//! wgpu compute runtime for hybrid GPU stepping.

use std::borrow::Cow;
use std::sync::{Arc, Mutex};

use bytemuck::{cast_slice, Zeroable};
use wgpu::util::DeviceExt;

use super::types::{
    GpuBody, GpuCollider, GpuManifold, GpuSimParams, GPU_MAX_BODIES, GPU_MAX_COLLIDERS,
    GPU_MAX_PAIRS, GPU_POLY_BUFFER_FLOATS,
};

const SHADER_WGSL: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/shaders/physics.wgsl"));

/// Wait for `map_async` without `mpsc::recv()`. On WASM, blocking `recv` on the JS main thread can
/// deadlock with the map callback; polling drives completion on the same thread.
///
/// On any error, calls [`Buffer::unmap`] so the buffer is never left mapped for the next
/// `copy_buffer_to_buffer` / `submit` (Chrome: "used in submit while mapped").
fn buffer_map_read_wait(
    buffer: &wgpu::Buffer,
    device: &wgpu::Device,
    slice: &wgpu::BufferSlice<'_>,
    context: &'static str,
) -> Result<(), String> {
    let slot: Arc<Mutex<Option<Result<(), wgpu::BufferAsyncError>>>> =
        Arc::new(Mutex::new(None));
    let slot_cb = Arc::clone(&slot);
    slice.map_async(wgpu::MapMode::Read, move |status| {
        if let Ok(mut g) = slot_cb.lock() {
            *g = Some(status);
        }
    });

    let mut polls = 0u32;
    while slot.lock().map(|g| g.is_none()).unwrap_or(true) {
        device.poll(wgpu::Maintain::Wait);
        polls += 1;
        if polls > 65_536 {
            buffer.unmap();
            return Err(format!(
                "{context}: buffer map did not complete after {polls} polls (GPU readback stuck)"
            ));
        }
    }

    let taken = slot.lock().ok().and_then(|mut g| g.take());
    match taken {
        Some(Ok(())) => Ok(()),
        Some(Err(e)) => {
            buffer.unmap();
            Err(format!("{context}: map_async failed: {e:?}"))
        }
        None => {
            buffer.unmap();
            Err(format!("{context}: map_async state missing"))
        }
    }
}

pub struct GpuPhysicsRuntime {
    device: wgpu::Device,
    queue: wgpu::Queue,
    bind_group: wgpu::BindGroup,
    pipeline_integrate: wgpu::ComputePipeline,
    pipeline_clear_atomics: wgpu::ComputePipeline,
    pipeline_narrowphase: wgpu::ComputePipeline,
    pipeline_solve: wgpu::ComputePipeline,
    pipeline_apply_deltas: wgpu::ComputePipeline,
    params_buffer: wgpu::Buffer,
    bodies_buffer: wgpu::Buffer,
    bodies_staging: wgpu::Buffer,
    colliders_buffer: wgpu::Buffer,
    poly_vertices_buffer: wgpu::Buffer,
    pairs_buffer: wgpu::Buffer,
    manifolds_buffer: wgpu::Buffer,
    _atomics_buffer: wgpu::Buffer,
}

impl GpuPhysicsRuntime {
    pub async fn new() -> Result<Self, String> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| "no suitable GPU adapter for Matcha2D physics".to_string())?;

        let mut limits = wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits());
        limits.max_storage_buffers_per_shader_stage =
            limits.max_storage_buffers_per_shader_stage.max(8);
        limits.max_buffer_size = limits.max_buffer_size.max(64 << 20);

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("matcha2d-physics-gpu"),
                    required_features: wgpu::Features::empty(),
                    required_limits: limits,
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .map_err(|e| format!("request_device failed: {e}"))?;

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("matcha2d-physics"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(SHADER_WGSL)),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("physics-bg-layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: Some(std::num::NonZeroU64::new(256).unwrap()),
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 6,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let body_stride = std::mem::size_of::<GpuBody>() as u64;
        let collider_stride = std::mem::size_of::<GpuCollider>() as u64;
        let manifold_stride = std::mem::size_of::<GpuManifold>() as u64;

        let params_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("physics-params"),
            contents: cast_slice(&[GpuSimParams::zeroed()]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bodies_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-bodies"),
            size: body_stride * GPU_MAX_BODIES as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let bodies_staging = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-bodies-staging"),
            size: body_stride * GPU_MAX_BODIES as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let colliders_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-colliders"),
            size: collider_stride * GPU_MAX_COLLIDERS as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let poly_vertices_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-poly-verts"),
            size: (GPU_POLY_BUFFER_FLOATS * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let pairs_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-pairs"),
            size: (GPU_MAX_PAIRS * 2 * std::mem::size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let manifolds_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-manifolds"),
            size: manifold_stride * GPU_MAX_PAIRS as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let atomics_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("physics-body-atomics"),
            size: (GPU_MAX_BODIES * 4 * std::mem::size_of::<i32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("physics-bg"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bodies_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: colliders_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: poly_vertices_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: pairs_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: manifolds_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: atomics_buffer.as_entire_binding(),
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("physics-pl"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let make_pipeline = |entry: &'static str| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&pipeline_layout),
                module: &shader,
                entry_point: Some(entry),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            })
        };

        let pipeline_integrate = make_pipeline("integrate_main");
        let pipeline_clear_atomics = make_pipeline("clear_atomics_main");
        let pipeline_narrowphase = make_pipeline("narrowphase_main");
        let pipeline_solve = make_pipeline("solve_contacts_main");
        let pipeline_apply_deltas = make_pipeline("apply_deltas_main");

        Ok(Self {
            device,
            queue,
            bind_group,
            pipeline_integrate,
            pipeline_clear_atomics,
            pipeline_narrowphase,
            pipeline_solve,
            pipeline_apply_deltas,
            params_buffer,
            bodies_buffer,
            bodies_staging,
            colliders_buffer,
            poly_vertices_buffer,
            pairs_buffer,
            manifolds_buffer,
            _atomics_buffer: atomics_buffer,
        })
    }

    /// Upload packed scene data and run GPU substeps + solver iterations, then read bodies back into `out_bodies`.
    pub fn step(
        &self,
        params: GpuSimParams,
        bodies: &[GpuBody],
        colliders: &[GpuCollider],
        poly_vertices: &[f32],
        pairs: &[u32],
        out_bodies: &mut [GpuBody],
        out_manifolds: &mut [GpuManifold],
    ) -> Result<(), String> {
        let body_count = params.body_count as usize;
        let collider_count = params.collider_count as usize;
        let pair_count = params.pair_count as usize;

        if body_count > GPU_MAX_BODIES || collider_count > GPU_MAX_COLLIDERS || pair_count > GPU_MAX_PAIRS {
            return Err("GPU physics limits exceeded (bodies/colliders/pairs)".to_string());
        }
        if bodies.len() < body_count || out_bodies.len() < body_count {
            return Err("body slice length mismatch".to_string());
        }
        if colliders.len() < collider_count {
            return Err("collider slice length mismatch".to_string());
        }
        if pairs.len() < pair_count * 2 {
            return Err("pairs slice length mismatch".to_string());
        }
        if poly_vertices.len() > GPU_POLY_BUFFER_FLOATS {
            return Err("poly vertex buffer too large".to_string());
        }

        self.queue
            .write_buffer(&self.params_buffer, 0, cast_slice(&[params]));
        let body_bytes = bytemuck::cast_slice(&bodies[..body_count]);
        self.queue.write_buffer(&self.bodies_buffer, 0, body_bytes);
        let col_bytes = bytemuck::cast_slice(&colliders[..collider_count]);
        self.queue.write_buffer(&self.colliders_buffer, 0, col_bytes);
        let mut poly_pad = vec![0.0f32; GPU_POLY_BUFFER_FLOATS];
        poly_pad[..poly_vertices.len()].copy_from_slice(poly_vertices);
        self.queue
            .write_buffer(&self.poly_vertices_buffer, 0, cast_slice(&poly_pad));
        self.queue
            .write_buffer(&self.pairs_buffer, 0, cast_slice(&pairs[..pair_count * 2]));

        let wg_bodies = ((body_count + 63) / 64).max(1) as u32;
        let wg_pairs = ((pair_count + 63) / 64).max(1) as u32;

        let substeps = params.substeps.max(1);
        let solver_iters = params.solver_iterations.max(1);

        // One encoder + one submit for all substeps (was 20× submits per substep → huge CPU/GPU sync cost).
        {
            let mut encoder = self.device.create_command_encoder(&Default::default());
            {
                let mut pass = encoder.begin_compute_pass(&Default::default());
                for _ in 0..substeps {
                    pass.set_pipeline(&self.pipeline_integrate);
                    pass.set_bind_group(0, &self.bind_group, &[]);
                    pass.dispatch_workgroups(wg_bodies, 1, 1);

                    pass.set_pipeline(&self.pipeline_narrowphase);
                    pass.set_bind_group(0, &self.bind_group, &[]);
                    pass.dispatch_workgroups(wg_pairs, 1, 1);

                    for _ in 0..solver_iters {
                        pass.set_pipeline(&self.pipeline_clear_atomics);
                        pass.set_bind_group(0, &self.bind_group, &[]);
                        pass.dispatch_workgroups(wg_bodies, 1, 1);

                        pass.set_pipeline(&self.pipeline_solve);
                        pass.set_bind_group(0, &self.bind_group, &[]);
                        pass.dispatch_workgroups(wg_pairs, 1, 1);

                        pass.set_pipeline(&self.pipeline_apply_deltas);
                        pass.set_bind_group(0, &self.bind_group, &[]);
                        pass.dispatch_workgroups(wg_bodies, 1, 1);
                    }
                }
            }
            self.queue.submit(Some(encoder.finish()));
        }

        self.device.poll(wgpu::Maintain::Wait);

        // Read back bodies
        let body_stride = std::mem::size_of::<GpuBody>() as u64;
        let body_size = body_stride * body_count as u64;
        let mut encoder = self.device.create_command_encoder(&Default::default());
        encoder.copy_buffer_to_buffer(&self.bodies_buffer, 0, &self.bodies_staging, 0, body_size);
        self.queue.submit(Some(encoder.finish()));
        self.device.poll(wgpu::Maintain::Wait);

        {
            let slice = self.bodies_staging.slice(..body_size);
            match buffer_map_read_wait(&self.bodies_staging, &self.device, &slice, "bodies readback") {
                Ok(()) => {
                    let data = slice.get_mapped_range();
                    let gpu_bodies: &[GpuBody] = bytemuck::cast_slice(&data[..body_size as usize]);
                    out_bodies[..body_count].copy_from_slice(gpu_bodies);
                }
                Err(e) => return Err(e),
            }
        }
        self.bodies_staging.unmap();

        // Read back manifolds for contact tracker
        let manifold_stride = std::mem::size_of::<GpuManifold>() as u64;
        let manifold_size = manifold_stride * pair_count as u64;
        if !out_manifolds.is_empty() && pair_count > 0 {
            let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("manifolds-staging-temp"),
                size: manifold_size,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let mut encoder = self.device.create_command_encoder(&Default::default());
            encoder.copy_buffer_to_buffer(
                &self.manifolds_buffer,
                0,
                &staging,
                0,
                manifold_size,
            );
            self.queue.submit(Some(encoder.finish()));
            self.device.poll(wgpu::Maintain::Wait);

            let slice = staging.slice(..manifold_size);
            match buffer_map_read_wait(&staging, &self.device, &slice, "manifolds readback") {
                Ok(()) => {
                    let data = slice.get_mapped_range();
                    let ms: &[GpuManifold] = bytemuck::cast_slice(&data[..manifold_size as usize]);
                    let n = pair_count.min(out_manifolds.len());
                    out_manifolds[..n].copy_from_slice(&ms[..n]);
                }
                Err(e) => return Err(e),
            }
            staging.unmap();
        }

        Ok(())
    }
}
