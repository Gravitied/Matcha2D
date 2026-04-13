//! WebGPU / wgpu compute backend for hybrid simulation (CPU broadphase, GPU narrowphase + Jacobi resolve).

mod runtime;
mod types;

pub use runtime::GpuPhysicsRuntime;
pub use types::{
    GpuBody, GpuCollider, GpuManifold, GpuSimParams, GPU_MAX_BODIES, GPU_MAX_COLLIDERS, GPU_MAX_PAIRS,
    GPU_MAX_POLY_VERTS, GPU_POLY_BUFFER_FLOATS,
};
