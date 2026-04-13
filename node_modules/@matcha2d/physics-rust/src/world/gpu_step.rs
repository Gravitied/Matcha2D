//! WebGPU stepping, buffer packing, and manifold readback for [`super::PhysicsWorld`].
use super::PhysicsWorld;
use crate::body::RigidBodyType;
use crate::collider::{ColliderHandle, Shape};
use crate::contact::{ContactManifold, ContactPoint};
use crate::gpu::{
    GpuBody, GpuCollider, GpuManifold, GpuSimParams, GPU_MAX_BODIES, GPU_MAX_COLLIDERS,
    GPU_MAX_PAIRS, GPU_MAX_POLY_VERTS, GPU_POLY_BUFFER_FLOATS,
};
use crate::math::{Rot, Vec2};
use crate::narrowphase;
use bytemuck::Zeroable;

impl PhysicsWorld {
    /// CPU broadphase + **step-start TOI recovery** (same as [`Self::step_cpu`]), then GPU
    /// narrowphase + Jacobi-style contact resolve + integration. In-step CCD is still GPU-discrete;
    /// use [`Self::step_cpu`] for full PGS + substepping parity on difficult scenes.
    pub(super) fn step_gpu(&mut self) -> Result<(), String> {
        let gpu = self
            .gpu_runtime
            .take()
            .ok_or_else(|| "GPU runtime missing".to_string())?;

        self.snapshot_body_poses_at_step_start();
        self.update_sleep();
        self.update_bvh();

        let pairs = if self.swept_broadphase {
            self.broadphase_swept()
        } else {
            self.bvh.query_pairs()
        };

        let mut pair_indices: Vec<u32> = Vec::new();
        for (ca, cb) in &pairs {
            if !self.can_colliders_collide(*ca, *cb) {
                continue;
            }
            pair_indices.push(ca.index() as u32);
            pair_indices.push(cb.index() as u32);
        }

        let pair_count = pair_indices.len() / 2;
        if pair_count > GPU_MAX_PAIRS {
            return Err(format!(
                "broadphase produced {pair_count} pairs; GPU_MAX_PAIRS is {GPU_MAX_PAIRS}"
            ));
        }

        let body_slots = self.bodies.slot_count();
        let collider_slots = self.colliders.slot_count();
        if body_slots > GPU_MAX_BODIES || collider_slots > GPU_MAX_COLLIDERS {
            return Err("body or collider capacity exceeds GPU limits".to_string());
        }

        // TOI / tunnel recovery at step-start poses (same order as `step_cpu`) before GPU pack.
        self.last_toi_debug_flat.clear();
        for (ca, cb) in &pairs {
            if !self.can_colliders_collide(*ca, *cb) {
                continue;
            }
            let hint = self.contact_tracker.poly_sat_hint(*ca, *cb);
            if narrowphase::detect_collision_with_hint(
                &self.bodies,
                &self.colliders,
                *ca,
                *cb,
                hint,
            )
            .is_some()
            {
                continue;
            }
            let _ = self.try_recover_tunnelled_manifold(*ca, *cb);
        }

        let mut gpu_bodies = vec![GpuBody::zeroed(); body_slots];
        self.pack_gpu_bodies(&mut gpu_bodies);

        let mut gpu_colliders = vec![GpuCollider::zeroed(); collider_slots];
        let mut poly_verts = vec![0.0f32; GPU_POLY_BUFFER_FLOATS];
        self.pack_gpu_colliders(&mut gpu_colliders, &mut poly_verts)?;

        let mut params = GpuSimParams::zeroed();
        // Match CPU `velocity_solver`: each outer "substep" uses `dt / num_substeps`, not full `dt`.
        let num_substeps = self.config.num_iterations.max(1) as f32;
        params.dt = self.dt / num_substeps;
        params.gravity_x = self.gravity.x;
        params.gravity_y = self.gravity.y;
        params.pair_count = pair_count as u32;
        params.body_count = body_slots as u32;
        params.collider_count = collider_slots as u32;
        params.solver_iterations = self.config.num_internal_iterations.max(1) as u32;
        params.substeps = num_substeps as u32;
        params.penetration_slop = self.penetration_slop;
        // GPU uniform must stay finite; uncapped CPU mode uses a large sentinel (still bounded per step by pen/w).
        params.max_corrective_velocity = if self.max_corrective_velocity.is_finite() {
            self.max_corrective_velocity
        } else {
            1.0e12
        };

        let mut out_bodies = vec![GpuBody::zeroed(); body_slots];
        let mut out_manifolds = vec![GpuManifold::zeroed(); pair_count.max(1)];

        let step_res = gpu.step(
            params,
            &gpu_bodies,
            &gpu_colliders,
            &poly_verts,
            &pair_indices,
            &mut out_bodies,
            &mut out_manifolds,
        );
        self.gpu_runtime = Some(gpu);
        step_res?;

        self.apply_gpu_body_readback(&out_bodies);

        let manifolds_cpu = self.gpu_manifolds_to_contact_manifolds(pair_count, &out_manifolds);
        self.contact_tracker.update(manifolds_cpu);

        self.wake_contacting_bodies();
        self.enforce_world_bounds();
        Ok(())
    }

    fn pack_gpu_bodies(&self, out: &mut [GpuBody]) {
        for idx in 0..out.len() {
            if !self.bodies.slot_is_live(idx) {
                out[idx] = GpuBody::zeroed();
                out[idx].body_type = 1;
                out[idx].inv_mass = 0.0;
                out[idx].inv_inertia = 0.0;
                continue;
            }
            let b = self.bodies.get_unchecked(idx);
            let body_type = match b.body_type {
                RigidBodyType::Dynamic => 0u32,
                RigidBodyType::Static => 1u32,
                RigidBodyType::Kinematic => 2u32,
            };
            let mut flags = 0u32;
            if b.sleeping {
                flags |= 1u32;
            }
            out[idx] = GpuBody {
                position: [b.pos.x, b.pos.y],
                angle: b.rot.angle(),
                angvel: b.angvel,
                linvel: [b.linvel.x, b.linvel.y],
                inv_mass: b.inv_mass,
                inv_inertia: b.inv_inertia,
                body_type,
                flags,
                linear_damping: b.linear_damping,
                angular_damping: b.angular_damping,
                gravity_scale: b.gravity_scale,
                _pad_b: [0.0; 3],
            };
        }
    }

    fn pack_gpu_colliders(
        &self,
        out: &mut [GpuCollider],
        poly_verts: &mut [f32],
    ) -> Result<(), String> {
        poly_verts.fill(0.0);
        for idx in 0..out.len() {
            if !self.colliders.slot_is_live(idx) {
                out[idx] = GpuCollider::zeroed();
                out[idx].shape_kind = 1;
                out[idx].param0 = 0.01;
                out[idx].param1 = 0.01;
                continue;
            }
            let c = self.colliders.get_unchecked(idx);
            let body_index = c
                .parent
                .map(|h| h.index() as u32)
                .unwrap_or(u32::MAX);
            let (shape_kind, param0, param1, vert_count, vert_start) = match &c.shape {
                Shape::Ball { radius } => (0u32, *radius, *radius, 0u32, 0u32),
                Shape::Cuboid { hx, hy } => (1u32, *hx, *hy, 0u32, 0u32),
                Shape::Polygon { vertices } => {
                    let n = vertices.len().min(GPU_MAX_POLY_VERTS as usize);
                    let start = idx * GPU_MAX_POLY_VERTS as usize * 2;
                    for i in 0..n {
                        poly_verts[start + i * 2] = vertices[i].x;
                        poly_verts[start + i * 2 + 1] = vertices[i].y;
                    }
                    let r = c.shape.max_support_radius_from_shape_origin();
                    (2u32, r, r, n as u32, (idx * GPU_MAX_POLY_VERTS as usize * 2) as u32)
                }
            };
            let mut collider_flags = 0u32;
            if c.is_sensor() {
                collider_flags |= 1u32;
            }
            let sa = c.local_rot.sin;
            let ca = c.local_rot.cos;
            out[idx] = GpuCollider {
                shape_kind,
                body_index,
                local_pos: [c.local_pos.x, c.local_pos.y],
                local_sin: sa,
                local_cos: ca,
                friction: c.friction,
                restitution: c.restitution,
                collider_flags,
                param0,
                param1,
                poly_vert_start: vert_start,
                poly_vert_count: vert_count,
                _p0: 0,
                _p1: 0,
                _p2: 0,
            };
        }
        Ok(())
    }

    fn apply_gpu_body_readback(&mut self, gpu_bodies: &[GpuBody]) {
        for idx in self.bodies.live_indices() {
            if idx >= gpu_bodies.len() {
                continue;
            }
            let gb = &gpu_bodies[idx];
            let h = self.bodies.handle_for_index(idx);
            if let Some(b) = self.bodies.get_mut(h) {
                if !(b.is_dynamic() || b.is_kinematic()) {
                    continue;
                }
                b.pos = Vec2::new(gb.position[0], gb.position[1]);
                b.rot = Rot::from_angle(gb.angle);
                b.linvel = Vec2::new(gb.linvel[0], gb.linvel[1]);
                b.angvel = gb.angvel;
            }
        }
    }

    fn gpu_manifolds_to_contact_manifolds(
        &self,
        pair_count: usize,
        gpu: &[GpuManifold],
    ) -> Vec<(ColliderHandle, ColliderHandle, ContactManifold)> {
        let mut out = Vec::with_capacity(pair_count);
        let n = pair_count.min(gpu.len());
        for i in 0..n {
            let m = gpu[i];
            if m.valid == 0 {
                continue;
            }
            let ia = m.collider_a as usize;
            let ib = m.collider_b as usize;
            if !self.colliders.slot_is_live(ia) || !self.colliders.slot_is_live(ib) {
                continue;
            }
            let ha = self.colliders.handle_for_index(ia);
            let hb = self.colliders.handle_for_index(ib);
            let normal = Vec2::new(m.normal[0], m.normal[1]);
            let mut mf = ContactManifold::new(normal);
            if m.contact_count >= 1 {
                mf.contacts.push(ContactPoint {
                    local_a: Vec2::new(m.c0.local_a[0], m.c0.local_a[1]),
                    local_b: Vec2::new(m.c0.local_b[0], m.c0.local_b[1]),
                    penetration: m.c0.penetration,
                    impulse_id: 0,
                    is_new: true,
                });
            }
            if m.contact_count >= 2 {
                mf.contacts.push(ContactPoint {
                    local_a: Vec2::new(m.c1.local_a[0], m.c1.local_a[1]),
                    local_b: Vec2::new(m.c1.local_b[0], m.c1.local_b[1]),
                    penetration: m.c1.penetration,
                    impulse_id: 1,
                    is_new: true,
                });
            }
            out.push((ha, hb, mf));
        }
        out
    }
}
