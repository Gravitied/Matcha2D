use crate::aabb::{self, AABB};
use crate::body::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::bvh::DynamicBvh;
use crate::collider::{Collider, ColliderHandle, ColliderSet, Shape};
use crate::contact::{ContactEvent, ContactTracker};
use crate::contact_constraint::{ContactConstraintSet, SolverDiagnostics};
use crate::gpu::GpuPhysicsRuntime;
use crate::math::Vec2;
use crate::narrowphase;
use crate::solver::{self, SolverConfig};
use crate::solver_body::SolverBodySet;

/// Default sleep velocity threshold (m/s) — keep strict so we do not sleep mid-settle.
const DEFAULT_SLEEP_VEL_THRESHOLD: f32 = 0.02;
/// Default sleep angular velocity threshold (rad/s)
const DEFAULT_SLEEP_ANG_VEL_THRESHOLD: f32 = 0.06;
/// Default time (seconds) a body must be below thresholds before sleeping
const DEFAULT_SLEEP_TIME_THRESHOLD: f32 = 2.0;
/// AABB margin for swept broadphase (extra padding against tunneling).
/// Keep small vs world scale — large values flood broadphase with false pairs and TOI work.
const SWEPT_AABB_MARGIN: f32 = 0.05;

mod toi_types;
mod toi;
mod gpu_step;

use toi_types::ToiStepSnapshot;

pub struct PhysicsWorld {
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    bvh: DynamicBvh,
    pub contact_tracker: ContactTracker,
    config: SolverConfig,
    gravity: Vec2,
    dt: f32,
    bvh_leaves: std::collections::HashMap<ColliderHandle, usize>,
    collider_body_indices: std::collections::HashMap<ColliderHandle, usize>,
    sleep_vel_threshold: f32,
    sleep_ang_vel_threshold: f32,
    sleep_time_threshold: f32,
    pub swept_broadphase: bool,
    position_correction_beta: f32,
    max_corrective_velocity: f32,
    penetration_slop: f32,
    spring_natural_frequency: f32,
    spring_damping_ratio: f32,
    /// Optional cap on |linear velocity| during PGS (`f32::INFINITY` disables).
    max_solver_linear_velocity: f32,
    /// Optional cap on |angular velocity| during PGS (`f32::INFINITY` disables).
    max_solver_angular_velocity: f32,
    /// PGS / SOR relaxation in `(0, 1]` — under-relaxation stabilizes stiff contact stacks without velocity clamps.
    pgs_relaxation: f32,
    /// Global cap on |v| before substep position integration (`f32::INFINITY` disables).
    max_integrate_linear_velocity: f32,
    max_integrate_angular_velocity: f32,
    /// Impulse / clamp stats from the last CPU [`Self::solve`] (GPU step does not update this).
    pub last_solver_diagnostics: SolverDiagnostics,
    /// Optional axis-aligned world bounds (min, max); dynamic bodies are clamped after each step.
    world_bounds: Option<(Vec2, Vec2)>,
    /// Per-body pose + velocity at the very start of `step()` — TOI recovery for discrete misses.
    body_toi_snapshot_at_step_start: Vec<Option<ToiStepSnapshot>>,
    /// WebGPU compute backend (optional). Use [`Self::init_gpu`] / [`Self::init_gpu_blocking`].
    pub gpu_runtime: Option<GpuPhysicsRuntime>,
    /// When `true` and [`Self::gpu_runtime`] is `Some`, [`Self::step`] uses the GPU pipeline.
    pub gpu_acceleration_enabled: bool,
    /// When true, append TOI recovery samples to [`Self::last_toi_debug_flat`] on each CPU step.
    pub debug_collision_logging: bool,
    /// Flat debug samples from the last CPU step, 18 `f32` per TOI event:
    /// `ca, cb, ha, hb, u, pre_ax, pre_ay, pre_bx, pre_by, post_ax, post_ay, post_bx, post_by, max_pen, pre_θa, pre_θb, post_θa, post_θb`.
    pub last_toi_debug_flat: Vec<f32>,
}

impl PhysicsWorld {
    pub fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            bvh: DynamicBvh::new(),
            contact_tracker: ContactTracker::new(),
            config: SolverConfig::default(),
            gravity: Vec2::new(0.0, -9.81),
            dt: 1.0 / 60.0,
            bvh_leaves: std::collections::HashMap::new(),
            collider_body_indices: std::collections::HashMap::new(),
            sleep_vel_threshold: DEFAULT_SLEEP_VEL_THRESHOLD,
            sleep_ang_vel_threshold: DEFAULT_SLEEP_ANG_VEL_THRESHOLD,
            sleep_time_threshold: DEFAULT_SLEEP_TIME_THRESHOLD,
            swept_broadphase: true,
            position_correction_beta: 0.02,
            // Spring bias only — no arbitrary separation-speed ceiling unless set from host.
            max_corrective_velocity: f32::INFINITY,
            penetration_slop: 0.02,
            spring_natural_frequency: 22.0,
            spring_damping_ratio: 5.5,
            max_solver_linear_velocity: f32::INFINITY,
            max_solver_angular_velocity: f32::INFINITY,
            pgs_relaxation: 0.88,
            max_integrate_linear_velocity: f32::INFINITY,
            max_integrate_angular_velocity: f32::INFINITY,
            last_solver_diagnostics: SolverDiagnostics::default(),
            world_bounds: None,
            body_toi_snapshot_at_step_start: Vec::new(),
            gpu_runtime: None,
            gpu_acceleration_enabled: false,
            debug_collision_logging: false,
            last_toi_debug_flat: Vec::new(),
        }
    }

    /// Enable accumulation of TOI recovery samples into [`Self::last_toi_debug_flat`] (CPU step only).
    pub fn set_debug_collision_logging(&mut self, enabled: bool) {
        self.debug_collision_logging = enabled;
    }

    /// Initialize WebGPU (async). WASM builds should call this from JS; native code can use [`Self::init_gpu_blocking`].
    pub async fn init_gpu(&mut self) -> Result<(), String> {
        let rt = GpuPhysicsRuntime::new().await?;
        self.gpu_runtime = Some(rt);
        // Do not enable stepping here: WASM WebGPU work must not run until the host opts in
        // (`set_gpu_acceleration_enabled`), otherwise the first `step` can block the JS main thread.
        self.gpu_acceleration_enabled = false;
        Ok(())
    }

    /// Initialize WebGPU synchronously (native targets only).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn init_gpu_blocking(&mut self) -> Result<(), String> {
        let rt = pollster::block_on(GpuPhysicsRuntime::new())?;
        self.gpu_runtime = Some(rt);
        self.gpu_acceleration_enabled = false;
        Ok(())
    }

    pub fn set_gpu_acceleration_enabled(&mut self, enabled: bool) {
        self.gpu_acceleration_enabled = enabled;
    }

    pub fn with_gravity(mut self, x: f32, y: f32) -> Self {
        self.gravity = Vec2::new(x, y);
        self
    }

    pub fn with_config(mut self, config: SolverConfig) -> Self {
        self.config = config;
        self
    }

    pub fn set_gravity(&mut self, x: f32, y: f32) {
        self.gravity = Vec2::new(x, y);
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.dt = dt.max(1e-6);
    }

    pub fn get_dt(&self) -> f32 {
        self.dt
    }

    pub fn set_sleep_thresholds(&mut self, vel: f32, ang_vel: f32, time: f32) {
        self.sleep_vel_threshold = vel;
        self.sleep_ang_vel_threshold = ang_vel;
        self.sleep_time_threshold = time;
    }

    pub fn set_position_correction(&mut self, beta: f32, max_velocity: f32) {
        self.position_correction_beta = beta;
        self.max_corrective_velocity = max_velocity;
    }

    pub fn set_penetration_slop(&mut self, slop: f32) {
        self.penetration_slop = slop.max(0.0);
    }

    pub fn set_spring_coefficients(&mut self, natural_frequency: f32, damping_ratio: f32) {
        self.spring_natural_frequency = natural_frequency;
        self.spring_damping_ratio = damping_ratio;
    }

    /// Pass non-finite values or `<= 0` to disable that axis (no post-PGS velocity clamp).
    pub fn set_max_solver_velocities(&mut self, max_linear: f32, max_angular: f32) {
        self.max_solver_linear_velocity = if max_linear.is_finite() && max_linear > 0.0 {
            max_linear
        } else {
            f32::INFINITY
        };
        self.max_solver_angular_velocity = if max_angular.is_finite() && max_angular > 0.0 {
            max_angular
        } else {
            f32::INFINITY
        };
    }

    /// PGS impulse under-relaxation in `(0, 1]` (typical 0.75–0.95 for stacks). Ignored outside range.
    pub fn set_pgs_relaxation(&mut self, relaxation: f32) {
        self.pgs_relaxation = relaxation.clamp(0.05, 1.0);
    }

    /// Hard cap on linear/angular speed before substep integration (`<= 0` or non-finite disables).
    pub fn set_max_integrate_velocities(&mut self, max_linear: f32, max_angular: f32) {
        self.max_integrate_linear_velocity = if max_linear.is_finite() && max_linear > 0.0 {
            max_linear
        } else {
            f32::INFINITY
        };
        self.max_integrate_angular_velocity = if max_angular.is_finite() && max_angular > 0.0 {
            max_angular
        } else {
            f32::INFINITY
        };
    }

    /// Axis-aligned bounds for dynamic bodies (meters). Pass `min_x = f32::NAN` to disable.
    pub fn set_world_bounds(&mut self, min_x: f32, min_y: f32, max_x: f32, max_y: f32) {
        if min_x.is_nan()
            || min_y.is_nan()
            || max_x.is_nan()
            || max_y.is_nan()
            || max_x <= min_x
            || max_y <= min_y
        {
            self.world_bounds = None;
        } else {
            self.world_bounds = Some((Vec2::new(min_x, min_y), Vec2::new(max_x, max_y)));
        }
    }

    pub fn set_body_linear_damping(&mut self, handle: u32, damping: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.linear_damping = damping.max(0.0);
        }
    }

    pub fn set_body_angular_damping(&mut self, handle: u32, damping: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.angular_damping = damping.max(0.0);
        }
    }

    pub fn set_solver_config(
        &mut self,
        num_iterations: usize,
        num_internal_iterations: usize,
        block_solver: bool,
    ) {
        self.config.num_iterations = num_iterations;
        self.config.num_internal_iterations = num_internal_iterations;
        self.config.block_solver = block_solver;
    }

    // ========== Body Management ==========

    pub fn create_dynamic_body(&mut self, x: f32, y: f32) -> u32 {
        self.bodies.insert(RigidBody::dynamic(Vec2::new(x, y))).0
    }

    pub fn create_static_body(&mut self, x: f32, y: f32) -> u32 {
        self.bodies
            .insert(RigidBody::static_body(Vec2::new(x, y)))
            .0
    }

    pub fn create_kinematic_body(&mut self, x: f32, y: f32) -> u32 {
        self.bodies.insert(RigidBody::kinematic(Vec2::new(x, y))).0
    }

    pub fn remove_body(&mut self, handle: u32) {
        let body_handle = RigidBodyHandle(handle);
        let colliders_to_remove: Vec<ColliderHandle> = self
            .colliders
            .live_indices()
            .into_iter()
            .filter_map(|idx| {
                let c = self.colliders.get_unchecked(idx);
                if c.parent == Some(body_handle) {
                    Some(self.colliders.handle_for_index(idx))
                } else {
                    None
                }
            })
            .collect();

        for ch in colliders_to_remove {
            self.remove_collider(ch.0);
        }

        self.bodies.remove(body_handle);
    }

    pub fn set_body_position(&mut self, handle: u32, x: f32, y: f32) {
        let body_handle = RigidBodyHandle(handle);
        let colliders: Vec<ColliderHandle> = self
            .colliders
            .live_indices()
            .into_iter()
            .filter_map(|idx| {
                let c = self.colliders.get_unchecked(idx);
                if c.parent == Some(body_handle) {
                    Some(self.colliders.handle_for_index(idx))
                } else {
                    None
                }
            })
            .collect();

        if let Some(body) = self.bodies.get_mut(body_handle) {
            body.pos = Vec2::new(x, y);
            body.wake();
        }

        for ch in colliders {
            self.update_collider_bvh(ch);
        }
    }

    pub fn set_body_velocity(&mut self, handle: u32, vx: f32, vy: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.linvel = Vec2::new(vx, vy);
            body.wake();
        }
    }

    pub fn set_body_angle(&mut self, handle: u32, angle: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.rot = crate::math::Rot::from_angle(angle);
            body.wake();
        }
    }

    pub fn set_body_collision_groups(&mut self, handle: u32, groups: u32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.collision_groups = groups;
        }
    }

    pub fn set_body_collision_mask(&mut self, handle: u32, mask: u32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.collision_mask = mask;
        }
    }

    pub fn wake_body(&mut self, handle: u32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.wake();
        }
    }

    pub fn sleep_body(&mut self, handle: u32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.sleep();
        }
    }

    pub fn is_body_sleeping(&self, handle: u32) -> bool {
        self.bodies
            .get(RigidBodyHandle(handle))
            .map(|b| b.sleeping)
            .unwrap_or(false)
    }

    pub fn get_body_position(&self, handle: u32) -> Vec<f32> {
        if let Some(body) = self.bodies.get(RigidBodyHandle(handle)) {
            vec![body.pos.x, body.pos.y]
        } else {
            vec![0.0, 0.0]
        }
    }

    pub fn get_body_velocity(&self, handle: u32) -> Vec<f32> {
        if let Some(body) = self.bodies.get(RigidBodyHandle(handle)) {
            vec![body.linvel.x, body.linvel.y]
        } else {
            vec![0.0, 0.0]
        }
    }

    pub fn get_body_angle(&self, handle: u32) -> f32 {
        if let Some(body) = self.bodies.get(RigidBodyHandle(handle)) {
            body.rot.angle()
        } else {
            0.0
        }
    }

    pub fn set_body_angular_velocity(&mut self, handle: u32, angvel: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.angvel = angvel;
            body.wake();
        }
    }

    pub fn get_body_angular_velocity(&self, handle: u32) -> f32 {
        if let Some(body) = self.bodies.get(RigidBodyHandle(handle)) {
            body.angvel
        } else {
            0.0
        }
    }

    pub fn apply_impulse(&mut self, handle: u32, fx: f32, fy: f32) {
        if let Some(body) = self.bodies.get_mut(RigidBodyHandle(handle)) {
            body.apply_impulse(Vec2::new(fx, fy));
        }
    }

    // ========== Collider Management ==========

    pub fn create_ball_collider(&mut self, radius: f32, body_handle: u32) -> u32 {
        let bh = RigidBodyHandle(body_handle);
        let collider = crate::collider::ColliderBuilder::ball(radius)
            .parent(bh)
            .build();
        self.insert_collider(collider, bh)
    }

    pub fn create_box_collider(&mut self, hx: f32, hy: f32, body_handle: u32) -> u32 {
        let bh = RigidBodyHandle(body_handle);
        let collider = crate::collider::ColliderBuilder::cuboid(hx, hy)
            .parent(bh)
            .build();
        self.insert_collider(collider, bh)
    }

    pub fn create_polygon_collider(
        &mut self,
        vertices_x: &[f32],
        vertices_y: &[f32],
        body_handle: u32,
    ) -> u32 {
        let bh = RigidBodyHandle(body_handle);
        let verts: Vec<Vec2> = vertices_x
            .iter()
            .zip(vertices_y.iter())
            .map(|(&x, &y)| Vec2::new(x, y))
            .collect();
        let verts = ensure_ccw_polygon_vertices(verts);
        if Shape::validate_polygon_vertices(&verts).is_err() {
            return ColliderHandle::INVALID.0;
        }
        let collider = crate::collider::ColliderBuilder::polygon(verts)
            .parent(bh)
            .build();
        self.insert_collider(collider, bh)
    }

    /// Push a dynamic body out of penetrations with nearby colliders (spawn / teleport helper).
    /// Uses current poses and narrowphase; updates the BVH for the body’s colliders.
    pub fn resolve_overlaps_for_body(&mut self, body_handle: u32) {
        let bh = RigidBodyHandle(body_handle);
        let Some(body) = self.bodies.get(bh) else {
            return;
        };
        if !body.is_dynamic() {
            return;
        }

        let my_colliders: Vec<ColliderHandle> = self
            .colliders
            .live_indices()
            .into_iter()
            .filter_map(|idx| {
                let c = self.colliders.get_unchecked(idx);
                if c.parent == Some(bh) {
                    Some(self.colliders.handle_for_index(idx))
                } else {
                    None
                }
            })
            .collect();

        let slop = self.penetration_slop;
        let mut total_delta = Vec2::zero();

        for &me in &my_colliders {
            let Some(col_me) = self.colliders.get(me) else {
                continue;
            };
            let aabb = self.compute_collider_aabb(col_me);
            let neighbors = self.bvh.query_aabb(aabb);

            for oth in neighbors {
                if oth == me {
                    continue;
                }
                if !self.can_colliders_collide(me, oth) {
                    continue;
                }

                let manifold = if let Some(m) = narrowphase::detect_collision_with_hint(
                    &self.bodies,
                    &self.colliders,
                    me,
                    oth,
                    self.contact_tracker.poly_sat_hint(me, oth),
                ) {
                    m
                } else if let Some(m) = narrowphase::detect_collision_with_hint(
                    &self.bodies,
                    &self.colliders,
                    oth,
                    me,
                    self.contact_tracker.poly_sat_hint(oth, me),
                ) {
                    m.flipped_across_bodies()
                } else {
                    continue;
                };

                let pen = manifold.max_penetration();
                if pen <= 1e-6 {
                    continue;
                }
                let Some(n) = manifold.normal.try_normalize() else {
                    continue;
                };

                let col_a = self.colliders.get(me).unwrap();
                let col_b = self.colliders.get(oth).unwrap();
                let Some(ha) = col_a.parent else {
                    continue;
                };
                let Some(hb) = col_b.parent else {
                    continue;
                };
                let im_a = self.bodies.get(ha).map(|b| b.inv_mass).unwrap_or(0.0);
                let im_b = self.bodies.get(hb).map(|b| b.inv_mass).unwrap_or(0.0);
                let correction = pen + slop;
                let total_im = im_a + im_b;
                if total_im <= 1e-20 {
                    continue;
                }

                if ha == bh {
                    let da = correction * (im_a / total_im);
                    total_delta -= n * da;
                }
                if hb == bh {
                    let db = correction * (im_b / total_im);
                    total_delta += n * db;
                }
            }
        }

        if let Some(body) = self.bodies.get_mut(bh) {
            body.pos += total_delta;
            body.wake();
        }
        for ch in my_colliders {
            self.update_collider_bvh(ch);
        }
    }

    /// Debug draw data: per contact point `[cx, cy, nx, ny, penetration, normal_impulse, ...]`.
    pub fn debug_contact_details_flat(&self) -> Vec<f32> {
        let mut out = Vec::new();
        for pair in self.contact_tracker.pairs() {
            let Some(col_a) = self.colliders.get(pair.collider_a) else {
                continue;
            };
            let Some(col_b) = self.colliders.get(pair.collider_b) else {
                continue;
            };
            let (pos_a, rot_a) = narrowphase::collider_world_transform(&self.bodies, col_a);
            let (pos_b, rot_b) = narrowphase::collider_world_transform(&self.bodies, col_b);
            let nn = pair
                .manifold
                .normal
                .try_normalize()
                .unwrap_or(Vec2::zero());
            for (_i, c) in pair.manifold.contacts.iter().enumerate() {
                let r_a = rot_a.mul_vec(c.local_a);
                let r_b = rot_b.mul_vec(c.local_b);
                let wa = pos_a + r_a;
                let wb = pos_b + r_b;
                let mid = (wa + wb) * 0.5;
                let impulse = 0.0;
                out.extend_from_slice(&[
                    mid.x,
                    mid.y,
                    nn.x,
                    nn.y,
                    c.penetration,
                    impulse,
                ]);
            }
        }
        out
    }

    fn insert_collider(&mut self, collider: Collider, body_handle: RigidBodyHandle) -> u32 {
        let ch = self.colliders.insert(collider);
        let body_idx = body_handle.index();
        self.collider_body_indices.insert(ch, body_idx);
        let col = self.colliders.get(ch).unwrap();
        let aabb = self.compute_collider_aabb(col);
        let leaf = self.bvh.insert_leaf(aabb, ch);
        self.bvh_leaves.insert(ch, leaf);

        // Update body mass properties from all colliders
        self.recompute_body_mass(body_handle);

        ch.0
    }

    /// Recompute mass properties for a body from all its colliders.
    /// Normalizes mass to a reasonable range (1-100) regardless of pixel scale
    /// so the solver can produce meaningful velocity corrections.
    fn recompute_body_mass(&mut self, body_handle: RigidBodyHandle) {
        if let Some(body) = self.bodies.get_mut(body_handle) {
            if !body.is_dynamic() {
                return;
            }
            let mut total_area = 0.0;
            let mut total_inertia_unscaled = 0.0;
            for idx in self.colliders.live_indices() {
                let col = self.colliders.get_unchecked(idx);
                if col.parent == Some(body_handle) {
                    let (mass, inertia) = col.shape.compute_mass_properties(col.density);
                    total_area += mass;
                    total_inertia_unscaled += inertia;
                }
            }
            if total_area > 0.0 {
                body.mass = total_area;
                body.inv_mass = 1.0 / total_area;
                body.inertia = total_inertia_unscaled;
                body.inv_inertia = if total_inertia_unscaled > 0.0 {
                    1.0 / total_inertia_unscaled
                } else {
                    0.0
                };
            } else {
                body.mass = 1.0;
                body.inv_mass = 1.0;
                body.inertia = 1.0;
                body.inv_inertia = 0.0;
            }
        }
    }

    pub fn remove_collider(&mut self, handle: u32) {
        let ch = ColliderHandle(handle);
        // Get the body handle before removing the collider
        let body_handle = self
            .collider_body_indices
            .get(&ch)
            .copied()
            .map(|idx| RigidBodyHandle(idx as u32));
        if let Some(&leaf) = self.bvh_leaves.get(&ch) {
            self.bvh.remove_leaf(leaf);
            self.bvh_leaves.remove(&ch);
        }
        self.collider_body_indices.remove(&ch);
        self.colliders.remove(ch);

        // Recompute body mass properties after collider removal
        if let Some(bh) = body_handle {
            self.recompute_body_mass(bh);
        }
    }

    pub fn set_collider_friction(&mut self, handle: u32, friction: f32) {
        if let Some(c) = self.colliders.get_mut(ColliderHandle(handle)) {
            c.friction = friction;
            // Single API matches Rapier-style `.friction(μ)` and compare.html sliders:
            // keep Coulomb static/kinetic thresholds aligned unless static is set explicitly.
            c.static_friction = friction;
        }
    }

    pub fn set_collider_static_friction(&mut self, handle: u32, static_friction: f32) {
        if let Some(c) = self.colliders.get_mut(ColliderHandle(handle)) {
            c.static_friction = static_friction;
        }
    }

    pub fn set_collider_restitution(&mut self, handle: u32, restitution: f32) {
        if let Some(c) = self.colliders.get_mut(ColliderHandle(handle)) {
            c.restitution = restitution;
        }
    }

    pub fn set_collider_sensor(&mut self, handle: u32, sensor: bool) {
        if let Some(c) = self.colliders.get_mut(ColliderHandle(handle)) {
            c.collider_type = if sensor {
                crate::collider::ColliderType::Sensor
            } else {
                crate::collider::ColliderType::Solid
            };
        }
    }

    // ========== Simulation ==========

    pub fn step(&mut self) {
        if self.gpu_acceleration_enabled && self.gpu_runtime.is_some() {
            if let Err(e) = self.step_gpu() {
                #[cfg(not(target_arch = "wasm32"))]
                eprintln!("Matcha2D GPU step failed ({e}); falling back to CPU.");
                let _ = e;
                self.step_cpu();
            }
        } else {
            self.step_cpu();
        }
    }

    /// CPU broadphase → narrowphase (GJK + TOI) → PGS solver.
    pub fn step_cpu(&mut self) {
        self.snapshot_body_poses_at_step_start();

        // 1. Update sleep states (before detection so sleeping bodies don't wake from stale contacts)
        self.update_sleep();

        // 2. Update BVH
        self.update_bvh();

        // 3. Broadphase (with optional swept AABB for CCD)
        let pairs = if self.swept_broadphase {
            self.broadphase_swept()
        } else {
            self.bvh.query_pairs()
        };

        // 4. Narrowphase with collision filtering (+ TOI recovery when swept broadphase still paired).
        self.last_toi_debug_flat.clear();
        let mut manifolds = Vec::new();
        for (ca, cb) in &pairs {
            if !self.can_colliders_collide(*ca, *cb) {
                continue;
            }
            let hint = self.contact_tracker.poly_sat_hint(*ca, *cb);
            let manifold = if let Some(m) = narrowphase::detect_collision_with_hint(
                &self.bodies,
                &self.colliders,
                *ca,
                *cb,
                hint,
            ) {
                Some(m)
            } else {
                self.try_recover_tunnelled_manifold(*ca, *cb)
            };
            if let Some(manifold) = manifold {
                manifolds.push((*ca, *cb, manifold));
            }
        }

        // Swept AABB broadphase finds candidate pairs; TOI recovery integrates linear accel (g+F/m),
        // angular accel (τ/I), step-start pose/velocity snapshots, ball–ball analytic separation,
        // GJK signed separation + bisection for other convex pairs, then generic pose sampling fallback.

        // 5. Update contact tracker (persistent manifolds, is_new / poly SAT hints)
        self.contact_tracker.update(manifolds);

        // 6. Wake bodies that are in contact
        self.wake_contacting_bodies();

        // 7. Blueprint-style solver: solver bodies → constraint build → PGS solve → integrate → writeback
        self.solve();

        // 8. Optional world bounds (safety net against tunneling / escape)
        self.enforce_world_bounds();
    }

    /// Full Blueprint-style solver pipeline with substepping.
    /// Each outer iteration is a substep that integrates positions, so
    /// constraint penetration is refreshed between iterations.
    fn solve(&mut self) {
        let num_substeps = self.config.num_iterations.max(1);
        let substep_dt = self.dt / num_substeps as f32;

        // Create solver body copies with substep_dt (force increments are per-substep)
        let mut solver_bodies = SolverBodySet::new();
        solver_bodies.max_integrate_linear_velocity = self.max_integrate_linear_velocity;
        solver_bodies.max_integrate_angular_velocity = self.max_integrate_angular_velocity;
        solver_bodies.copy_from_bodies(&self.bodies, substep_dt, self.gravity);

        // Build constraints from contact pairs with substep_dt (for ERP/CFM)
        let mut constraints = ContactConstraintSet::new();
        constraints
            .set_position_correction(self.position_correction_beta, self.max_corrective_velocity);
        constraints.allowed_linear_error = self.penetration_slop;
        constraints
            .set_spring_coefficients(self.spring_natural_frequency, self.spring_damping_ratio);
        constraints.max_solver_linear_velocity = self.max_solver_linear_velocity;
        constraints.max_solver_angular_velocity = self.max_solver_angular_velocity;
        constraints.pgs_relaxation = self.pgs_relaxation;
        constraints.build_from_pairs(
            &self.contact_tracker,
            &self.colliders,
            &self.bodies,
            &solver_bodies,
            substep_dt,
            self.config.block_solver,
        );

        // Run the full solver loop with substepping
        solver::solve(
            &mut solver_bodies,
            &mut constraints,
            &mut self.contact_tracker,
            num_substeps,
            self.config.num_internal_iterations,
            substep_dt,
            self.config.block_solver,
            &mut self.bodies,
        );
        self.last_solver_diagnostics = constraints.last_diagnostics;
    }

    fn compute_body_world_aabb(&self, body_handle: RigidBodyHandle) -> Option<AABB> {
        let mut union: Option<AABB> = None;
        for idx in self.colliders.live_indices() {
            let col = self.colliders.get_unchecked(idx);
            if col.parent == Some(body_handle) {
                let aabb = self.compute_collider_aabb(col);
                union = Some(match union {
                    None => aabb,
                    Some(u) => u.merge(aabb),
                });
            }
        }
        union
    }

    /// Push dynamic bodies back inside optional world bounds and damp velocity into walls.
    fn enforce_world_bounds(&mut self) {
        let Some((bmin, bmax)) = self.world_bounds else {
            return;
        };
        let handles: Vec<RigidBodyHandle> = self.bodies.iter().map(|(h, _)| h).collect();
        for body_handle in handles {
            let Some(body) = self.bodies.get(body_handle) else {
                continue;
            };
            if !body.is_dynamic() {
                continue;
            }
            let Some(aabb) = self.compute_body_world_aabb(body_handle) else {
                continue;
            };
            let mut dx = 0.0f32;
            let mut dy = 0.0f32;
            if aabb.min.x < bmin.x {
                dx = bmin.x - aabb.min.x;
            }
            if aabb.max.x > bmax.x {
                dx = bmax.x - aabb.max.x;
            }
            if aabb.min.y < bmin.y {
                dy = bmin.y - aabb.min.y;
            }
            if aabb.max.y > bmax.y {
                dy = bmax.y - aabb.max.y;
            }
            if dx == 0.0 && dy == 0.0 {
                continue;
            }
            if let Some(body) = self.bodies.get_mut(body_handle) {
                body.pos.x += dx;
                body.pos.y += dy;
                if dx != 0.0 && body.linvel.x * dx < 0.0 {
                    body.linvel.x = 0.0;
                }
                if dy != 0.0 && body.linvel.y * dy < 0.0 {
                    body.linvel.y = 0.0;
                }
                body.wake();
            }
            let colliders: Vec<ColliderHandle> = self
                .colliders
                .live_indices()
                .into_iter()
                .filter_map(|idx| {
                    let c = self.colliders.get_unchecked(idx);
                    if c.parent == Some(body_handle) {
                        Some(self.colliders.handle_for_index(idx))
                    } else {
                        None
                    }
                })
                .collect();
            for ch in colliders {
                self.update_collider_bvh(ch);
            }
        }
    }

    // ========== Sleep System ==========

    fn update_sleep(&mut self) {
        let dt = self.dt;
        let vel_thresh = self.sleep_vel_threshold;
        let ang_thresh = self.sleep_ang_vel_threshold;
        let time_thresh = self.sleep_time_threshold;

        let body_handles: Vec<RigidBodyHandle> = self.bodies.iter().map(|(h, _)| h).collect();
        for body_handle in body_handles {
            let in_contact = self.body_has_nonempty_contact(body_handle);
            let Some(body) = self.bodies.get_mut(body_handle) else {
                continue;
            };
            if !body.is_dynamic() || !body.enabled {
                continue;
            }
            if body.sleeping {
                continue;
            }

            let speed = body.linvel.length();
            let ang_speed = body.angvel.abs();

            // Bodies in sustained contact need longer to sleep — avoids freezing while
            // the solver is still bleeding tiny tangential / angular motion from stacks.
            let required_time = if in_contact {
                time_thresh * 1.35
            } else {
                time_thresh
            };

            if speed < vel_thresh && ang_speed < ang_thresh {
                body.sleep_timer += dt;
                if body.sleep_timer >= required_time {
                    body.sleep();
                }
            } else {
                body.sleep_timer = 0.0;
            }
        }
    }

    /// Uses last frame's contact tracker (pairs not yet updated for this step).
    fn body_has_nonempty_contact(&self, body_handle: RigidBodyHandle) -> bool {
        for pair in self.contact_tracker.pairs() {
            if pair.manifold.contacts.is_empty() {
                continue;
            }
            for h in [pair.collider_a, pair.collider_b] {
                if let Some(c) = self.colliders.get(h) {
                    if c.parent == Some(body_handle) {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn wake_contacting_bodies(&mut self) {
        for pair in self.contact_tracker.pairs() {
            if pair.manifold.contacts.is_empty() {
                continue;
            }
            // Only wake bodies when there are NEW contacts (first collision frame).
            // Persistent resting contacts (is_new = false) must NOT reset the sleep
            // timer — otherwise resting bodies can never accumulate enough sleep time.
            let has_new_contact = pair.manifold.contacts.iter().any(|c| c.is_new);
            if !has_new_contact {
                continue;
            }
            let (ha, hb) = (pair.collider_a, pair.collider_b);
            if let Some(col) = self.colliders.get(ha) {
                if let Some(ph) = col.parent {
                    if let Some(body) = self.bodies.get_mut(ph) {
                        body.wake();
                    }
                }
            }
            if let Some(col) = self.colliders.get(hb) {
                if let Some(ph) = col.parent {
                    if let Some(body) = self.bodies.get_mut(ph) {
                        body.wake();
                    }
                }
            }
        }
    }

    // ========== Collision Filtering ==========

    fn can_colliders_collide(&self, ca: ColliderHandle, cb: ColliderHandle) -> bool {
        let col_a = match self.colliders.get(ca) {
            Some(c) => c,
            None => return false,
        };
        let col_b = match self.colliders.get(cb) {
            Some(c) => c,
            None => return false,
        };

        if col_a.is_sensor() && col_b.is_sensor() {
            return false;
        }

        let (ha, hb) = match (col_a.parent, col_b.parent) {
            (Some(a), Some(b)) => (a, b),
            _ => return true,
        };

        let body_a = match self.bodies.get(ha) {
            Some(b) => b,
            None => return false,
        };
        let body_b = match self.bodies.get(hb) {
            Some(b) => b,
            None => return false,
        };

        // At least one body must be dynamic for a collision to matter
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return false;
        }

        if body_a.sleeping && body_b.sleeping {
            return false;
        }

        body_a.can_collide_with(body_b)
    }

    // ========== Swept AABB Broadphase (CCD) ==========

    fn broadphase_swept(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let indices = self.colliders.live_indices();
        let mut swept_aabbs: Vec<(ColliderHandle, AABB)> = Vec::with_capacity(indices.len());
        for &idx in &indices {
            let col = self.colliders.get_unchecked(idx);
            let ch = self.colliders.handle_for_index(idx);
            let base_aabb = self.compute_collider_aabb(col);

            let expansion = if let Some(ph) = col.parent {
                if let Some(body) = self.bodies.get(ph) {
                    if body.sleeping {
                        Vec2::zero()
                    } else if body.is_dynamic() || body.is_kinematic() {
                        let g = self.gravity * body.gravity_scale;
                        let v0 = body.linvel;
                        let v1 = v0 + g * self.dt;
                        let accel_pad = Vec2::new(
                            0.5 * g.x.abs() * self.dt * self.dt,
                            0.5 * g.y.abs() * self.dt * self.dt,
                        );
                        let lin = Vec2::new(
                            (v0.x * self.dt).abs().max((v1.x * self.dt).abs()) + accel_pad.x,
                            (v0.y * self.dt).abs().max((v1.y * self.dt).abs()) + accel_pad.y,
                        );
                        let r_max = col.max_radius_from_body_com();
                        let ang_pad = body.angvel.abs() * r_max * self.dt * 1.25;
                        lin + Vec2::new(ang_pad, ang_pad)
                    } else {
                        Vec2::zero()
                    }
                } else {
                    Vec2::zero()
                }
            } else {
                Vec2::zero()
            };

            let swept = AABB::new(
                base_aabb.min - expansion - Vec2::new(SWEPT_AABB_MARGIN, SWEPT_AABB_MARGIN),
                base_aabb.max + expansion + Vec2::new(SWEPT_AABB_MARGIN, SWEPT_AABB_MARGIN),
            );
            swept_aabbs.push((ch, swept));
        }

        // Temporary BVH over swept AABBs: O(n log n) pair discovery vs O(n²) all-pairs.
        let mut swept_bvh = DynamicBvh::new();
        for (ch, swept) in swept_aabbs {
            // No fat margin: must match the old all-pairs check on raw swept AABBs (see `insert_leaf_with_margin`).
            swept_bvh.insert_leaf_with_margin(swept, ch, 0.0);
        }
        let mut pairs = swept_bvh.query_pairs();
        pairs.retain(|(ca, cb)| self.can_colliders_collide(*ca, *cb));
        pairs
    }

    // ========== BVH Helpers ==========

    fn update_collider_bvh(&mut self, ch: ColliderHandle) {
        if let Some(&leaf) = self.bvh_leaves.get(&ch) {
            let col = self.colliders.get(ch);
            if let Some(col) = col {
                let aabb = self.compute_collider_aabb(col);
                let new_leaf = self.bvh.update_leaf(leaf, aabb);
                if new_leaf != leaf {
                    self.bvh_leaves.insert(ch, new_leaf);
                }
            }
        }
    }

    fn update_bvh(&mut self) {
        let indices = self.colliders.live_indices();
        for idx in indices {
            let col = self.colliders.get_unchecked(idx);
            let ch = self.colliders.handle_for_index(idx);

            if let Some(&leaf) = self.bvh_leaves.get(&ch) {
                let aabb = self.compute_collider_aabb(col);
                let new_leaf = self.bvh.update_leaf(leaf, aabb);
                if new_leaf != leaf {
                    self.bvh_leaves.insert(ch, new_leaf);
                }
            }
        }
    }

    fn compute_collider_aabb(&self, collider: &Collider) -> AABB {
        let (pos, rot) = narrowphase::collider_world_transform(&self.bodies, collider);

        match &collider.shape {
            Shape::Ball { radius } => aabb::aabb_ball(pos, *radius),
            Shape::Cuboid { hx, hy } => aabb::aabb_cuboid(pos, rot, *hx, *hy),
            Shape::Polygon { vertices } => aabb::aabb_polygon(pos, rot, vertices),
        }
    }

    // ========== Query ==========

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    /// World-space outlines for every live collider — same transform as narrowphase / AABB (`collider_world_transform`).
    ///
    /// Flat layout (all `f32`):
    /// `[entry_count,` then `entry_count` times:
    ///   `collider_handle, body_handle, shape_kind, flags, point_count,` then `point_count` pairs `(x,y)` in world meters (Y-up).
    /// - `shape_kind`: `0` = ball (circle polyline), `1` = cuboid (4 corners CCW), `2` = convex polygon (CCW after engine normalize).
    /// - `flags`: `1` = sensor, `0` = solid.
    /// - `body_handle`: rigid-body handle as float, or NaN if unparented.
    pub fn debug_narrowphase_collider_outlines_flat(&self) -> Vec<f32> {
        const BALL_SEGMENTS: usize = 32;
        let mut out = Vec::new();
        let indices = self.colliders.live_indices();
        out.push(indices.len() as f32);
        for idx in indices {
            let ch = self.colliders.handle_for_index(idx);
            let col = self.colliders.get_unchecked(idx);
            let (wp, wr) = narrowphase::collider_world_transform(&self.bodies, col);
            let body_h = col
                .parent
                .map(|h| h.0 as f32)
                .unwrap_or(f32::NAN);
            let flags = if col.is_sensor() { 1.0 } else { 0.0 };
            match &col.shape {
                Shape::Ball { radius } => {
                    out.push(ch.0 as f32);
                    out.push(body_h);
                    out.push(0.0);
                    out.push(flags);
                    out.push(BALL_SEGMENTS as f32);
                    for i in 0..BALL_SEGMENTS {
                        let t = (i as f32 / BALL_SEGMENTS as f32) * std::f32::consts::TAU;
                        let local = Vec2::new(t.cos() * *radius, t.sin() * *radius);
                        let w = wp + wr.mul_vec(local);
                        out.push(w.x);
                        out.push(w.y);
                    }
                }
                Shape::Cuboid { hx, hy } => {
                    let corners = [
                        Vec2::new(-*hx, -*hy),
                        Vec2::new(*hx, -*hy),
                        Vec2::new(*hx, *hy),
                        Vec2::new(-*hx, *hy),
                    ];
                    out.push(ch.0 as f32);
                    out.push(body_h);
                    out.push(1.0);
                    out.push(flags);
                    out.push(4.0);
                    for p in corners {
                        let w = wp + wr.mul_vec(p);
                        out.push(w.x);
                        out.push(w.y);
                    }
                }
                Shape::Polygon { vertices } => {
                    let n = vertices.len();
                    out.push(ch.0 as f32);
                    out.push(body_h);
                    out.push(2.0);
                    out.push(flags);
                    out.push(n as f32);
                    for p in vertices {
                        let w = wp + wr.mul_vec(*p);
                        out.push(w.x);
                        out.push(w.y);
                    }
                }
            }
        }
        out
    }

    pub fn get_positions_x(&self) -> Vec<f32> {
        self.bodies
            .live_indices()
            .into_iter()
            .map(|idx| self.bodies.get_unchecked(idx).pos.x)
            .collect()
    }

    pub fn get_positions_y(&self) -> Vec<f32> {
        self.bodies
            .live_indices()
            .into_iter()
            .map(|idx| self.bodies.get_unchecked(idx).pos.y)
            .collect()
    }

    pub fn get_angles(&self) -> Vec<f32> {
        self.bodies
            .live_indices()
            .into_iter()
            .map(|idx| self.bodies.get_unchecked(idx).rot.angle())
            .collect()
    }

    pub fn get_angular_velocities(&self) -> Vec<f32> {
        self.bodies
            .live_indices()
            .into_iter()
            .map(|idx| self.bodies.get_unchecked(idx).angvel)
            .collect()
    }

    pub fn contact_events(&self) -> &Vec<ContactEvent> {
        &self.contact_tracker.events
    }
}

/// SAT / clipping assume counter‑clockwise polygon winding (positive signed area).
fn ensure_ccw_polygon_vertices(mut verts: Vec<Vec2>) -> Vec<Vec2> {
    let n = verts.len();
    if n < 3 {
        return verts;
    }
    let mut twice_area = 0.0f32;
    for i in 0..n {
        let j = (i + 1) % n;
        twice_area += verts[i].cross(verts[j]);
    }
    if twice_area < 0.0 {
        verts.reverse();
    }
    verts
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;

    #[test]
    fn test_world_creation() {
        let world = PhysicsWorld::new();
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.collider_count(), 0);
    }

    #[test]
    fn test_create_bodies() {
        let mut world = PhysicsWorld::new();
        let b1 = world.create_dynamic_body(0.0, 10.0);
        let _b2 = world.create_static_body(0.0, -5.0);
        assert_eq!(world.body_count(), 2);

        let pos = world.get_body_position(b1);
        assert!((pos[0] - 0.0).abs() < 1e-5);
        assert!((pos[1] - 10.0).abs() < 1e-5);
    }

    #[test]
    fn test_step_no_crash() {
        let mut world = PhysicsWorld::new();
        world.create_dynamic_body(0.0, 10.0);
        world.step();
        assert!(world.get_positions_y()[0] < 10.0);
    }

    #[test]
    fn test_spinning_cuboid_collision() {
        let mut world = PhysicsWorld::new();

        let static_body = world.create_static_body(0.0, 0.0);
        let _static_col = world.create_box_collider(0.5, 0.5, static_body);

        let dynamic_body = world.create_dynamic_body(3.0, 0.0);
        world.set_body_velocity(dynamic_body, -10.0, 0.0);
        let _dynamic_col = world.create_box_collider(0.5, 0.5, dynamic_body);

        if let Some(body) = world
            .bodies
            .get_mut(crate::body::RigidBodyHandle(dynamic_body))
        {
            body.angvel = 15.0;
        }

        let mut collision_detected = false;
        let mut collision_step = 0;
        let mut passed_through = false;
        let mut max_penetration = 0.0f32;

        for step in 0..120 {
            let _pos_before = world.get_body_position(dynamic_body);
            world.step();
            let pos_after = world.get_body_position(dynamic_body);
            let vel = world.get_body_velocity(dynamic_body);

            let contact_count = world.contact_tracker.pair_count();

            let dynamic_left_edge = pos_after[0] - 0.707;
            let static_right_edge = 0.5;

            if !collision_detected && contact_count > 0 {
                collision_detected = true;
                collision_step = step;
            }

            if dynamic_left_edge < static_right_edge {
                let penetration = static_right_edge - dynamic_left_edge;
                max_penetration = max_penetration.max(penetration);
            }

            if pos_after[0] < -0.5 && vel[0] < -1.0 {
                passed_through = true;
                eprintln!(
                    "=== SPINNING COLLISION FAILURE ===\n\
                     Step: {}\n\
                     Position: ({:.4}, {:.4})\n\
                     Velocity: ({:.4}, {:.4})\n\
                     Contact pairs: {}\n\
                     Collision first detected at step: {}\n\
                     Max penetration observed: {:.4}",
                    step,
                    pos_after[0],
                    pos_after[1],
                    vel[0],
                    vel[1],
                    contact_count,
                    if collision_detected {
                        collision_step.to_string()
                    } else {
                        "never".to_string()
                    },
                    max_penetration,
                );
            }

            if pos_after[0] < -1.0 && !passed_through {
                passed_through = true;
            }
        }

        assert!(
            collision_detected,
            "Spinning cuboid never detected collision with static cuboid!"
        );
        assert!(
            !passed_through,
            "Spinning cuboid passed through static cuboid! Max penetration: {:.4}",
            max_penetration
        );

        let final_vel = world.get_body_velocity(dynamic_body);
        assert!(
            final_vel[0] > -10.0,
            "Dynamic body velocity not affected by collision: vx={:.4}",
            final_vel[0]
        );
    }

    #[test]
    fn test_spinning_cuboid_vertex_collision() {
        let mut world = PhysicsWorld::new();

        let static_body = world.create_static_body(0.0, 0.0);
        let _sc = world.create_box_collider(0.5, 0.5, static_body);

        let dynamic_body = world.create_dynamic_body(3.0, 0.0);
        world.set_body_angle(dynamic_body, std::f32::consts::FRAC_PI_4);
        world.set_body_velocity(dynamic_body, -10.0, 0.0);
        let _dc = world.create_box_collider(0.5, 0.5, dynamic_body);

        if let Some(body) = world
            .bodies
            .get_mut(crate::body::RigidBodyHandle(dynamic_body))
        {
            body.angvel = -20.0;
        }

        let mut collided = false;
        let mut passed_through = false;

        for _step in 0..120 {
            world.step();
            let pos = world.get_body_position(dynamic_body);
            let vel = world.get_body_velocity(dynamic_body);

            if world.contact_tracker.pair_count() > 0 {
                collided = true;
            }

            if pos[0] < -0.5 && vel[0] < -1.0 {
                passed_through = true;
            }
        }

        assert!(
            collided,
            "45-degree spinning cuboid never collided with static cuboid!"
        );
        assert!(
            !passed_through,
            "45-degree spinning cuboid passed through static cuboid!"
        );
    }

    #[test]
    fn test_extreme_spin_collision() {
        let mut world = PhysicsWorld::new();

        let static_body = world.create_static_body(0.0, 0.0);
        let _sc = world.create_box_collider(0.5, 0.5, static_body);

        let dynamic_body = world.create_dynamic_body(3.0, 0.0);
        world.set_body_velocity(dynamic_body, -8.0, 0.0);
        let _dc = world.create_box_collider(0.5, 0.5, dynamic_body);

        if let Some(body) = world
            .bodies
            .get_mut(crate::body::RigidBodyHandle(dynamic_body))
        {
            body.angvel = 60.0;
        }

        let mut collided = false;
        let mut passed_through = false;

        for _step in 0..120 {
            world.step();
            let pos = world.get_body_position(dynamic_body);
            let vel = world.get_body_velocity(dynamic_body);

            if world.contact_tracker.pair_count() > 0 {
                collided = true;
            }

            if pos[0] < -0.5 && vel[0] < -1.0 {
                passed_through = true;
            }
        }

        assert!(collided, "Extremely spinning cuboid never collided!");
        assert!(
            !passed_through,
            "Extremely spinning cuboid passed through static cuboid!"
        );
    }

    #[test]
    fn test_fast_linear_tunneling_prevented() {
        let mut world = PhysicsWorld::new();
        world.swept_broadphase = true;

        let wall = world.create_static_body(5.0, 0.0);
        world.create_box_collider(0.5, 2.0, wall);

        let bullet = world.create_dynamic_body(0.0, 0.0);
        world.set_body_velocity(bullet, 100.0, 0.0);
        world.create_ball_collider(0.1, bullet);

        let mut collided = false;
        let mut max_bullet_x = 0.0f32;

        for _ in 0..60 {
            world.step();
            let pos = world.get_body_position(bullet);
            max_bullet_x = max_bullet_x.max(pos[0]);
            if world.contact_tracker.pair_count() > 0 {
                collided = true;
                break;
            }
        }

        assert!(
            collided,
            "Fast bullet (100 m/s) never detected collision with wall! Max bullet x: {}",
            max_bullet_x
        );
    }

    #[test]
    fn test_set_body_position_updates_bvh() {
        let mut world = PhysicsWorld::new();

        let wall = world.create_static_body(5.0, 0.0);
        world.create_box_collider(0.5, 2.0, wall);

        let ball = world.create_dynamic_body(0.0, 0.0);
        world.create_ball_collider(0.5, ball);

        world.set_body_position(ball, 5.0, 0.0);
        world.step();

        assert!(
            world.contact_tracker.pair_count() > 0,
            "After teleporting body into wall, no collision detected!"
        );
    }

    #[test]
    fn test_set_body_position_wakes_body() {
        let mut world = PhysicsWorld::new();
        world.set_sleep_thresholds(100.0, 100.0, 0.0);

        let ball = world.create_dynamic_body(0.0, 0.0);
        world.create_ball_collider(0.5, ball);

        world.step();
        assert!(world.is_body_sleeping(ball), "Body should be sleeping");

        world.set_body_position(ball, 1.0, 1.0);
        assert!(
            !world.is_body_sleeping(ball),
            "Body should be awake after set_body_position"
        );
    }

    #[test]
    fn test_sleep_system() {
        let mut world = PhysicsWorld::new();

        let ball = world.create_dynamic_body(0.0, 0.0);
        world.create_ball_collider(0.5, ball);

        assert!(!world.is_body_sleeping(ball), "Body should start awake");

        world.sleep_body(ball);
        assert!(
            world.is_body_sleeping(ball),
            "Body should be sleeping after sleep_body"
        );

        world.wake_body(ball);
        assert!(
            !world.is_body_sleeping(ball),
            "Body should be awake after wake_body"
        );

        world.set_sleep_thresholds(10.0, 10.0, 0.0);
        world.step();
        assert!(
            world.is_body_sleeping(ball),
            "Body with zero velocity should sleep with aggressive thresholds"
        );
    }

    #[test]
    fn test_debug_narrowphase_outlines_non_empty() {
        let mut world = PhysicsWorld::new();
        let body = world.create_dynamic_body(1.0, 2.0);
        world.create_box_collider(0.4, 0.6, body);
        let flat = world.debug_narrowphase_collider_outlines_flat();
        assert!(flat.len() >= 6, "expected header + one collider entry");
        assert_eq!(flat[0] as usize, 1, "one collider");
        // 4 corners for cuboid
        assert_eq!(flat[5] as usize, 4, "cuboid point count");
    }

    #[test]
    fn test_polygon_cw_vertices_still_collide() {
        // Clockwise triangle in local space; engine should re-wind to CCW for SAT.
        let mut world = PhysicsWorld::new();
        let floor = world.create_static_body(0.0, 0.0);
        world.create_box_collider(5.0, 0.5, floor);

        let tri = world.create_dynamic_body(0.0, 0.55);
        let cw_x = [0.0_f32, -0.4, 0.4];
        let cw_y = [0.35_f32, -0.25, -0.25];
        world.create_polygon_collider(&cw_x, &cw_y, tri);

        world.step();
        assert!(
            world.contact_tracker.pair_count() > 0,
            "CW polygon should be normalized and collide with floor"
        );
    }

    #[test]
    fn test_collision_filtering() {
        let mut world = PhysicsWorld::new();

        let a = world.create_dynamic_body(0.0, 0.0);
        world.set_body_collision_groups(a, 0x01);
        world.set_body_collision_mask(a, 0x02);
        world.create_ball_collider(0.5, a);

        let b = world.create_dynamic_body(100.0, 100.0);
        world.set_body_collision_groups(b, 0x04);
        world.set_body_collision_mask(b, 0xFF);
        world.create_ball_collider(0.5, b);

        let c = world.create_dynamic_body(0.0, 0.8);
        world.set_body_collision_groups(c, 0x02);
        world.set_body_collision_mask(c, 0xFF);
        world.create_ball_collider(0.5, c);

        world.step();

        assert_eq!(
            world.contact_tracker.pair_count(),
            1,
            "Expected exactly 1 collision pair (A+C), got {}",
            world.contact_tracker.pair_count()
        );
    }

    #[test]
    fn test_bvh_update_leaf_no_panic() {
        let mut world = PhysicsWorld::new();

        let b1 = world.create_dynamic_body(0.0, 0.0);
        world.create_box_collider(0.5, 0.5, b1);

        let b2 = world.create_dynamic_body(5.0, 0.0);
        world.create_box_collider(0.5, 0.5, b2);

        for _ in 0..60 {
            world.set_body_position(b1, 10.0, 0.0);
            world.step();
        }

        assert_eq!(world.body_count(), 2);
    }

    #[test]
    fn test_contact_maintained_no_passthrough() {
        // Test: a box resting on a static floor should not sink through over time.
        // This reproduces the "passthrough with maintained contact" bug where:
        // - missing position bias / insufficient substeps can cause sinking or passthrough
        let mut world = PhysicsWorld::new();
        world.set_sleep_thresholds(100.0, 100.0, 100.0); // Disable sleeping for this test

        // Floor
        let floor = world.create_static_body(0.0, -2.0);
        world.create_box_collider(10.0, 0.5, floor);

        // Box resting on floor
        let box_body = world.create_dynamic_body(0.0, 0.0);
        world.create_box_collider(0.5, 0.5, box_body);

        let initial_y = world.get_body_position(box_body)[1];

        // Run for 120 steps (2 seconds) and track position
        let mut min_y = initial_y;
        let mut max_penetration = 0.0f32;
        let mut contact_frames = 0;

        for step in 0..120 {
            world.step();
            let pos = world.get_body_position(box_body);
            let vel = world.get_body_velocity(box_body);
            min_y = min_y.min(pos[1]);

            // Track contact persistence
            if world.contact_tracker.pair_count() > 0 {
                contact_frames += 1;
            }

            // Check penetration: box bottom should not go below floor top
            let box_bottom = pos[1] - 0.5;
            let floor_top = -2.0 + 0.5;
            if box_bottom < floor_top {
                max_penetration = max_penetration.max(floor_top - box_bottom);
            }

            // Box should not fall through floor
            if pos[1] < -5.0 {
                panic!(
                    "Box fell through floor at step {}! y={:.4}, vel=({:.4},{:.4}), contacts={}",
                    step,
                    pos[1],
                    vel[0],
                    vel[1],
                    world.contact_tracker.pair_count()
                );
            }
        }

        // Box should have been in contact most of the time
        assert!(
            contact_frames > 10,
            "Box should have been in contact for most frames, got {}",
            contact_frames
        );

        // Box should not have sunk significantly below its starting position
        // Floor top is at y=-1.5, box center should settle near y=-1.0 (box bottom at -1.5)
        assert!(
            min_y > -1.5,
            "Box sank too far! min_y={:.4}, initial_y={:.4}",
            min_y,
            initial_y
        );

        // Velocity should settle near zero (not keep accelerating downward)
        let final_vel = world.get_body_velocity(box_body);
        assert!(
            final_vel[1].abs() < 5.0,
            "Box still has significant vertical velocity after 120 steps: vy={:.4}",
            final_vel[1]
        );
    }

    #[test]
    fn test_stacking_stability() {
        // Test: 3 boxes stacked on a static floor should not immediately explode
        let mut world = PhysicsWorld::new();

        // Floor
        let floor = world.create_static_body(0.0, -0.5);
        world.create_box_collider(10.0, 0.5, floor);

        // Stack 3 boxes
        for i in 0..3 {
            let body = world.create_dynamic_body(0.0, 0.5 + i as f32);
            world.create_box_collider(0.5, 0.5, body);
        }

        // Run for 30 steps (0.5 seconds)
        for step in 0..30 {
            world.step();

            // Check for immediate explosion
            let positions_y = world.get_positions_y();
            let max_y = positions_y
                .iter()
                .cloned()
                .fold(f32::NEG_INFINITY, f32::max);
            if max_y > 20.0 {
                panic!("Boxes exploded at step {}! Max y = {}", step, max_y);
            }
        }

        // All boxes should be above the floor (y > -2.0, allowing some penetration)
        let positions_y = world.get_positions_y();
        for (i, &y) in positions_y.iter().enumerate() {
            if i == 0 {
                continue;
            }
            assert!(y > -2.0, "Box {} fell through floor! y = {}", i, y);
        }

        // Boxes should not have exploded
        let max_y = positions_y
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(max_y < 20.0, "Boxes exploded! Max y = {}", max_y);
    }

    #[test]
    fn test_ultrafast_ball_does_not_pass_thin_wall() {
        let mut world = PhysicsWorld::new();
        world.swept_broadphase = true;
        let wall = world.create_static_body(40.0, 0.0);
        let wall_col = world.create_box_collider(0.04, 4.0, wall);
        world.set_collider_restitution(wall_col, 0.0);

        let bullet = world.create_dynamic_body(0.0, 0.0);
        let ball_col = world.create_ball_collider(0.12, bullet);
        world.set_collider_restitution(ball_col, 0.0);
        world.set_body_velocity(bullet, 480.0, 0.0);

        let wall_face_x = 40.0 - 0.04;
        let mut collided = false;
        for _ in 0..100 {
            world.step();
            let px = world.get_body_position(bullet)[0];
            assert!(
                px < wall_face_x + 0.14,
                "ball tunnelled past thin wall face: x={px:.3} (face {wall_face_x:.3})"
            );
            if world.contact_tracker.pair_count() > 0 {
                collided = true;
            }
        }
        assert!(
            collided,
            "ultrafast ball never registered contact with thin wall"
        );
    }

    #[test]
    fn test_falling_ball_does_not_drop_through_thin_shelf() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(0.0, -9.81);
        world.swept_broadphase = true;
        // Shelf top surface y = 1.0 (center 0.99, half-height 0.01)
        let shelf = world.create_static_body(0.0, 0.99);
        let shelf_col = world.create_box_collider(6.0, 0.01, shelf);
        world.set_collider_restitution(shelf_col, 0.0);

        let ball = world.create_dynamic_body(0.0, 12.0);
        let ball_col = world.create_ball_collider(0.22, ball);
        world.set_collider_restitution(ball_col, 0.0);

        for step in 0..240 {
            world.step();
            let py = world.get_body_position(ball)[1];
            assert!(
                py > 0.75,
                "ball fell through thin shelf (step {step}, y={py:.4})"
            );
        }
        let final_y = world.get_body_position(ball)[1];
        assert!(
            final_y < 2.5 && final_y > 0.85,
            "expected rest on shelf near y≈1.22, got y={final_y:.4}"
        );
    }

    #[test]
    fn test_two_dynamic_balls_head_on_tunnel_recovered() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(0.0, 0.0);
        world.swept_broadphase = true;

        let a = world.create_dynamic_body(0.0, 0.0);
        world.create_ball_collider(0.35, a);
        world.set_body_velocity(a, 220.0, 0.0);

        let b = world.create_dynamic_body(28.0, 0.0);
        world.create_ball_collider(0.35, b);
        world.set_body_velocity(b, -220.0, 0.0);

        let mut ever_contact = false;
        for step in 0..150 {
            world.step();
            if world.contact_tracker.pair_count() > 0 {
                ever_contact = true;
            }
            let pa = world.get_body_position(a)[0];
            let pb = world.get_body_position(b)[0];
            if !ever_contact && step < 40 {
                assert!(
                    pa < pb - 0.15,
                    "A crossed B without contact (step {step}): pa={pa:.3} pb={pb:.3}"
                );
            }
        }
        assert!(
            ever_contact,
            "head-on dynamic balls never registered contact — dynamic–dynamic tunnel likely"
        );
    }

    /// Head-on dynamic **boxes** at high speed: TOI recovery must use the convex GJK separation path
    /// (not the ball–ball analytic branch).
    #[test]
    fn test_two_dynamic_boxes_head_on_tunnel_recovered() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(0.0, 0.0);
        world.swept_broadphase = true;

        let a = world.create_dynamic_body(0.0, 0.0);
        world.create_box_collider(0.35, 0.35, a);
        world.set_body_velocity(a, 220.0, 0.0);

        let b = world.create_dynamic_body(28.0, 0.0);
        world.create_box_collider(0.35, 0.35, b);
        world.set_body_velocity(b, -220.0, 0.0);

        let mut ever_contact = false;
        for step in 0..150 {
            world.step();
            if world.contact_tracker.pair_count() > 0 {
                ever_contact = true;
            }
            let pa = world.get_body_position(a)[0];
            let pb = world.get_body_position(b)[0];
            if !ever_contact && step < 40 {
                assert!(
                    pa < pb - 0.15,
                    "A crossed B without contact (step {step}): pa={pa:.3} pb={pb:.3}"
                );
            }
        }
        assert!(
            ever_contact,
            "head-on dynamic boxes never registered contact — GJK TOI tunnel recovery likely broken"
        );
    }

    /// Polygon–polygon uses the same SAT/TOI path as boxes but is more sensitive to grazing axes.
    #[test]
    fn test_two_dynamic_triangles_head_on_tunnel_recovered() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(0.0, 0.0);
        world.swept_broadphase = true;

        let a = world.create_dynamic_body(0.0, 0.0);
        let ax = [-0.35_f32, 0.35, 0.0];
        let ay = [-0.35_f32, -0.35, 0.35];
        world.create_polygon_collider(&ax, &ay, a);
        world.set_body_velocity(a, 200.0, 0.0);

        let b = world.create_dynamic_body(26.0, 0.0);
        let bx = [-0.35_f32, 0.35, 0.0];
        let by = [-0.35_f32, -0.35, 0.35];
        world.create_polygon_collider(&bx, &by, b);
        world.set_body_velocity(b, -200.0, 0.0);

        let mut ever_contact = false;
        for step in 0..150 {
            world.step();
            if world.contact_tracker.pair_count() > 0 {
                ever_contact = true;
            }
            let pa = world.get_body_position(a)[0];
            let pb = world.get_body_position(b)[0];
            if !ever_contact && step < 45 {
                assert!(
                    pa < pb - 0.12,
                    "triangle A crossed B without contact (step {step}): pa={pa:.3} pb={pb:.3}"
                );
            }
        }
        assert!(
            ever_contact,
            "head-on dynamic triangles never registered contact — polygon TOI / SAT likely broken"
        );
    }

    #[test]
    fn test_toi_trajectory_quadratic_fall() {
        let dt = 1.0_f32;
        let traj = super::toi_types::ToiTrajectory::Integrated {
            p0: Vec2::zero(),
            v0: Vec2::zero(),
            a_lin: Vec2::new(0.0, -10.0),
            a0: 0.0,
            omega0: 0.0,
            alpha: 0.0,
        };
        let (p, _) = traj.pose(dt, 1.0);
        assert!((p.y + 5.0).abs() < 1e-3, "expected y=-5 at u=1, got {}", p.y);
        let (pm, _) = traj.pose(dt, 0.5);
        assert!(
            (pm.y + 1.25).abs() < 1e-3,
            "expected y=-1.25 at u=0.5, got {}",
            pm.y
        );
    }

    #[test]
    fn test_toi_trajectory_constant_angular_accel() {
        let dt = 1.0_f32;
        let traj = super::toi_types::ToiTrajectory::Integrated {
            p0: Vec2::zero(),
            v0: Vec2::zero(),
            a_lin: Vec2::zero(),
            a0: 0.0,
            omega0: 0.0,
            alpha: 2.0,
        };
        let (_, r) = traj.pose(dt, 1.0);
        let ang = r.angle();
        assert!(
            (ang - 1.0).abs() < 1e-4,
            "expected θ=1 rad at u=1 (½α dt²), got {}",
            ang
        );
    }

    #[test]
    fn test_fast_kinematic_advancing_into_ball_contacts() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(0.0, 0.0);
        world.swept_broadphase = true;

        let kin = world.create_kinematic_body(-14.0, 0.0);
        world.set_body_velocity(kin, 220.0, 0.0);
        world.create_box_collider(0.45, 1.6, kin);

        let ball = world.create_dynamic_body(10.0, 0.0);
        world.create_ball_collider(0.36, ball);

        let mut ever_contact = false;
        for step in 0..55 {
            world.step();
            if world.contact_tracker.pair_count() > 0 {
                ever_contact = true;
            }
            let pk = world.get_body_position(kin)[0];
            let pb = world.get_body_position(ball)[0];
            if !ever_contact {
                assert!(
                    pk < pb + 0.35,
                    "kinematic centre passed ball without contact (step {step}) pk={pk:.3} pb={pb:.3}"
                );
            }
        }
        assert!(
            ever_contact,
            "kinematic wall vs ball never registered contact"
        );
    }

    /// WebGPU path: resting box should not tunnel through floor (skips if no adapter).
    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn test_gpu_step_resting_box_no_passthrough() {
        let mut world = PhysicsWorld::new();
        world.set_sleep_thresholds(100.0, 100.0, 100.0);
        if world.init_gpu_blocking().is_err() {
            eprintln!("skip test_gpu_step_resting_box_no_passthrough: WebGPU unavailable");
            return;
        }
        world.set_gpu_acceleration_enabled(true);

        let floor = world.create_static_body(0.0, -2.0);
        world.create_box_collider(10.0, 0.5, floor);
        let box_body = world.create_dynamic_body(0.0, 0.0);
        world.create_box_collider(0.5, 0.5, box_body);

        let mut min_y = world.get_body_position(box_body)[1];
        for _ in 0..120 {
            world.step();
            let pos = world.get_body_position(box_body);
            min_y = min_y.min(pos[1]);
            assert!(
                pos[1] > -5.0,
                "GPU step: box fell through floor y={}",
                pos[1]
            );
        }
        assert!(
            min_y > -1.52,
            "GPU step: box sank too far min_y={min_y}"
        );
    }
}
