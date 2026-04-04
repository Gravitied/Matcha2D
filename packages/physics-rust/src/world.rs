use crate::aabb::{self, AABB};
use crate::body::{RigidBody, RigidBodyHandle, RigidBodySet};
use crate::bvh::DynamicBvh;
use crate::collider::{Collider, ColliderHandle, ColliderSet, Shape};
use crate::contact::{ContactEvent, ContactTracker};
use crate::contact_constraint::ContactConstraintSet;
use crate::math::Vec2;
use crate::narrowphase;
use crate::solver::{self, SolverConfig};
use crate::solver_body::SolverBodySet;

/// Default sleep velocity threshold (m/s)
const DEFAULT_SLEEP_VEL_THRESHOLD: f32 = 0.05;
/// Default sleep angular velocity threshold (rad/s)
const DEFAULT_SLEEP_ANG_VEL_THRESHOLD: f32 = 0.1;
/// Default time (seconds) a body must be below thresholds before sleeping
const DEFAULT_SLEEP_TIME_THRESHOLD: f32 = 1.0;
/// AABB margin for swept broadphase
const SWEPT_AABB_MARGIN: f32 = 0.5;

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
    spring_natural_frequency: f32,
    spring_damping_ratio: f32,
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
            position_correction_beta: 0.01,
            max_corrective_velocity: 50.0,
            spring_natural_frequency: 15.0,
            spring_damping_ratio: 1.0,
        }
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

    pub fn set_spring_coefficients(&mut self, natural_frequency: f32, damping_ratio: f32) {
        self.spring_natural_frequency = natural_frequency;
        self.spring_damping_ratio = damping_ratio;
    }

    pub fn set_solver_config(
        &mut self,
        num_iterations: usize,
        num_internal_iterations: usize,
        num_stabilization_iterations: usize,
        warmstart_coefficient: f32,
        block_solver: bool,
    ) {
        self.config.num_iterations = num_iterations;
        self.config.num_internal_iterations = num_internal_iterations;
        self.config.num_stabilization_iterations = num_stabilization_iterations;
        self.config.warmstart_coefficient = warmstart_coefficient;
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
        let collider = crate::collider::ColliderBuilder::polygon(verts)
            .parent(bh)
            .build();
        self.insert_collider(collider, bh)
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
            // Normalize: target mass proportional to sqrt(area) to keep values in 1-100 range
            // for typical pixel-scale colliders (10-100px). This prevents huge masses
            // that make solver impulses ineffective.
            if total_area > 0.0 {
                let target_mass = total_area.sqrt().clamp(1.0, 100.0);
                let scale = target_mass / total_area;
                body.mass = target_mass;
                body.inv_mass = 1.0 / target_mass;
                body.inertia = total_inertia_unscaled * scale;
                body.inv_inertia = if body.inertia > 0.0 {
                    1.0 / body.inertia
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

        // 4. Narrowphase with collision filtering
        let mut manifolds = Vec::new();
        for (ca, cb) in &pairs {
            if !self.can_colliders_collide(*ca, *cb) {
                continue;
            }
            if let Some(manifold) =
                narrowphase::detect_collision(&self.bodies, &self.colliders, *ca, *cb)
            {
                manifolds.push((*ca, *cb, manifold));
            }
        }

        // 5. Update contact tracker (persistent manifolds for warmstarting)
        self.contact_tracker.update(manifolds);

        // 6. Wake bodies that are in contact
        self.wake_contacting_bodies();

        // 7. Blueprint-style solver: solver bodies → constraint build → warmstart → PGS solve → integrate → stabilize → writeback
        self.solve();
    }

    /// Full Blueprint-style solver pipeline
    fn solve(&mut self) {
        // Create solver body copies from rigid bodies
        let mut solver_bodies = SolverBodySet::new();
        solver_bodies.copy_from_bodies(&self.bodies, self.dt, self.gravity);

        // Build constraints from contact pairs
        let mut constraints = ContactConstraintSet::new();
        constraints
            .set_position_correction(self.position_correction_beta, self.max_corrective_velocity);
        constraints
            .set_spring_coefficients(self.spring_natural_frequency, self.spring_damping_ratio);
        constraints.build_from_pairs(
            &self.contact_tracker,
            &self.colliders,
            &solver_bodies,
            self.dt,
            self.config.block_solver,
        );

        // Run the full solver loop
        solver::solve(
            &mut solver_bodies,
            &mut constraints,
            &mut self.contact_tracker,
            self.config.num_iterations,
            self.config.num_internal_iterations,
            self.config.num_stabilization_iterations,
            self.config.warmstart_coefficient,
            self.dt,
            self.config.block_solver,
            &mut self.bodies,
        );
    }

    // ========== Sleep System ==========

    fn update_sleep(&mut self) {
        let dt = self.dt;
        let vel_thresh = self.sleep_vel_threshold;
        let ang_thresh = self.sleep_ang_vel_threshold;
        let time_thresh = self.sleep_time_threshold;

        let indices = self.bodies.live_indices();
        for idx in indices {
            let body = self.bodies.get_unchecked_mut(idx);
            if !body.is_dynamic() || !body.enabled {
                continue;
            }
            if body.sleeping {
                continue;
            }

            let speed = body.linvel.length();
            let ang_speed = body.angvel.abs();

            if speed < vel_thresh && ang_speed < ang_thresh {
                body.sleep_timer += dt;
                if body.sleep_timer >= time_thresh {
                    body.sleep();
                }
            } else {
                body.sleep_timer = 0.0;
            }
        }
    }

    fn wake_contacting_bodies(&mut self) {
        for pair in self.contact_tracker.pairs() {
            if pair.manifold.contacts.is_empty() {
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

        if body_a.sleeping && body_b.sleeping {
            return false;
        }

        body_a.can_collide_with(body_b)
    }

    // ========== Swept AABB Broadphase (CCD) ==========

    fn broadphase_swept(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let mut pairs = Vec::new();
        let indices = self.colliders.live_indices();
        let n = indices.len();

        let mut swept_aabbs: Vec<(ColliderHandle, AABB)> = Vec::with_capacity(n);
        for &idx in &indices {
            let col = self.colliders.get_unchecked(idx);
            let ch = self.colliders.handle_for_index(idx);
            let base_aabb = self.compute_collider_aabb(col);

            let expansion = if let Some(ph) = col.parent {
                if let Some(body) = self.bodies.get(ph) {
                    if body.is_dynamic() && !body.sleeping {
                        let v = body.linvel * self.dt;
                        Vec2::new(v.x.abs(), v.y.abs())
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

        for i in 0..n {
            for j in (i + 1)..n {
                let (ca, aabb_a) = &swept_aabbs[i];
                let (cb, aabb_b) = &swept_aabbs[j];
                if aabb_a.overlaps(*aabb_b) {
                    let (ha, hb) = if ca.0 < cb.0 { (*ca, *cb) } else { (*cb, *ca) };
                    pairs.push((ha, hb));
                }
            }
        }

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
        let (pos, rot) = if let Some(parent_handle) = collider.parent {
            if let Some(body) = self.bodies.get(parent_handle) {
                let world_pos = body.pos + body.rot.mul_vec(collider.local_pos);
                let world_rot = body.rot.mul_rot(collider.local_rot);
                (world_pos, world_rot)
            } else {
                (collider.local_pos, collider.local_rot)
            }
        } else {
            (collider.local_pos, collider.local_rot)
        };

        match &collider.shape {
            Shape::Ball { radius } => aabb::aabb_ball(pos, *radius),
            Shape::Cuboid { hx, hy } => aabb::aabb_cuboid(pos, rot, *hx, *hy),
            Shape::Polygon { vertices } => aabb::aabb_polygon(pos, rot, vertices),
        }
    }

    fn body_index(&self, handle: RigidBodyHandle) -> Option<usize> {
        if self.bodies.get(handle).is_some() {
            Some(handle.index())
        } else {
            None
        }
    }

    // ========== Query ==========

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
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

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let b2 = world.create_static_body(0.0, -5.0);
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
            let pos_before = world.get_body_position(dynamic_body);
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
        // - warmstart_coefficient = 0.0 causes impulses to reset each frame
        // - rhs_bias = 0.0 means no position correction during the main solve
        // - num_stabilization_iterations = 0 means no post-integration cleanup
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
}
