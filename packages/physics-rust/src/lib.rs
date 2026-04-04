mod aabb;
mod body;
mod bvh;
mod collider;
pub mod contact;
mod contact_constraint;
mod math;
mod narrowphase;
pub mod solver;
mod solver_body;
mod velocity_solver;
pub mod world;

use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PhysicsEngine {
    world: world::PhysicsWorld,
}

#[wasm_bindgen]
impl PhysicsEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> PhysicsEngine {
        PhysicsEngine {
            world: world::PhysicsWorld::new(),
        }
    }

    pub fn with_gravity(mut self, x: f32, y: f32) -> PhysicsEngine {
        self.world.set_gravity(x, y);
        self
    }

    pub fn set_gravity(&mut self, x: f32, y: f32) {
        self.world.set_gravity(x, y);
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.world.set_dt(dt);
    }

    pub fn get_dt(&self) -> f32 {
        self.world.get_dt()
    }

    pub fn set_swept_broadphase(&mut self, enabled: bool) {
        self.world.swept_broadphase = enabled;
    }

    pub fn set_sleep_thresholds(&mut self, vel: f32, ang_vel: f32, time: f32) {
        self.world.set_sleep_thresholds(vel, ang_vel, time);
    }

    // ========== Solver Configuration ==========

    pub fn set_solver_config(
        &mut self,
        num_iterations: u32,
        num_internal_iterations: u32,
        num_stabilization_iterations: u32,
        warmstart_coefficient: f32,
        block_solver: bool,
    ) {
        self.world.set_solver_config(
            num_iterations as usize,
            num_internal_iterations as usize,
            num_stabilization_iterations as usize,
            warmstart_coefficient,
            block_solver,
        );
    }

    pub fn set_position_correction(&mut self, beta: f32, max_velocity: f32) {
        self.world.set_position_correction(beta, max_velocity);
    }

    pub fn set_spring_coefficients(&mut self, natural_frequency: f32, damping_ratio: f32) {
        self.world
            .set_spring_coefficients(natural_frequency, damping_ratio);
    }

    // ========== Body Management ==========

    pub fn create_dynamic_body(&mut self, x: f32, y: f32) -> u32 {
        self.world.create_dynamic_body(x, y)
    }

    pub fn create_static_body(&mut self, x: f32, y: f32) -> u32 {
        self.world.create_static_body(x, y)
    }

    pub fn create_kinematic_body(&mut self, x: f32, y: f32) -> u32 {
        self.world.create_kinematic_body(x, y)
    }

    pub fn remove_body(&mut self, handle: u32) {
        self.world.remove_body(handle);
    }

    pub fn set_body_position(&mut self, handle: u32, x: f32, y: f32) {
        self.world.set_body_position(handle, x, y);
    }

    pub fn set_body_velocity(&mut self, handle: u32, vx: f32, vy: f32) {
        self.world.set_body_velocity(handle, vx, vy);
    }

    pub fn set_body_angle(&mut self, handle: u32, angle: f32) {
        self.world.set_body_angle(handle, angle);
    }

    pub fn set_body_collision_groups(&mut self, handle: u32, groups: u32) {
        self.world.set_body_collision_groups(handle, groups);
    }

    pub fn set_body_collision_mask(&mut self, handle: u32, mask: u32) {
        self.world.set_body_collision_mask(handle, mask);
    }

    pub fn wake_body(&mut self, handle: u32) {
        self.world.wake_body(handle);
    }

    pub fn sleep_body(&mut self, handle: u32) {
        self.world.sleep_body(handle);
    }

    pub fn is_body_sleeping(&self, handle: u32) -> bool {
        self.world.is_body_sleeping(handle)
    }

    pub fn get_body_angle(&self, handle: u32) -> f32 {
        self.world.get_body_angle(handle)
    }

    pub fn apply_impulse(&mut self, handle: u32, fx: f32, fy: f32) {
        self.world.apply_impulse(handle, fx, fy);
    }

    pub fn get_body_position(&self, handle: u32) -> Vec<f32> {
        self.world.get_body_position(handle)
    }

    pub fn get_body_velocity(&self, handle: u32) -> Vec<f32> {
        self.world.get_body_velocity(handle)
    }

    pub fn set_body_angular_velocity(&mut self, handle: u32, angvel: f32) {
        self.world.set_body_angular_velocity(handle, angvel);
    }

    pub fn get_body_angular_velocity(&self, handle: u32) -> f32 {
        self.world.get_body_angular_velocity(handle)
    }

    // ========== Collider Management ==========

    pub fn create_ball_collider(&mut self, radius: f32, body: u32) -> u32 {
        self.world.create_ball_collider(radius, body)
    }

    pub fn create_box_collider(&mut self, hx: f32, hy: f32, body: u32) -> u32 {
        self.world.create_box_collider(hx, hy, body)
    }

    pub fn create_polygon_collider(
        &mut self,
        vertices_x: &[f32],
        vertices_y: &[f32],
        body: u32,
    ) -> u32 {
        self.world
            .create_polygon_collider(vertices_x, vertices_y, body)
    }

    pub fn remove_collider(&mut self, handle: u32) {
        self.world.remove_collider(handle);
    }

    pub fn set_collider_friction(&mut self, handle: u32, friction: f32) {
        self.world.set_collider_friction(handle, friction);
    }

    pub fn set_collider_restitution(&mut self, handle: u32, restitution: f32) {
        self.world.set_collider_restitution(handle, restitution);
    }

    pub fn set_collider_sensor(&mut self, handle: u32, sensor: bool) {
        self.world.set_collider_sensor(handle, sensor);
    }

    // ========== Simulation ==========

    pub fn step(&mut self) {
        self.world.step();
    }

    // ========== Query ==========

    pub fn body_count(&self) -> usize {
        self.world.body_count()
    }

    pub fn collider_count(&self) -> usize {
        self.world.collider_count()
    }

    pub fn get_positions_x(&self) -> Vec<f32> {
        self.world.get_positions_x()
    }

    pub fn get_positions_y(&self) -> Vec<f32> {
        self.world.get_positions_y()
    }

    pub fn get_angles(&self) -> Vec<f32> {
        self.world.get_angles()
    }

    pub fn get_angular_velocities(&self) -> Vec<f32> {
        self.world.get_angular_velocities()
    }

    // ========== Diagnostics ==========

    /// Returns the number of active contact pairs this frame
    pub fn contact_pair_count(&self) -> usize {
        self.world.contact_tracker.pair_count()
    }

    /// Returns the total number of contact points across all pairs
    pub fn contact_point_count(&self) -> usize {
        self.world
            .contact_tracker
            .pairs()
            .map(|p| p.manifold.contacts.len())
            .sum()
    }

    /// Returns the maximum penetration depth across all contacts this frame
    pub fn max_penetration(&self) -> f32 {
        self.world
            .contact_tracker
            .pairs()
            .flat_map(|p| p.manifold.contacts.iter().map(|c| c.penetration))
            .fold(0.0f32, f32::max)
    }

    /// Returns contact normal for the first contact pair (if any)
    pub fn first_contact_normal(&self) -> Vec<f32> {
        if let Some(pair) = self.world.contact_tracker.pairs().next() {
            vec![pair.manifold.normal.x, pair.manifold.normal.y]
        } else {
            vec![0.0, 0.0]
        }
    }

    /// Returns a diagnostic string for the current frame
    pub fn debug_dump(&self) -> String {
        let pairs = self.world.contact_tracker.pair_count();
        let points: usize = self
            .world
            .contact_tracker
            .pairs()
            .map(|p| p.manifold.contacts.len())
            .sum();
        let max_pen = self
            .world
            .contact_tracker
            .pairs()
            .flat_map(|p| p.manifold.contacts.iter().map(|c| c.penetration))
            .fold(0.0f32, f32::max);
        let sleep_count = self
            .world
            .bodies
            .live_indices()
            .iter()
            .filter(|&&i| self.world.bodies.get_unchecked(i).sleeping)
            .count();
        let dynamic_count = self
            .world
            .bodies
            .live_indices()
            .iter()
            .filter(|&&i| self.world.bodies.get_unchecked(i).is_dynamic())
            .count();
        format!(
            "pairs={pairs} pts={points} max_pen={max_pen:.6} sleeping={sleep_count} dynamic={dynamic_count}"
        )
    }
}

impl Default for PhysicsEngine {
    fn default() -> Self {
        Self::new()
    }
}
