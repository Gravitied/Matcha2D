use crate::body::RigidBodySet;
use crate::math::{Rot, Vec2};

/// Mutable velocity state the solver operates on
#[derive(Clone, Copy, Debug, Default)]
pub struct SolverVel {
    pub linear: Vec2,
    pub angular: f32,
}

/// Read-only pose + inverse mass properties for constraint solving
#[derive(Clone, Copy, Debug)]
pub struct SolverPose {
    pub rotation: Rot,
    pub translation: Vec2,
    pub inv_inertia: f32,
    pub inv_mass: f32,
}

impl Default for SolverPose {
    fn default() -> Self {
        Self {
            rotation: Rot::identity(),
            translation: Vec2::zero(),
            inv_inertia: 0.0,
            inv_mass: 0.0,
        }
    }
}

/// Parallel arrays of solver bodies, indexed by body handle index.
/// This matches the Blueprint approach where solver bodies are indexed by body ID.
pub struct SolverBodySet {
    pub vels: Vec<SolverVel>,
    pub poses: Vec<SolverPose>,
    /// Precomputed force increments (gravity + accumulated forces) * dt * inv_mass
    pub force_incr_linear: Vec<Vec2>,
    pub force_incr_angular: Vec<f32>,
    /// Damping coefficients
    pub linear_damping: Vec<f32>,
    pub angular_damping: Vec<f32>,
    /// Maximum body index + 1 (size of all arrays)
    pub capacity: usize,
}

impl SolverBodySet {
    pub fn new() -> Self {
        Self {
            vels: Vec::new(),
            poses: Vec::new(),
            force_incr_linear: Vec::new(),
            force_incr_angular: Vec::new(),
            linear_damping: Vec::new(),
            angular_damping: Vec::new(),
            capacity: 0,
        }
    }

    /// Initialize solver bodies from rigid bodies.
    /// Arrays are sized to max(body_index) + 1 so they can be indexed by body handle index.
    pub fn copy_from_bodies(&mut self, bodies: &RigidBodySet, dt: f32, gravity: Vec2) {
        let indices = bodies.live_indices();
        if indices.is_empty() {
            self.capacity = 0;
            self.vels.clear();
            self.poses.clear();
            self.force_incr_linear.clear();
            self.force_incr_angular.clear();
            self.linear_damping.clear();
            self.angular_damping.clear();
            return;
        }

        let max_idx = indices.iter().copied().max().unwrap();
        let n = max_idx + 1;
        self.capacity = n;

        self.vels.resize_with(n, SolverVel::default);
        self.poses.resize_with(n, SolverPose::default);
        self.force_incr_linear.resize_with(n, Vec2::zero);
        self.force_incr_angular.resize_with(n, || 0.0);
        self.linear_damping.resize_with(n, || 0.0);
        self.angular_damping.resize_with(n, || 0.0);

        for &body_idx in &indices {
            let body = bodies.get_unchecked(body_idx);

            // Copy velocities
            self.vels[body_idx].linear = body.linvel;
            self.vels[body_idx].angular = body.angvel;

            // Copy pose
            self.poses[body_idx].rotation = body.rot;
            self.poses[body_idx].translation = body.pos;

            // Copy inverse mass properties (zero for static bodies)
            if body.is_dynamic() {
                self.poses[body_idx].inv_mass = body.inv_mass;
                self.poses[body_idx].inv_inertia = body.inv_inertia;
            } else {
                self.poses[body_idx].inv_mass = 0.0;
                self.poses[body_idx].inv_inertia = 0.0;
            }

            // Compute force increments: (gravity * gravity_scale + force * inv_mass) * dt
            if body.is_dynamic() && !body.sleeping {
                let grav_incr = gravity * body.gravity_scale * dt;
                let force_incr = body.force * body.inv_mass * dt;
                self.force_incr_linear[body_idx] = grav_incr + force_incr;
                self.force_incr_angular[body_idx] = body.torque * body.inv_inertia * dt;
            } else {
                self.force_incr_linear[body_idx] = Vec2::zero();
                self.force_incr_angular[body_idx] = 0.0;
            }

            // Store damping
            self.linear_damping[body_idx] = body.linear_damping;
            self.angular_damping[body_idx] = body.angular_damping;
        }
    }

    /// Add precomputed force increments to solver velocities.
    pub fn apply_force_increments(&mut self) {
        for i in 0..self.capacity {
            self.vels[i].linear += self.force_incr_linear[i];
            self.vels[i].angular += self.force_incr_angular[i];
        }
    }

    /// Apply damping to solver velocities.
    pub fn apply_damping(&mut self, dt: f32) {
        for i in 0..self.capacity {
            if self.linear_damping[i] > 0.0 {
                let ld = 1.0 / (1.0 + self.linear_damping[i] * dt);
                self.vels[i].linear *= ld;
            }
            if self.angular_damping[i] > 0.0 {
                let ad = 1.0 / (1.0 + self.angular_damping[i] * dt);
                self.vels[i].angular *= ad;
            }
        }
    }

    /// Integrate positions from solver velocities.
    pub fn integrate_positions(&mut self, dt: f32) {
        for i in 0..self.capacity {
            let pose = &mut self.poses[i];
            let vel = &mut self.vels[i];

            if pose.inv_mass > 0.0 {
                // NaN guard - sanitize corrupted values
                if vel.linear.x.is_nan() || vel.linear.y.is_nan() || vel.angular.is_nan() {
                    vel.linear = Vec2::zero();
                    vel.angular = 0.0;
                }
                if pose.translation.x.is_nan() || pose.translation.y.is_nan() {
                    pose.translation = Vec2::zero();
                }

                // Hard velocity clamp to prevent explosion
                const MAX_LINEAR_VEL: f32 = 500.0;
                const MAX_ANGULAR_VEL: f32 = 500.0;
                let lin_len = vel.linear.length();
                if lin_len > MAX_LINEAR_VEL {
                    vel.linear = vel.linear * (MAX_LINEAR_VEL / lin_len);
                }
                if vel.angular.abs() > MAX_ANGULAR_VEL {
                    vel.angular = vel.angular.signum() * MAX_ANGULAR_VEL;
                }

                // Clamp position delta to prevent explosion from extreme velocities
                let delta = vel.linear * dt;
                const MAX_POSITION_DELTA: f32 = 10.0;
                let delta_len = delta.length();
                let clamped_delta = if delta_len > MAX_POSITION_DELTA {
                    delta * (MAX_POSITION_DELTA / delta_len)
                } else {
                    delta
                };

                pose.translation += clamped_delta;
                let angle_delta = vel.angular * dt;
                let angle_delta = angle_delta.clamp(-10.0, 10.0);
                let angle = pose.rotation.angle() + angle_delta;
                pose.rotation = Rot::from_angle(angle);
            }
        }
    }

    /// Write solver velocities and positions back to rigid bodies.
    pub fn writeback_to_bodies(&self, bodies: &mut RigidBodySet) {
        let indices = bodies.live_indices();
        for &body_idx in &indices {
            let body = bodies.get_unchecked_mut(body_idx);
            if !body.is_dynamic() || body.sleeping {
                continue;
            }

            let vel = &self.vels[body_idx];
            let pose = &self.poses[body_idx];

            // Final NaN guard before writing back
            let linvel = if vel.linear.x.is_nan() || vel.linear.y.is_nan() {
                Vec2::zero()
            } else {
                vel.linear
            };
            let angvel = if vel.angular.is_nan() {
                0.0
            } else {
                vel.angular
            };
            let pos = if pose.translation.x.is_nan() || pose.translation.y.is_nan() {
                body.pos
            } else {
                pose.translation
            };

            body.linvel = linvel;
            body.angvel = angvel;
            body.pos = pos;
            body.rot = pose.rotation;
        }
    }

    #[inline]
    pub fn vel(&self, body_idx: usize) -> &SolverVel {
        &self.vels[body_idx]
    }

    #[inline]
    pub fn vel_mut(&mut self, body_idx: usize) -> &mut SolverVel {
        &mut self.vels[body_idx]
    }

    #[inline]
    pub fn pose(&self, body_idx: usize) -> &SolverPose {
        &self.poses[body_idx]
    }

    #[inline]
    pub fn pose_mut(&mut self, body_idx: usize) -> &mut SolverPose {
        &mut self.poses[body_idx]
    }

    pub fn len(&self) -> usize {
        self.capacity
    }
}

impl Default for SolverBodySet {
    fn default() -> Self {
        Self::new()
    }
}
