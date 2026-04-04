use crate::contact::ContactTracker;
use crate::math::Vec2;
use crate::solver_body::SolverBodySet;

/// Spring-based ERP/CFM coefficients for contact softness
pub struct ContactSpringCoefficients {
    pub natural_frequency: f32,
    pub damping_ratio: f32,
}

impl ContactSpringCoefficients {
    pub fn default_contact() -> Self {
        Self {
            natural_frequency: 15.0,
            damping_ratio: 1.0,
        }
    }

    pub fn erp_inv_dt(&self, dt: f32) -> f32 {
        let ang_freq = self.natural_frequency * 2.0 * std::f32::consts::PI;
        ang_freq / (dt * ang_freq + 2.0 * self.damping_ratio)
    }

    pub fn cfm_factor(&self, dt: f32) -> f32 {
        let erp = self.erp(dt);
        if erp == 0.0 {
            return 1.0;
        }
        let inv_erp_minus_one = 1.0 / erp - 1.0;
        let denom = (1.0 + inv_erp_minus_one) * 4.0 * self.damping_ratio * self.damping_ratio;
        if denom == 0.0 {
            return 1.0;
        }
        let cfm_coeff = (inv_erp_minus_one * inv_erp_minus_one) / denom;
        1.0 / (1.0 + cfm_coeff)
    }

    fn erp(&self, dt: f32) -> f32 {
        let ang_freq = self.natural_frequency * 2.0 * std::f32::consts::PI;
        ang_freq * dt / (dt * ang_freq + 2.0 * self.damping_ratio)
    }
}

/// Per-contact-point normal constraint
#[derive(Clone, Debug)]
pub struct ContactConstraintNormal {
    pub body_a: usize,
    pub body_b: usize,
    pub normal: Vec2,
    pub torque_dir_a: f32,
    pub torque_dir_b: f32,
    pub ii_torque_dir_a: f32,
    pub ii_torque_dir_b: f32,
    pub rhs: f32,
    pub rhs_wo_bias: f32,
    pub impulse: f32,
    pub projected_mass: f32,
    pub cfm_factor: f32,
    pub r_mat_diag: f32,
    pub r_mat_offdiag: f32,
    pub penetration: f32,
}

/// Per-contact-point tangent (friction) constraint
#[derive(Clone, Debug)]
pub struct ContactConstraintTangent {
    pub body_a: usize,
    pub body_b: usize,
    pub tangent: Vec2,
    pub torque_dir_a: f32,
    pub torque_dir_b: f32,
    pub ii_torque_dir_a: f32,
    pub ii_torque_dir_b: f32,
    pub rhs: f32,
    pub impulse: f32,
    pub projected_mass: f32,
    pub friction_coeff: f32,
    pub normal_idx: usize,
}

/// A group of constraints for a single contact pair
#[derive(Debug)]
pub struct ContactPairConstraints {
    pub collider_a: crate::collider::ColliderHandle,
    pub collider_b: crate::collider::ColliderHandle,
    pub normals: Vec<ContactConstraintNormal>,
    pub tangents: Vec<ContactConstraintTangent>,
}

/// Full constraint set for all active contact pairs
pub struct ContactConstraintSet {
    pub pairs: Vec<ContactPairConstraints>,
    pub spring: ContactSpringCoefficients,
    pub allowed_linear_error: f32,
    pub max_corrective_velocity: f32,
    pub baumgarte_factor: f32,
}

impl ContactConstraintSet {
    pub fn new() -> Self {
        Self {
            pairs: Vec::new(),
            spring: ContactSpringCoefficients::default_contact(),
            allowed_linear_error: 0.001,
            max_corrective_velocity: 50.0,
            baumgarte_factor: 0.01,
        }
    }

    pub fn set_position_correction(&mut self, beta: f32, max_velocity: f32) {
        self.baumgarte_factor = beta;
        self.max_corrective_velocity = max_velocity;
    }

    pub fn set_spring_coefficients(&mut self, natural_frequency: f32, damping_ratio: f32) {
        self.spring.natural_frequency = natural_frequency;
        self.spring.damping_ratio = damping_ratio;
    }

    /// Build constraints from contact pairs and solver bodies
    pub fn build_from_pairs(
        &mut self,
        tracker: &ContactTracker,
        colliders: &crate::collider::ColliderSet,
        solver_bodies: &SolverBodySet,
        dt: f32,
        block_solver: bool,
    ) {
        self.pairs.clear();

        let cfm_factor = self.spring.cfm_factor(dt);
        let erp_inv_dt = self.spring.erp_inv_dt(dt);
        let inv_dt = 1.0 / dt;

        for pair in tracker.pairs() {
            if pair.manifold.contacts.is_empty() {
                continue;
            }

            let col_a = match colliders.get(pair.collider_a) {
                Some(c) => c,
                None => continue,
            };
            let col_b = match colliders.get(pair.collider_b) {
                Some(c) => c,
                None => continue,
            };

            let body_a_idx = match col_a.parent {
                Some(h) => h.index(),
                None => continue,
            };
            let body_b_idx = match col_b.parent {
                Some(h) => h.index(),
                None => continue,
            };

            let normal = pair.manifold.normal;
            let tangent = normal.perp();
            let friction = (col_a.friction * col_b.friction).sqrt();
            let restitution = col_a.restitution.max(col_b.restitution);

            let mut normals = Vec::new();
            let mut tangents = Vec::new();

            for (ci, contact) in pair.manifold.contacts.iter().enumerate() {
                let pose_a = solver_bodies.pose(body_a_idx);
                let pose_b = solver_bodies.pose(body_b_idx);

                let r_a = pose_a.rotation.mul_vec(contact.local_a);
                let r_b = pose_b.rotation.mul_vec(contact.local_b);

                let torque_a = r_a.cross(normal);
                let torque_b = r_b.cross(-normal);

                let ii_td_a = pose_a.inv_inertia * torque_a;
                let ii_td_b = pose_b.inv_inertia * torque_b;

                let inv_mass_sum = pose_a.inv_mass + pose_b.inv_mass;
                let k = inv_mass_sum + torque_a * ii_td_a + torque_b * ii_td_b;
                let proj_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

                // Store effective mass (k) for block solver, not projected mass.
                // The block solver solves K_eff * Δλ = -dvel, so diagonal must be k.
                let r_mat_diag = k;
                let r_mat_offdiag = 0.0;

                let vel_a = solver_bodies.vel(body_a_idx);
                let vel_b = solver_bodies.vel(body_b_idx);
                let v_a = vel_a.linear + Vec2::new(-vel_a.angular * r_a.y, vel_a.angular * r_a.x);
                let v_b = vel_b.linear + Vec2::new(-vel_b.angular * r_b.y, vel_b.angular * r_b.x);
                let rel_vel = v_b - v_a;
                let normal_vel = rel_vel.dot(normal);

                let is_new = contact.is_new;
                let is_bouncy = if is_new {
                    restitution > 0.0
                } else {
                    restitution >= 1.0
                };

                let rhs_wo_bias = 0.0;

                let dist = -contact.penetration;
                // No Baumgarte velocity bias — it explodes at pixel scale where
                // penetration * inv_dt produces massive values. Position correction
                // is handled by the stabilization pass instead.
                let rhs_bias = 0.0;

                let rhs = if is_bouncy {
                    rhs_wo_bias + restitution * normal_vel.min(0.0) + rhs_bias
                } else {
                    rhs_wo_bias + rhs_bias
                };

                let warmstart_impulse = if ci < pair.prev_accumulated_impulses.len() {
                    pair.prev_accumulated_impulses[ci].0
                } else {
                    0.0
                };

                normals.push(ContactConstraintNormal {
                    body_a: body_a_idx,
                    body_b: body_b_idx,
                    normal,
                    torque_dir_a: torque_a,
                    torque_dir_b: torque_b,
                    ii_torque_dir_a: ii_td_a,
                    ii_torque_dir_b: ii_td_b,
                    rhs,
                    rhs_wo_bias,
                    impulse: warmstart_impulse,
                    projected_mass: proj_mass,
                    cfm_factor,
                    r_mat_diag,
                    r_mat_offdiag,
                    penetration: contact.penetration,
                });

                let rt_a = r_a.cross(tangent);
                let rt_b = r_b.cross(tangent);
                let ii_rt_a = pose_a.inv_inertia * rt_a;
                let ii_rt_b = pose_b.inv_inertia * rt_b;
                let k_t = inv_mass_sum + rt_a * ii_rt_a + rt_b * ii_rt_b;
                let proj_mass_t = if k_t > 0.0 { 1.0 / k_t } else { 0.0 };

                let tangent_vel = rel_vel.dot(tangent);
                let warmstart_tangent = if ci < pair.prev_accumulated_impulses.len() {
                    pair.prev_accumulated_impulses[ci].1
                } else {
                    0.0
                };

                tangents.push(ContactConstraintTangent {
                    body_a: body_a_idx,
                    body_b: body_b_idx,
                    tangent,
                    torque_dir_a: rt_a,
                    torque_dir_b: rt_b,
                    ii_torque_dir_a: ii_rt_a,
                    ii_torque_dir_b: ii_rt_b,
                    rhs: 0.0,
                    impulse: warmstart_tangent,
                    projected_mass: proj_mass_t,
                    friction_coeff: friction,
                    normal_idx: ci,
                });
            }

            if block_solver && normals.len() >= 2 {
                for i in (0..normals.len().saturating_sub(1)).step_by(2) {
                    let na = &normals[i];
                    let nb = &normals[i + 1];
                    let pose_a = solver_bodies.pose(na.body_a);
                    let pose_b = solver_bodies.pose(na.body_b);
                    let inv_mass_sum = pose_a.inv_mass + pose_b.inv_mass;
                    let offdiag = inv_mass_sum
                        + na.torque_dir_a * pose_a.inv_inertia * nb.torque_dir_a
                        + na.torque_dir_b * pose_b.inv_inertia * nb.torque_dir_b;
                    normals[i].r_mat_offdiag = offdiag;
                    normals[i + 1].r_mat_offdiag = offdiag;
                }
            }

            self.pairs.push(ContactPairConstraints {
                collider_a: pair.collider_a,
                collider_b: pair.collider_b,
                normals,
                tangents,
            });
        }
    }

    /// Apply warmstart impulses to solver velocities
    pub fn warmstart(&mut self, solver_bodies: &mut SolverBodySet, coeff: f32) {
        for pair in &mut self.pairs {
            for n in &mut pair.normals {
                if n.normal.x.is_nan() || n.normal.y.is_nan() || n.impulse.is_nan() {
                    continue;
                }
                let impulse = n.impulse * coeff;
                n.impulse = impulse;
                if impulse.abs() < 1e-10 {
                    continue;
                }
                let ba = n.body_a;
                let bb = n.body_b;
                let normal = n.normal;
                let ii_a = n.ii_torque_dir_a;
                let ii_b = n.ii_torque_dir_b;
                let im_a = solver_bodies.pose(ba).inv_mass;
                let im_b = solver_bodies.pose(bb).inv_mass;
                let va = solver_bodies.vel_mut(ba);
                va.linear -= normal * (impulse * im_a);
                va.angular -= ii_a * impulse;
                let vb = solver_bodies.vel_mut(bb);
                vb.linear += normal * (impulse * im_b);
                vb.angular -= ii_b * impulse;
            }
        }
    }

    /// Update constraints: recompute RHS bias from current solver body velocities
    /// and current penetration depths. Only stores bias terms in rhs — the relative
    /// velocity is computed fresh in the solve step.
    pub fn update(&mut self, solver_bodies: &SolverBodySet, dt: f32) {
        let cfm_factor = self.spring.cfm_factor(dt);
        let inv_dt = 1.0 / dt;
        let beta = self.baumgarte_factor;

        for pair in &mut self.pairs {
            for n in &mut pair.normals {
                let vel_a = solver_bodies.vel(n.body_a);
                let vel_b = solver_bodies.vel(n.body_b);

                let dvel = n.normal.dot(vel_b.linear - vel_a.linear)
                    - n.torque_dir_a * vel_a.angular
                    - n.torque_dir_b * vel_b.angular;

                let dist = -n.penetration;
                let rhs_bias = if dist < 0.0 {
                    (dist * beta * inv_dt).clamp(-self.max_corrective_velocity, 0.0)
                } else {
                    0.0
                };

                n.rhs_wo_bias = dvel;
                // rhs contains ONLY bias terms — relative velocity is computed in solve
                n.rhs = rhs_bias;
                n.cfm_factor = cfm_factor;
            }
        }
    }

    /// Solve velocity constraints without position correction bias.
    /// Used for PGS iterations after the first one — the bias was already
    /// applied in the first iteration, so subsequent iterations should only
    /// resolve relative velocity, not re-apply position correction.
    pub fn solve_wo_bias_vel(&mut self, solver_bodies: &mut SolverBodySet, block_solver: bool) {
        for pi in 0..self.pairs.len() {
            let pair = &mut self.pairs[pi];

            if block_solver && pair.normals.len() >= 2 {
                for i in (0..pair.normals.len().saturating_sub(1)).step_by(2) {
                    let (left, right) = pair.normals.split_at_mut(i + 1);
                    solve_normal_pair_wo_bias(solver_bodies, &mut left[i], &mut right[0]);
                }
                if pair.normals.len() % 2 == 1 {
                    let last = pair.normals.len() - 1;
                    solve_normal_single_wo_bias(solver_bodies, &mut pair.normals[last]);
                }
            } else {
                for n in &mut pair.normals {
                    solve_normal_single_wo_bias(solver_bodies, n);
                }
            }

            for ti in 0..pair.tangents.len() {
                let normal_impulse = if pair.tangents[ti].normal_idx < pair.normals.len() {
                    pair.normals[pair.tangents[ti].normal_idx].impulse
                } else {
                    0.0
                };
                let friction_limit = pair.tangents[ti].friction_coeff * normal_impulse;
                solve_tangent(solver_bodies, &mut pair.tangents[ti], friction_limit);
            }
        }

        // Hard velocity clamp during PGS to prevent divergence
        const MAX_REASONABLE_VEL: f32 = 100.0;
        for i in 0..solver_bodies.capacity {
            let vel = &mut solver_bodies.vels[i];
            let lin_len = vel.linear.length();
            if lin_len > MAX_REASONABLE_VEL {
                vel.linear = vel.linear * (MAX_REASONABLE_VEL / lin_len);
            }
            if vel.angular.abs() > MAX_REASONABLE_VEL {
                vel.angular = vel.angular.signum() * MAX_REASONABLE_VEL;
            }
            if vel.linear.x.is_nan() || vel.linear.y.is_nan() {
                vel.linear = Vec2::zero();
            }
            if vel.angular.is_nan() {
                vel.angular = 0.0;
            }
        }
    }

    /// Solve all constraints with bias (main PGS iterations)
    pub fn solve(&mut self, solver_bodies: &mut SolverBodySet, block_solver: bool) {
        for pi in 0..self.pairs.len() {
            let pair = &mut self.pairs[pi];

            if block_solver && pair.normals.len() >= 2 {
                for i in (0..pair.normals.len().saturating_sub(1)).step_by(2) {
                    let (left, right) = pair.normals.split_at_mut(i + 1);
                    solve_normal_pair(solver_bodies, &mut left[i], &mut right[0]);
                }
                if pair.normals.len() % 2 == 1 {
                    let last = pair.normals.len() - 1;
                    solve_normal_single(solver_bodies, &mut pair.normals[last]);
                }
            } else {
                for n in &mut pair.normals {
                    solve_normal_single(solver_bodies, n);
                }
            }

            for ti in 0..pair.tangents.len() {
                let normal_impulse = if pair.tangents[ti].normal_idx < pair.normals.len() {
                    pair.normals[pair.tangents[ti].normal_idx].impulse
                } else {
                    0.0
                };
                let friction_limit = pair.tangents[ti].friction_coeff * normal_impulse;
                solve_tangent(solver_bodies, &mut pair.tangents[ti], friction_limit);
            }
        }

        // Hard velocity clamp during PGS to prevent divergence
        const MAX_REASONABLE_VEL: f32 = 100.0;
        for i in 0..solver_bodies.capacity {
            let vel = &mut solver_bodies.vels[i];
            let lin_len = vel.linear.length();
            if lin_len > MAX_REASONABLE_VEL {
                vel.linear = vel.linear * (MAX_REASONABLE_VEL / lin_len);
            }
            if vel.angular.abs() > MAX_REASONABLE_VEL {
                vel.angular = vel.angular.signum() * MAX_REASONABLE_VEL;
            }
            if vel.linear.x.is_nan() || vel.linear.y.is_nan() {
                vel.linear = Vec2::zero();
            }
            if vel.angular.is_nan() {
                vel.angular = 0.0;
            }
        }
    }

    /// Rebuild constraint RHS from current solver velocities.
    /// Call this between PGS iterations to keep the RHS fresh.
    pub fn rebuild_rhs(&mut self, solver_bodies: &SolverBodySet, dt: f32) {
        let inv_dt = 1.0 / dt;
        let beta = self.baumgarte_factor;

        for pair in &mut self.pairs {
            for n in &mut pair.normals {
                let vel_a = solver_bodies.vel(n.body_a);
                let vel_b = solver_bodies.vel(n.body_b);
                let normal_vel = n.normal.dot(vel_b.linear - vel_a.linear)
                    + n.torque_dir_b * vel_b.angular
                    - n.torque_dir_a * vel_a.angular;

                let dist = -n.penetration;
                let rhs_bias = if dist < 0.0 {
                    (dist * beta * inv_dt).clamp(-self.max_corrective_velocity, 0.0)
                } else {
                    0.0
                };

                n.rhs_wo_bias = normal_vel;
                // rhs contains ONLY bias — relative velocity computed in solve
                n.rhs = rhs_bias;
            }
        }
    }

    /// Solve without bias (stabilization pass)
    /// Performs direct position correction to resolve remaining penetration.
    /// Does NOT modify velocities — the main velocity solve already resolved
    /// relative velocities. This just fixes up positions that drifted.
    ///
    /// IMPORTANT: Correction is applied ONCE per contact pair (using max penetration),
    /// not once per contact point. Applying per-point with N contacts would multiply
    /// the correction by N, causing massive over-correction and explosion.
    pub fn solve_wo_bias(&mut self, solver_bodies: &mut SolverBodySet, _block_solver: bool) {
        let correction_fraction = 0.3;
        for pair in &mut self.pairs {
            // Find the maximum penetration in this pair — only correct once per pair
            let mut max_pen = 0.0f32;
            let mut max_normal = crate::math::Vec2::zero();
            let mut body_a = 0usize;
            let mut body_b = 0usize;

            for n in &pair.normals {
                if n.penetration > max_pen {
                    max_pen = n.penetration;
                    max_normal = n.normal;
                    body_a = n.body_a;
                    body_b = n.body_b;
                }
            }

            if max_pen <= 0.0 {
                continue;
            }

            let im_a = solver_bodies.pose(body_a).inv_mass;
            let im_b = solver_bodies.pose(body_b).inv_mass;
            let total_im = im_a + im_b;
            if total_im == 0.0 {
                continue;
            }

            let correction = max_pen * correction_fraction;
            let move_a = correction * (im_a / total_im);
            let move_b = correction * (im_b / total_im);
            let pose_a = &mut solver_bodies.poses[body_a];
            pose_a.translation -= max_normal * move_a;
            let pose_b = &mut solver_bodies.poses[body_b];
            pose_b.translation += max_normal * move_b;
        }
    }

    /// Write back accumulated impulses to contact tracker for next-frame warmstart
    pub fn writeback_impulses(&self, tracker: &mut ContactTracker) {
        for pair_constraints in &self.pairs {
            if let Some(pair) = tracker.pairs_mut().find(|p| {
                p.collider_a == pair_constraints.collider_a
                    && p.collider_b == pair_constraints.collider_b
            }) {
                let n = &pair_constraints.normals;
                let t = &pair_constraints.tangents;
                pair.prev_accumulated_impulses = (0..n.len().max(t.len()))
                    .map(|i| {
                        let ni = if i < n.len() { n[i].impulse } else { 0.0 };
                        let ti = if i < t.len() { t[i].impulse } else { 0.0 };
                        (ni, ti)
                    })
                    .collect();
            }
        }
    }
}

impl Default for ContactConstraintSet {
    fn default() -> Self {
        Self::new()
    }
}

// ---- Free functions to avoid borrow checker issues ----

fn solve_normal_single(solver_bodies: &mut SolverBodySet, n: &mut ContactConstraintNormal) {
    let ba = n.body_a;
    let bb = n.body_b;
    let normal = n.normal;
    let td_a = n.torque_dir_a;
    let td_b = n.torque_dir_b;
    let ii_a = n.ii_torque_dir_a;
    let ii_b = n.ii_torque_dir_b;
    let rhs = n.rhs;
    let pm = n.projected_mass;
    let cf = n.cfm_factor;

    // NaN guard on constraint data
    if normal.x.is_nan() || normal.y.is_nan() || pm.is_nan() || rhs.is_nan() {
        return;
    }

    let va_lin = solver_bodies.vel(ba).linear;
    let va_ang = solver_bodies.vel(ba).angular;
    let vb_lin = solver_bodies.vel(bb).linear;
    let vb_ang = solver_bodies.vel(bb).angular;

    // NaN guard on velocities
    if va_lin.x.is_nan()
        || va_lin.y.is_nan()
        || va_ang.is_nan()
        || vb_lin.x.is_nan()
        || vb_lin.y.is_nan()
        || vb_ang.is_nan()
    {
        return;
    }

    let dvel = normal.dot(vb_lin - va_lin) - td_a * va_ang - td_b * vb_ang + rhs;

    let new_impulse = cf * (n.impulse - pm * dvel).max(0.0);
    let mut dlambda = new_impulse - n.impulse;
    // Clamp impulse delta to prevent runaway accumulation in PGS iterations.
    // Use a scale-relative clamp based on the projected mass and expected velocity range.
    let max_dlambda = (pm * 1000.0).max(1.0);
    dlambda = dlambda.clamp(-max_dlambda, max_dlambda);
    n.impulse += dlambda;

    let im_a = solver_bodies.pose(ba).inv_mass;
    let im_b = solver_bodies.pose(bb).inv_mass;

    let va = solver_bodies.vel_mut(ba);
    va.linear -= normal * (dlambda * im_a);
    va.angular -= ii_a * dlambda;

    let vb = solver_bodies.vel_mut(bb);
    vb.linear += normal * (dlambda * im_b);
    vb.angular -= ii_b * dlambda;
}

fn solve_normal_single_wo_bias(solver_bodies: &mut SolverBodySet, n: &mut ContactConstraintNormal) {
    let ba = n.body_a;
    let bb = n.body_b;
    let normal = n.normal;
    let td_a = n.torque_dir_a;
    let td_b = n.torque_dir_b;
    let ii_a = n.ii_torque_dir_a;
    let ii_b = n.ii_torque_dir_b;
    let rhs = n.rhs_wo_bias;
    let pm = n.projected_mass;

    if normal.x.is_nan() || normal.y.is_nan() || pm.is_nan() || rhs.is_nan() {
        return;
    }

    let va_lin = solver_bodies.vel(ba).linear;
    let va_ang = solver_bodies.vel(ba).angular;
    let vb_lin = solver_bodies.vel(bb).linear;
    let vb_ang = solver_bodies.vel(bb).angular;

    if va_lin.x.is_nan()
        || va_lin.y.is_nan()
        || va_ang.is_nan()
        || vb_lin.x.is_nan()
        || vb_lin.y.is_nan()
        || vb_ang.is_nan()
    {
        return;
    }

    let dvel = normal.dot(vb_lin - va_lin) - td_a * va_ang - td_b * vb_ang + rhs;
    let dvel = dvel.clamp(-100000.0, 100000.0);

    let new_impulse = (n.impulse - pm * dvel).max(0.0);
    let dlambda = new_impulse - n.impulse;
    n.impulse = new_impulse;

    let im_a = solver_bodies.pose(ba).inv_mass;
    let im_b = solver_bodies.pose(bb).inv_mass;

    let va = solver_bodies.vel_mut(ba);
    va.linear -= normal * (dlambda * im_a);
    va.angular -= ii_a * dlambda;

    let vb = solver_bodies.vel_mut(bb);
    vb.linear += normal * (dlambda * im_b);
    vb.angular -= ii_b * dlambda;
}

fn solve_normal_pair(
    solver_bodies: &mut SolverBodySet,
    na: &mut ContactConstraintNormal,
    nb: &mut ContactConstraintNormal,
) {
    let ba = na.body_a;
    let bb = na.body_b;
    let na_normal = na.normal;
    let nb_normal = nb.normal;
    let na_td_a = na.torque_dir_a;
    let na_td_b = na.torque_dir_b;
    let na_ii_a = na.ii_torque_dir_a;
    let na_ii_b = na.ii_torque_dir_b;
    let nb_ii_a = nb.ii_torque_dir_a;
    let nb_ii_b = nb.ii_torque_dir_b;
    let na_rhs = na.rhs;
    let nb_rhs = nb.rhs;
    let na_pm = na.projected_mass;
    let nb_pm = nb.projected_mass;
    let na_cf = na.cfm_factor;
    let k00 = na.r_mat_diag;
    let k11 = nb.r_mat_diag;
    let k01 = na.r_mat_offdiag;
    let k10 = k01;

    if na_normal.x.is_nan()
        || na_normal.y.is_nan()
        || nb_normal.x.is_nan()
        || nb_normal.y.is_nan()
        || na_pm.is_nan()
        || nb_pm.is_nan()
        || na_rhs.is_nan()
        || nb_rhs.is_nan()
    {
        return;
    }

    let va_lin = solver_bodies.vel(ba).linear;
    let va_ang = solver_bodies.vel(ba).angular;
    let vb_lin = solver_bodies.vel(bb).linear;
    let vb_ang = solver_bodies.vel(bb).angular;

    if va_lin.x.is_nan()
        || va_lin.y.is_nan()
        || va_ang.is_nan()
        || vb_lin.x.is_nan()
        || vb_lin.y.is_nan()
        || vb_ang.is_nan()
    {
        return;
    }

    let dvel_a = na_normal.dot(vb_lin - va_lin) - na_td_a * va_ang - na_td_b * vb_ang + na_rhs;
    let nb_td_a = nb.torque_dir_a;
    let nb_td_b = nb.torque_dir_b;
    let dvel_b = nb_normal.dot(vb_lin - va_lin) - nb_td_a * va_ang - nb_td_b * vb_ang + nb_rhs;

    let dvel_a = dvel_a.clamp(-100000.0, 100000.0);
    let dvel_b = dvel_b.clamp(-100000.0, 100000.0);

    let det = k00 * k11 - k01 * k10;
    let x: (f32, f32);

    // Check if the K matrix is positive definite (det > 0 and diagonal > 0).
    // A negative or near-zero determinant means the system is ill-conditioned
    // (e.g., two contact points with opposite torques and high inv_inertia).
    // Fall back to independent solving in this case.
    if det > 1e-6 && k00 > 1e-6 && k11 > 1e-6 {
        let inv_det = 1.0 / det;
        let cx = na.impulse - (k11 * dvel_a - k01 * dvel_b) * inv_det;
        let cy = nb.impulse - (-k10 * dvel_a + k00 * dvel_b) * inv_det;
        if cx >= 0.0 && cy >= 0.0 {
            x = (cx, cy);
        } else if cx >= 0.0 {
            x = (cx, 0.0);
        } else if cy >= 0.0 {
            x = (0.0, cy);
        } else {
            x = (0.0, 0.0);
        }
    } else {
        // Fallback: solve independently
        x = (
            (na.impulse - na_pm * dvel_a).max(0.0),
            (nb.impulse - nb_pm * dvel_b).max(0.0),
        );
    }

    let x0 = na_cf * x.0;
    let x1 = na_cf * x.1;

    let d0 = x0 - na.impulse;
    let d1 = x1 - nb.impulse;
    na.impulse = x0;
    nb.impulse = x1;

    let im_a = solver_bodies.pose(ba).inv_mass;
    let im_b = solver_bodies.pose(bb).inv_mass;

    let va = solver_bodies.vel_mut(ba);
    va.linear -= (na_normal * d0 + nb_normal * d1) * im_a;
    va.angular -= na_ii_a * d0 + nb_ii_a * d1;

    let vb = solver_bodies.vel_mut(bb);
    vb.linear += (na_normal * d0 + nb_normal * d1) * im_b;
    vb.angular -= na_ii_b * d0 + nb_ii_b * d1;
}

fn solve_normal_pair_wo_bias(
    solver_bodies: &mut SolverBodySet,
    na: &mut ContactConstraintNormal,
    nb: &mut ContactConstraintNormal,
) {
    let ba = na.body_a;
    let bb = na.body_b;
    let na_normal = na.normal;
    let nb_normal = nb.normal;
    let na_td_a = na.torque_dir_a;
    let na_td_b = na.torque_dir_b;
    let na_ii_a = na.ii_torque_dir_a;
    let na_ii_b = na.ii_torque_dir_b;
    let nb_ii_a = nb.ii_torque_dir_a;
    let nb_ii_b = nb.ii_torque_dir_b;
    let na_rhs = na.rhs_wo_bias;
    let nb_rhs = nb.rhs_wo_bias;
    let na_pm = na.projected_mass;
    let nb_pm = nb.projected_mass;

    if na_normal.x.is_nan()
        || na_normal.y.is_nan()
        || nb_normal.x.is_nan()
        || nb_normal.y.is_nan()
        || na_pm.is_nan()
        || nb_pm.is_nan()
        || na_rhs.is_nan()
        || nb_rhs.is_nan()
    {
        return;
    }

    let va_lin = solver_bodies.vel(ba).linear;
    let va_ang = solver_bodies.vel(ba).angular;
    let vb_lin = solver_bodies.vel(bb).linear;
    let vb_ang = solver_bodies.vel(bb).angular;

    if va_lin.x.is_nan()
        || va_lin.y.is_nan()
        || va_ang.is_nan()
        || vb_lin.x.is_nan()
        || vb_lin.y.is_nan()
        || vb_ang.is_nan()
    {
        return;
    }

    let dvel_a = na_normal.dot(vb_lin - va_lin) - na_td_a * va_ang - na_td_b * vb_ang + na_rhs;
    let nb_td_a = nb.torque_dir_a;
    let nb_td_b = nb.torque_dir_b;
    let dvel_b = nb_normal.dot(vb_lin - va_lin) - nb_td_a * va_ang - nb_td_b * vb_ang + nb_rhs;

    let dvel_a = dvel_a.clamp(-100000.0, 100000.0);
    let dvel_b = dvel_b.clamp(-100000.0, 100000.0);

    let new_a = (na.impulse - na_pm * dvel_a).max(0.0);
    let new_b = (nb.impulse - nb_pm * dvel_b).max(0.0);

    let da = new_a - na.impulse;
    let db = new_b - nb.impulse;
    na.impulse = new_a;
    nb.impulse = new_b;

    let im_a = solver_bodies.pose(ba).inv_mass;
    let im_b = solver_bodies.pose(bb).inv_mass;

    let va = solver_bodies.vel_mut(ba);
    va.linear -= (na_normal * da + nb_normal * db) * im_a;
    va.angular -= na_ii_a * da + nb_ii_a * db;

    let vb = solver_bodies.vel_mut(bb);
    vb.linear += (na_normal * da + nb_normal * db) * im_b;
    vb.angular -= na_ii_b * da + nb_ii_b * db;
}

fn solve_tangent(
    solver_bodies: &mut SolverBodySet,
    t: &mut ContactConstraintTangent,
    friction_limit: f32,
) {
    let ba = t.body_a;
    let bb = t.body_b;
    let tangent = t.tangent;
    let td_a = t.torque_dir_a;
    let td_b = t.torque_dir_b;
    let ii_a = t.ii_torque_dir_a;
    let ii_b = t.ii_torque_dir_b;
    let rhs = t.rhs;
    let pm = t.projected_mass;

    if tangent.x.is_nan() || tangent.y.is_nan() || pm.is_nan() || rhs.is_nan() {
        return;
    }

    let va_lin = solver_bodies.vel(ba).linear;
    let va_ang = solver_bodies.vel(ba).angular;
    let vb_lin = solver_bodies.vel(bb).linear;
    let vb_ang = solver_bodies.vel(bb).angular;

    if va_lin.x.is_nan()
        || va_lin.y.is_nan()
        || va_ang.is_nan()
        || vb_lin.x.is_nan()
        || vb_lin.y.is_nan()
        || vb_ang.is_nan()
    {
        return;
    }

    let dvel = tangent.dot(vb_lin - va_lin) - td_a * va_ang + td_b * vb_ang + rhs;
    let dvel = dvel.clamp(-100000.0, 100000.0);

    let limit = friction_limit.max(0.0);
    let new_impulse = (t.impulse - pm * dvel).clamp(-limit, limit);
    let dlambda = new_impulse - t.impulse;
    t.impulse = new_impulse;

    let im_a = solver_bodies.pose(ba).inv_mass;
    let im_b = solver_bodies.pose(bb).inv_mass;

    let va = solver_bodies.vel_mut(ba);
    va.linear -= tangent * (dlambda * im_a);
    va.angular -= ii_a * dlambda;

    let vb = solver_bodies.vel_mut(bb);
    vb.linear += tangent * (dlambda * im_b);
    vb.angular += ii_b * dlambda;
}
