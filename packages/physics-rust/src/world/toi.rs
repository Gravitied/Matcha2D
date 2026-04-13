//! Time-of-impact recovery and pose sampling for [`super::PhysicsWorld`].
use super::toi_types::{ToiStepSnapshot, ToiTrajectory};
use super::PhysicsWorld;
use crate::body::RigidBodyHandle;
use crate::collider::{Collider, ColliderHandle, Shape};
use crate::contact::ContactManifold;
use crate::math::{Rot, Vec2};
use crate::narrowphase;
use crate::shape_cast;

impl PhysicsWorld {
    pub(super) fn snapshot_body_poses_at_step_start(&mut self) {
        let indices = self.bodies.live_indices();
        let Some(&max_idx) = indices.iter().max() else {
            self.body_toi_snapshot_at_step_start.clear();
            return;
        };
        let n = max_idx + 1;
        if self.body_toi_snapshot_at_step_start.len() < n {
            self.body_toi_snapshot_at_step_start.resize(n, None);
        }
        for slot in self.body_toi_snapshot_at_step_start.iter_mut() {
            *slot = None;
        }
        for &idx in &indices {
            let b = self.bodies.get_unchecked(idx);
            if (b.is_dynamic() || b.is_kinematic()) && !b.sleeping {
                self.body_toi_snapshot_at_step_start[idx] = Some(ToiStepSnapshot {
                    pos: b.pos,
                    angle: b.rot.angle(),
                    linvel: b.linvel,
                    angvel: b.angvel,
                });
            }
        }
    }

    fn detect_collision_at_two_body_poses(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ha: RigidBodyHandle,
        pos_a: Vec2,
        rot_a: Rot,
        hb: RigidBodyHandle,
        pos_b: Vec2,
        rot_b: Rot,
    ) -> Option<ContactManifold> {
        let (bak_a_p, bak_a_r) = {
            let b = self.bodies.get(ha)?;
            (b.pos, b.rot)
        };
        let (bak_b_p, bak_b_r) = {
            let b = self.bodies.get(hb)?;
            (b.pos, b.rot)
        };
        {
            let a = self.bodies.get_mut(ha)?;
            a.pos = pos_a;
            a.rot = rot_a;
        }
        {
            let b = self.bodies.get_mut(hb)?;
            b.pos = pos_b;
            b.rot = rot_b;
        }
        let hint = self.contact_tracker.poly_sat_hint(ca, cb);
        let m = narrowphase::detect_collision_with_hint(
            &self.bodies,
            &self.colliders,
            ca,
            cb,
            hint,
        );
        {
            let a = self.bodies.get_mut(ha)?;
            a.pos = bak_a_p;
            a.rot = bak_a_r;
        }
        {
            let b = self.bodies.get_mut(hb)?;
            b.pos = bak_b_p;
            b.rot = bak_b_r;
        }
        m
    }

    /// Motion model for TOI: static = fixed; dynamic = constant linear accel (`g` + `F/m`) and angular accel `τ/I`;
    /// kinematic = translation from `v₀` only (matches solver), rotation `θ₀ + ω₀τ`.
    fn body_toi_trajectory(&self, h: RigidBodyHandle) -> Option<ToiTrajectory> {
        let b = self.bodies.get(h)?;
        if b.is_static() {
            return Some(ToiTrajectory::Fixed {
                pos: b.pos,
                ang: b.rot.angle(),
            });
        }
        if !b.is_dynamic() && !b.is_kinematic() {
            return None;
        }
        let idx = h.index();
        let snap = self
            .body_toi_snapshot_at_step_start
            .get(idx)
            .copied()
            .flatten();
        let (p0, a0, v0, omega0) = if let Some(s) = snap {
            (s.pos, s.angle, s.linvel, s.angvel)
        } else {
            (b.pos, b.rot.angle(), b.linvel, b.angvel)
        };

        if b.is_kinematic() {
            return Some(ToiTrajectory::Integrated {
                p0,
                v0,
                a_lin: Vec2::zero(),
                a0,
                omega0,
                alpha: 0.0,
            });
        }

        let grav = self.gravity * b.gravity_scale;
        let a_lin = grav + b.force * b.inv_mass;
        let alpha = b.torque * b.inv_inertia;

        Some(ToiTrajectory::Integrated {
            p0,
            v0,
            a_lin,
            a0,
            omega0,
            alpha,
        })
    }

    fn max_shape_radius_on_body(&self, body_h: RigidBodyHandle) -> f32 {
        let mut r = 0.0f32;
        for idx in self.colliders.live_indices() {
            let c = self.colliders.get_unchecked(idx);
            if c.parent == Some(body_h) {
                r = r.max(c.shape.max_support_radius_from_shape_origin() + c.local_pos.length());
            }
        }
        r.max(0.01)
    }

    fn collider_handles_for_body(&self, body_h: RigidBodyHandle) -> Vec<ColliderHandle> {
        self.colliders
            .live_indices()
            .into_iter()
            .filter_map(|idx| {
                let c = self.colliders.get_unchecked(idx);
                if c.parent == Some(body_h) {
                    Some(self.colliders.handle_for_index(idx))
                } else {
                    None
                }
            })
            .collect()
    }

    fn tunnel_sample_pair_at_u(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ha: RigidBodyHandle,
        hb: RigidBodyHandle,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
        u: f32,
    ) -> bool {
        let (pa, ra) = ta.pose(self.dt, u);
        let (pb, rb) = tb.pose(self.dt, u);
        self.detect_collision_at_two_body_poses(ca, cb, ha, pa, ra, hb, pb, rb)
            .is_some()
    }

    /// World-space ball centres at normalized time `u ∈ [0,1]`, signed gap (negative = overlap).
    fn ball_ball_signed_separation_at_u(
        &self,
        col_a: &Collider,
        col_b: &Collider,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
        u: f32,
    ) -> Option<f32> {
        let (Shape::Ball { radius: ra }, Shape::Ball { radius: rb }) =
            (&col_a.shape, &col_b.shape)
        else {
            return None;
        };
        let (pa, rot_a) = ta.pose(self.dt, u);
        let (pb, rot_b) = tb.pose(self.dt, u);
        let cwa = pa + rot_a.mul_vec(col_a.local_pos);
        let cwb = pb + rot_b.mul_vec(col_b.local_pos);
        Some(cwa.distance(cwb) - (*ra + *rb))
    }

    /// First `u` where two moving balls touch: scan separation along the integrated trajectories,
    /// then bisect on signed gap (tighter than boolean overlap sampling alone).
    fn ball_ball_first_toi_u(
        &self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
    ) -> Option<f32> {
        let col_a = self.colliders.get(ca)?;
        let col_b = self.colliders.get(cb)?;
        const SCAN: usize = 32;
        let sep0 = self.ball_ball_signed_separation_at_u(col_a, col_b, ta, tb, 0.0)?;
        if sep0 <= 1e-5 {
            return None;
        }
        let mut prev_u = 0.0f32;
        for i in 1..=SCAN {
            let u = i as f32 / SCAN as f32;
            let sep = self.ball_ball_signed_separation_at_u(col_a, col_b, ta, tb, u)?;
            if sep <= 0.0 {
                let mut lo = prev_u;
                let mut hi = u;
                for _ in 0..10 {
                    let mid = (lo + hi) * 0.5;
                    let sm = self.ball_ball_signed_separation_at_u(col_a, col_b, ta, tb, mid)?;
                    if sm <= 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return Some(hi);
            }
            prev_u = u;
        }
        None
    }

    /// Signed separation along trajectories at normalized time `u ∈ [0,1]`.
    ///
    /// **Negative:** penetrating — uses narrowphase manifold depth (`-max_penetration()`).
    /// **Positive:** separated — minimum distance from [`shape_cast::gjk_distance`].
    /// If GJK reports overlap but the manifold query missed (rare), returns a small negative slack.
    fn gjk_signed_separation_at_u(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ha: RigidBodyHandle,
        hb: RigidBodyHandle,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
        u: f32,
    ) -> f32 {
        let (pa, ra) = ta.pose(self.dt, u);
        let (pb, rb) = tb.pose(self.dt, u);
        if let Some(m) = self.detect_collision_at_two_body_poses(ca, cb, ha, pa, ra, hb, pb, rb) {
            return -m.max_penetration();
        }
        let Some(col_a) = self.colliders.get(ca) else {
            return 0.0;
        };
        let Some(col_b) = self.colliders.get(cb) else {
            return 0.0;
        };
        let (oa, rsa) = shape_cast::collider_world_frame(pa, ra, col_a);
        let (ob, rsb) = shape_cast::collider_world_frame(pb, rb, col_b);
        match shape_cast::gjk_distance(
            &col_a.shape,
            oa,
            rsa,
            &col_b.shape,
            ob,
            rsb,
        ) {
            Some(d) => d,
            None => -1e-4,
        }
    }

    /// First `u` where signed separation crosses non-positive: uniform scan in `u`, then bisection.
    /// Uses [`Self::gjk_signed_separation_at_u`] (pose from `ToiTrajectory` + GJK / manifold), not swept CA.
    fn convex_gjk_first_toi_u(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ha: RigidBodyHandle,
        hb: RigidBodyHandle,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
    ) -> Option<f32> {
        const SCAN: usize = 32;
        let sep0 = self.gjk_signed_separation_at_u(ca, cb, ha, hb, ta, tb, 0.0);
        if sep0 <= 1e-5 {
            return None;
        }
        let mut prev_u = 0.0f32;
        for i in 1..=SCAN {
            let u = i as f32 / SCAN as f32;
            let sep = self.gjk_signed_separation_at_u(ca, cb, ha, hb, ta, tb, u);
            if sep <= 0.0 {
                let mut lo = prev_u;
                let mut hi = u;
                for _ in 0..10 {
                    let mid = (lo + hi) * 0.5;
                    let sm = self.gjk_signed_separation_at_u(ca, cb, ha, hb, ta, tb, mid);
                    if sm <= 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                return Some(hi);
            }
            prev_u = u;
        }
        None
    }

    fn generic_toi_contact_u(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
        ha: RigidBodyHandle,
        hb: RigidBodyHandle,
        ta: ToiTrajectory,
        tb: ToiTrajectory,
        travel: f32,
    ) -> Option<f32> {
        let steps = ((travel / 0.04).ceil() as usize).clamp(16, 32);
        let mut i_hit: Option<usize> = None;
        for i in 1..=steps {
            let u = i as f32 / steps as f32;
            if self.tunnel_sample_pair_at_u(ca, cb, ha, hb, ta, tb, u) {
                i_hit = Some(i);
                break;
            }
        }
        let i_hit = i_hit?;
        let mut hi = i_hit as f32 / steps as f32;
        let mut lo_u = (i_hit.saturating_sub(1)) as f32 / steps as f32;
        if self.tunnel_sample_pair_at_u(ca, cb, ha, hb, ta, tb, lo_u) {
            return None;
        }
        for _ in 0..8 {
            let mid = (lo_u + hi) * 0.5;
            if self.tunnel_sample_pair_at_u(ca, cb, ha, hb, ta, tb, mid) {
                hi = mid;
            } else {
                lo_u = mid;
            }
        }
        Some(hi)
    }

    fn strip_dynamic_inward_normal(&mut self, h: RigidBodyHandle, n: Vec2) {
        if let Some(b) = self.bodies.get_mut(h) {
            if !b.is_dynamic() {
                return;
            }
            let vn = b.linvel.dot(n);
            if vn < -1e-3 {
                b.linvel -= n * vn;
            }
        }
    }

    /// TOI search: ball–ball analytic separation; other convex pairs GJK separation + bisection;
    /// fallback pose sampling for robustness. Mutates body poses at the contact time-of-impact.
    fn try_recover_two_body_toi(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
    ) -> Option<ContactManifold> {
        let (ha, hb, ball_ball) = {
            let col_a = self.colliders.get(ca)?;
            let col_b = self.colliders.get(cb)?;
            let ha = col_a.parent?;
            let hb = col_b.parent?;
            let ball_ball = matches!(
                (&col_a.shape, &col_b.shape),
                (Shape::Ball { .. }, Shape::Ball { .. })
            );
            (ha, hb, ball_ball)
        };
        if ha == hb {
            return None;
        }

        let (pre_pos_a, pre_angle_a, pre_pos_b, pre_angle_b) = {
            let ba = self.bodies.get(ha)?;
            let bb = self.bodies.get(hb)?;
            (ba.pos, ba.rot.angle(), bb.pos, bb.rot.angle())
        };

        let r_a = self.max_shape_radius_on_body(ha);
        let r_b = self.max_shape_radius_on_body(hb);

        // Cheap reject (ball–ball only): COM separation + low spin is safe for spheres; for boxes/polygons
        // corners can close while COMs separate — skipping TOI there causes missed recovery and velocity spikes.
        if ball_ball {
            if let (Some(ba), Some(bb)) = (self.bodies.get(ha), self.bodies.get(hb)) {
                let rel = bb.pos - ba.pos;
                let rel_v = bb.linvel - ba.linvel;
                let rel_sq = rel.length_sq();
                if rel_sq > 1e-12 && rel_v.dot(rel) > 0.0 {
                    let w_max = ba.angvel.abs().max(bb.angvel.abs());
                    let spin_travel = w_max * r_a.max(r_b) * self.dt;
                    if spin_travel < 0.02 {
                        return None;
                    }
                }
            }
        }

        let ta = self.body_toi_trajectory(ha)?;
        let tb = self.body_toi_trajectory(hb)?;
        if !ta.is_moving(self.dt, r_a) && !tb.is_moving(self.dt, r_b) {
            return None;
        }

        let travel = ta
            .travel_extent(self.dt, r_a)
            .max(tb.travel_extent(self.dt, r_b));

        let u_refined = if ball_ball {
            self.ball_ball_first_toi_u(ca, cb, ta, tb)
        } else {
            self.convex_gjk_first_toi_u(ca, cb, ha, hb, ta, tb)
        };
        let mut u_contact = u_refined.unwrap_or_else(|| {
            self.generic_toi_contact_u(ca, cb, ha, hb, ta, tb, travel)
                .unwrap_or(f32::NAN)
        });
        if !u_contact.is_finite() {
            return None;
        }
        let (mut pos_a, mut rot_a) = ta.pose(self.dt, u_contact);
        let (mut pos_b, mut rot_b) = tb.pose(self.dt, u_contact);
        let mut m =
            self.detect_collision_at_two_body_poses(ca, cb, ha, pos_a, rot_a, hb, pos_b, rot_b);
        // GJK root can sit in a ≤0 separation wedge without a SAT manifold (grazing / numeric mismatch).
        // Overlap-based TOI uses the same trajectories and is a cheap second pass before giving up.
        if m.is_none() {
            if let Some(u2) = self.generic_toi_contact_u(ca, cb, ha, hb, ta, tb, travel) {
                u_contact = u2;
                let (pa, ra) = ta.pose(self.dt, u_contact);
                let (pb, rb) = tb.pose(self.dt, u_contact);
                pos_a = pa;
                rot_a = ra;
                pos_b = pb;
                rot_b = rb;
                m = self.detect_collision_at_two_body_poses(ca, cb, ha, pos_a, rot_a, hb, pos_b, rot_b);
            }
        }
        let m = m?;

        if let Some(ba) = self.bodies.get_mut(ha) {
            ba.pos = pos_a;
            ba.rot = rot_a;
            ba.wake();
        }
        if let Some(bb) = self.bodies.get_mut(hb) {
            bb.pos = pos_b;
            bb.rot = rot_b;
            bb.wake();
        }

        let n = m.normal;
        let a_dyn = self.bodies.get(ha)?.is_dynamic();
        let b_dyn = self.bodies.get(hb)?.is_dynamic();
        if a_dyn && !b_dyn {
            self.strip_dynamic_inward_normal(ha, n);
        } else if b_dyn && !a_dyn {
            self.strip_dynamic_inward_normal(hb, n);
        }

        for ch in self.collider_handles_for_body(ha) {
            self.update_collider_bvh(ch);
        }
        for ch in self.collider_handles_for_body(hb) {
            self.update_collider_bvh(ch);
        }

        if self.debug_collision_logging {
            let max_pen = m.max_penetration();
            self.last_toi_debug_flat.extend_from_slice(&[
                ca.0 as f32,
                cb.0 as f32,
                ha.0 as f32,
                hb.0 as f32,
                u_contact,
                pre_pos_a.x,
                pre_pos_a.y,
                pre_pos_b.x,
                pre_pos_b.y,
                pos_a.x,
                pos_a.y,
                pos_b.x,
                pos_b.y,
                max_pen,
                pre_angle_a,
                pre_angle_b,
                rot_a.angle(),
                rot_b.angle(),
            ]);
        }

        Some(m)
    }

    pub(super) fn try_recover_tunnelled_manifold(
        &mut self,
        ca: ColliderHandle,
        cb: ColliderHandle,
    ) -> Option<ContactManifold> {
        self.try_recover_two_body_toi(ca, cb)
    }
}
