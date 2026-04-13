//! Time-of-impact integration poses for tunnel recovery (used only by [`super::PhysicsWorld`]).
use crate::math::{Rot, Vec2};

/// Full kinematic state at the start of `step()`, used so TOI motion matches pose/velocity together
/// (avoids stale `v` after `set_body_position` teleports) and matches pre-solve forces/torques.
#[derive(Clone, Copy)]
pub(crate) struct ToiStepSnapshot {
    pub pos: Vec2,
    pub angle: f32,
    pub linvel: Vec2,
    pub angvel: f32,
}

/// Pose along the frame interval for TOI search.
///
/// `τ = u·dt`, `u ∈ [0,1]`.
#[derive(Clone, Copy)]
pub(crate) enum ToiTrajectory {
    Fixed { pos: Vec2, ang: f32 },
    Integrated {
        p0: Vec2,
        v0: Vec2,
        a_lin: Vec2,
        a0: f32,
        omega0: f32,
        alpha: f32,
    },
}

impl ToiTrajectory {
    pub(crate) fn pose(self, dt: f32, u: f32) -> (Vec2, Rot) {
        let tau = u * dt;
        match self {
            ToiTrajectory::Fixed { pos, ang } => (pos, Rot::from_angle(ang)),
            ToiTrajectory::Integrated {
                p0,
                v0,
                a_lin,
                a0,
                omega0,
                alpha,
            } => {
                let pos = p0 + v0 * tau + a_lin * (0.5 * tau * tau);
                let ang = a0 + omega0 * tau + alpha * (0.5 * tau * tau);
                (pos, Rot::from_angle(ang))
            }
        }
    }

    pub(crate) fn travel_extent(self, dt: f32, shape_radius: f32) -> f32 {
        match self {
            ToiTrajectory::Fixed { .. } => 0.0,
            ToiTrajectory::Integrated {
                p0,
                v0,
                a_lin,
                omega0,
                alpha,
                ..
            } => {
                let p1 = p0 + v0 * dt + a_lin * (0.5 * dt * dt);
                let linear = (p1 - p0).length();
                let delta_theta = (omega0 * dt + 0.5 * alpha * dt * dt).abs();
                let omega1 = omega0 + alpha * dt;
                let rotational = delta_theta
                    .max(omega0.abs().max(omega1.abs()) * dt)
                    * shape_radius;
                linear + rotational
            }
        }
    }

    pub(crate) fn is_moving(self, dt: f32, shape_radius: f32) -> bool {
        self.travel_extent(dt, shape_radius) > 1e-6
    }
}
