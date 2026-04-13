//! Floor + ball: geometric invariant using `max_penetration()` (same contract as TS collision test).
//!
//! `GEOM_SLACK` is **not** a physics tolerance for tunneling — it accounts for `f32` integration /
//! solver reporting so the test does not false-positive on sub‑mm gaps when `max_penetration` is tiny.

use matcha2d_physics::PhysicsEngine;

const GEOM_SLACK: f32 = 2.0e-4;

#[test]
fn ball_on_floor_respects_surface_minus_reported_penetration() {
    let r = 0.25_f32;
    let floor_half_y = 0.5_f32;
    let floor_top = floor_half_y;

    let mut e = PhysicsEngine::new();
    e.set_gravity(0.0, -12.0);
    e.set_dt(1.0 / 60.0);
    e.set_swept_broadphase(true);

    let floor = e.create_static_body(0.0, 0.0);
    let _floor_col = e.create_box_collider(8.0, floor_half_y, floor);

    let ball = e.create_dynamic_body(0.0, 3.0);
    let ball_col = e.create_ball_collider(r, ball);
    e.set_collider_friction(ball_col, 0.3);
    e.set_collider_restitution(ball_col, 0.0);

    for step in 0..900 {
        e.step();
        let pen = e.max_penetration();
        assert!(pen.is_finite(), "step {step}: max_penetration not finite");
        let p = e.get_body_position(ball);
        assert!(p[0].is_finite() && p[1].is_finite(), "step {step}: position not finite");
        let ball_bottom = p[1] - r;
        let allowed = floor_top - pen - GEOM_SLACK;
        assert!(
            ball_bottom >= allowed,
            "step {step}: ball_bottom={ball_bottom:.6} < allowed={allowed:.6} (floor_top={floor_top} max_pen={pen:.6})\n\
             HINT: compare narrowphase contacts vs TOI/swept broadphase if ball_bottom is **far** below (>> GEOM_SLACK) with small pen."
        );
    }
}
