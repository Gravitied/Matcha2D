//! Native `PhysicsEngine` invariants (no Vitest / no JS). Failing assertions print concrete metrics.

use matcha2d_physics::PhysicsEngine;

fn assert_finite2(label: &str, x: f32, y: f32) {
    assert!(
        x.is_finite() && y.is_finite(),
        "{label}: non-finite ({x}, {y})"
    );
}

#[test]
fn step_preserves_finite_positions_and_velocities() {
    let mut e = PhysicsEngine::new();
    e.set_gravity(0.0, -9.81);
    e.set_dt(1.0 / 60.0);
    let b = e.create_dynamic_body(0.0, 50.0);
    e.create_ball_collider(0.25, b);

    for step in 0..120 {
        e.step();
        let p = e.get_body_position(b);
        let v = e.get_body_velocity(b);
        assert_finite2(&format!("step {step} pos"), p[0], p[1]);
        assert_finite2(&format!("step {step} vel"), v[0], v[1]);
        let pen = e.max_penetration();
        assert!(pen.is_finite(), "step {step}: max_penetration non-finite");
        assert!(
            pen >= 0.0,
            "step {step}: max_penetration negative ({pen}) — depth sign convention broken?"
        );
    }
}

#[test]
fn gravity_first_substep_matches_g_times_dt_with_zero_damping() {
    let mut e = PhysicsEngine::new();
    e.set_gravity(0.0, -10.0);
    e.set_dt(1.0 / 60.0);
    let b = e.create_dynamic_body(0.0, 100.0);
    e.create_ball_collider(0.25, b);
    e.set_body_linear_damping(b, 0.0);
    e.set_body_angular_damping(b, 0.0);

    e.step();
    let v = e.get_body_velocity(b);
    let g = -10.0_f32;
    let dt = 1.0_f32 / 60.0;
    let expected_vy = g * dt;
    assert!(
        v[0].abs() < 1e-6,
        "expected vx≈0 after gravity-only step, got {}",
        v[0]
    );
    assert!(
        (v[1] - expected_vy).abs() < 1e-5,
        "expected vy≈g*dt={expected_vy}, got {}",
        v[1]
    );
}
