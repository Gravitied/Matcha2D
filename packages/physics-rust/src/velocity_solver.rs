use crate::body::RigidBodySet;
use crate::contact::ContactTracker;
use crate::contact_constraint::ContactConstraintSet;
use crate::solver_body::SolverBodySet;

/// Full Blueprint-style solver loop:
/// 1. Apply force increments to solver velocities
/// 2. Update constraints (recompute RHS from current positions)
/// 3. Warmstart (apply retained impulses)
/// 4. PGS solve with bias (position correction + restitution)
/// 5. Integrate positions
/// 6. Stabilization pass (solve without bias)
/// 7. Writeback to rigid bodies
pub fn solve(
    solver_bodies: &mut SolverBodySet,
    constraints: &mut ContactConstraintSet,
    tracker: &mut ContactTracker,
    num_iterations: usize,
    num_internal_iterations: usize,
    num_stabilization_iterations: usize,
    warmstart_coeff: f32,
    dt: f32,
    block_solver: bool,
    bodies: &mut RigidBodySet,
) {
    // Step 1: Apply force increments (gravity + accumulated forces) once
    solver_bodies.apply_force_increments();

    // Step 1.5: Apply damping
    solver_bodies.apply_damping(dt);

    // Step 2: Warmstart — apply retained impulses from previous frame once
    if warmstart_coeff > 0.0 {
        constraints.warmstart(solver_bodies, warmstart_coeff);
    }

    // Step 3: Main PGS iterations with bias (position correction + restitution)
    // Baumgarte bias is applied only in the FIRST PGS iteration to avoid
    // compounding position correction across iterations. After the first
    // iteration, solve without bias so impulses accumulate purely from
    // velocity resolution, not repeated position correction.
    constraints.update(solver_bodies, dt);
    for iter in 0..num_iterations {
        for inner in 0..num_internal_iterations {
            if iter == 0 && inner == 0 {
                // First iteration: solve with Baumgarte bias for position correction
                constraints.solve(solver_bodies, block_solver);
            } else {
                // Subsequent iterations: solve without bias, only resolve velocity
                constraints.solve_wo_bias_vel(solver_bodies, block_solver);
            }
        }
    }

    // Step 4: Integrate positions once using corrected solver velocities
    solver_bodies.integrate_positions(dt);

    // Step 5: Stabilization pass — direct position correction to resolve remaining penetration
    for _ in 0..num_stabilization_iterations {
        constraints.solve_wo_bias(solver_bodies, block_solver);
    }

    // Step 6: Write solver state back to rigid bodies
    solver_bodies.writeback_to_bodies(bodies);

    // Step 7: Write accumulated impulses back to contact tracker for next-frame warmstart
    constraints.writeback_impulses(tracker);
}
