pub use crate::contact_constraint::{ContactConstraintSet, ContactSpringCoefficients};
pub use crate::solver_body::SolverBodySet;
pub use crate::velocity_solver::solve;

/// Solver configuration matching Blueprint-style architecture
pub struct SolverConfig {
    /// Number of outer solver iterations (substeps)
    pub num_iterations: usize,
    /// PGS iterations per substep
    pub num_internal_iterations: usize,
    /// Stabilization iterations after position integration
    pub num_stabilization_iterations: usize,
    /// Warmstart coefficient (0.0 = disabled, 1.0 = full)
    pub warmstart_coefficient: f32,
    /// Use block solver for 2-contact manifolds
    pub block_solver: bool,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            num_iterations: 10,
            num_internal_iterations: 4,
            num_stabilization_iterations: 4,
            warmstart_coefficient: 1.0,
            block_solver: true,
        }
    }
}
