# Fix Collision Overlap: Matcha2D vs Template-Repo Reference

## Problem

Matcha2D exhibits constant shape overlap across all shape type combinations (box-box, box-circle, circle-polygon, etc.), while the Template-Repo (Sopiro/Rapier-style) implementation has perfect collision with zero overlap. The root causes are in the **contact solver**, not the narrowphase detection.

## Root Cause Analysis

### Primary: Missing Block Solver (CRITICAL)

**Template-Repo** (`Template-Repo/src/contact.ts:166-365`): Has a full `BlockSolver` class that solves 2-contact manifolds as a **coupled 2x2 linear system**. When a manifold has 2 contacts, it builds a 2x2 effective mass matrix `K` and solves for both impulses simultaneously, accounting for cross-coupling between contacts.

**Matcha2D** (`packages/physics-rust/src/world.rs:566-643`): `solve_velocity_batch` solves each contact **independently** — no block solver. Each contact's impulse is computed as if it were the only contact on the manifold.

This is the single largest difference and the primary cause of overlap. Without the block solver:
- Two contacts on the same face fight each other (each tries to correct independently)
- Resting contacts (box on floor, stacked objects) are inherently 2-contact manifolds
- Results in jitter, bounce, and visible overlap/passthrough

### Secondary: Restitution Bias Formula

**Template-Repo** (`Template-Repo/src/contact.ts:86`):
```typescript
this.bias += this.restitution * Math.min(normalVelocity + Settings.restitutionSlop, 0.0);
```
Continuous: applies restitution proportionally to approach velocity whenever `vn < -restitutionSlop`.

**Matcha2D** (`packages/physics-rust/src/solver.rs:117-121`):
```rust
let bias = if vel_along_normal < -config.restitution_slop {
    -restitution * vel_along_normal
} else {
    0.0
};
```
Step function: applies full restitution only when `vn < -restitutionSlop`, otherwise zero. This creates a discontinuity at the threshold that can cause inconsistent bounce behavior.

### Tertiary: Solver Iterations

- Template-Repo: 10 velocity iterations
- Matcha2D: 8 velocity iterations
- Minor impact but fewer iterations = less convergence

## Fixes

### Fix 1: Implement Block Solver for 2-Contact Manifolds

**File**: `packages/physics-rust/src/solver.rs`

Add a `solve_velocity_block` function that handles 2-contact manifolds as a coupled system, modeled after Template-Repo's `BlockSolver`:

```
For a 2-contact manifold:
1. Build 2x2 effective mass matrix K:
   K[0][0] = invMass_a + invMass_b + invInertia_a * (ra1 × n)² + invInertia_b * (rb1 × n)²
   K[1][1] = invMass_a + invMass_b + invInertia_a * (ra2 × n)² + invInertia_b * (rb2 × n)²
   K[0][1] = K[1][0] = invMass_a + invInertia_a * (ra1 × n)(ra2 × n)
                     + invMass_b + invInertia_b * (rb1 × n)(rb2 × n)

2. Compute velocity constraints:
   vn1 = J1 · v  (normal velocity at contact 1)
   vn2 = J2 · v  (normal velocity at contact 2)
   b = (vn1 + bias1, vn2 + bias2)

3. Adjust for accumulated impulse:
   b' = b - K * a  (where a = old accumulated impulse vector)

4. Solve via total enumeration (4 cases from Box2D):
   Case 1: Both contacts active → x = -K⁻¹ * b'
   Case 2: Contact 1 active, x2 = 0 → x1 = -b'.x / K[0][0]
   Case 3: x1 = 0, Contact 2 active → x2 = -b'.y / K[1][1]
   Case 4: Both x = 0 → check if b' >= 0

5. Clamp accumulated impulses to x >= 0
6. Compute incremental impulse d = x - a
7. Apply incremental impulse to both bodies' velocities
```

The existing `solve_contact_constraint` (per-contact solver) stays for 1-contact manifolds.

**File**: `packages/physics-rust/src/world.rs`

Modify `solve_velocity_batch` to:
1. First solve friction (tangent) per-contact (same as Template-Repo order)
2. Then for 2-contact manifolds, use block solver for normal impulses
3. For 1-contact manifolds, use existing per-contact solver

### Fix 2: Correct Restitution Bias Formula

**File**: `packages/physics-rust/src/solver.rs:117-121`

Change from step function to continuous formula:

```rust
// Before:
let bias = if vel_along_normal < -config.restitution_slop {
    -restitution * vel_along_normal
} else {
    0.0
};

// After:
let bias = restitution * vel_along_normal.min(-config.restitution_slop);
```

This matches Template-Repo's formula: `restitution * min(vn + restitutionSlop, 0)` which is equivalent to `restitution * vn` when `vn < -restitutionSlop`, and `0` otherwise — but continuous at the boundary.

### Fix 3: Bump Solver Iterations

**File**: `packages/physics-rust/src/solver.rs:18`

Change `velocity_iterations` default from 8 to 10 to match Template-Repo.

## Files to Modify

| File | Change |
|------|--------|
| `packages/physics-rust/src/solver.rs` | Add `solve_velocity_block`, fix restitution bias, bump iterations |
| `packages/physics-rust/src/world.rs` | Update `solve_velocity_batch` to use block solver for 2-contact manifolds |

## Verification

1. Run existing Rust tests: `cargo test -p physics-rust`
2. The existing `test_spinning_cuboid_collision`, `test_spinning_cuboid_vertex_collision`, and `test_extreme_spin_collision` tests in `world.rs` should still pass (they test for no passthrough)
3. Build WASM: `npm run build -w packages/physics-rust`
4. Run full test suite: `npm test`
5. Manual visual verification: shapes should show zero visible overlap in demo

## Dependencies

None — all changes are internal to `packages/physics-rust/src/`. No changes to the public API, TypeScript types, or WASM bindings.
