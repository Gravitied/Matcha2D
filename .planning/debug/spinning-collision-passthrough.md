---
status: awaiting_human_verify
trigger: "Whenever objects are spinning as they collide, they pass through one another. Also there is a lot of jittering with collisions."
created: 2026-04-02T12:00:00Z
updated: 2026-04-02T12:30:00Z
---

## Current Focus

hypothesis: CONFIRMED — All bugs found and fixed (4 total)
test: 32/32 tests pass including 3 spinning collision tests
expecting: user confirms spinning collisions work and jitter is gone
next_action: await human verification

## Symptoms
<!-- Written during gathering, then IMMUTABLE -->

expected: Objects should collide and bounce off each other even when spinning; collisions should be smooth without jitter
actual: Spinning objects pass through each other during collision; collisions exhibit heavy jittering
errors: unknown
reproduction: Any scene with spinning dynamic bodies colliding
started: Since Rust solver implementation

## Eliminated
<!-- APPEND only - prevents re-investigating -->

- hypothesis: Warmstarting broken (impulses not persisted)
  evidence: Fixed — impulses now transferred across frames via impulse_id matching
  timestamp: 2026-04-02T12:10:00Z

- hypothesis: Position correction ignores rotation
  evidence: Fixed — added angular correction terms to apply_position_correction
  timestamp: 2026-04-02T12:10:00Z

- hypothesis: restitution_slop too high (1.0 vs 0.5)
  evidence: Fixed — changed to 0.5
  timestamp: 2026-04-02T12:10:00Z

- hypothesis: SAT reference/incident face selection wrong when normal flipped
  evidence: Fixed — when normal is flipped for A→B consistency, ref/incident faces now swap too. Test confirms no pass-through.
  timestamp: 2026-04-02T12:25:00Z

## Evidence
<!-- APPEND only - facts discovered -->

- timestamp: 2026-04-02T12:01:00Z
  checked: contact.rs ContactTracker::update()
  found: Warmstarting impulses always reset to (0.0, 0.0)
  implication: Fixed

- timestamp: 2026-04-02T12:02:00Z
  checked: world.rs solve_velocity_batch()
  found: Passed &mut 0.0 instead of persisted values
  implication: Fixed

- timestamp: 2026-04-02T12:03:00Z
  checked: solver.rs apply_position_correction()
  found: Only corrected linear position, not angular
  implication: Fixed

- timestamp: 2026-04-02T12:20:00Z
  checked: narrowphase.rs sat_polygon_polygon() lines 291-302
  found: CRITICAL BUG — When SAT flips normal to ensure A→B direction, the reference/incident face assignment stays based on the original (unflipped) normal. This causes the edge clipping to use the wrong face as reference, producing incorrect contact points. Example: if B's face axis had minimum penetration, B becomes reference. But after normal flip, the normal now points along A's face normal — A should be the reference.
  implication: This was the primary cause of spinning pass-through — wrong contact points meant wrong impulse direction

- timestamp: 2026-04-02T12:25:00Z
  checked: test_spinning_cuboid_collision, test_spinning_cuboid_vertex_collision, test_extreme_spin_collision
  found: All 3 tests pass. Cuboids at 0°, 45°, and with 60 rad/s spin all collide correctly without pass-through.
  implication: Fix verified

## Resolution
<!-- OVERWRITE as understanding evolves -->

root_cause: Four bugs:
1. **SAT reference/incident face swap** (narrowphase.rs): When the SAT collision normal is flipped to ensure A→B direction, the reference and incident face assignments must also swap. Without this, edge clipping uses the wrong face as reference, producing incorrect contact manifold for rotated/spinning objects. This was the PRIMARY cause of spinning pass-through.
2. **Warmstarting broken** (contact.rs + world.rs): Impulses not persisted across frames or within iterations.
3. **Position correction ignores rotation** (solver.rs): Linear-only correction can't separate spinning objects.
4. **restitution_slop too high** (solver.rs): 1.0 → 0.5.

fix:
- narrowphase.rs: Added `flipped` flag tracking normal flip; ref/incident swap when flipped
- contact.rs: Warmstarting transfers impulses via impulse_id matching
- world.rs: Solver reads/writes persisted impulses from ContactPair
- solver.rs: Position correction includes angular terms; restitution_slop = 0.5

files_changed:
- packages/physics-rust/src/narrowphase.rs: SAT reference/incident face swap fix
- packages/physics-rust/src/contact.rs: Warmstarting impulse transfer
- packages/physics-rust/src/world.rs: Solver impulse persistence + diagnostic tests
- packages/physics-rust/src/solver.rs: Angular position correction + restitution_slop
