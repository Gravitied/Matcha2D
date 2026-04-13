---
title: "Solver Concepts: Warm Starting, Baumgarte, Impulse Accumulation"
type: concept
tags: [solver, performance, algorithm]
related: [sequential-impulse-solver.md, world-config.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Solver Concepts

## Sequential Impulse (SI)

The solver iterates over contacts multiple times per step, applying velocity corrections (impulses) to satisfy constraints. Each iteration brings contacts closer to resolution. More iterations = more stable but more expensive.

Config: `velocityIterations` (default 10), `positionIterations` (default 4).

## Warm Starting

Cached impulses from the previous frame are applied at the start of the current frame. Since contacts usually persist, this gives the solver a better initial guess, reducing the iterations needed for convergence.

Config: `warmStarting` (bool), `warmStartThreshold` (float — discard cached impulse if bodies moved too far).

## Impulse Accumulation

During the velocity solve, per-contact accumulated impulses are clamped to physically valid ranges. This prevents impulses from going negative (objects pulling each other). Also enables the block solver for 2-contact manifolds.

Config: `impulseAccumulation` (bool).

## Block Solver

For 2-contact manifolds (e.g., box resting on a surface), a 2×2 block system is solved simultaneously rather than iterating contacts independently. Reduces jitter significantly for flat surfaces.

Config: `blockSolver` (bool, default `true`).

## Baumgarte Position Correction

After velocity solving, a small position correction is applied to push overlapping bodies apart. The `baumgarteFactor` controls how aggressively penetration is corrected per step. Too high = jitter; too low = sinking.

Config: `baumgarteFactor`, `penetrationSlop` (how much overlap is tolerated before correction kicks in).

## Position Correction (separate pass)

An additional position-level correction pass runs after velocity solving, controlled by `positionCorrection` (bool) and `positionCorrectionBeta`.

## Restitution

Bounciness. `defaultRestitution` controls how much kinetic energy is preserved in collisions. `restitutionSlop` is a velocity threshold below which restitution is zeroed (prevents micro-bouncing at rest).

## Related

- [[sequential-impulse-solver]] — the concrete implementation
- [[world-config]] — all tuning knobs
