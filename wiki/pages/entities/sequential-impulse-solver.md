---
title: "Sequential Impulse Solver"
type: entity
tags: [solver, algorithm, dev-a, performance]
related: [collision-pipeline.md, solver-concepts.md, world-config.md, physics-backend-interface.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Sequential Impulse Solver

## Location

`packages/core/src/solver/sequential-impulse.ts`

## What It Is

The constraint solver for Matcha2D's TypeScript backend. Resolves contact constraints by applying velocity impulses iteratively.

## Methods

```typescript
solveVelocity(buffers, manifolds, config) -> void
solvePosition(buffers, manifolds, config) -> void
integrate(buffers, count, dt, gravity) -> void
```

## Features

- **Block solver for 2-contact manifolds** — solves the 2×2 system simultaneously, reducing jitter on flat surfaces. Enabled via `config.blockSolver`.
- **Impulse accumulation** — clamps accumulated impulses to physically valid ranges. Enabled via `config.impulseAccumulation`.
- **Warm starting** — applies cached impulses from the previous frame for faster convergence. Enabled via `config.warmStarting`.
- **Baumgarte position correction** — pushes overlapping bodies apart each step. Controlled by `config.baumgarteFactor` and `config.penetrationSlop`.

## `integrate`

Applies gravity and integrates velocity → position for all active, non-static bodies. Called once per step after velocity solving.

## Iteration Counts

- `config.velocityIterations` (default 10) — inner loop for velocity solve
- `config.positionIterations` (default 4) — inner loop for position correction

## Related

- [[solver-concepts]] — explanation of warm starting, Baumgarte, etc.
- [[world-config]] — all solver config knobs
- [[collision-pipeline]] — produces manifolds this solver consumes
