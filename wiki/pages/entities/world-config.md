---
title: "WorldConfig"
type: entity
tags: [api, architecture, solver, dev-b]
related: [world-class.md, solver-concepts.md, sequential-impulse-solver.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# WorldConfig

## Location

`packages/types/src/config.ts`

## What It Is

Configuration struct passed to `World.create*()`. All fields are optional — `DEFAULT_WORLD_CONFIG` provides sensible defaults.

## Fields

### Simulation

| Field | Default | Description |
|-------|---------|-------------|
| `gravity` | `{ x: 0, y: -9.81 }` | World gravity vector |
| `fixedTimestep` | `1/60` | Physics step size in seconds |

### Solver Iterations

| Field | Default | Description |
|-------|---------|-------------|
| `velocityIterations` | `10` | Sequential impulse velocity solve iterations |
| `positionIterations` | `4` | Position correction iterations |

### Broadphase / Narrowphase

| Field | Default | Options |
|-------|---------|---------|
| `broadphaseMethod` | `'dynamicTree'` | `'dynamicTree'`, `'sap'`, `'bvh'` |
| `narrowphaseMethod` | `'gjk'` | `'gjk'` (only option) |

### Position Correction (Baumgarte)

| Field | Default | Description |
|-------|---------|-------------|
| `baumgarteFactor` | — | Aggressiveness of Baumgarte correction |
| `penetrationSlop` | — | Overlap tolerance before correction fires |
| `positionCorrection` | — | Enable/disable position correction pass |
| `positionCorrectionBeta` | — | Beta for position correction pass |

### Impulse Solver Tuning

| Field | Default | Description |
|-------|---------|-------------|
| `blockSolver` | `true` | 2-contact block solver (reduces jitter) |
| `impulseAccumulation` | — | Enable impulse clamping/accumulation |
| `warmStarting` | — | Reuse previous-frame impulses |
| `warmStartThreshold` | — | Discard cached impulse if bodies moved this far |

### Restitution & Friction

| Field | Default | Description |
|-------|---------|-------------|
| `defaultFriction` | — | Default friction coefficient |
| `defaultRestitution` | — | Default restitution (bounciness) |
| `restitutionSlop` | — | Velocity threshold to zero restitution |

### Sleep

| Field | Default | Description |
|-------|---------|-------------|
| `sleepEnabled` | — | Enable/disable sleep system |
| `sleepTimeThreshold` | — | Time before a body can sleep |
| `sleepVelocityThreshold` | — | Velocity below which a body is sleep-eligible |

## Related

- [[world-class]] — consumes config
- [[solver-concepts]] — explains the solver fields
- [[fixed-timestep]] — `fixedTimestep` field
