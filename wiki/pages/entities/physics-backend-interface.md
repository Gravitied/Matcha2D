---
title: "PhysicsBackend Interface"
type: entity
tags: [api, architecture, dev-a, dev-b]
related: [wasm-dual-backend.md, world-class.md, sequential-impulse-solver.md, wasm-physics-backend.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# PhysicsBackend Interface

## Location

`packages/types/src/backend.ts`

## What It Is

The single integration point between Dev A (physics core) and Dev B (engine shell). Dev B calls these methods from the simulation loop; Dev A implements them.

## Interface Definition

```typescript
interface PhysicsBackend {
  broadphase(buffers, count, method?) -> CollisionPair[]
  narrowphase(buffers, pairs, method?) -> ContactManifold[]
  collide(buffers, count, broadphaseMethod?, narrowphaseMethod?) -> ContactManifold[]
  solveVelocity(buffers, manifolds, config) -> void
  integrate(buffers, count, dt, gravity) -> void
  solvePosition(buffers, manifolds, config) -> void
}
```

## Key Notes

- `collide()` is the **primary entry point** — runs full broadphase → narrowphase pipeline
- In WASM mode: `broadphase`, `narrowphase`, `solveVelocity`, `integrate`, `solvePosition` are **no-ops** — Box2D handles everything inside `collide()`
- In TS mode: all methods are active and called individually by the simulation loop

## Implementations

| Class | Location | Backend |
|-------|----------|---------|
| `TsPhysicsBackend` | `packages/core/src/` | TypeScript (GJK, SI solver) |
| `WasmPhysicsBackend` | `packages/core/src/wasm/` | Rust/Box2D WASM |

## Related

- [[wasm-dual-backend]] — design rationale
- [[world-class]] — how backends are created and injected
- [[sequential-impulse-solver]] — TS backend solver
- [[wasm-physics-backend]] — WASM backend
