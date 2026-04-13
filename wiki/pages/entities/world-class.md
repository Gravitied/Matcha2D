---
title: "World Class"
type: entity
tags: [api, dev-b, architecture]
related: [physics-backend-interface.md, body-manager.md, simulation-loop.md, island-manager.md, fixed-timestep.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# World Class

## Location

`packages/world/src/world.ts`

## What It Is

The top-level user-facing API class. Owns the body buffers, creates/destroys bodies, and orchestrates the simulation loop.

## Factory Methods

```typescript
World.createWithTS(config?)          // TypeScript backend (sync)
World.createWithWasm(config?)        // WASM backend (async — must await)
World.createWithBackend(backend, config?)  // custom PhysicsBackend
```

## Key API

```typescript
createBody(def)              // allocate a body, returns BodyHandle
destroyBody(handle)          // free a body slot
step(dt)                     // advance simulation by dt seconds
renderAlpha                  // 0..1 interpolation factor for renderer
setCollisionCallbacks(callbacks)  // begin/stay/end contact events
```

## Collision Callbacks

```typescript
setCollisionCallbacks({ onBegin?, onStay?, onEnd? })
```

Fired by the `ContactTracker`. Each callback receives the two `BodyHandle`s involved.

## Internals

| Component | File | Role |
|-----------|------|------|
| `BodyManager` | `body-manager.ts` | flat-array allocation helpers |
| `SimulationLoop` | `simulation-loop.ts` | fixed-timestep accumulator + `renderAlpha` |
| `IslandManager` | `island.ts` | union-find for sleep optimization |

## Related

- [[physics-backend-interface]] — what `World` delegates physics to
- [[body-manager]] — body allocation internals
- [[simulation-loop]] — fixed-timestep logic
- [[island-manager]] — sleep/island tracking
- [[world-config]] — configuration options
