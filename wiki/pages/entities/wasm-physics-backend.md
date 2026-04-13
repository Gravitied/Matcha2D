---
title: "WasmPhysicsBackend"
type: entity
tags: [wasm, dev-a, architecture]
related: [physics-backend-interface.md, wasm-dual-backend.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# WasmPhysicsBackend

## Location

`packages/core/src/wasm/`

## What It Is

Implements `PhysicsBackend` using Rust/Box2D compiled to WebAssembly via Emscripten.

## Key Files

| File | Purpose |
|------|---------|
| `WasmPhysicsBackend` (class) | Implements `PhysicsBackend`, bridges Matcha2D ↔ Box2D |
| `WasmModule` loader (`loadWasmModule`) | Async loader for the WASM module |
| `box2d.d.ts` | TypeScript declarations for Box2D Emscripten bindings |

## WASM Artifact

`wasm/build/box2d.js` — gitignored. Must be built from Rust source in `packages/physics-rust/`.

## Behavior

In WASM mode, individual methods (`broadphase`, `narrowphase`, `solveVelocity`, `integrate`, `solvePosition`) are **no-ops**. Box2D handles the entire pipeline internally inside `collide()`.

## Loading

```typescript
const world = await World.createWithWasm(config)
// Must await — WASM module loads asynchronously
```

## Rust Source

`packages/physics-rust/` — the Rust physics core. Contains `body.rs`, `collider.rs`, `contact.rs`, `contact_constraint.rs`, `velocity_solver.rs`, `world.rs`, `solver_body.rs`, `lib.rs`.

## Related

- [[physics-backend-interface]] — interface this class implements
- [[wasm-dual-backend]] — design rationale
- [[world-class]] — `createWithWasm` factory
