---
title: "WASM / TypeScript Dual Backend Design"
type: concept
tags: [wasm, architecture, dev-a, dev-b]
related: [physics-backend-interface.md, wasm-physics-backend.md, architecture-overview.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# WASM / TypeScript Dual Backend Design

## What It Is

Matcha2D supports two physics backends behind a single `PhysicsBackend` interface:

1. **TypeScript backend** — pure TS implementation (GJK+EPA, sequential impulse solver, dynamic AABB tree)
2. **WASM backend** — Rust/Box2D compiled to WebAssembly via Emscripten

The backend is swapped at construction time. Dev B's simulation loop calls the same interface regardless of which is active.

## Creation API

```typescript
World.createWithTS(config?)        // TypeScript backend (default, sync)
World.createWithWasm(config?)      // WASM backend (async — must await)
World.createWithBackend(backend, config?)  // custom PhysicsBackend
```

## WASM Mode Behavior

In WASM mode, `broadphase`, `narrowphase`, `solveVelocity`, `integrate`, and `solvePosition` are **no-ops** on the TS side. Box2D handles the entire pipeline internally during `collide()`.

The `WasmPhysicsBackend` (`packages/core/src/wasm/`) bridges between Matcha2D's buffer layout and Box2D's internal representation.

## Why Two Backends?

- **TypeScript backend** — zero setup, works everywhere, easy to debug, educational
- **WASM backend** — higher performance for complex scenes, production use case
- **Shared interface** — Dev B writes the simulation loop once; swapping backends is one line

## WASM Build Artifact

`wasm/build/box2d.js` — gitignored (not committed). Must be built separately from the Rust source in `packages/physics-rust/`.

## Related

- [[physics-backend-interface]] — the shared contract
- [[wasm-physics-backend]] — the concrete WASM implementation
- [[world-class]] — `createWithTS`, `createWithWasm`, `createWithBackend`
