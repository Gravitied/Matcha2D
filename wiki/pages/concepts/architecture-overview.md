---
title: "Matcha2D Architecture Overview"
type: concept
tags: [architecture, api, dev-a, dev-b]
related: [physics-backend-interface.md, buffer-layout.md, wasm-dual-backend.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Matcha2D Architecture Overview

## Pipeline

```
User API (TS)  ->  Simulation Loop (TS)  ->  Physics Core (TS/WASM)  ->  Renderer (TS)
```

## Package Structure

| Package | Path | Owner | Purpose |
|---------|------|-------|---------|
| `@matcha2d/types` | `packages/types/` | Shared | Shared contract: buffer layout, interfaces, config, collision types |
| `@matcha2d/core` | `packages/core/` | Dev A | Physics math, collision detection, constraint solver |
| `@matcha2d/world` | `packages/world/` | Dev B | World API, body management, simulation loop |
| `@matcha2d/render` | `packages/render/` | Dev B | Canvas2D / WebGL debug renderer |
| `@matcha2d/tools` | `packages/tools/` | Dev B | Serialization, profiling, debug utilities |

## Dependency Graph

```
@matcha2d/types      (no deps — build first)
    ├── @matcha2d/core    (types)
    │       │
    ├───────┴── @matcha2d/world  (types + core)
    ├── @matcha2d/render  (types)
    └── @matcha2d/tools   (types)
```

Build order: types → core → world/render/tools. Root `npm run build` handles this automatically.

## Design Principles

- **TypeScript-first** — strict mode, ESM only, no runtime deps
- **Data-Oriented Design** — flat typed arrays (SoA) for all body data
- **Dual backend** — TypeScript (default) or Rust/WASM (Box2D) via `PhysicsBackend` interface
- **Ease of Use** — simple API for users, detailed for power users
- **Zero allocations** in hot paths — pass output arrays + indices, not new objects

## Dev Ownership Boundaries

Dev A owns `packages/core/` and `packages/types/`. Dev B owns `packages/world/`, `packages/render/`, `packages/tools/`. Neither should touch the other's packages.

## Related

- [[physics-backend-interface]] — the integration contract between Dev A and Dev B
- [[buffer-layout]] — the SoA data layout
- [[wasm-dual-backend]] — WASM/TypeScript backend strategy
