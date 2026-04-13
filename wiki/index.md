# Matcha2D Wiki Index

A catalog of all wiki pages. Updated on every ingest, query save, or lint pass.
Read this first when searching for relevant pages before answering a query.

---

## Entities

- [PhysicsBackend Interface](pages/entities/physics-backend-interface.md) — Integration contract between Dev A (core) and Dev B (world)
- [World Class](pages/entities/world-class.md) — Top-level user API: body creation, step, collision callbacks
- [WorldConfig](pages/entities/world-config.md) — All configuration fields with defaults (gravity, solver, broadphase, sleep)
- [Collision Pipeline](pages/entities/collision-pipeline.md) — Full broadphase → narrowphase → solver flow
- [Broadphase (Dynamic AABB Tree / SAP)](pages/entities/broadphase.md) — Spatial culling; dynamic tree (default), SAP, BVH
- [Narrowphase: GJK + EPA](pages/entities/narrowphase-gjk.md) — Exact contact detection; GJK+EPA + circle-circle analytical
- [Sequential Impulse Solver](pages/entities/sequential-impulse-solver.md) — Velocity + position constraint solver; block solver, warm starting
- [Contact Tracker](pages/entities/contact-tracker.md) — Begin/stay/end event tracking across frames
- [WasmPhysicsBackend](pages/entities/wasm-physics-backend.md) — Box2D Rust/WASM backend; loading, no-op methods, artifact location
- [Math Primitives (Vec2, Mat2, AABB)](pages/entities/math-primitives.md) — Flat-array math; Vec2, Mat2, AABB functions
- [Render Package (@matcha2d/render)](pages/entities/render-package.md) — IRenderer interface, Canvas2DRenderer, renderAlpha usage
- [Tools Package (@matcha2d/tools)](pages/entities/tools-package.md) — serializeBuffers, Profiler

---

## Concepts

- [Matcha2D Architecture Overview](pages/concepts/architecture-overview.md) — Package structure, pipeline, dev ownership, design principles
- [Buffer Layout (SoA Typed Arrays)](pages/concepts/buffer-layout.md) — MatchaBuffers interface, SoA layout, MAX_BODIES, BodyHandle
- [Data-Oriented Design (DoD)](pages/concepts/data-oriented-design.md) — Why SoA over AoS; cache, WASM, GC trade-offs
- [Fixed Timestep & Render Interpolation](pages/concepts/fixed-timestep.md) — Accumulator pattern, renderAlpha, determinism
- [WASM / TypeScript Dual Backend Design](pages/concepts/wasm-dual-backend.md) — Dual backend rationale, creation API, WASM no-op behavior
- [Solver Concepts](pages/concepts/solver-concepts.md) — Warm starting, Baumgarte, impulse accumulation, block solver, restitution

---

## Sources

<!-- Source summary pages go here — one per raw document ingested -->

---

## Analyses

<!-- Saved query results and explorations go here -->

---

*Last updated: 2026-04-06 | Total pages: 18*
