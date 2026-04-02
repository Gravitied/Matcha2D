# Matcha2D Roadmap

## Phase 01: WASM Physics Core
**Goal:** Port Box2D C engine to WASM, integrate with existing TypeScript monorepo as the physics backend.

**Requirements:**
- WASM-01: Box2D compiles to WASM via Emscripten
- WASM-02: WASM module exposes C API compatible with MatchaBuffers (SoA format)
- WASM-03: TypeScript wrapper implements PhysicsBackend interface
- WASM-04: WASM backend works with existing World simulation loop
- WASM-05: Build pipeline produces .wasm + .js glue, integrated into npm build

**Plans:**
4/4 plans complete
- [x] 01-wasm-core-01-PLAN.md — WASM build pipeline and C bridge API ✅
- [x] 01-wasm-core-02-PLAN.md — TypeScript WASM wrapper and PhysicsBackend ✅
- [x] 01-wasm-core-03-PLAN.md — Integration with World simulation loop ✅
- [ ] 01-wasm-core-04-PLAN.md — Gap closure: filename alignment, timestep fix, .gitignore

## Phase 02: Collision Pipeline & WASM Integration Fix
**Goal:** Fix WASM backend collision detection so contact normals/frames are reported correctly, ensure shapes render properly (including complex shapes), rewrite Box2D bridge scripts as matcha2D-specific, and add thorough tests.

**Requirements:**
- COL-01: WASM backend produces correct contact normals and contact frames matching the TypeScript backend
- COL-02: All shape types (box, circle, polygon) render correctly in the demo using both backends
- COL-03: Box2D WASM bridge C code renamed/rewritten as matcha_physics (matcha2D-specific, no generic Box2D naming)
- COL-04: Collision algorithms consistent with standard 2D physics engine conventions (normal direction, contact point, penetration depth)
- COL-05: Comprehensive test suite covering WASM backend collision results, contact manifold correctness, and shape rendering correctness

**Plans:**
0/4 plans complete
- [ ] 02-collision-pipeline-wasm-fix-01-PLAN.md — Rename b2_ to m2_ in C bridge, fix m2_get_contacts contact extraction
- [ ] 02-collision-pipeline-wasm-fix-02-PLAN.md — Update WasmModule.ts cwrap names and contact array extraction, add World.manifolds getter
- [ ] 02-collision-pipeline-wasm-fix-03-PLAN.md — Fix demos to visualize contacts/normals in WASM mode
- [ ] 02-collision-pipeline-wasm-fix-04-PLAN.md — Comprehensive contact manifold test suite

## Phase 03: World & Simulation (Future)
- [ ] To be planned

## Phase 04: Renderer (Future)
- [ ] To be planned
