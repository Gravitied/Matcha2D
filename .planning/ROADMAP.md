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
3/4 plans complete
- [x] 01-wasm-core-01-PLAN.md — WASM build pipeline and C bridge API ✅
- [x] 01-wasm-core-02-PLAN.md — TypeScript WASM wrapper and PhysicsBackend ✅
- [x] 01-wasm-core-03-PLAN.md — Integration with World simulation loop ✅
- [ ] 01-wasm-core-04-PLAN.md — Gap closure: filename alignment, timestep fix, .gitignore

## Phase 02: Collision Pipeline (Future)
- [ ] To be planned

## Phase 03: World & Simulation (Future)
- [ ] To be planned

## Phase 04: Renderer (Future)
- [ ] To be planned
