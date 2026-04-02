# Matcha2D - Phase 01: WASM Physics Core

## Position
Starting fresh — porting Box2D C engine to WASM as the physics core backend.

## Goal
Replace the TypeScript physics core with a WASM-compiled Box2D engine that conforms to the existing `PhysicsBackend` interface, providing production-grade 2D physics.

## Decisions
- Use Emscripten to compile Box2D C code to WASM (not Rust — Box2D is already C)
- Box2D source in `box2d-main/` is the reference implementation
- Must conform to existing `@matcha2d/types` contract (buffer layout, PhysicsBackend interface)
- TypeScript wrappers in `packages/core/src/wasm/` will bridge WASM to the PhysicsBackend interface
- Keep existing TypeScript implementations as fallback during transition

## Blockers
- Emscripten SDK must be installed on the system
- WASM module needs to read/write MatchaBuffers (SoA format) which differs from Box2D's internal data model

## Pending Todos
- Set up Emscripten build pipeline for Box2D
- Create WASM module with C API that accepts SoA buffer pointers
- Write TypeScript bindings (glue code) that implement PhysicsBackend
- Integrate WASM backend into the World simulation loop
- Test with existing test suite
