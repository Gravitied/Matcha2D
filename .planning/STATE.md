---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
current_plan: 2 of 3
status: executing
last_updated: "2026-04-02T09:58:32.655Z"
progress:
  total_phases: 4
  completed_phases: 0
  total_plans: 3
  completed_plans: 1
---

# Matcha2D - Phase 01: WASM Physics Core

## Position
- **Current Plan:** 2 of 3
- **Status:** In Progress
- **Last session:** 2026-04-02T09:58:32.653Z

## Goal
Replace the TypeScript physics core with a WASM-compiled Box2D engine that conforms to the existing `PhysicsBackend` interface, providing production-grade 2D physics.

## Decisions
- Use Emscripten to compile Box2D C code to WASM (not Rust — Box2D is already C)
- Box2D source in `box2d-main/` is the reference implementation
- Must conform to existing `@matcha2d/types` contract (buffer layout, PhysicsBackend interface)
- TypeScript wrappers in `packages/core/src/wasm/` will bridge WASM to the PhysicsBackend interface
- Keep existing TypeScript implementations as fallback during transition
- Used add_executable (not add_library) for Emscripten WASM output
- Shape recreation on type change rather than incremental updates
- Contact query via body iteration using b2Body_GetContactData

## Blockers
- Emscripten SDK must be installed on the system to build WASM module

## Completed Plans
- **Plan 01:** WASM build pipeline and C bridge API (01-wasm-core-01-SUMMARY.md) — Complete

## Pending Todos
- Write TypeScript bindings (glue code) that implement PhysicsBackend
- Integrate WASM backend into the World simulation loop
- Test with existing test suite
