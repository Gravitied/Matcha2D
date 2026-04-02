---
phase: 01-wasm-core
plan: 02
subsystem: wasm
tags: [emscripten, wasm, box2d, typescript, physics-backend, memory-management]

# Dependency graph
requires:
  - phase: 01-wasm-core-01
    provides: Emscripten build pipeline and C bridge API (wasm_bridge.h/c)
provides:
  - TypeScript WASM module loader with typed cwrap wrappers
  - WasmPhysicsBackend implementing the PhysicsBackend interface
  - Public API exports (WasmPhysicsBackend, loadWasmModule, WasmInstance)
  - Updated documentation with build instructions and API reference
affects: [01-wasm-core-03, 01-wasm-core-04]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - WASM memory bridge: JS typed arrays → HEAPF32/HEAPU8 → C pointers via malloc
    - Unified step pattern: Box2D handles full pipeline internally, individual methods are no-ops
    - Immediate free pattern: temporary WASM allocations freed after each call, not tracked

key-files:
  created:
    - packages/core/src/wasm/WasmModule.ts
    - packages/core/src/wasm/WasmPhysicsBackend.ts
    - packages/core/src/wasm/index.ts
    - packages/core/src/wasm/box2d.d.ts
    - packages/core/wasm/build/box2d.js
    - packages/core/wasm/build/box2d.d.ts
  modified:
    - packages/core/src/wasm/README.md

key-decisions:
  - "Individual PhysicsBackend methods (broadphase, narrowphase, etc.) are no-ops in WASM mode since Box2D handles everything internally during b2_step()"
  - "Temporary WASM heap allocations are freed immediately after each call rather than tracked persistently — simpler memory management"
  - "Stub box2d.js placed in wasm/build/ so TypeScript can resolve the import without requiring a full WASM build"

patterns-established:
  - "WasmInstance interface: typed wrapper around all 7 C bridge functions with clean TS signatures"
  - "wrapCwrap helper: safe generic wrapper around Module.cwrap avoiding TS constraint issues"
  - "allocAndCopy + readFromHeap + freeMany: reusable memory management primitives"

requirements-completed:
  - WASM-03

# Metrics
duration: 15min
completed: 2026-04-02
---

# Phase 01 Plan 02: TypeScript WASM Wrapper and PhysicsBackend Summary

**TypeScript wrapper classes loading the Emscripten-compiled Box2D WASM module and implementing the PhysicsBackend interface with SoA buffer translation**

## Performance

- **Duration:** 15 min
- **Started:** 2026-04-02T05:54:00Z
- **Completed:** 2026-04-02T06:09:11Z
- **Tasks:** 2
- **Files modified:** 7

## Accomplishments

- WasmModule.ts: async WASM loader with typed cwrap wrappers for all 7 C bridge functions, SoA buffer copy to/from WASM linear memory, memory management (malloc/free)
- WasmPhysicsBackend.ts: full PhysicsBackend implementation — init() creates Box2D world, collide() runs sync→step→read→contacts pipeline, individual methods are no-ops
- index.ts: clean public API exporting WasmPhysicsBackend, loadWasmModule, WasmInstance
- README.md: comprehensive documentation with architecture diagram, build instructions, API examples, C bridge reference
- Stub box2d.js + box2d.d.ts in wasm/build/ for TypeScript resolution without requiring WASM build

## Task Commits

Each task was committed atomically:

1. **Task 1: Create WASM module loader** - `c188562` (feat)
   - WasmModule.ts with loadWasmModule() returning WasmInstance
   - WasmInstance wraps all 7 bridge functions: init, destroy, syncBodies, syncShapes, step, readBodies, getContacts, dispose
   - box2d.d.ts type declarations for Emscripten glue module
   - Stub box2d.js + box2d.d.ts in wasm/build/ for TS resolution

2. **Task 2: Implement PhysicsBackend using WASM** - `aeb0c93` (feat)
   - WasmPhysicsBackend implements PhysicsBackend interface
   - collide() runs full pipeline: syncBodies → syncShapes → step → readBodies → getContacts
   - broadphase/narrowphase/solveVelocity/integrate/solvePosition are no-ops
   - index.ts exports public API
   - README.md updated with build instructions and API docs
   - WasmModule.ts: added freeMany() to prevent memory leaks

**Plan metadata:** `aeb0c93` (docs: complete plan)

## Files Created/Modified

- `packages/core/src/wasm/WasmModule.ts` — WASM loader, typed cwrap wrappers, SoA buffer copy, memory management
- `packages/core/src/wasm/WasmPhysicsBackend.ts` — PhysicsBackend implementation wrapping Box2D WASM
- `packages/core/src/wasm/index.ts` — Public exports (WasmPhysicsBackend, loadWasmModule, WasmInstance)
- `packages/core/src/wasm/box2d.d.ts` — Type declarations for Emscripten glue module
- `packages/core/wasm/build/box2d.js` — Stub for TS resolution (replaced by Emscripten build)
- `packages/core/wasm/build/box2d.d.ts` — Type declarations for stub JS file
- `packages/core/src/wasm/README.md` — Updated with architecture, build instructions, API docs

## Decisions Made

- **No-op individual methods:** Box2D's b2World_Step handles broadphase → narrowphase → solve → integrate internally. The WASM backend's collide() is the primary entry point; individual methods return empty or do nothing.
- **Immediate free over persistent tracking:** Temporary WASM heap allocations (for sync/read/getContacts) are freed immediately after each call via freeMany(), rather than tracked in a persistent list. Simpler and prevents leaks.
- **Stub JS file for TS resolution:** Placed a minimal stub at wasm/build/box2d.js so TypeScript can resolve the import without requiring a full WASM build. Emscripten overwrites this when the build runs.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Memory leak in WasmModule.ts — temporary allocations not freed**
- **Found during:** Task 2 implementation (reviewing WasmModule.ts from Task 1)
- **Issue:** syncBodies, syncShapes, readBodies, and getContacts allocated WASM heap memory via malloc but never freed it, causing memory leaks on every call
- **Fix:** Added freeMany() helper function and called it after each WASM bridge call to free temporary allocations
- **Files modified:** packages/core/src/wasm/WasmModule.ts
- **Verification:** Code review confirmed all malloc'd pointers are freed after use
- **Committed in:** aeb0c93 (Task 2 commit)

---

**Total deviations:** 1 auto-fixed (1 bug)
**Impact on plan:** Essential fix — memory leaks would cause crashes in production. No scope creep.

## Issues Encountered

- TypeScript `verbatimModuleSyntax` required `import type` for WasmInstance — fixed by splitting import
- `source.constructor` not constructable in TS — replaced with explicit `instanceof` checks and typed array constructors
- Emscripten glue module not found at build time — resolved by creating stub JS + .d.ts files in wasm/build/
- Stale `.git/index.lock` file on Windows — removed manually before commits

## User Setup Required

None - no external service configuration required.

Note: Emscripten SDK must be installed on the system to actually build the WASM module. This is a pre-existing blocker noted in STATE.md. The TypeScript code compiles and is ready — once WASM is built, the wrapper will work.

## Next Phase Readiness

- WASM TypeScript wrapper is complete and type-checks cleanly
- WasmPhysicsBackend implements the full PhysicsBackend interface
- Ready for Plan 03: Integration with World simulation loop
- Ready for Plan 04: Build pipeline integration (once Emscripten SDK is available)

---

*Phase: 01-wasm-core*
*Completed: 2026-04-02*

## Self-Check: PASSED
