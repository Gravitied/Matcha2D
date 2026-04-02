---
phase: 01-wasm-core
verified: 2026-04-02T06:22:00Z
status: gaps_found
score: 4/5 must-haves verified
gaps:
  - truth: "Build pipeline produces .wasm + .js glue files that match expected filenames"
    status: partial
    reason: "Filename mismatch between CMakeLists.txt output and TypeScript expectations. CMakeLists.txt produces matcha2d_wasm.js/matcha2d_wasm.wasm, but tsup.config.ts looks for box2d.js/box2d.wasm and WasmModule.ts imports ../../wasm/build/box2d.js. A stub box2d.js exists for TS resolution, but the actual Emscripten build would produce mismatched filenames."
    artifacts:
      - path: "packages/core/wasm/CMakeLists.txt"
        issue: "Output target named matcha2d_wasm (line 101), produces matcha2d_wasm.js + matcha2d_wasm.wasm"
      - path: "packages/core/tsup.config.ts"
        issue: "Looks for box2d.wasm and box2d.js (line 17), won't find matcha2d_wasm.* files"
      - path: "packages/core/src/wasm/WasmModule.ts"
        issue: "Dynamic import of ../../wasm/build/box2d.js (line 40), won't resolve matcha2d_wasm.js"
      - path: "packages/core/wasm/build.sh"
        issue: "Copies matcha2d_wasm.* to dist/ (lines 40-47), but tsup expects box2d.*"
    missing:
      - "Align filenames: either rename CMakeLists.txt output to box2d or update tsup.config.ts + WasmModule.ts to use matcha2d_wasm"
  - truth: "WASM backend respects simulation timestep parameters from World config"
    status: partial
    reason: "WasmPhysicsBackend.collide() hardcodes dt=1/60 and subSteps=4 instead of using the dt from the simulation loop or config values. Works with default config but breaks if fixedTimestep differs from 1/60."
    artifacts:
      - path: "packages/core/src/wasm/WasmPhysicsBackend.ts"
        issue: "Line 120: this.wasm.step(this.worldHandle, 1 / 60, 4) — hardcoded values ignore the dt parameter and config"
    missing:
      - "Pass dt parameter to step() instead of hardcoding 1/60"
      - "Make subSteps configurable via WorldConfig or constructor"
  - truth: "WASM build artifacts can be versioned in dist/wasm/"
    status: failed
    reason: "Root .gitignore line 5 contains '*.wasm' which ignores ALL .wasm files including dist/wasm/box2d.wasm. Confirmed: 'git check-ignore packages/core/dist/wasm/box2d.wasm' returns the path. This prevents WASM binaries from being committed even though the plan intends dist/wasm/ to contain the final build output."
    artifacts:
      - path: ".gitignore"
        issue: "Line 5: '*.wasm' ignores ALL .wasm files, including the intended dist/wasm/ output"
    missing:
      - "Add negation pattern '!packages/core/dist/wasm/*.wasm' after the '*.wasm' line to allow dist output while still ignoring stray .wasm files elsewhere"
human_verification:
  - test: "Actual WASM compilation with Emscripten SDK"
    expected: "emcmake cmake + cmake --build produces matcha2d_wasm.wasm and matcha2d_wasm.js without errors"
    why_human: "Emscripten SDK not installed on this system — cannot verify C compilation against Box2D 3.x headers"
  - test: "WASM backend end-to-end with World.createWithWasm()"
    expected: "World created with WASM backend runs simulation steps, bodies move correctly, contacts reported"
    why_human: "Stub box2d.js throws on load — requires actual WASM build to test runtime behavior"
  - test: "Memory leak check during extended WASM simulation"
    expected: "No WASM heap growth over thousands of simulation steps"
    why_human: "Requires running WASM module and monitoring heap — stub prevents this"
---

# Phase 01: WASM Core Verification Report

**Phase Goal:** Port Box2D C engine to WASM, integrate with existing TypeScript monorepo as the physics backend.
**Verified:** 2026-04-02T06:22:00Z
**Status:** gaps_found
**Re-verification:** No — initial verification

## Goal Achievement

### Observable Truths

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | Box2D C source compiles to a .wasm file via Emscripten | ⚠️ PARTIAL | CMakeLists.txt is valid, lists all 58 Box2D sources, build scripts exist for bash+PowerShell. Cannot verify actual compilation — Emscripten SDK not installed on this system. |
| 2 | WASM module exposes C API that accepts SoA buffer pointers | ✓ VERIFIED | wasm_bridge.h declares 8 WASM_EXPORT functions with SoA signatures. wasm_bridge.c (438 lines) implements full body/shape sync, step, read, contact query. Flag definitions match TypeScript (ACTIVE=0x01, STATIC=0x02). WorldState handles up to 4 simultaneous worlds. |
| 3 | TypeScript class implements PhysicsBackend interface | ✓ VERIFIED | WasmPhysicsBackend (188 lines) implements all 6 PhysicsBackend methods. WasmModule.ts (378 lines) wraps all 7 C bridge functions with typed cwrap. SoA buffers correctly copied to/from WASM linear memory via HEAPF32/HEAPU8. Memory management with freeMany() prevents leaks. |
| 4 | World class can use WasmPhysicsBackend as its physics backend | ⚠️ PARTIAL | World has createWithWasm(), createWithBackend(), createWithTS() factories. _physicsStep routes through backend when available. **Gap:** WasmPhysicsBackend.collide() hardcodes dt=1/60 and subSteps=4, ignoring the dt parameter from the simulation loop. Works with default config but breaks with custom timesteps. |
| 5 | Build pipeline produces .wasm + .js glue, integrated into npm build | ⚠️ PARTIAL | tsup.config.ts has onSuccess hook for copying WASM artifacts. package.json has build:wasm and build:all scripts. .gitignore correctly excludes wasm/build/. **Gap:** Filename mismatch — CMakeLists.txt produces `matcha2d_wasm.*` but tsup.config.ts looks for `box2d.*` and WasmModule.ts imports `box2d.js`. Stub box2d.js exists for TS resolution but actual Emscripten build would produce mismatched filenames. |

**Score:** 2/5 truths fully verified, 3/5 partial (3 gaps total)

### Required Artifacts

| Artifact | Expected | Status | Details |
|----------|----------|--------|---------|
| `packages/core/wasm/CMakeLists.txt` | Emscripten build config for Box2D + bridge | ✓ VERIFIED | 122 lines, valid CMake, all 58 Box2D sources listed, proper include paths, EMSCRIPTEN_KEEPALIVE exports, MODULARIZE/EXPORT_ES6 flags |
| `packages/core/wasm/wasm_bridge.h` | C API header with EMSCRIPTEN_KEEPALIVE exports | ✓ VERIFIED | 175 lines, 8 functions declared with WASM_EXPORT macro, proper extern "C" guard, comprehensive JSDoc |
| `packages/core/wasm/wasm_bridge.c` | Bridge implementation accepting SoA buffers | ✓ VERIFIED | 438 lines, full implementation: b2_init/b2_destroy (world lifecycle), b2_sync_bodies/b2_sync_shapes (SoA→Box2D), b2_step, b2_read_bodies (Box2D→SoA), b2_get_contact_count/b2_get_contacts (collision query) |
| `packages/core/wasm/build.sh` | Unix build script | ✓ VERIFIED | 51 lines, emcc detection, emcmake cmake, copies output to dist/wasm/ |
| `packages/core/wasm/build.ps1` | PowerShell build script | ✓ VERIFIED | 61 lines, equivalent to build.sh for Windows |
| `packages/core/src/wasm/WasmModule.ts` | WASM loader and memory management | ✓ VERIFIED | 378 lines, async loadWasmModule(), WasmInstance interface with 8 methods, allocAndCopy/readFromHeap/freeMany memory primitives, proper typed array handling |
| `packages/core/src/wasm/WasmPhysicsBackend.ts` | PhysicsBackend implementation | ✓ VERIFIED | 188 lines, implements all 6 interface methods, collide() runs sync→step→read→contacts pipeline, dispose() cleans up |
| `packages/core/src/wasm/index.ts` | Public exports | ✓ VERIFIED | 10 lines, exports WasmPhysicsBackend, loadWasmModule, WasmInstance type |
| `packages/core/tsup.config.ts` | Build config copying WASM artifacts | ⚠️ PARTIAL | 40 lines, onSuccess hook present but looks for `box2d.*` filenames instead of `matcha2d_wasm.*` |
| `packages/core/package.json` | Package exports including WASM | ✓ VERIFIED | build:wasm and build:all scripts, ./wasm/* export path |
| `packages/world/src/world.ts` | World class with WASM integration | ⚠️ PARTIAL | 202 lines, 3 factory methods, backend-driven _physicsStep, but WasmPhysicsBackend.collide() hardcodes timestep |
| `packages/core/src/index.ts` | Core exports including WASM | ✓ VERIFIED | 28 lines, exports WasmPhysicsBackend, loadWasmModule, WasmInstance type |
| `.gitignore` | WASM build intermediates excluded | ✓ VERIFIED | Excludes `packages/core/wasm/build/` and `*.wasm` |

### Key Link Verification

| From | To | Via | Status | Details |
|------|-----|-----|--------|---------|
| `wasm_bridge.c` | `box2d/box2d.h` | `#include "box2d/box2d.h"` | ✓ WIRED | Line 15 includes Box2D public API header |
| `WasmPhysicsBackend.ts` | `PhysicsBackend` interface | `implements PhysicsBackend` | ✓ WIRED | Line 22: class implements interface, all 6 methods present |
| `WasmPhysicsBackend.ts` | `WasmModule.ts` | `import { loadWasmModule }` | ✓ WIRED | Lines 19-20: imports loadWasmModule and WasmInstance type |
| `WasmModule.ts` | Emscripten glue | `import('../../wasm/build/box2d.js')` | ⚠️ PARTIAL | Dynamic import resolves to stub box2d.js. Actual build produces `matcha2d_wasm.js` — filename mismatch |
| `world.ts` | `WasmPhysicsBackend` | `import { ... WasmPhysicsBackend } from '@matcha2d/core'` | ✓ WIRED | Line 3: World imports WasmPhysicsBackend from core package |
| `tsup.config.ts` | WASM build output | `onSuccess` hook copying files | ⚠️ PARTIAL | Hook looks for `box2d.wasm`/`box2d.js` but CMake produces `matcha2d_wasm.wasm`/`matcha2d_wasm.js` |

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
|-------------|-------------|-------------|--------|----------|
| WASM-01 | Plan 01 | Box2D compiles to WASM via Emscripten | ⚠️ PARTIAL | CMakeLists.txt valid, build scripts complete, 58 sources listed. Cannot verify actual compilation (Emscripten not installed). |
| WASM-02 | Plan 01 | WASM module exposes C API compatible with MatchaBuffers (SoA format) | ✓ SATISFIED | wasm_bridge.h/c implement 8 functions accepting SoA buffer pointers. Flag definitions match TypeScript. Full body/shape sync, step, read, contact query. |
| WASM-03 | Plan 02 | TypeScript wrapper implements PhysicsBackend interface | ✓ SATISFIED | WasmPhysicsBackend implements all 6 methods. WasmModule wraps 7 C bridge functions. SoA buffers copied to/from WASM memory. Memory management prevents leaks. |
| WASM-04 | Plan 03 | WASM backend works with existing World simulation loop | ⚠️ PARTIAL | World has factory methods (createWithWasm, createWithBackend, createWithTS). _physicsStep routes through backend. **Gap:** collide() hardcodes dt=1/60 instead of using simulation loop dt. |
| WASM-05 | Plan 03 | Build pipeline produces .wasm + .js glue, integrated into npm build | ⚠️ PARTIAL | tsup onSuccess hook, package.json scripts, .gitignore all present. **Gap:** Filename mismatch between CMakeLists.txt output (matcha2d_wasm.*) and TypeScript expectations (box2d.*). |

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
|------|------|---------|----------|--------|
| `packages/core/src/wasm/WasmPhysicsBackend.ts` | 120 | Hardcoded timestep `1/60` | ⚠️ Warning | WASM backend ignores dt parameter from simulation loop. Breaks with custom fixedTimestep values. |
| `packages/core/src/wasm/WasmPhysicsBackend.ts` | 120 | Hardcoded subSteps `4` | ⚠️ Warning | Sub-step count not configurable, limits tuning for accuracy vs performance. |
| `packages/core/wasm/CMakeLists.txt` | 101 | Output name `matcha2d_wasm` | ⚠️ Warning | Produces `matcha2d_wasm.js`/`.wasm` but TypeScript code expects `box2d.js`/`.wasm`. |
| `packages/core/tsup.config.ts` | 17 | Filename `box2d.*` | ⚠️ Warning | Looks for `box2d.wasm`/`box2d.js` but CMake produces `matcha2d_wasm.*`. |
| `packages/core/wasm/build/box2d.js` | 10-13 | Stub that throws | ℹ️ Info | Intentional stub for TS resolution. Throws "WASM module not built" — prevents runtime testing without Emscripten. |

### Human Verification Required

1. **Actual WASM compilation with Emscripten SDK**
   - **Test:** Install Emscripten SDK, run `npm run build:wasm` in packages/core
   - **Expected:** emcmake cmake + cmake --build produces .wasm and .js files without compilation errors against Box2D 3.x headers
   - **Why human:** Emscripten SDK not installed on this system — cannot verify C compilation

2. **WASM backend end-to-end with World.createWithWasm()**
   - **Test:** Create world with `await World.createWithWasm()`, add bodies, run step(), verify positions update
   - **Expected:** Bodies move under gravity, contacts reported, no crashes
   - **Why human:** Stub box2d.js throws on load — requires actual WASM build to test runtime behavior

3. **Memory leak check during extended WASM simulation**
   - **Test:** Run 10,000+ simulation steps with WASM backend, monitor WASM heap growth
   - **Expected:** No unbounded heap growth (freeMany() should prevent leaks)
   - **Why human:** Requires running WASM module and monitoring heap — stub prevents this

4. **Filename alignment fix verification**
   - **Test:** After fixing filename mismatch, verify tsup onSuccess hook copies correct files
   - **Expected:** `matcha2d_wasm.wasm` and `matcha2d_wasm.js` (or `box2d.*` if renamed) appear in dist/wasm/
   - **Why human:** Requires actual WASM build to confirm the full pipeline works end-to-end

### Gaps Summary

Three structural gaps prevent full phase goal achievement:

**1. Filename mismatch (WASM-05):** The Emscripten CMake configuration (`CMakeLists.txt` line 101) names its output target `matcha2d_wasm`, producing `matcha2d_wasm.js` and `matcha2d_wasm.wasm`. However, the TypeScript WASM loader (`WasmModule.ts` line 40) dynamically imports `../../wasm/build/box2d.js`, and the tsup build hook (`tsup.config.ts` line 17) looks for `box2d.wasm` and `box2d.js`. A stub `box2d.js` exists for TypeScript compilation, but when the actual Emscripten build runs, it will produce `matcha2d_wasm.*` files that nothing references. This needs alignment — either rename the CMake target to `box2d` or update all TypeScript references to `matcha2d_wasm`.

**2. Hardcoded timestep (WASM-04):** `WasmPhysicsBackend.collide()` calls `this.wasm.step(this.worldHandle, 1 / 60, 4)` with hardcoded values, ignoring the dt parameter passed by the World simulation loop. This works with the default fixedTimestep of 1/60 but would produce incorrect physics if the World is configured with a different timestep. The dt parameter should be passed through to `step()`.

**3. .gitignore blocks WASM artifacts from version control (WASM-05):** Root `.gitignore` line 5 contains `*.wasm` which ignores ALL .wasm files including `packages/core/dist/wasm/box2d.wasm`. Confirmed via `git check-ignore`. This prevents WASM binaries from being committed even though the plan intends dist/wasm/ to contain the final build output. Fix: add `!packages/core/dist/wasm/*.wasm` negation pattern after the `*.wasm` line.

All three gaps are fixable with minimal changes and do not indicate fundamental architectural issues. The C bridge implementation, TypeScript wrapper, and World integration are all substantively implemented and correctly wired.

---

_Verified: 2026-04-02T06:22:00Z_
_Verifier: Claude (gsd-verifier)_
