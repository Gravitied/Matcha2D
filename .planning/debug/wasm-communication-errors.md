---
status: awaiting_human_verify
trigger: "collision.html:169 WASM backend initialized successfully + Module._malloc is not a function"
created: "2026-04-02T09:06:00.000Z"
updated: "2026-04-02T09:30:00.000Z"
---

## Current Focus

hypothesis: All three issues fixed: (1) WASM path regex, (2) Race condition guard, (3) _malloc/_free missing from exported functions
test: verify _malloc and _free are now exported in WASM module
expecting: WASM backend fully works, no more _malloc errors
next_action: await human verification in browser

## Symptoms

expected: WASM backend loads and collision detection works
actual: WASM returns 404, collision.html crashes on undefined.shape, then _malloc is not a function
errors:
  - "Cannot read properties of undefined (reading 'shape')" at collision.html:364
  - "Failed to load resource: the server responded with a status of 404" for box2d.wasm
  - "wasm streaming compile failed"
  - "Aborted(both async and sync fetching of the wasm failed)"
  - "Module._malloc is not a function" at WasmModule.ts:127
reproduction: Open collision.html, select WASM backend
started: Current session

## Eliminated

- hypothesis: WASM build artifacts missing
  evidence: box2d.wasm and box2d.js exist in packages/core/dist/wasm/
  timestamp: "2026-04-02T09:12:00.000Z"

- hypothesis: WASM file path regex works correctly
  evidence: Regex /\/dist\/[^/]+$/ doesn't match /dist/ at end of URL, causing wrong path
  timestamp: "2026-04-02T09:11:00.000Z"

- hypothesis: _malloc and _free are exported by default
  evidence: Emscripten 5.x requires explicit export via EXPORTED_FUNCTIONS, not EXPORTED_RUNTIME_METHODS
  timestamp: "2026-04-02T09:25:00.000Z"

## Evidence

- timestamp: "2026-04-02T09:10:00.000Z"
  checked: WasmModule.ts line 42 regex
  found: /\/dist\/[^/]+$/ doesn't match URLs ending with /dist/ (no trailing path component)
  implication: wasmDir becomes .../dist/ instead of .../dist/wasm/, causing 404 for box2d.wasm

- timestamp: "2026-04-02T09:11:00.000Z"
  checked: Built file locations
  found: box2d.js and box2d.wasm are in packages/core/dist/wasm/
  implication: wasmDir should point to .../dist/wasm/

- timestamp: "2026-04-02T09:12:00.000Z"
  checked: collision.html error line 364
  found: bodyData[i] is undefined when accessing b.shape
  implication: bodyData array not populated, likely due to init() failing before loop completes

- timestamp: "2026-04-02T09:13:00.000Z"
  checked: WasmModule.ts source code
  found: Source already has correct regex /\/dist\/?$/ but built file had old regex
  implication: Source was fixed but not rebuilt

- timestamp: "2026-04-02T09:14:00.000Z"
  checked: packages/core/dist/index.js after rebuild
  found: wasmDir regex now correct: /\/dist\/?$/
  implication: WASM file path should now resolve correctly

- timestamp: "2026-04-02T09:15:00.000Z"
  checked: collision.html animation loop timing
  found: loop() runs via requestAnimationFrame independently of init() completion
  implication: step() and render() can be called while bodyData is empty during async init

- timestamp: "2026-04-02T09:16:00.000Z"
  checked: collision.html resetAll() function
  found: resetAll() calls await init() but animation loop continues during await
  implication: Race condition between init() and step()/render()

- timestamp: "2026-04-02T09:17:00.000Z"
  checked: npm test results
  found: All 125 tests pass across 6 test files
  implication: Fixes don't break existing functionality

- timestamp: "2026-04-02T09:22:00.000Z"
  checked: box2d.js exports
  found: _malloc and _free not found in exported symbols
  implication: WasmModule.ts calls Module._malloc which doesn't exist

- timestamp: "2026-04-02T09:23:00.000Z"
  checked: build.ps1 EXPORTED_RUNTIME_METHODS
  found: Only exports ccall, cwrap, HEAPF32, HEAPU8, HEAP32 — not malloc/free
  implication: Need to add _malloc and _free to EXPORTED_FUNCTIONS

- timestamp: "2026-04-02T09:24:00.000Z"
  checked: Emscripten 5.x export behavior
  found: malloc/free must be in EXPORTED_FUNCTIONS (with underscore prefix), not EXPORTED_RUNTIME_METHODS
  implication: Updated build.ps1 and CMakeLists.txt to include _malloc, _free in EXPORTED_FUNCTIONS

- timestamp: "2026-04-02T09:25:00.000Z"
  checked: Rebuilt WASM module
  found: box2d.wasm is now 30KB (was 8 bytes), _malloc and _free are exported
  implication: WASM module now has all required functions

- timestamp: "2026-04-02T09:26:00.000Z"
  checked: build.ps1 copy destination
  found: Script copied to dist/box2d.wasm instead of dist/wasm/box2d.wasm
  implication: Updated build.ps1 to copy to dist/wasm/ directory

- timestamp: "2026-04-02T09:27:00.000Z"
  checked: dist/wasm/ files after rebuild
  found: box2d.wasm = 30,630 bytes, box2d.js = 10,723 bytes with _malloc/_free exported
  implication: All WASM files correctly built and in place

## Resolution

root_cause: Three issues: (1) WasmModule.ts regex /\/dist\/[^/]+$/ doesn't match URLs ending with /dist/, causing 404 for box2d.wasm. Source was fixed but not rebuilt. (2) Race condition in collision.html where animation loop calls step()/render() while init() is still running, causing bodyData[i] to be undefined. (3) Emscripten build didn't export _malloc/_free functions — they were missing from EXPORTED_FUNCTIONS, causing "Module._malloc is not a function" error.
fix: 
  1. Rebuilt packages/core to apply fixed regex /\/dist\/?$/
  2. Added isInitializing flag to collision.html to prevent step/render during init
  3. Added guard to render() to skip drawing when bodyData is empty
  4. Added _malloc and _free to EXPORTED_FUNCTIONS in build.ps1 and CMakeLists.txt
  5. Rebuilt WASM module with emcc (30KB output with all exports)
  6. Fixed build.ps1 to copy WASM files to dist/wasm/ instead of dist/
verification: WASM module rebuilt with _malloc/_free exports, dist/wasm/ contains correct files. Awaiting browser testing.
files_changed:
  - packages/core/dist/index.js (rebuilt with fixed regex)
  - packages/core/dist/index.js.map (rebuilt)
  - packages/core/dist/index.d.ts (rebuilt)
  - packages/core/dist/wasm/box2d.js (rebuilt with _malloc/_free exports)
  - packages/core/dist/wasm/box2d.wasm (rebuilt, 30KB)
  - packages/core/wasm/build/box2d.js (rebuilt)
  - packages/core/wasm/build/box2d.wasm (rebuilt, 30KB)
  - packages/core/wasm/build.ps1 (added _malloc/_free to exports, fixed copy destination)
  - packages/core/wasm/CMakeLists.txt (added _malloc/_free to exports)
  - demo/collision.html (added isInitializing guard, step/render guards)

## Symptoms

expected: WASM backend loads and collision detection works
actual: WASM returns 404, collision.html crashes on undefined.shape
errors:
  - "Cannot read properties of undefined (reading 'shape')" at collision.html:364
  - "Failed to load resource: the server responded with a status of 404" for box2d.wasm
  - "wasm streaming compile failed"
  - "Aborted(both async and sync fetching of the wasm failed)"
reproduction: Open collision.html, select WASM backend
started: Current session

## Eliminated

- hypothesis: WASM build artifacts missing
  evidence: box2d.wasm and box2d.js exist in packages/core/dist/wasm/
  timestamp: "2026-04-02T09:12:00.000Z"

- hypothesis: WASM file path regex works correctly
  evidence: Regex /\/dist\/[^/]+$/ doesn't match /dist/ at end of URL, causing wrong path
  timestamp: "2026-04-02T09:11:00.000Z"

## Evidence

- timestamp: "2026-04-02T09:10:00.000Z"
  checked: WasmModule.ts line 42 regex
  found: /\/dist\/[^/]+$/ doesn't match URLs ending with /dist/ (no trailing path component)
  implication: wasmDir becomes .../dist/ instead of .../dist/wasm/, causing 404 for box2d.wasm

- timestamp: "2026-04-02T09:11:00.000Z"
  checked: Built file locations
  found: box2d.js and box2d.wasm are in packages/core/dist/wasm/
  implication: wasmDir should point to .../dist/wasm/

- timestamp: "2026-04-02T09:12:00.000Z"
  checked: collision.html error line 364
  found: bodyData[i] is undefined when accessing b.shape
  implication: bodyData array not populated, likely due to init() failing before loop completes

- timestamp: "2026-04-02T09:13:00.000Z"
  checked: WasmModule.ts source code
  found: Source already has correct regex /\/dist\/?$/ but built file had old regex
  implication: Source was fixed but not rebuilt

- timestamp: "2026-04-02T09:14:00.000Z"
  checked: packages/core/dist/index.js after rebuild
  found: wasmDir regex now correct: /\/dist\/?$/
  implication: WASM file path should now resolve correctly

- timestamp: "2026-04-02T09:15:00.000Z"
  checked: collision.html animation loop timing
  found: loop() runs via requestAnimationFrame independently of init() completion
  implication: step() and render() can be called while bodyData is empty during async init

- timestamp: "2026-04-02T09:16:00.000Z"
  checked: collision.html resetAll() function
  found: resetAll() calls await init() but animation loop continues during await
  implication: Race condition between init() and step()/render()

- timestamp: "2026-04-02T09:17:00.000Z"
  checked: npm test results
  found: All 125 tests pass across 6 test files
  implication: Fixes don't break existing functionality

## Resolution

root_cause: Two issues: (1) WasmModule.ts regex /\/dist\/[^/]+$/ doesn't match URLs ending with /dist/, causing 404 for box2d.wasm. Source was fixed but not rebuilt. (2) Race condition in collision.html where animation loop calls step()/render() while init() is still running, causing bodyData[i] to be undefined.
fix: 
  1. Rebuilt packages/core to apply fixed regex /\/dist\/?$/
  2. Added isInitializing flag to collision.html to prevent step/render during init
  3. Added guard to render() to skip drawing when bodyData is empty
verification: All 125 tests pass. Code changes verified. Awaiting browser testing.
files_changed:
  - packages/core/dist/index.js (rebuilt with fixed regex)
  - packages/core/dist/index.js.map (rebuilt)
  - packages/core/dist/index.d.ts (rebuilt)
  - demo/collision.html (added isInitializing guard, step/render guards)
