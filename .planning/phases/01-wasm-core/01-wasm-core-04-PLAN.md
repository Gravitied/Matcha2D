---
phase: 01-wasm-core
plan: 04
type: execute
wave: 1
depends_on: []
files_modified:
  - packages/core/wasm/CMakeLists.txt
  - packages/core/tsup.config.ts
  - packages/core/src/wasm/WasmModule.ts
  - packages/core/wasm/build.sh
  - packages/core/wasm/build.ps1
  - packages/core/src/wasm/WasmPhysicsBackend.ts
  - .gitignore
autonomous: true
requirements:
  - WASM-04
  - WASM-05
gap_closure: true
must_haves:
  truths:
    - "Build pipeline produces .wasm + .js files with matching filenames across CMake and TypeScript"
    - "WASM backend uses dt parameter from simulation loop instead of hardcoded value"
    - "WASM build, subSteps configurable"
    - "dist/wasm/*.wasm files not ignored by .gitignore"
  artifacts:
    - path: "packages/core/wasm/CMakeLists.txt"
      provides: "Emscripten build config with consistent output name"
      contains: "box2d"
    - path: "packages/core/tsup.config.ts"
      provides: "Build config looking for correct WASM filenames"
      contains: "box2d"
    - path: "packages/core/src/wasm/WasmModule.ts"
      provides: "WASM loader importing correct filename"
      contains: "box2d.js"
    - path: "packages/core/src/wasm/WasmPhysicsBackend.ts"
      provides: "Backend using dt parameter from collide() call"
      pattern: "this\\.wasm\\.step.*dt"
    - path: ".gitignore"
      provides: "Negation pattern for dist/wasm/*.wasm"
      contains: "!packages/core/dist/wasm/*.wasm"
  key_links:
    - from: "packages/core/wasm/CMakeLists.txt"
      to: "packages/core/tsup.config.ts"
      via: "matching output filenames"
      pattern: "box2d\\.(wasm|js)"
    - from: "packages/core/src/wasm/WasmPhysicsBackend.ts"
      to: "packages/world/src/world.ts"
      via: "dt parameter from simulation loop"
      pattern: "collide.*dt"
---

<objective>
Close three verification gaps: filename mismatch between CMake and TypeScript, hardcoded timestep in WASM backend, and .gitignore blocking WASM artifacts from version control.

Purpose: Ensure the build pipeline produces consistently-named WASM artifacts, the WASM backend respects simulation timestep configuration, and dist output can be committed to version control.

Output: Fixed CMakeLists.txt, tsup.config.ts, WasmModule.ts, WasmPhysicsBackend.ts, and .gitignore
</objective>

<execution_context>
@C:/Users/winsi/.config/opencode/get-shit-done/workflows/execute-plan.md
@C:/Users/winsi/.config/opencode/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@.planning/phases/01-wasm-core/01-wasm-core-VERIFICATION.md
@.planning/phases/01-wasm-core/01-wasm-core-01-SUMMARY.md
@.planning/phases/01-wasm-core/01-wasm-core-02-SUMMARY.md
@.planning/phases/01-wasm-core/01-wasm-core-03-SUMMARY.md

# Gap 1: Filename mismatch
# CMakeLists.txt line 101: output target named "matcha2d_wasm" -> produces matcha2d_wasm.js + matcha2d_wasm.wasm
# tsup.config.ts line 17: looks for box2d.wasm and box2d.js
# WasmModule.ts line 40: imports ../../wasm/build/box2d.js
# build.sh lines 40-47: copies matcha2d_wasm.* to dist/
# Fix: Rename CMake output to "box2d" to match TypeScript expectations

# Gap 2: Hardcoded timestep
# WasmPhysicsBackend.collide() line 120: this.wasm.step(this.worldHandle, 1 / 60, 4)
# Should use dt parameter from collide() signature instead of hardcoding 1/60
# subSteps=4 should be configurable

# Gap 3: .gitignore blocks WASM artifacts
# Root .gitignore line 5: "*.wasm" ignores ALL .wasm files including dist/wasm/box2d.wasm
# Fix: Add negation pattern "!packages/core/dist/wasm/*.wasm" after the *.wasm line
</context>

<tasks>

<task type="auto">
  <name>Task 1: Align WASM filenames across build pipeline</name>
  <files>packages/core/wasm/CMakeLists.txt, packages/core/wasm/build.sh, packages/core/wasm/build.ps1, packages/core/tsup.config.ts, packages/core/src/wasm/WasmModule.ts</files>
  <action>
    Fix the filename mismatch so CMake output matches TypeScript expectations:

    1. **CMakeLists.txt** (line ~101): Change output target name from `matcha2d_wasm` to `box2d`
       - Find: `set_target_properties(matcha2d_wasm ...)` or `add_executable(matcha2d_wasm ...)`
       - Replace target name with `box2d`
       - This will produce `box2d.js` and `box2d.wasm` matching TypeScript expectations

    2. **build.sh** (lines ~40-47): Update copy commands to use `box2d.*` instead of `matcha2d_wasm.*`
       - Change any references from `matcha2d_wasm.wasm`/`matcha2d_wasm.js` to `box2d.wasm`/`box2d.js`

    3. **build.ps1**: Same updates as build.sh for PowerShell version

    4. **tsup.config.ts** (line ~17): Verify it looks for `box2d.wasm` and `box2d.js` (should already be correct)

    5. **WasmModule.ts** (line ~40): Verify dynamic import path is `../../wasm/build/box2d.js` (should already be correct)

    Reference existing code from SUMMARIES: CMakeLists.txt uses add_executable with .js suffix for Emscripten output. Build scripts copy from wasm/build/ to dist/wasm/.

    Gap reason: Filename mismatch prevents WASM build artifacts from being found by TypeScript tooling.
  </action>
  <verify>
    <automated>grep -q "box2d" packages/core/wasm/CMakeLists.txt && grep -q "box2d" packages/core/wasm/build.sh && grep -q "box2d" packages/core/tsup.config.ts && grep -q "box2d.js" packages/core/src/wasm/WasmModule.ts</automated>
  </verify>
  <done>All files reference consistent "box2d" filename for WASM output, no references to "matcha2d_wasm" remain in build pipeline files</done>
</task>

<task type="auto">
  <name>Task 2: Fix hardcoded timestep and .gitignore WASM exclusion</name>
  <files>packages/core/src/wasm/WasmPhysicsBackend.ts, .gitignore</files>
  <action>
    Fix two remaining gaps:

    **Gap 2 - Hardcoded timestep in WasmPhysicsBackend:**

    1. **WasmPhysicsBackend.ts** (line ~120): Replace hardcoded values with parameters
       - Current: `this.wasm.step(this.worldHandle, 1 / 60, 4)`
       - Change to: `this.wasm.step(this.worldHandle, dt, this.subSteps)`
       - Add `subSteps` as a private field with default value 4
       - Make it configurable via constructor or init() method
       - The `dt` parameter should come from the `collide()` method signature

    2. Update `collide()` method to accept and pass through the dt parameter:
       - Current signature: `collide(buffers, count, ...)`
       - The dt should be available from World simulation loop context
       - If collide() doesn't receive dt, add it as a parameter or store it as instance state during init()

    Reference from SUMMARY: collide() runs syncBodies → syncShapes → step → readBodies → getContacts pipeline. The step() call needs dt from simulation loop.

    Gap reason: WASM backend ignores dt parameter, breaks with custom timesteps.

    **Gap 3 - .gitignore blocking WASM artifacts:**

    3. **.gitignore** (line ~5): Add negation pattern after `*.wasm`
       - Add: `!packages/core/dist/wasm/*.wasm`
       - This allows dist output while still ignoring stray .wasm files elsewhere
       - Place immediately after the `*.wasm` line so git processes the negation correctly

    Gap reason: *.wasm pattern prevents WASM binaries from being committed to version control.
  </action>
  <verify>
    <automated>grep -q "this.wasm.step.*dt" packages/core/src/wasm/WasmPhysicsBackend.ts && grep -q "!packages/core/dist/wasm/\*.wasm" .gitignore</automated>
  </verify>
  <done>WasmPhysicsBackend.step() receives dt parameter instead of hardcoded 1/60, subSteps is configurable, .gitignore has negation pattern for dist/wasm/*.wasm</done>
</task>

</tasks>

<verification>
- CMakeLists.txt output target named "box2d" (not "matcha2d_wasm")
- Build scripts copy box2d.wasm and box2d.js to dist/wasm/
- tsup.config.ts looks for box2d.* files
- WasmModule.ts imports box2d.js
- WasmPhysicsBackend.step() receives dt parameter from collide()
- subSteps configurable via constructor or init()
- .gitignore allows dist/wasm/*.wasm while ignoring other .wasm files
- TypeScript compilation passes for all modified files
</verification>

<success_criteria>
- All three verification gaps closed
- Filename consistency across entire build pipeline (CMake → build scripts → tsup → TypeScript)
- WASM backend respects simulation timestep configuration
- WASM artifacts can be committed to version control
- No TypeScript compilation errors
</success_criteria>

<output>
After completion, create `.planning/phases/01-wasm-core/01-wasm-core-04-SUMMARY.md`
</output>