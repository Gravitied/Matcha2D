---
phase: 01-wasm-core
plan: 03
type: execute
wave: 3
depends_on: ["01-wasm-core-02"]
files_modified:
  - packages/core/package.json
  - packages/core/tsup.config.ts
  - packages/world/src/World.ts
  - packages/core/src/index.ts
  - .gitignore
autonomous: true
requirements:
  - WASM-04
  - WASM-05
must_haves:
  truths:
    - "npm run build produces .wasm + .js files in packages/core/dist/wasm/"
    - "World class can use WasmPhysicsBackend as its physics backend"
    - "Existing TypeScript code compiles without errors after integration"
  artifacts:
    - path: "packages/core/tsup.config.ts"
      provides: "Build config that copies WASM artifacts to dist/"
      contains: "onSuccess|copy"
    - path: "packages/world/src/World.ts"
      provides: "World class updated to accept WasmPhysicsBackend"
      contains: "WasmPhysicsBackend|PhysicsBackend"
    - path: "packages/core/package.json"
      provides: "Package exports including WASM files"
      contains: "wasm"
  key_links:
    - from: "packages/world/src/World.ts"
      to: "packages/core/src/wasm/WasmPhysicsBackend.ts"
      via: "import"
      pattern: "import.*WasmPhysicsBackend"
    - from: "packages/core/tsup.config.ts"
      to: "packages/core/wasm/build/"
      via: "copy WASM output"
      pattern: "copy|onSuccess"
---

<objective>
Integrate WASM build artifacts into the npm build pipeline and wire WasmPhysicsBackend into the World simulation loop.

Purpose: The WASM module and TypeScript wrapper from Plans 01-02 need to be part of the normal build flow. The World class needs to use the WASM backend. Build artifacts (.wasm + .js) must be copied to dist/ automatically.

Output: Updated build configs, World integration, package.json exports.
</objective>

<execution_context>
@C:/Users/winsi/.config/opencode/get-shit-done/workflows/execute-plan.md
@C:/Users/winsi/.config/opencode/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@packages/core/package.json
@packages/core/tsup.config.ts
@packages/world/src/World.ts
@packages/world/package.json
@package.json
@.gitignore

# Current build order (from root package.json):
# 1. npm run build -w packages/types
# 2. npm run build -w packages/core
# 3. npm run build --workspaces --if-present (world, render, tools)

# World dependency chain:
# types -> core -> world
# The World class currently uses the TS PhysicsBackend directly.
# It needs to be updated to optionally use WasmPhysicsBackend.

# tsup.config.ts currently:
# - Builds ESM output with .d.ts generation and sourcemaps
# - Entry: src/index.ts
# - Output: dist/
</context>

<interfaces>
<!-- Key types and contracts the executor needs. Extracted from codebase. -->

From packages/core/src/index.ts:
```typescript
// Math exports
export { vec2Set, vec2Add, vec2Sub, vec2Scale, vec2Dot, vec2Cross, vec2LengthSq, vec2Length, vec2Normalize, vec2DistanceSq, dot, cross, lengthSq, length } from './math/vec2.js'
export { mat2FromAngle, mat2MulVec, mat2TransposeMulVec } from './math/mat2.js'
// Collision exports
export { aabbOverlap, aabbMerge, aabbContains, aabbArea, aabbPerimeter, computeBodyAABB } from './collision/aabb.js'
export { broadphase, broadphaseBVH, DynamicTree } from './collision/broadphase.js'
export { narrowphase } from './collision/narrowphase.js'
export { gjkNarrowphase } from './collision/gjk.js'
export { collide, narrowphaseDispatch } from './collision/pipeline.js'
export { registerShapeHandler, getShapeHandler } from './collision/shapes.js'
export { ContactTracker } from './collision/contact-tracker.js'
export { Simplex } from './collision/simplex.js'
export { Polytope } from './collision/polytope.js'
// Solver exports
export { solveVelocity, solvePosition, integrate } from './solver/sequential-impulse.js'
```

From packages/types/src/backend.ts:
```typescript
interface PhysicsBackend {
  broadphase(buffers: MatchaBuffers, count: number, method?: BroadphaseMethod): CollisionPair[]
  narrowphase(buffers: MatchaBuffers, pairs: CollisionPair[], method?: NarrowphaseMethod): ContactManifold[]
  collide(buffers: MatchaBuffers, count: number, broadphaseMethod?: BroadphaseMethod, narrowphaseMethod?: NarrowphaseMethod): ContactManifold[]
  solveVelocity(buffers: MatchaBuffers, manifolds: ContactManifold[], config: WorldConfig): void
  integrate(buffers: MatchaBuffers, count: number, dt: number, gravity: { x: number; y: number }): void
  solvePosition(buffers: MatchaBuffers, manifolds: ContactManifold[], config: WorldConfig): void
}
```
</interfaces>

<tasks>

<task type="auto">
  <name>Task 1: Wire WASM artifacts into build pipeline</name>
  <files>packages/core/tsup.config.ts, packages/core/package.json, .gitignore</files>
  <action>
    Update `packages/core/tsup.config.ts` to:
    1. Add an `onSuccess` hook (or use tsup's `publicDir` / copy plugin) that copies WASM build artifacts from `wasm/build/box2d.wasm` and `wasm/build/box2d.js` to `dist/wasm/` after the TypeScript build completes
    2. If the WASM build hasn't been run (files don't exist), print a warning but don't fail the build — the WASM is optional for development
    3. Add `wasm/` to the package exports so consumers can import WASM files:
       ```json
       "./wasm/*": "./dist/wasm/*"
       ```

    Update `packages/core/package.json` to:
    1. Add a `build:wasm` script: `bash wasm/build.sh` (or `powershell wasm/build.ps1` on Windows)
    2. Add a `build:all` script: `npm run build:wasm && npm run build`
    3. Add the WASM file exports in the "exports" field

    Update root `.gitignore` to:
    1. Ignore `packages/core/wasm/build/` directory (build artifacts)
    2. Ignore `*.wasm` in dist (or include them — decision: include .wasm in dist since they're build output, but ignore the intermediate build/ directory)
    3. Ensure `packages/core/wasm/build/` is gitignored but `packages/core/dist/wasm/` is not
  </action>
  <verify>
    <automated>grep -q "wasm" packages/core/tsup.config.ts && grep -q "build:wasm" packages/core/package.json && grep -q "wasm/build" .gitignore</automated>
  </verify>
  <done>tsup config copies WASM artifacts to dist/, package.json has build:wasm script, .gitignore excludes wasm/build/ but not dist/wasm/, WASM exports added to package.json</done>
</task>

<task type="auto">
  <name>Task 2: Integrate WasmPhysicsBackend into World class</name>
  <files>packages/world/src/World.ts, packages/core/src/index.ts</files>
  <action>
    Update `packages/world/src/World.ts`:
    1. Import `WasmPhysicsBackend` from `@matcha2d/core` (or the relative path)
    2. The World class should accept a `PhysicsBackend` in its constructor (dependency injection), defaulting to the existing TypeScript backend for backwards compatibility
    3. Add a convenience factory method or option: `World.createWithWasm(config?)` that creates a World using WasmPhysicsBackend
    4. If using WasmPhysicsBackend, call `await backend.init(config)` before the first simulation step
    5. The simulation loop should work identically regardless of backend — the PhysicsBackend interface is the contract

    Update `packages/core/src/index.ts`:
    1. Add exports for `WasmPhysicsBackend` and `loadWasmModule` from `./wasm/index.js`
    2. This makes them available to the World package via `@matcha2d/core`

    The World class simulation loop pattern:
    ```typescript
    // Current pattern (TS backend):
    const manifolds = this.backend.collide(this.buffers, count)
    this.backend.solveVelocity(this.buffers, manifolds, config)
    this.backend.integrate(this.buffers, count, dt, gravity)

    // With WASM backend, collide() does everything internally, so:
    // - collide() returns manifolds for event reporting
    // - solveVelocity/integrate are no-ops (already done in step)
    // This is handled inside WasmPhysicsBackend — the World doesn't need to know
    ```

    Keep the TypeScript backend as the default for now. WASM is opt-in.
  </action>
  <verify>
    <automated>npx tsc --noEmit -p packages/world/tsconfig.json 2>&1 | head -20</automated>
  </verify>
  <done>World class accepts PhysicsBackend via constructor, WasmPhysicsBackend is exported from @matcha2d/core, TypeScript compiles without errors for both core and world packages, existing tests still pass</done>
</task>

</tasks>

<verification>
- npm run build -w packages/types succeeds
- npm run build -w packages/core succeeds (with or without WASM artifacts present)
- TypeScript type-checking passes for all packages (npm run lint)
- World class can be instantiated with either TS or WASM backend
- WASM build artifacts are copied to dist/wasm/ when available
- .gitignore correctly excludes build intermediates but keeps dist output
</verification>

<success_criteria>
- Build pipeline produces .wasm + .js files in packages/core/dist/wasm/
- World class works with both TypeScript and WASM backends
- All TypeScript packages compile without errors
- Package exports correctly expose WASM-related types
- .gitignore is correctly configured
</success_criteria>

<output>
After completion, create `.planning/phases/01-wasm-core/01-wasm-core-03-SUMMARY.md`
</output>
