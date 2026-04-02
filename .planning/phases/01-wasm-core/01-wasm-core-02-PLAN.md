---
phase: 01-wasm-core
plan: 02
type: execute
wave: 2
depends_on: ["01-wasm-core-01"]
files_modified:
  - packages/core/src/wasm/WasmPhysicsBackend.ts
  - packages/core/src/wasm/WasmModule.ts
  - packages/core/src/wasm/index.ts
autonomous: true
requirements:
  - WASM-03
must_haves:
  truths:
    - "TypeScript class implements PhysicsBackend interface"
    - "WASM module loads asynchronously and exposes bridge functions"
    - "SoA buffers are passed to WASM via typed array views of WASM memory"
  artifacts:
    - path: "packages/core/src/wasm/WasmModule.ts"
      provides: "WASM loader and memory management"
      exports: ["loadWasmModule", "WasmInstance"]
    - path: "packages/core/src/wasm/WasmPhysicsBackend.ts"
      provides: "PhysicsBackend implementation wrapping WASM calls"
      exports: ["WasmPhysicsBackend"]
    - path: "packages/core/src/wasm/index.ts"
      provides: "Public exports for WASM backend"
      exports: ["WasmPhysicsBackend", "loadWasmModule"]
  key_links:
    - from: "packages/core/src/wasm/WasmPhysicsBackend.ts"
      to: "packages/types/src/backend.ts"
      via: "implements PhysicsBackend"
      pattern: "class.*implements.*PhysicsBackend"
    - from: "packages/core/src/wasm/WasmModule.ts"
      to: "packages/core/wasm/build.sh"
      via: "imports generated .js glue"
      pattern: "import.*from.*wasm.*js"
---

<objective>
Create TypeScript wrapper classes that load the WASM module and implement the PhysicsBackend interface, bridging SoA buffers to WASM function calls.

Purpose: The WASM module compiled in Plan 01 needs a TypeScript wrapper that the World class can use. This wrapper implements the existing PhysicsBackend interface, translating buffer data into WASM memory calls.

Output: WasmModule.ts (loader), WasmPhysicsBackend.ts (interface implementation), index.ts (exports).
</objective>

<execution_context>
@C:/Users/winsi/.config/opencode/get-shit-done/workflows/execute-plan.md
@C:/Users/winsi/.config/opencode/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@packages/types/src/backend.ts
@packages/types/src/buffers.ts
@packages/types/src/collision.ts
@packages/types/src/config.ts
@packages/core/src/wasm/README.md
@packages/core/src/solver/sequential-impulse.ts
@packages/core/src/collision/pipeline.ts

# PhysicsBackend interface to implement:
# broadphase(buffers, count, method?) -> CollisionPair[]
# narrowphase(buffers, pairs, method?) -> ContactManifold[]
# collide(buffers, count, broadphaseMethod?, narrowphaseMethod?) -> ContactManifold[]
# solveVelocity(buffers, manifolds, config) -> void
# integrate(buffers, count, dt, gravity) -> void
# solvePosition(buffers, manifolds, config) -> void

# WASM bridge functions (from Plan 01):
# b2_init(gravityX, gravityY, maxBodies) -> worldHandle
# b2_destroy(worldHandle)
# b2_sync_bodies(worldHandle, posX, posY, ...) — writes SoA to WASM memory
# b2_sync_shapes(worldHandle, posX, posY, ...) — writes shape data to WASM memory
# b2_step(worldHandle, dt, subSteps)
# b2_read_bodies(worldHandle, posX, posY, ...) — reads WASM state back to SoA
# b2_get_contact_count(worldHandle) -> int
# b2_get_contacts(worldHandle, bodyA, bodyB, nx, ny, px, py, penetration, maxContacts)

# Emscripten MODULARIZE=1 + EXPORT_ES6=1 produces a default export function
# that returns a Promise<Module>. The Module object has:
# - HEAPF32, HEAPU8 — typed array views into WASM linear memory
# - _b2_init, _b2_sync_bodies, etc. — the C functions (prefixed with _)
# - ccall/cwrap — convenient function wrappers
</context>

<interfaces>
<!-- Key types and contracts the executor needs. Extracted from codebase. -->

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

From packages/types/src/buffers.ts:
```typescript
interface MatchaBuffers {
  positionX: Float32Array; positionY: Float32Array
  velocityX: Float32Array; velocityY: Float32Array
  angle: Float32Array; angularVel: Float32Array
  mass: Float32Array; invMass: Float32Array
  inertia: Float32Array; invInertia: Float32Array
  flags: Uint8Array
  halfExtentX: Float32Array; halfExtentY: Float32Array
  shapeType: Uint8Array; shapeRadius: Float32Array
  shapeVertexCount: Uint8Array
  shapeVerticesX: Float32Array; shapeVerticesY: Float32Array
}
```

From packages/types/src/collision.ts:
```typescript
interface ContactPoint { localA: { x: number; y: number }; localB: { x: number; y: number }; penetration: number }
interface ContactManifold { bodyA: BodyHandle; bodyB: BodyHandle; normal: { x: number; y: number }; contacts: ContactPoint[] }
interface CollisionPair { a: BodyHandle; b: BodyHandle }
```
</interfaces>

<tasks>

<task type="auto">
  <name>Task 1: Create WASM module loader</name>
  <files>packages/core/src/wasm/WasmModule.ts</files>
  <action>
    Create `WasmModule.ts` that:
    1. Imports the Emscripten-generated JS glue module from `../../wasm/build/box2d.js` (relative to src/wasm/)
    2. Exports an async `loadWasmModule()` function that:
       - Calls the Emscripten module factory
       - Waits for WASM to be instantiated
       - Returns a `WasmInstance` object with typed methods wrapping the C bridge functions
    3. The `WasmInstance` interface wraps all 7 bridge functions with clean TypeScript signatures:
       ```typescript
       interface WasmInstance {
         init(gravityX: number, gravityY: number, maxBodies: number): number
         destroy(worldHandle: number): void
         syncBodies(worldHandle: number, buffers: MatchaBuffers, count: number): void
         syncShapes(worldHandle: number, buffers: MatchaBuffers, count: number): void
         step(worldHandle: number, dt: number, subSteps: number): void
         readBodies(worldHandle: number, buffers: MatchaBuffers, count: number): void
         getContacts(worldHandle: number): ContactManifold[]
         dispose(): void
       }
       ```
    4. For passing SoA buffers to WASM: allocate typed arrays in WASM memory (HEAPF32/HEAPU8), copy data from JS buffers using `.set()`, pass pointers (byteOffset) to C functions
    5. For reading back: read from WASM HEAP arrays and copy into JS buffers
    6. Handle memory cleanup in `dispose()` — free WASM-allocated arrays
    7. Use `Module.cwrap()` for clean function signatures rather than raw `_functionName` calls

    The glue module path should be configurable via an import path. Since the .js file is built into `packages/core/wasm/build/`, the TypeScript source at `packages/core/src/wasm/` imports it as `../../wasm/build/box2d.js`. Add a type declaration for the Emscripten module shape since it won't have .d.ts.
  </action>
  <verify>
    <automated>npx tsc --noEmit packages/core/src/wasm/WasmModule.ts 2>&1 | head -20</automated>
  </verify>
  <done>WasmModule.ts compiles without errors, exports loadWasmModule() returning WasmInstance with all 7 methods, memory management (allocate/copy/dispose) is implemented</done>
</task>

<task type="auto">
  <name>Task 2: Implement PhysicsBackend using WASM</name>
  <files>packages/core/src/wasm/WasmPhysicsBackend.ts, packages/core/src/wasm/index.ts</files>
  <action>
    Create `WasmPhysicsBackend.ts` that implements the `PhysicsBackend` interface:

    ```typescript
    export class WasmPhysicsBackend implements PhysicsBackend {
      private wasm: WasmInstance | null = null
      private worldHandle: number = 0
      private initialized = false

      async init(config: WorldConfig): Promise<void>
      broadphase(buffers, count, method?): CollisionPair[]
      narrowphase(buffers, pairs, method?): ContactManifold[]
      collide(buffers, count, broadMethod?, narrowMethod?): ContactManifold[]
      solveVelocity(buffers, manifolds, config): void
      integrate(buffers, count, dt, gravity): void
      solvePosition(buffers, manifolds, config): void
      dispose(): void
    }
    ```

    Implementation strategy — the WASM backend takes a different approach than the TS backend:
    - `init(config)`: loads WASM module, calls `b2_init(config.gravity.x, config.gravity.y, MAX_BODIES)`
    - `collide(buffers, count, ...)`: calls `b2_sync_bodies()` + `b2_sync_shapes()` to push buffer data into WASM, then `b2_step()`, then `b2_read_bodies()` to pull results back, then `b2_get_contacts()` to get contact manifolds
    - `broadphase()` / `narrowphase()`: For WASM mode, these are subsumed by `collide()` which runs the full pipeline. Return empty arrays if called directly (WASM handles collision internally).
    - `solveVelocity()` / `solvePosition()`: For WASM mode, these are handled inside `b2_step()`. No-op or call sync+read for consistency.
    - `integrate()`: For WASM mode, handled inside `b2_step()`. No-op.
    - `dispose()`: calls `b2_destroy()` and `wasm.dispose()`

    The key insight: Box2D's `b2World_Step` internally does broadphase → narrowphase → solve → integrate all in one call. So the WASM backend's `collide()` method is the primary entry point, and the individual methods are no-ops or thin wrappers.

    Create `index.ts` that exports `WasmPhysicsBackend` and `loadWasmModule`.

    Update `wasm/README.md` with actual build instructions now that the code exists.
  </action>
  <verify>
    <automated>npx tsc --noEmit packages/core/src/wasm/WasmPhysicsBackend.ts packages/core/src/wasm/index.ts 2>&1 | head -20</automated>
  </verify>
  <done>WasmPhysicsBackend implements all 6 PhysicsBackend methods, init() loads WASM and creates world, collide() runs full pipeline, dispose() cleans up, index.ts exports both classes, README.md updated</done>
</task>

</tasks>

<verification>
- WasmModule.ts compiles and correctly wraps all 7 C bridge functions
- WasmPhysicsBackend implements the full PhysicsBackend interface
- SoA buffers are correctly copied to/from WASM linear memory
- Memory is properly allocated and freed (no leaks)
- TypeScript type-checking passes for all new files
</verification>

<success_criteria>
- WasmModule.ts loads WASM module and provides typed wrapper methods
- WasmPhysicsBackend class implements PhysicsBackend interface with all 6 methods
- Buffer data flows correctly: JS SoA → WASM memory → Box2D → WASM memory → JS SoA
- No TypeScript compilation errors in new files
- index.ts provides clean public API
</success_criteria>

<output>
After completion, create `.planning/phases/01-wasm-core/01-wasm-core-02-SUMMARY.md`
</output>
