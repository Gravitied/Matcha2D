# Matcha2D

A high-performance 2D physics engine. TypeScript-first, with a Rust/WASM core planned.

## Commands

```bash
npm run build        # Build all packages (types -> core -> rest)
npm test             # Run all tests (vitest)
npm run test:watch   # Watch mode for tests
npm run lint         # Type-check entire project (tsc -b --noEmit)
npm run clean        # Remove all dist/ folders

# Single package
npm run build -w packages/core
npm test -w packages/core
```
## Template
Template-Repo folder : C:\Users\winsi\Projects\Matcha2D\Template-Repo
Use the Template-Repo folder to reference all algorithms and code as the working version. The Template-Repo should be treated as a perfect project and all errors should be assumed to be something elses fault if the fault is assumed to be the Template-Repos. Do not change any code inside of the template repo only use it for reference.

## Architecture

```
User API (TS)  ->  Simulation Loop (TS)  ->  Physics Core (TS/WASM)  ->  Renderer (TS)
```

### Packages

| Package | Path | Owner | Purpose |
|---------|------|-------|---------|
| `@matcha2d/types` | `packages/types/` | Shared | Shared contract: buffer layout, interfaces, config, collision types |
| `@matcha2d/core` | `packages/core/` | Dev A | Physics math, collision detection, constraint solver |
| `@matcha2d/world` | `packages/world/` | Dev B | World API, body management, simulation loop |
| `@matcha2d/render` | `packages/render/` | Dev B | Canvas2D / WebGL debug renderer |
| `@matcha2d/tools` | `packages/tools/` | Dev B | Serialization, profiling, debug utilities |

### Dependency Graph

```
@matcha2d/types      (no deps — build first)
    ├── @matcha2d/core    (types)
    │       │
    ├───────┴── @matcha2d/world  (types + core)
    ├── @matcha2d/render  (types)
    └── @matcha2d/tools   (types)
```

Build order matters: types must build before everything else, core before world. The root `npm run build` script handles this automatically.

## The Shared Contract

Both devs depend on `@matcha2d/types`. Changes here require agreement from both sides. No developer should interact with the other developers packages or files no matter what (Neither dev A or dev B).

### Buffer Layout (`packages/types/src/buffers.ts`)

Data-Oriented Design: all body data lives in flat `Float32Array`/`Uint8Array` buffers (SoA), not object arrays. This is critical for cache performance and future WASM shared memory.

```typescript
interface MatchaBuffers {
  positionX / positionY       // Float32Array — world position
  velocityX / velocityY       // Float32Array — linear velocity
  angle / angularVel          // Float32Array — rotation (radians) and angular velocity
  mass / invMass              // Float32Array — mass and precomputed 1/mass (0 = static)
  inertia / invInertia        // Float32Array — rotational inertia and inverse
  flags                       // Uint8Array — bitfield (ACTIVE, STATIC, SLEEPING, SENSOR)
  halfExtentX / halfExtentY   // Float32Array — AABB half-extents, set at body creation
  shapeType                   // Uint8Array — 0=Box, 1=Circle, 2=Polygon
  shapeRadius                 // Float32Array — circle radius
  shapeVertexCount            // Uint8Array — polygon vertex count
  shapeVerticesX / shapeVerticesY  // Float32Array — polygon vertices (capacity × MAX_VERTICES_PER_SHAPE, local space)
}
```

Max capacity: 8192 bodies per world (`MAX_BODIES`). Max polygon vertices: 16 (`MAX_VERTICES_PER_SHAPE`). Use `createBuffers(capacity)` to allocate.

### Body & Shape Identification

- `BodyHandle` is a branded `number` (index into buffer arrays). Cast with `as BodyHandle`.
- `BodyFlags` — bitfield: `ACTIVE (0x01)`, `STATIC (0x02)`, `SLEEPING (0x04)`, `SENSOR (0x08)`
- `BodyType` — enum: `Dynamic (0)`, `Static (1)`, `Kinematic (2)`
- `ShapeType` — enum: `Box (0)`, `Circle (1)`, `Polygon (2)`
- `CollisionCallbacks` — `{ onBegin?, onStay?, onEnd? }` — fired by `ContactTracker`
- `JointType` / `ConstraintDef` — joint/constraint types (future use)
- `Vec2Readonly`, `AABB`, `Transform` — math types

### PhysicsBackend Interface (`packages/types/src/backend.ts`)

The single integration point between Dev A and Dev B:

```typescript
interface PhysicsBackend {
  broadphase(buffers, count, method?) -> CollisionPair[]
  narrowphase(buffers, pairs, method?) -> ContactManifold[]
  collide(buffers, count, broadphaseMethod?, narrowphaseMethod?) -> ContactManifold[]  // full pipeline
  solveVelocity(buffers, manifolds, config) -> void
  integrate(buffers, count, dt, gravity) -> void
  solvePosition(buffers, manifolds, config) -> void
}
```

Dev B calls these methods from the simulation loop. Dev A implements them. The `collide()` method is the primary entry point for the full broadphase → narrowphase pipeline. In WASM mode, `broadphase`/`narrowphase`/`solveVelocity`/`integrate`/`solvePosition` are no-ops — Box2D handles everything internally during `collide()`.

## Dev A: Physics Core (`packages/core/`)

### Implemented
- **Vec2 math** (`src/math/vec2.ts`) — flat-array operations (`vec2Add`, `vec2Dot`, `vec2Cross`, `vec2Normalize`, etc.) + scalar helpers (`dot`, `cross`, `length`)
- **Mat2 rotation** (`src/math/mat2.ts`) — `mat2FromAngle`, `mat2MulVec`, `mat2TransposeMulVec`
- **AABB** (`src/collision/aabb.ts`) — `aabbOverlap`, `aabbMerge`, `aabbContains`, `aabbArea`, `aabbPerimeter`, `computeBodyAABB`
- **Broadphase** (`src/collision/broadphase.ts`) — Sort-and-Sweep (`sap`), incremental Dynamic AABB Tree (`dynamicTree`, default), legacy BVH alias; dispatches via `broadphase()`
- **Dynamic Tree** (`src/collision/dynamic-tree.ts`) — incremental AABB tree with `insert`, `remove`, `updateAll`, `queryPairs`
- **GJK + EPA narrowphase** (`src/collision/gjk.ts`) — full GJK+EPA for convex shapes including circle-circle analytical shortcut; exported as `gjkNarrowphase`
- **Narrowphase dispatch** (`src/collision/narrowphase.ts`) — thin wrapper that routes to `gjkNarrowphase`
- **Collision pipeline** (`src/collision/pipeline.ts`) — `collide()` (broadphase → narrowphase), `narrowphaseDispatch()`
- **Shape system** (`src/collision/shapes.ts`) — `ShapeHandler` interface + registry (`registerShapeHandler`, `getShapeHandler`); built-in handlers for Circle, Box, Polygon
- **Simplex / Polytope** (`src/collision/simplex.ts`, `src/collision/polytope.ts`) — GJK/EPA support structures
- **Contact tracker** (`src/collision/contact-tracker.ts`) — tracks begin/stay/end contact events across frames
- **Sequential impulse solver** (`src/solver/sequential-impulse.ts`) — `solveVelocity`, `solvePosition`, `integrate`; block solver for 2-contact manifolds, impulse accumulation, Baumgarte position correction
- **WASM backend** (`src/wasm/`) — `WasmPhysicsBackend` (implements `PhysicsBackend`), `WasmModule` loader (`loadWasmModule`), Box2D Emscripten bridge (`box2d.d.ts`); WASM build artifact at `wasm/build/box2d.js` (gitignored)

### Collision Pipeline
```
Broadphase (DynamicTree / SAP)
  -> Narrowphase (GJK + EPA for all convex shapes; circle-circle analytical)
    -> Contact manifolds
      -> Sequential impulse solver (velocity + position)
```

## Dev B: Engine Shell

### `@matcha2d/world` (`packages/world/`)
- `World` class — owns buffers, creates/destroys bodies, orchestrates simulation
  - `World.createWithTS(config?)` — TypeScript backend (default)
  - `World.createWithWasm(config?)` — WASM backend (async, Box2D)
  - `World.createWithBackend(backend, config?)` — custom `PhysicsBackend`
  - `createBody(def)` / `destroyBody(handle)` — body lifecycle
  - `step(dt)` — fixed-timestep accumulator loop
  - `renderAlpha` — interpolation factor for rendering
  - `setCollisionCallbacks(callbacks)` — begin/stay/end contact events
- `BodyManager` (`body-manager.ts`) — flat-array allocation helpers
- `SimulationLoop` (`simulation-loop.ts`) — fixed-timestep accumulator with interpolation alpha
- `IslandManager` (`island.ts`) — union-find for sleep optimization

### `@matcha2d/render` (`packages/render/`)
- `IRenderer` interface — `begin()`, `drawBodies()`, `end()`, `destroy()`
- `Canvas2DRenderer` — debug renderer (reads buffers, draws shapes)

### `@matcha2d/tools` (`packages/tools/`)
- `serializeBuffers` / `deserializeBuffers` — scene save/load
- `Profiler` — per-step timing (broadphase, narrowphase, solver, integrate)

### Fixed Timestep Pattern
```typescript
accumulator += deltaTime
while (accumulator >= fixedStep) {
  physicsStep(fixedStep)
  accumulator -= fixedStep
}
renderAlpha = accumulator / fixedStep  // interpolation factor
```

### WorldConfig (`packages/types/src/config.ts`)
Key fields (all optional — `DEFAULT_WORLD_CONFIG` provides sensible defaults):
- `gravity` — `{ x, y }`, default `{ x: 0, y: -9.81 }`
- `fixedTimestep` — default `1/60`
- `velocityIterations` / `positionIterations` — solver iteration counts (10 / 4)
- `broadphaseMethod` — `'dynamicTree'` (default) | `'sap'` | `'bvh'`
- `narrowphaseMethod` — `'gjk'` (only option)
- `baumgarteFactor` / `penetrationSlop` — position correction tuning
- `defaultFriction` / `defaultRestitution` / `restitutionSlop`
- `blockSolver` — 2-contact block solver (reduces jitter), default `true`
- `impulseAccumulation` / `warmStarting` / `warmStartThreshold`
- `positionCorrection` / `positionCorrectionBeta`
- `sleepEnabled` / `sleepTimeThreshold` / `sleepVelocityThreshold`

## Conventions

### Code Style
- **Strict TypeScript** — `strict: true`, `verbatimModuleSyntax: true`
- **ESM only** — all packages use `"type": "module"`, imports use `.js` extensions
- **ES2017 target** — broad browser compatibility (~98%)
- **No runtime dependencies** — zero mandatory deps
- Use `import type` for type-only imports (enforced by `verbatimModuleSyntax`)

### API QoL
- **Ease of Use API** — The APIs core design should be meant for ease of use and customizability.
- All terms functions and syntax should be simple yet customizable enough for unique management of scenes.
- The focus is easy user experience, yet detailed for more familar users that want to delve deeper.

### Performance Rules
- Hot-path math uses flat typed arrays (SoA), not objects
- Precompute inverse values (`invMass`, `invInertia`) at body creation, not per-frame
- Avoid allocations in tight loops — pass output arrays + indices instead of returning new objects
- Vec2 array functions signature: `(outX, outY, outIdx, aX, aY, aIdx, ...)`

### Code Modularity
- Any code/function that can be modular, should always be modular to support easy development and integration for later features.


### Testing
- Tests go in `__tests__/` directories within each package
- Framework: vitest
- Run all: `npm test` | Run one package: `npm test -w packages/core`

### Adding a New Package
1. Create `packages/<name>/` with `package.json`, `tsconfig.json`, `tsup.config.ts`, `src/index.ts`
2. Add `{ "path": "packages/<name>" }` to root `tsconfig.json` references
3. If it depends on types: add `"@matcha2d/types": "*"` to dependencies and `{ "path": "../types" }` to tsconfig references
4. If it needs DOM APIs: add `"lib": ["ES2017", "DOM"]` to its tsconfig
5. Run `npm install` to link workspaces

### Build Configuration
- **tsup** builds ESM output with `.d.ts` generation and sourcemaps
- Each package has its own `tsup.config.ts` (identical template, can diverge)
- Root build script ensures correct order: types -> core -> everything else

## Toolchain
- **Node.js** + **npm workspaces** (monorepo)
- **TypeScript** ^5.7 — type-checking and declarations
- **tsup** ^8.3 — ESM bundling + DTS generation (wraps esbuild)
- **vitest** ^3.0 — test runner
- **webGL** for render engine.
