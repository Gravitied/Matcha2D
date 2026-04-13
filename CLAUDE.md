# Matcha2D

-It is assumed all bugs/fixes or requests are meant as a fix/implementation for Matcha2D and its demos only, unless stated otherwise.

A high-performance 2D physics engine. TypeScript-first, with a Rust/WASM core planned. This will be used for web animation and 2d web game development.

## Commands

```bash
npm run build        # Build all packages (types -> core -> rest)
npm test             # Run all tests (vitest)
npm run test:watch   # Watch mode for tests
npm run lint         # Type-check entire project (tsc -b --noEmit)
npm run clean        # Remove all dist/ folders
D
# Single package
npm run build -w packages/core
npm test -w packages/core
```
## Wiki

A persistent knowledge base lives at `wiki/`. The LLM maintains it; you source and direct.

- **Drop sources** into `wiki/raw/` (markdown preferred; use Obsidian Web Clipper for articles)
- **Ingest**: tell Claude "ingest wiki/raw/<file>" — it reads, extracts, updates pages, updates index + log
- **Query**: ask any question — Claude searches `wiki/index.md`, reads relevant pages, answers with citations
- **Lint**: ask "lint the wiki" — Claude checks for contradictions, orphans, gaps
- **Schema**: `wiki/SCHEMA.md` — read this to understand conventions before any wiki operation

## Template
Matcha2D_Blueprint folder : C:\Users\winsi\Projects\Matcha2D\Project_References
Use the Project_References folder to reference all algorithms and code as the working version. The Project_References should be treated as a perfect project and all errors should be assumed to be something elses fault if the fault is assumed to be the Project_References, Do not change any code inside of the Project_References only use it for reference.

## Architecture

```
User API (TS)  ->  Simulation Loop (TS)  ->  Physics Core (TS/WASM)  ->  Renderer (TS)
```

### Packages

| Package | Path | Owner | Purpose |
|---------|------|-------|---------|
| `@matcha2d/types` | `packages/types/` | Shared | Shared contract: buffer layout, interfaces, config, collision types |
| `@matcha2d/physics-rust` | `packages/physics-rust/` | Shared | Rust 2D engine + `wasm-pack` JS/WASM (`pkg/`) — `PhysicsEngine` |
| `@matcha2d/world` | `packages/world/` | Shell | World API around `PhysicsEngine`, fixed timestep, body/collider helpers |
| `@matcha2d/render` | `packages/render/` | Dev B | Canvas2D / WebGL debug renderer |
| `@matcha2d/tools` | `packages/tools/` | Dev B | Serialization, profiling, debug utilities |

### Dependency Graph

```
@matcha2d/types           (no deps — build first)
    ├── @matcha2d/physics-rust  (WASM build; no TS deps except toolchain)
    ├── @matcha2d/world         (types + physics-rust)
    ├── @matcha2d/render        (types)
    └── @matcha2d/tools       (types)
```

Build order: `types` → `physics-rust` (WASM) → `world` → other workspaces. Root `npm run build` runs them in that order.

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

Optional contract for buffer-driven integrations (the default `World` path uses `PhysicsEngine` directly):

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

The `PhysicsBackend` interface in `packages/types` documents a buffer-oriented pipeline used when integrating alternate backends. The shipped engine is **`PhysicsEngine`** from `@matcha2d/physics-rust` (Rust + wasm-bindgen).

## Physics engine (`packages/physics-rust/`)

Single Rust crate (`matcha2d-physics`): dynamic tree broadphase, SAT-style narrowphase + manifolds, sequential impulse / PGS solver, optional sleep, TOI-style recovery on CPU step, optional WebGPU path. Built with `wasm-pack` to `packages/physics-rust/pkg/` for the web.

### Collision / solve pipeline (CPU)
```
Broadphase (dynamic BVH; swept AABB variant optional)
  -> Narrowphase (convex tests + contact manifolds)
    -> Contact tracker (persistence / warm-start hints)
      -> Velocity solver + substep integration + position stabilization
```

## Engine shell

### `@matcha2d/world` (`packages/world/`)
- `World` class — thin wrapper over `PhysicsEngine`, maps `WorldConfig` into Rust setters
  - `World.create(config?)` — async: loads WASM glue, constructs `PhysicsEngine`, applies config
  - `createBody(def)` / `destroyBody(handle)` — body lifecycle
  - `step(dt)` — fixed-timestep accumulator loop
  - `renderAlpha` — interpolation factor for rendering
  - `setCollisionCallbacks(callbacks)` — begin/stay/end contact events

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

## Rust physics GPU (`packages/physics-rust`)

Optional **WebGPU / wgpu** hybrid pipeline (off by default):

- **CPU:** sleep, BVH broadphase, collision filtering, pair list, **step-start TOI recovery** (same as CPU step) before packing GPU buffers.
- **GPU:** `integrate_main` uses **`dt / num_substeps`** per outer iteration (matches CPU PGS substepping). Pipeline: integrate → narrowphase (ball/ball, ball/box, box/box; polygons use conservative sphere) → Jacobi contact resolve (`clear_atomics_main`, `solve_contacts_main`, `apply_deltas_main`) with fixed-point atomics; normal correction clamped by `max_corrective_velocity * dt`; read back bodies and manifolds.
- **`PhysicsWorld`:** `init_gpu()` (async) or `init_gpu_blocking()` (native); when `gpu_acceleration_enabled` and `gpu_runtime` are set, `step()` uses GPU (`step_gpu()`), otherwise `step_cpu()` (GJK + TOI + PGS). GPU failures fall back to CPU with a stderr message (native). Use **`step_cpu()`** for full in-step CCD/PGS parity on extreme tunneling scenes.
- **WASM:** `PhysicsEngine::init_gpu()` async; `setGpuAccelerationEnabled` in JS bindings.
- **Limits:** `GPU_MAX_BODIES`, `GPU_MAX_COLLIDERS`, `GPU_MAX_PAIRS` in `packages/physics-rust/src/gpu/types.rs`.
- **Shaders:** `packages/physics-rust/shaders/physics.wgsl`.
