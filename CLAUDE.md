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
| `@matcha2d/types` | `packages/types/` | Shared | Shared contract: buffer layout, interfaces, config |
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
  positionX / positionY   // Float32Array — world position
  velocityX / velocityY   // Float32Array — linear velocity
  angle / angularVel      // Float32Array — rotation (radians) and angular velocity
  mass / invMass          // Float32Array — mass and precomputed 1/mass (0 = static)
  inertia / invInertia    // Float32Array — rotational inertia and inverse
  flags                   // Uint8Array — bitfield (ACTIVE, STATIC, SLEEPING, SENSOR)
}
```

Max capacity: 8192 bodies per world. Use `createBuffers(capacity)` to allocate.

### Body Identification

- `BodyHandle` is a branded `number` (index into buffer arrays). Cast with `as BodyHandle`.
- `BodyFlags` — bitfield: `ACTIVE (0x01)`, `STATIC (0x02)`, `SLEEPING (0x04)`, `SENSOR (0x08)`
- `BodyType` — enum: `Dynamic (0)`, `Static (1)`, `Kinematic (2)`

### PhysicsBackend Interface (`packages/types/src/backend.ts`)

The single integration point between Dev A and Dev B:

```typescript
interface PhysicsBackend {
  broadphase(buffers, count) -> CollisionPair[]
  narrowphase(buffers, pairs) -> ContactManifold[]
  solveVelocity(buffers, manifolds, config) -> void
  integrate(buffers, count, dt, gravity) -> void
  solvePosition(buffers, manifolds, config) -> void
}
```

Dev B calls these methods from the simulation loop. Dev A implements them. Until real implementations exist, stubs return empty arrays / no-ops.

## Dev A: Physics Core (`packages/core/`)

### Implemented
- **Vec2 math** (`src/math/vec2.ts`) — flat-array operations (`vec2Add`, `vec2Dot`, `vec2Cross`, `vec2Normalize`, etc.) + scalar helpers (`dot`, `cross`, `length`)
- **Mat2 rotation** (`src/math/mat2.ts`) — `mat2FromAngle`, `mat2MulVec`, `mat2TransposeMulVec`
- **AABB** (`src/collision/aabb.ts`) — `aabbOverlap`, `aabbMerge`, `aabbContains`, `aabbArea`, `aabbPerimeter`

### Stubs (to implement)
- `src/collision/broadphase.ts` — sort-and-sweep, then dynamic BVH
- `src/collision/narrowphase.ts` — SAT for polygons + circles
- `src/collision/gjk.ts` — GJK + EPA for convex shapes
- `src/solver/sequential-impulse.ts` — velocity and position constraint solving
- `src/wasm/` — future Rust/WASM port

### Collision Pipeline (target)
```
Broadphase (AABB tree / sort-and-sweep)
  -> Midphase (AABB vs AABB)
    -> Narrowphase (SAT for polygons, GJK+EPA for convex)
      -> Contact manifolds
        -> Sequential impulse solver
```

## Dev B: Engine Shell

### `@matcha2d/world` (`packages/world/`)
- `World` class — owns buffers, creates bodies, orchestrates simulation
- `BodyManager` — flat-array allocation, free-list, compaction
- `SimulationLoop` — fixed-timestep accumulator with interpolation alpha
- `IslandManager` — union-find for sleep optimization

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
