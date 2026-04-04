# Plan: Custom Rust Physics Engine (Rapier-Inspired) → WASM

## Goal

Write a custom 2D physics engine in Rust, inspired by Rapier's architecture, compile it to WASM, and integrate it with Matcha2D. The engine uses a **BVH broadphase**, **SAT + GJK/EPA narrowphase**, and **sequential impulse solver**. It exposes a **handle-based API** (bodies + colliders) similar to Rapier, compiled to WASM via wasm-pack.

## Reference

Study `Matcha2D_Blueprint/` (a Rapier codebase copy) for algorithm patterns. Key files:
- `src/geometry/broad_phase_bvh.rs` — BVH pair detection, incremental optimization
- `src/pipeline/physics_pipeline.rs` — Pipeline order (broadphase → narrowphase → islands → solve → integrate)
- `src/dynamics/solver/velocity_solver.rs` — PGS iterations, warmstarting, bias/no-bias passes
- `src/geometry/narrow_phase.rs` — Contact manifold computation, persistent contacts
- `src/dynamics/integration_parameters.rs` — Solver config (iterations, softness, warmstarting)
- `src/dynamics/rigid_body.rs` — Component-based body design
- `src/geometry/collider.rs` — Separate collider with shape + material + parent body

---

## Architecture

```
packages/physics-rust/     Rust crate → WASM
  src/
    math.rs                Vec2 (no external deps)
    aabb.rs                AABB operations
    body.rs                RigidBody, RigidBodyType, RigidBodySet (generational arena)
    collider.rs            Collider, Shape, ColliderSet (generational arena)
    bvh.rs                 Dynamic AABB Tree (broadphase)
    narrowphase.rs         SAT + GJK/EPA contact detection
    contact.rs             ContactManifold, ContactPoint, ContactTracker
    solver.rs              Sequential impulse, integration
    world.rs               PhysicsWorld — main API, pipeline orchestration
    lib.rs                 WASM exports via #[wasm_bindgen]

@matcha2d/core             Thin re-export of physics-rust WASM package
@matcha2d/world            World class wrapping WASM PhysicsWorld
```

**Zero external dependencies** — all math, collision, and solver code is custom.
**WASM owns all data** — JS calls WASM functions, no buffer copies.

---

## Phase 1: Rust Core (`packages/physics-rust/`)

### 1.1 Project Setup

```
physics-rust/
  Cargo.toml
  package.json
  src/
    lib.rs
    math.rs
    aabb.rs
    body.rs
    collider.rs
    bvh.rs
    narrowphase.rs
    contact.rs
    solver.rs
    world.rs
```

**Cargo.toml**:
```toml
[package]
name = "matcha2d-physics"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
wasm-bindgen = "0.2"

[profile.release]
opt-level = "z"
lto = true
codegen-units = 1
```

**package.json**:
```json
{
  "name": "@matcha2d/physics-rust",
  "version": "0.0.1",
  "private": true,
  "type": "module",
  "main": "pkg/matcha2d_physics.js",
  "types": "pkg/matcha2d_physics.d.ts",
  "files": ["pkg"],
  "scripts": {
    "build": "wasm-pack build --target web --out-dir pkg",
    "clean": "rm -rf pkg target"
  }
}
```

### 1.2 Module Details

#### `math.rs` — Vector Math
- `Vec2 { x: f32, y: f32 }` with: add, sub, scale, dot, cross, length, length_sq, normalize, negate, perp (rotate 90°)
- `Rot { cos: f32, sin: f32 }` — 2D rotation with: mul_vec, inv_mul_vec, from_angle
- All inline, no heap allocation, Copy trait

#### `aabb.rs` — Axis-Aligned Bounding Box
- `AABB { min: Vec2, max: Vec2 }`
- `overlap(a, b) -> bool`, `merge(a, b) -> AABB`, `contains(a, b) -> bool`
- `area() -> f32`, `perimeter() -> f32`, `center() -> Vec2`
- `compute_body_aabb(pos, rot, shape) -> AABB` — world-space tight AABB
- `fat_aabb(aabb, margin) -> AABB` — expanded for broadphase skin

#### `body.rs` — Rigid Bodies
- `RigidBodyType` enum: `Dynamic`, `Static`, `Kinematic`
- `RigidBody` struct:
  - `pos: Vec2`, `rot: Rot` — position/rotation
  - `linvel: Vec2`, `angvel: f32` — velocities
  - `force: Vec2`, `torque: f32` — accumulated forces
  - `mass: f32`, `inv_mass: f32` — mass properties
  - `inertia: f32`, `inv_inertia: f32` — rotational inertia
  - `body_type: RigidBodyType`
  - `enabled: bool`
  - `linear_damping: f32`, `angular_damping: f32`
  - `gravity_scale: f32`
  - `user_data: u32`
- `RigidBodyBuilder` — fluent builder pattern
- `RigidBodySet` — generational arena (Vec + generation counters)
  - `insert(body) -> RigidBodyHandle`
  - `remove(handle) -> Option<RigidBody>`
  - `get(handle) -> Option<&RigidBody>`
  - `get_mut(handle) -> Option<&mut RigidBody>`
  - `iter() -> impl Iterator`
- `RigidBodyHandle(u32)` — generational handle (index in upper 24 bits, generation in lower 8)

#### `collider.rs` — Colliders
- `Shape` enum:
  - `Ball { radius: f32 }`
  - `Cuboid { hx: f32, hy: f32 }` (box half-extents)
  - `Polygon { vertices: Vec<Vec2> }` (convex, up to 16 verts)
- `Collider` struct:
  - `shape: Shape`
  - `parent: Option<RigidBodyHandle>`
  - `local_pos: Vec2`, `local_rot: Rot` — offset from parent
  - `friction: f32`, `restitution: f32`
  - `is_sensor: bool`
  - `density: f32` — for automatic mass computation
  - `user_data: u32`
- `ColliderBuilder` — fluent builder
- `ColliderSet` — generational arena (same pattern as RigidBodySet)
- `ColliderHandle(u32)` — generational handle
- Mass property computation: `compute_mass_properties(shape, density) -> (mass, inertia)`

#### `bvh.rs` — Dynamic AABB Tree (Broadphase)
- `BvhNode` struct: `aabb: AABB`, `parent: Option<usize>`, `child1: Option<usize>`, `child2: Option<usize>`, `height: i32`, `leaf_data: Option<ColliderHandle>`
- `DynamicBvh` struct: `nodes: Vec<BvhNode>`, `root: Option<usize>`, `free_list: Vec<usize>`
- Operations:
  - `insert_leaf(aabb, handle) -> usize` — find best sibling via area heuristic, create parent
  - `remove_leaf(node_index)` — remove and balance
  - `update_leaf(node_index, new_aabb)` — remove + reinsert if aabb doesn't fit
  - `balance(index) -> usize` — AVL-style rotation for tree balancing
  - `query_pairs() -> Vec<(ColliderHandle, ColliderHandle)>` — traverse dual-subtree, collect overlapping leaf pairs
  - `compute_height() -> usize` — tree height
- Fat AABB margin: expand leaf AABBs by velocity * dt + skin constant
- Pattern: Rapier's broad_phase_bvh.rs + Box2D's b2_dynamic_tree

#### `narrowphase.rs` — SAT + GJK/EPA
- **SAT** (primary):
  - `sat_ball_ball(a, b) -> Option<ContactManifold>` — distance check
  - `sat_ball_polygon(ball, poly) -> Option<ContactManifold>` — find closest edge, project center
  - `sat_polygon_polygon(a, b) -> Option<ContactManifold>` — test edge normals of both shapes
  - Contact point generation via reference/incident edge clipping
- **GJK + EPA** (fallback for complex shapes):
  - `gjk_distance(a, b, shape_a, shape_b) -> Option<(distance, normal, point_a, point_b)>`
  - `gjk_penetration(a, b, shape_a, shape_b) -> Option<ContactManifold>` — EPA for depth/normal
  - Support function for each shape type
  - Simplex management (2D: point, line, triangle)
- Shape pair dispatch: match on (Shape, Shape) → call appropriate algorithm
- SAT for: ball-ball, ball-polygon, polygon-polygon (up to ~20 verts)
- GJK+EPA for: complex polygons, general convex shapes

#### `contact.rs` — Contact Data
- `ContactPoint` struct: `local_a: Vec2`, `local_b: Vec2`, `penetration: f32`, `id: u32` (for warmstarting persistence)
- `ContactManifold` struct: `body_a: RigidBodyHandle`, `body_b: RigidBodyHandle`, `normal: Vec2`, `contacts: Vec<ContactPoint>` (max 2 for 2D block solver)
- `ContactPair` struct: `collider_a: ColliderHandle`, `collider_b: ColliderHandle`, `manifold: ContactManifold`, `prev_manifold: Option<ContactManifold>`
- `ContactTracker` struct:
  - `pairs: HashMap<(ColliderHandle, ColliderHandle), ContactPair>`
  - `active_pairs: Vec<(ColliderHandle, ColliderHandle)>`
  - `update(new_manifolds)` — merge new contacts with existing, detect begin/stay/end
  - `begin_contacts()`, `end_contacts()` — event queries

#### `solver.rs` — Sequential Impulse
- `SolverConfig` struct:
  - `velocity_iterations: usize` (default 8)
  - `position_iterations: usize` (default 3)
  - `baumgarte_factor: f32` (default 0.2)
  - `penetration_slop: f32` (default 0.02)
  - `restitution_slop: f32` (default 1.0)
  - `warmstarting: bool` (default true)
  - `block_solver: bool` (default true)
- `integrate(bodies, dt, gravity)` — Euler: pos += vel * dt, vel += gravity * dt, apply damping
- `solve_velocity(bodies, manifolds, config)`:
  - For each iteration:
    - For each manifold:
      - Compute Jacobian (normal + tangent)
      - Compute effective mass
      - Apply normal impulse (clamp 0..max)
      - Apply friction impulse (Coulomb cone: clamp -μ*λ_n..+μ*λ_n)
      - Block solver for 2-contact manifolds (solve 2x2 system)
  - Warmstarting: apply previous frame's accumulated impulses as initial guess
- `solve_position(bodies, manifolds, config)`:
  - Baumgarte position correction: apply velocity bias proportional to penetration depth
  - For each manifold, correct body positions to reduce penetration
- Precompute inv_mass/inv_inertia at body creation

#### `world.rs` — PhysicsWorld (Main API)
- `PhysicsWorld` struct:
  - `bodies: RigidBodySet`
  - `colliders: ColliderSet`
  - `bvh: DynamicBvh`
  - `contact_tracker: ContactTracker`
  - `config: SolverConfig`
  - `gravity: Vec2`
  - `dt: f32` (fixed timestep)
  - `contact_events: Vec<ContactEvent>`
- `step()`:
  1. Clear forces on all dynamic bodies
  2. Apply gravity
  3. Integrate velocities and positions
  4. Update BVH (compute AABBs, update leaves)
  5. Broadphase: query BVH pairs
  6. Narrowphase: SAT/GJK on each pair → contact manifolds
  7. Update contact tracker (begin/stay/end events)
  8. Solve velocity (sequential impulse, N iterations)
  9. Solve position (Baumgarte correction)
  10. Write back positions/rotations

#### `lib.rs` — WASM Exports
```rust
#[wasm_bindgen]
impl PhysicsWorld {
  #[wasm_bindgen(constructor)]
  pub fn new() -> PhysicsWorld;

  // Body management
  pub fn create_dynamic_body(&mut self, x: f32, y: f32) -> u32;
  pub fn create_static_body(&mut self, x: f32, y: f32) -> u32;
  pub fn create_kinematic_body(&mut self, x: f32, y: f32) -> u32;
  pub fn remove_body(&mut self, handle: u32);

  // Body properties
  pub fn set_body_position(&mut self, handle: u32, x: f32, y: f32);
  pub fn set_body_velocity(&mut self, handle: u32, vx: f32, vy: f32);
  pub fn set_body_angle(&mut self, handle: u32, angle: f32);
  pub fn get_body_position(&self, handle: u32) -> Vec<f32>;
  pub fn get_body_velocity(&self, handle: u32) -> Vec<f32>;
  pub fn get_body_angle(&self, handle: u32) -> f32;
  pub fn apply_impulse(&mut self, handle: u32, fx: f32, fy: f32);

  // Collider management
  pub fn create_ball_collider(&mut self, radius: f32, body: u32) -> u32;
  pub fn create_box_collider(&mut self, hx: f32, hy: f32, body: u32) -> u32;
  pub fn create_polygon_collider(&mut self, vertices_x: &[f32], vertices_y: &[f32], body: u32) -> u32;
  pub fn remove_collider(&mut self, handle: u32);
  pub fn set_collider_friction(&mut self, handle: u32, friction: f32);
  pub fn set_collider_restitution(&mut self, handle: u32, restitution: f32);
  pub fn set_collider_sensor(&mut self, handle: u32, sensor: bool);

  // Simulation
  pub fn set_gravity(&mut self, x: f32, y: f32);
  pub fn step(&mut self);

  // Query
  pub fn body_count(&self) -> usize;
  pub fn collider_count(&self) -> usize;

  // Contact events (JSON for simplicity)
  pub fn get_contact_events_json(&self) -> String;
  // Bulk position read (for rendering)
  pub fn get_positions_x(&self) -> Vec<f32>;
  pub fn get_positions_y(&self) -> Vec<f32>;
  pub fn get_angles(&self) -> Vec<f32>;
}
```

---

## Phase 2: TypeScript Types Update (`packages/types/`)

### 2.1 Delete
- `src/backend.ts` — PhysicsBackend interface (no longer needed)

### 2.2 Simplify `config.ts`
- Remove `BroadphaseMethod`, `NarrowphaseMethod` types
- Remove `broadphaseMethod`, `narrowphaseMethod` from WorldConfig
- Keep: gravity, fixedTimestep, solver iterations, friction, restitution, sleep

### 2.3 Keep (may be simplified)
- `src/buffers.ts` — MatchaBuffers may no longer be needed (WASM owns data), but keep for compatibility
- `src/body.ts` — BodyHandle, BodyFlags, BodyType
- `src/collision.ts` — ShapeType, ContactManifold, CollisionCallbacks
- `src/math.ts` — Vec2Readonly, AABB, Transform

---

## Phase 3: Core Package Update (`packages/core/`)

### 3.1 Delete ALL existing code
- `src/math/`, `src/collision/`, `src/solver/`, `src/wasm/`
- `wasm/` (C source, build scripts, build artifacts)
- `tsup.config.ts`
- `__tests__/`

### 3.2 Create thin re-export
- `src/index.ts`: `export * from '@matcha2d/physics-rust'`
- `package.json`: depends on `@matcha2d/physics-rust`

---

## Phase 4: World Package Update (`packages/world/`)

### 4.1 Rewrite `world.ts`
- `World` class wraps WASM `PhysicsWorld`
- `World.create(config?)` — async factory, calls `init()` on WASM module
- Body/collider management delegates to WASM
- `step(dt)` — fixed-timestep accumulator, calls WASM step
- `renderAlpha` for interpolation
- Collision callbacks via WASM contact events

### 4.2 Delete
- `body-manager.ts`, `simulation-loop.ts`, `island.ts`

### 4.3 Updated API (Rapier-style)
```typescript
const world = await World.create({ gravity: { x: 0, y: -9.81 } });
const body = world.createBody({ type: 'dynamic', position: { x: 0, y: 10 } });
const collider = world.createCollider({ shape: 'box', hx: 0.5, hy: 0.5, body });
world.step(dt);
const pos = world.getBodyPosition(body);
```

---

## Phase 5: Build Pipeline

### 5.1 Root package.json
```json
{
  "workspaces": [
    "packages/types",
    "packages/physics-rust",
    "packages/core",
    "packages/world",
    "packages/render",
    "packages/tools"
  ],
  "scripts": {
    "build:wasm": "npm run build -w packages/physics-rust",
    "build": "npm run build:wasm && npm run build -w packages/types && npm run build -w packages/core && npm run build -w packages/world",
    "test": "vitest run",
    "test:wasm": "cargo test --manifest-path packages/physics-rust/Cargo.toml",
    "lint": "tsc -b"
  }
}
```

### 5.2 WASM file loading
- wasm-pack `--target web` produces JS glue with async `init()`
- Demo uses importmap to resolve `@matcha2d/physics-rust`
- Test environment uses `vite-plugin-wasm` or direct init

---

## Phase 6: Demo Update (`demo/`)

### 6.1 `collision.html`
- Remove backend/broadphase/narrowphase selectors
- Always WASM mode
- Update to use new World.create() API (async)
- Update body creation to use createBody() + createCollider()
- Remove inline collision resolver

### 6.2 `collision-wasm.html`
- Merge with collision.html or delete

---

## Phase 7: Tests

### 7.1 Rust native tests (`cargo test`)
Unit tests in Rust source files:
- `math.rs` — Vec2 operations
- `aabb.rs` — AABB operations
- `bvh.rs` — tree insert/remove/query
- `narrowphase.rs` — SAT and GJK for all shape combos
- `solver.rs` — impulse solving
- `world.rs` — full pipeline integration

### 7.2 TypeScript integration tests
- Import from WASM, test through World API
- Create bodies, step, verify contacts and positions

---

## Implementation Order

1. **packages/physics-rust/** — Rust crate
   - math.rs → aabb.rs → body.rs → collider.rs → bvh.rs → narrowphase.rs → contact.rs → solver.rs → world.rs → lib.rs
   - Cargo test for each module
   - Verify `wasm-pack build` succeeds

2. **packages/types/** — Update types (delete backend.ts, simplify config)

3. **packages/core/** — Replace with thin re-export

4. **packages/world/** — Rewrite World class

5. **Build pipeline** — Integrate wasm-pack

6. **demo/** — Update for new API

7. **Tests** — Rust + TypeScript integration

8. **Cleanup** — Remove dead code

---

## Rapier Patterns to Follow

| Pattern | Rapier | Matcha2D Custom |
|---------|--------|-----------------|
| Handle-based identity | Generational Arena<u32> | Simple u32 index + generation in lower bits |
| Body/Collider separation | Full separation | Same — bodies own dynamics, colliders own shapes |
| BVH broadphase | parry::Bvh + incremental optimizer | Custom DynamicBvh (Box2D-style AVL tree) |
| Contact persistence | ContactPair + prev_manifold | ContactTracker with HashMap |
| Solver | PGS with warmstarting + bias/no-bias | Sequential impulse with warmstarting + Baumgarte |
| Pipeline order | broadphase → narrowphase → solve → integrate | integrate → broadphase → narrowphase → solve (simpler) |

## Risks

| Risk | Mitigation |
|------|-----------|
| Rust compilation complexity | Start with simplest modules (math, aabb), build up |
| WASM binary size | Use opt-level="z" + LTO, no external deps |
| wasm-pack not installed | Add install check + docs |
| GJK complexity | Implement SAT first, add GJK as fallback |
| API mismatch with existing demos | Update demos after core is working |
