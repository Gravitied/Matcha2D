# Plan: Replace Collision System with Sopiro/Physics Approach

## Context

**Current state:** The Matcha2D codebase has substantial collision code but suffers from critical bugs causing lag-back and passthroughs:
- Restitution hardcoded to zero (`sequential-impulse.ts:96`)
- `integrate` not exported from core
- Vertex indexing bug in `getWorldVertices` (reads from offset 0 instead of body's slot)
- Warm starting non-functional due to per-frame solver recreation
- Position solver double-transforms contact points
- No collision callback system
- No contact persistence between frames
- World layer is entirely stubbed (no `step()` method)

**Reference:** Sopiro/Physics — a proven 2D physics engine with:
- Dynamic AABB Tree with incremental insert/remove + tree rotations
- GJK with simplex shrink/closest-point approach
- EPA with polytope expansion
- Contact manifold generation via edge clipping with feature IDs
- Impulse-based solver with warm starting, friction, restitution, 2-contact block LCP solver
- Contact persistence via feature ID matching (`tryWarmStart`)
- Collision callback system via `ContactInfo`

## Approach

Rather than copy-pasting the reference code (which uses OOP class-based architecture incompatible with our SoA buffer design), we will **rewrite our existing modules** using the reference's algorithms and proven formulas while preserving our data-oriented architecture.

---

## Phase 1: Dynamic AABB Tree (Broadphase)

**Goal:** Replace the rebuild-every-frame BVH with an incremental dynamic AABB tree matching the reference's insert/remove/rotate pattern.

### 1.1 `packages/core/src/collision/dynamic-tree.ts` (new file)

**What to implement:**
- `DynamicTree` class that manages a persistent AABB tree across frames
- Node structure: `{ id, parent, child1, child2, isLeaf, minX, minY, maxX, maxY, body }`
- `insert(bodyIdx, buffers)`: Find best sibling via cost-based descent (union area + inherited cost), create new parent, walk up refitting AABBs with tree rotations
- `remove(bodyIdx)`: Replace node with sibling, update parent links, refit ancestors
- `update(bodyIdx, buffers)`: Remove + reinsert if AABB moved outside fat margin
- `queryPairs()`: Dual-subtree traversal matching reference's `checkCollision()` — self-pairs within subtrees + cross-pairs between siblings
- `reset()`: Clear entire tree
- Fat AABB margin (`aabbMargin = 0.05`) for static bodies to reduce reinsertions

**Key differences from current BVH:**
- Current: rebuilds tree from scratch every frame (O(n log n) build)
- New: incremental insert/remove with tree rotations (amortized O(log n))
- Reference uses BFS priority queue for best-sibling search; we'll use the same
- Tree rotation heuristic: try 4 swap types, pick lowest area cost

### 1.2 `packages/core/src/collision/broadphase.ts` (rewrite)

**What to change:**
- Replace `broadphaseBVH` call with `DynamicTree.queryPairs()`
- Keep `broadphaseSAP` as-is (it works correctly)
- Add `createDynamicTree()` factory function
- World layer will own a `DynamicTree` instance and call `insert`/`remove`/`update` per body

### 1.3 `packages/types/src/buffers.ts` (extend)

**What to add:**
- No buffer changes needed — the tree stores node data internally, not in SoA buffers

---

## Phase 2: GJK + EPA (Narrowphase — Primary Path)

**Goal:** Replace the current GJK implementation with the reference's cleaner simplex-based approach, and fix EPA to produce reliable penetration depth.

### 2.1 `packages/core/src/collision/simplex.ts` (new file)

**What to implement:**
- `Simplex` class/struct for managing GJK simplex vertices (up to 3 in 2D)
- `getClosest(qx, qy)`: Returns closest point on simplex to origin + contributor indices
  - 1-point: return the point
  - 2-point: barycentric projection onto segment, shrink to closest feature
  - 3-point: test 7 Voronoi regions (3 vertices, 3 edges, interior) using barycentric coords
- `addVertex(wx, wy)`: Add vertex (max 3)
- `shrink(indices)`: Rebuild simplex from contributor indices
- `containsVertex(wx, wy)`: Check for duplicate vertices

**Why new file:** Current GJK embeds simplex logic inline — extracting it makes the algorithm clearer and matches the reference's separation of concerns.

### 2.2 `packages/core/src/collision/gjk.ts` (rewrite)

**What to change:**

**GJK collision detection:**
- Use reference's cleaner loop: `csoSupport → addVertex → getClosest → shrink → new direction`
- CSO (Configuration Space Object) support function: already correct in current code, keep it
- Early termination: if new support point doesn't extend past closest point → no collision
- Duplicate vertex check → no collision
- Expand degenerate simplex (1 or 2 points) to triangle before EPA — matching reference's switch/case

**EPA penetration depth:**
- `Polytope` class: initialized from GJK's 3-vertex simplex
- `getClosestEdge()`: Find edge closest to origin, compute outward normal
- Iterative expansion: `csoSupport(edgeNormal) → if (|supportDist - edgeDist| > epsilon) → insert vertex`
- Fallback: if loop exhausts, return closest edge of final polytope

**Contact manifold generation:**
- `findFarthestEdge(body, normal)`: Returns the reference edge (for polygons: edge at support vertex; for circles: tangent edge)
- `clipEdge()`: Sutherland-Hodgman clipping against reference edge planes
- `findContactPoints(normal, bodyA, bodyB)`: Full contact generation pipeline
  - Determine reference/incident edge by perpendicularness to collision normal
  - Clip incident edge against reference edge's side planes
  - Merge close contact points (threshold: `1.415 * TANGENT_MIN_LENGTH`)
  - Return 1 or 2 contact points with feature IDs

### 2.3 `packages/core/src/collision/narrowphase.ts` (rewrite)

**What to change:**
- **Remove SAT entirely** — the reference uses GJK+EPA for all shape combinations (except circle-circle which has an analytical shortcut)
- New dispatch logic:
  - Circle vs Circle → analytical (keep current `circleCircleSAT`, it's correct)
  - All other pairs → `gjkCollision() → epaPenetration() → buildContactManifold()`
- Manifold building now uses the reference's `findContactPoints()` for proper multi-point contact with feature IDs

**Why remove SAT:** The reference demonstrates that GJK+EPA is more robust and handles all convex shapes uniformly. Our current SAT has the vertex indexing bug and complex clipping logic that's error-prone.

---

## Phase 3: Contact Solver & Collision Resolution

**Goal:** Fix restitution, make warm starting actually work, and match the reference's proven solver formulas.

### 3.1 `packages/core/src/solver/sequential-impulse.ts` (rewrite)

**Critical fixes:**

**Restitution (BUG FIX):**
```typescript
// Current (broken):
const restitution = Math.max(0, Math.max(0, 0))

// New (from reference):
const restitution = bodyA.restitution * bodyB.restitution
const relVelNormal = ... // relative velocity along normal
const bias = restitution * Math.min(relVelNormal + restitutionSlop, 0.0)
```

**ContactSolver class structure (match reference):**
- `prepare(dir, contactType, featureFlipped)`: Compute Jacobian `J = [-dir, -ra×dir, dir, rb×dir]` and effective mass `M = (J·M⁻¹·Jᵀ)⁻¹`
- `solve()`: Compute `λ = effectiveMass * -(J·v + bias)`, clamp impulse, apply
- Normal contact: `impulseSum = max(0, impulseSum + λ)` (non-penetration)
- Tangent contact: `impulseSum = clamp(impulseSum + λ, -maxFriction, maxFriction)` (Coulomb friction cone)

**Warm starting (BUG FIX):**
- Current: solvers are recreated each frame → impulses always 0
- New: `ContactManifold` stores `normalImpulse`/`tangentImpulse` per contact point
- World layer persists manifolds across frames
- `tryWarmStart(oldManifold, newManifold)`: Match contact points by feature ID, copy accumulated impulses
- Add distance threshold: skip warm start if contact points moved too far (prevents overshoot on deep penetration)

**Block solver (2-contact LCP):**
- Keep existing structure but fix the formulas to match reference exactly:
  - `K = J·M⁻¹·Jᵀ` (2×2 matrix)
  - `b' = b - K·a` (where `a` = old accumulated impulses)
  - Total enumeration: test 4 cases (both active, first only, second only, neither)
  - First valid solution breaks

**Position solver (BUG FIX):**
- Remove the broken double-transformation of contact points
- Use the reference's approach: bias = `-(beta/dt) * max(penetration - slop, 0)`
- Clamp position correction to prevent overshoot: `clampedLambda = max(lambda, -penetration)`

### 3.2 `packages/types/src/collision.ts` (extend)

**What to add:**
```typescript
export interface ContactPoint {
  localA: Vec2Readonly
  localB: Vec2Readonly
  penetration: number
  /** Feature ID for contact persistence (vertex index or -1 for circles) */
  idA: number
  idB: number
}

export interface ContactManifold {
  bodyA: BodyHandle
  bodyB: BodyHandle
  normal: Vec2Readonly
  contacts: ContactPoint[]
  /** Accumulated impulses for warm starting (persisted across frames) */
  accumulatedImpulses?: number[]
  /** Whether this manifold persisted from the previous frame */
  persistent?: boolean
}
```

### 3.3 `packages/types/src/config.ts` (extend)

**What to add:**
```typescript
export interface WorldConfig {
  // ... existing fields ...
  /** Restitution slop: velocity threshold below which restitution is ignored */
  restitutionSlop: number
  /** Enable impulse accumulation (warm starting) */
  impulseAccumulation: boolean
  /** Position correction mode */
  positionCorrection: boolean
  /** Position correction beta factor */
  positionCorrectionBeta: number
  /** Warm start distance threshold — skip if contact points moved further */
  warmStartThreshold: number
}
```

---

## Phase 4: Collision Callbacks

**Goal:** Add a contact listener system so users can react to collisions.

### 4.1 `packages/types/src/callbacks.ts` (new file)

```typescript
export interface ContactInfo {
  bodyA: BodyHandle
  bodyB: BodyHandle
  contactNormal: { x: number; y: number }
  contactPoints: Array<{ x: number; y: number }>
  totalImpulse: number
}

export interface CollisionCallbacks {
  /** Called when two bodies first start touching */
  onBeginContact?(info: ContactInfo): void
  /** Called every frame while two bodies remain in contact */
  onStayContact?(info: ContactInfo): void
  /** Called when two bodies stop touching */
  onEndContact?(bodyA: BodyHandle, bodyB: BodyHandle): void
  /** Called for sensor/trigger bodies (no physical response) */
  onTriggerEnter?(bodyA: BodyHandle, bodyB: BodyHandle): void
}
```

### 4.2 `packages/core/src/collision/contact-tracker.ts` (new file)

**What to implement:**
- Tracks which body pairs are currently in contact
- `update(manifolds)`: Compare new manifolds against previous frame
  - New pairs → `onBeginContact`
  - Existing pairs → `onStayContact`
  - Removed pairs → `onEndContact`
- Uses `BodyHandle` pair key (sorted: `min(a,b) << 16 | max(a,b)`) for O(1) lookup
- Stores `ContactInfo` per active pair for callback data

### 4.3 Integration into solveVelocity

- After solving, compute total impulse per manifold from accumulated impulses
- Pass to `ContactTracker.update()` which fires callbacks

---

## Phase 5: World Layer Integration

**Goal:** Wire everything together so the simulation actually runs.

### 5.1 `packages/world/src/world.ts` (rewrite)

**What to add:**
- `step(dt)`: Full physics step
  ```typescript
  step(dt: number) {
    this.accumulator += dt
    while (this.accumulator >= this.config.fixedTimestep) {
      this.physicsStep(this.config.fixedTimestep)
      this.accumulator -= this.config.fixedTimestep
    }
  }
  
  private physicsStep(dt: number) {
    // 1. Integrate velocities (gravity)
    integrate(this.buffers, this.bodyCount, dt, this.config.gravity)
    
    // 2. Update dynamic tree AABBs
    this.tree.updateAll(this.buffers)
    
    // 3. Broadphase → pairs
    const pairs = this.tree.queryPairs()
    
    // 4. Narrowphase → manifolds
    const newManifolds = gjkNarrowphase(this.buffers, pairs)
    
    // 5. Warm start: match old manifolds to new
    const manifolds = this.warmStartManifolds(this._prevManifolds, newManifolds)
    
    // 6. Solve velocity constraints
    solveVelocity(this.buffers, manifolds, this.config)
    
    // 7. Solve position constraints
    solvePosition(this.buffers, manifolds, this.config)
    
    // 8. Fire collision callbacks
    this.contactTracker.update(manifolds, this.config)
    
    // 9. Store manifolds for next frame
    this._prevManifolds = manifolds
  }
  ```
- `createBody(def)`: Proper body creation with position, shape, mass, friction, restitution
- `destroyBody(handle)`: Remove from dynamic tree, compact buffers
- `setCollisionCallbacks(callbacks)`: Register callback listener
- Own `DynamicTree` instance, `ContactTracker` instance

### 5.2 `packages/world/src/simulation-loop.ts` (rewrite)

**What to add:**
- Full accumulator-based stepping
- `renderAlpha` for interpolation
- `step(world, deltaTime)`: Calls `world.step(dt)` internally

### 5.3 `packages/core/src/index.ts` (update)

**What to add:**
- Export `integrate` from solver
- Export `DynamicTree` class
- Export `Simplex`, `Polytope` if needed for custom use
- Export `ContactTracker`
- Export `CollisionCallbacks` type

---

## Phase 6: Testing

### 6.1 Update existing tests

- `__tests__/broadphase.test.ts`: Test dynamic tree insert/remove/query against SAP for consistency
- `__tests__/gjk.test.ts`: Update for new simplex-based GJK + EPA
- `__tests__/pipeline.test.ts`: Remove SAT vs GJK comparison (SAT removed)
- `__tests__/narrowphase.test.ts`: Replace SAT tests with GJK+EPA tests

### 6.2 New tests

- `__tests__/dynamic-tree.test.ts`: Insert, remove, update, query, rotation, stress test
- `__tests__/simplex.test.ts`: Closest point for 1/2/3 vertex cases
- `__tests__/contact-tracker.test.ts`: Begin/stay/end callback firing
- `__tests__/solver.test.ts`: Restitution, friction, warm starting, block solver
- `__tests__/integration.test.ts`: Full step() with bodies falling, stacking, bouncing

---

## File Change Summary

| File | Action | Description |
|------|--------|-------------|
| `packages/core/src/collision/dynamic-tree.ts` | **NEW** | Incremental AABB tree with rotations |
| `packages/core/src/collision/simplex.ts` | **NEW** | GJK simplex management |
| `packages/core/src/collision/polytope.ts` | **NEW** | EPA polytope expansion |
| `packages/core/src/collision/gjk.ts` | **REWRITE** | GJK + EPA + contact generation (reference algorithm) |
| `packages/core/src/collision/narrowphase.ts` | **REWRITE** | Remove SAT, dispatch to GJK+EPA |
| `packages/core/src/collision/broadphase.ts` | **UPDATE** | Use dynamic tree instead of rebuild BVH |
| `packages/core/src/collision/contact-tracker.ts` | **NEW** | Contact persistence + callback firing |
| `packages/core/src/collision/bvh.ts` | **DELETE** | Replaced by dynamic-tree.ts |
| `packages/core/src/solver/sequential-impulse.ts` | **REWRITE** | Fix restitution, warm starting, position solver |
| `packages/core/src/index.ts` | **UPDATE** | Export new modules + integrate |
| `packages/types/src/collision.ts` | **UPDATE** | Add feature IDs, accumulated impulses to types |
| `packages/types/src/config.ts` | **UPDATE** | Add restitution/position correction settings |
| `packages/types/src/callbacks.ts` | **NEW** | Collision callback interfaces |
| `packages/types/src/index.ts` | **UPDATE** | Export new types |
| `packages/world/src/world.ts` | **REWRITE** | Full step(), body management, callback integration |
| `packages/world/src/simulation-loop.ts` | **REWRITE** | Accumulator-based stepping |
| `packages/core/__tests__/dynamic-tree.test.ts` | **NEW** | Dynamic tree tests |
| `packages/core/__tests__/simplex.test.ts` | **NEW** | Simplex closest-point tests |
| `packages/core/__tests__/contact-tracker.test.ts` | **NEW** | Callback tests |
| `packages/core/__tests__/solver.test.ts` | **NEW** | Solver behavior tests |
| `packages/core/__tests__/integration.test.ts` | **NEW** | End-to-end physics tests |
| `packages/core/__tests__/broadphase.test.ts` | **UPDATE** | Dynamic tree tests |
| `packages/core/__tests__/gjk.test.ts` | **UPDATE** | New GJK+EPA tests |
| `packages/core/__tests__/narrowphase.test.ts` | **UPDATE** | GJK-only tests |
| `packages/core/__tests__/pipeline.test.ts` | **UPDATE** | Remove SAT comparison |

---

## Key Algorithm Differences (Reference vs Current)

| Aspect | Current | Reference | New |
|--------|---------|-----------|-----|
| Broadphase tree | Rebuild every frame | Incremental insert/remove + rotations | Incremental + rotations |
| GJK simplex | Inline vertex management | `Simplex` class with `getClosest` + `shrink` | `Simplex` class |
| EPA | Edge list rebuild each iter | `Polytope` with vertex array insertion | `Polytope` class |
| Contact points | Single point from midpoint | Edge clipping with feature IDs | Multi-point with IDs |
| Restitution | Hardcoded to 0 | `bodyA.restitution * bodyB.restitution` | Product formula |
| Warm starting | Non-functional (per-frame recreation) | Feature ID matching + impulse copy | Persistent manifolds + ID matching |
| Position solver | Double-transforms points | Direct bias application | Fixed bias formula |
| Collision callbacks | None | `ContactInfo` from manifold | `ContactTracker` + callbacks |
| Narrowphase | SAT (per shape combo) + GJK fallback | GJK+EPA for all (except circle-circle) | GJK+EPA primary |

---

## Risk Mitigation

1. **GJK convergence issues:** Keep max iterations at 30, add epsilon checks, fallback to SAT for degenerate cases during transition
2. **EPA non-convergence:** Fallback to closest edge of final polytope (already in current code)
3. **Warm start instability:** Distance threshold prevents overshoot; disable warm start on first contact
4. **Performance regression:** Dynamic tree should be faster than rebuild BVH; benchmark with 1000+ bodies
5. **Breaking API changes:** `ContactPoint` gains `idA`/`idB` fields — backward compatible (optional)
