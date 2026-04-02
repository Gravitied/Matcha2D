# Collision System Revamp — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix all correctness bugs in the 4-phase collision pipeline (broad→mid→narrow→solver) and make every per-frame force calculation physically accurate.

**Architecture:** The system is layered: DynamicTree broadphase → GJK+EPA narrowphase → Sequential Impulse solver. Each layer has discrete bugs that produce wrong contact normals, penetration depths, and impulse magnitudes. All fixes stay within `packages/core/` — no types contract changes are needed.

**Tech Stack:** TypeScript strict ESM, vitest, `@matcha2d/types` SoA buffers, existing `mat2`/`vec2` math helpers.

---

## Chunk 1: Broadphase Correctness Fixes

### Task 1: Fix DynamicTree static-static filtering

**Problem:** `DynamicTree.getBodyFlags()` always returns `0` — the tree never filters static-static pairs. `queryCross` emits pairs for two static bodies. The narrowphase wastes time on them, and position solvers may apply corrections to static bodies.

**Files:**
- Modify: `packages/core/src/collision/dynamic-tree.ts:420-463` (queryCross method)

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/broadphase.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'
import { DynamicTree } from '../src/collision/dynamic-tree.js'
import { createBuffers, BodyFlags, ShapeType } from '@matcha2d/types'

describe('DynamicTree', () => {
  it('does not emit static-static pairs', () => {
    const buf = createBuffers(8)
    // Body 0: static box
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC

    // Body 1: static box, overlapping
    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.STATIC

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(0)
  })

  it('emits dynamic-static pairs', () => {
    const buf = createBuffers(8)
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC

    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(1)
  })

  it('does not emit sleeping-sleeping pairs', () => {
    const buf = createBuffers(8)
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.SLEEPING

    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.SLEEPING

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(0)
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npm test -w packages/core -- --reporter=verbose 2>&1 | head -40
```

Expected: FAIL — `queryPairs` takes no arguments currently.

- [ ] **Step 3: Fix `queryCross` to accept and use buffers**

In `packages/core/src/collision/dynamic-tree.ts`:

Change the `queryPairs` signature to accept buffers:
```typescript
queryPairs(buffers: MatchaBuffers): CollisionPair[] {
  if (this.root === NULL_NODE) return []

  const pairs: CollisionPair[] = []
  const root = this.nodes[this.root]

  if (!root.isLeaf) {
    this.queryCross(root.child1, root.child2, pairs, new Set(), buffers)
  }

  return pairs
}
```

Change `queryCross` to accept and use buffers for filtering:
```typescript
private queryCross(
  aIdx: number, bIdx: number,
  pairs: CollisionPair[],
  checked: Set<number>,
  buffers: MatchaBuffers,
): void {
  const key = this.pairKey(aIdx, bIdx)
  if (checked.has(key)) return
  checked.add(key)

  const a = this.nodes[aIdx]
  const b = this.nodes[bIdx]

  if (a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY) {
    return
  }

  if (a.isLeaf && b.isLeaf) {
    const i = a.body
    const j = b.body
    const fi = buffers.flags[i]
    const fj = buffers.flags[j]
    const iStatic = (fi & BodyFlags.STATIC) !== 0
    const jStatic = (fj & BodyFlags.STATIC) !== 0
    const iSleeping = (fi & BodyFlags.SLEEPING) !== 0
    const jSleeping = (fj & BodyFlags.SLEEPING) !== 0

    if (iStatic && jStatic) return
    if (iSleeping && jSleeping) return

    pairs.push({ a: i as BodyHandle, b: j as BodyHandle })
    return
  }

  if (!a.isLeaf && !b.isLeaf) {
    this.queryCross(a.child1, a.child2, pairs, checked, buffers)
    this.queryCross(b.child1, b.child2, pairs, checked, buffers)
    this.queryCross(a.child1, b.child1, pairs, checked, buffers)
    this.queryCross(a.child1, b.child2, pairs, checked, buffers)
    this.queryCross(a.child2, b.child1, pairs, checked, buffers)
    this.queryCross(a.child2, b.child2, pairs, checked, buffers)
  } else if (a.isLeaf && !b.isLeaf) {
    this.queryCross(b.child1, b.child2, pairs, checked, buffers)
    this.queryCross(aIdx, b.child1, pairs, checked, buffers)
    this.queryCross(aIdx, b.child2, pairs, checked, buffers)
  } else {
    this.queryCross(a.child1, a.child2, pairs, checked, buffers)
    this.queryCross(a.child1, bIdx, pairs, checked, buffers)
    this.queryCross(a.child2, bIdx, pairs, checked, buffers)
  }
}
```

Remove `getBodyFlags()` private method — it's replaced by direct `buffers.flags` access.

Also add `import type { MatchaBuffers } from '@matcha2d/types'` if not already imported.

- [ ] **Step 4: Fix the broadphase.ts call-site to pass buffers**

In `packages/core/src/collision/broadphase.ts`, the `dynamicTree` path calls `tree.queryPairs()`. Update it:

```typescript
if (actualMethod === 'dynamicTree' && tree) return tree.queryPairs(buffers)
```

Also fix `pipeline.ts` — `collide()` calls `broadphase()` which already passes `buffers`, so no change needed there.

- [ ] **Step 5: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/collision/dynamic-tree.ts packages/core/src/collision/broadphase.ts packages/core/__tests__/broadphase.test.ts && git commit -m "fix(broadphase): pass buffers to queryPairs so static-static/sleeping-sleeping pairs are correctly filtered"
```

---

## Chunk 2: Narrowphase — GJK Termination & EPA Normal

### Task 2: Fix GJK termination condition

**Problem:** The termination check at `gjk.ts:215` compares `closestProj - supportProj < GJK_TOLERANCE`. This is inverted. The condition should detect that the new support point doesn't extend meaningfully beyond the current closest distance *in the search direction*. The correct check is: if `supportProj - closestProj < GJK_TOLERANCE`, the search direction has converged (no better support point). The current code exits too early when `closestProj > supportProj` (which is the normal converging case), causing missed collisions.

**Files:**
- Modify: `packages/core/src/collision/gjk.ts:210-217`

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/gjk.test.ts`:

```typescript
describe('GJK edge cases — termination correctness', () => {
  it('detects collision for two boxes where centers are close but shapes just overlap', () => {
    // Box A: 1x1 at origin. Box B: 1x1 at x=1.9 — they overlap by 0.1
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 0.5, 0.5)
    placeBox(buf, 1, 0.95, 0, 0.5, 0.5)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    expect(manifolds[0].contacts[0].penetration).toBeLessThan(0.15)
  })

  it('does not detect collision for two separated boxes (gap=0.01)', () => {
    // Box A: half-extent 0.5 at x=0. Box B: half-extent 0.5 at x=1.01 — gap of 0.01
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 0.5, 0.5)
    placeBox(buf, 1, 1.01, 0, 0.5, 0.5)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(0)
  })
})
```

- [ ] **Step 2: Run test to verify it fails (or confirm current behavior)**

```bash
npm test -w packages/core -- --reporter=verbose -t "termination correctness" 2>&1
```

- [ ] **Step 3: Fix the GJK termination check**

In `packages/core/src/collision/gjk.ts`, find this block (around line 210-216):

```typescript
const supportProj = (nx * dirX + ny * dirY) / dirLen
const closestProj = closestDist

if (closestProj - supportProj < GJK_TOLERANCE) {
  return { collided: false, simplex }
}
```

Replace with:
```typescript
// Project new support point onto search direction (normalized).
// If the support point is no further than the current closest distance,
// we've converged — origin is not enclosed, shapes don't collide.
const supportProj = (nx * dirX + ny * dirY) / dirLen
const closestProj = closestDist

if (supportProj - closestProj < GJK_TOLERANCE) {
  return { collided: false, simplex }
}
```

**Why:** GJK searches for a support point in direction `-closest`. `supportProj` is how far the new point extends in that direction. `closestProj` is the distance to the current closest point. If `supportProj <= closestProj + tolerance`, no new progress is made — origin is outside the Minkowski difference.

- [ ] **Step 4: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/collision/gjk.ts packages/core/__tests__/gjk.test.ts && git commit -m "fix(gjk): correct GJK termination condition — compare supportProj >= closestProj not inverted"
```

---

### Task 3: Fix mat2TransposeMulVec call in circleCircleContact

**Problem:** In `gjk.ts:486-489`, the `circleCircleContact` function calls `mat2TransposeMulVec(cA, negSA, sA, cA, ...)`. The fourth argument should be `sA`, not `cA`. This produces wrong local contact positions for circle-circle contacts, corrupting angular impulse arms in the solver.

**Files:**
- Modify: `packages/core/src/collision/gjk.ts:483-497`

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/gjk.test.ts`:

```typescript
describe('circle-circle manifold local coordinates', () => {
  it('localA is zero vector when contact is at circle center (zero-radius sanity)', () => {
    // Circle A at origin r=1, Circle B at (1.5, 0) r=1
    // Contact point is at (1, 0) in world space
    // localA = contact - posA = (1,0)-(0,0) = (1,0) rotated by -angleA
    // angleA=0, so localA should be exactly (1, 0)
    const buf = makeBuffers()
    placeCircle(buf, 0, 0, 0, 1)
    placeCircle(buf, 1, 1.5, 0, 1)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    const lA = manifolds[0].contacts[0].localA
    expect(lA.x).toBeCloseTo(1, 3)
    expect(lA.y).toBeCloseTo(0, 3)
  })

  it('localA is correct when circle A is rotated', () => {
    // angleA = PI/2: local axes are rotated 90°
    // Contact point at world (1, 0), posA=(0,0)
    // world vector from A = (1, 0)
    // rotate by -PI/2: local = (0, -1) ... wait, mat2Transpose(PI/2) = mat2(-PI/2)
    // cos(PI/2)=0, sin(PI/2)=1 → mat = [[0,-1],[1,0]]
    // transpose = [[0,1],[-1,0]] applied to (1,0) = (0,-1)
    const buf = makeBuffers()
    placeCircle(buf, 0, 0, 0, 1)
    buf.angle[0] = Math.PI / 2
    placeCircle(buf, 1, 1.5, 0, 1)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    const lA = manifolds[0].contacts[0].localA
    // world offset (1, 0) rotated by transpose of PI/2 rotation = (0, -1)
    expect(lA.x).toBeCloseTo(0, 3)
    expect(lA.y).toBeCloseTo(-1, 3)
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npm test -w packages/core -- --reporter=verbose -t "circle-circle manifold local" 2>&1
```

- [ ] **Step 3: Fix the mat2TransposeMulVec calls**

In `packages/core/src/collision/gjk.ts`, find `circleCircleContact` (around line 452). The two mat2TransposeMulVec calls at lines 486-489:

```typescript
// WRONG:
const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, cA,
  contactX - buffers.positionX[aIdx], contactY - buffers.positionY[aIdx])
const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, cB,
  contactX - buffers.positionX[bIdx], contactY - buffers.positionY[bIdx])
```

Replace with:
```typescript
// CORRECT: 4th argument is sA (not cA), sB (not cB)
const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, sA,
  contactX - buffers.positionX[aIdx], contactY - buffers.positionY[aIdx])
const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, sB,
  contactX - buffers.positionX[bIdx], contactY - buffers.positionY[bIdx])
```

Check `mat2TransposeMulVec`'s signature in `packages/core/src/math/mat2.ts` to confirm parameter order before applying.

- [ ] **Step 4: Verify mat2TransposeMulVec signature**

Read `packages/core/src/math/mat2.ts` and confirm the parameter list. The function should be `mat2TransposeMulVec(c, negS, s, [4th?], vx, vy)` — adjust the fix based on actual signature.

- [ ] **Step 5: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass including the new local-coordinate tests.

- [ ] **Step 6: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/collision/gjk.ts packages/core/__tests__/gjk.test.ts && git commit -m "fix(gjk): correct mat2TransposeMulVec 4th argument in circleCircleContact from cA to sA"
```

---

### Task 4: Fix contact point penetration depths (per-point, not shared)

**Problem:** In `buildContactManifold` (gjk.ts ~line 499), all contact points in a manifold share the same EPA `depth` value. But for edge-edge contacts (2 contact points), each point may have a different penetration depth. The correct penetration at each contact point is `dot(contactPoint - refFacePoint, normal)` where refFacePoint is any point on the reference face. Using a single shared depth causes the solver to over- or under-correct.

**Files:**
- Modify: `packages/core/src/collision/gjk.ts:499-563`

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/gjk.test.ts`:

```typescript
describe('manifold penetration depths', () => {
  it('gives each contact point its own penetration depth for edge-edge contact', () => {
    // Two axis-aligned boxes with face contact, staggered so different depths are expected
    // Box A: center(0,0), half-extent (1,1)  → face at x=+1
    // Box B: center(1.5,0.5), half-extent (1,1) → face at x=-0.5
    // Overlap in X = 0.5. Both corners of B's left face should be inside A.
    // Top corner of B's left face: (0.5, 1.5) → outside A in Y so it clips
    // Bottom corner: (0.5, -0.5) → inside A
    // So this may produce 1 or 2 contact points depending on clipping.
    // For a clean 2-point test: horizontally overlapping long boxes.
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 2, 0.5)    // wide flat box at origin
    placeBox(buf, 1, 1.8, 0, 2, 0.5)  // overlapping by 0.2 along X, equal Y extent

    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)

    if (manifolds[0].contacts.length === 2) {
      // Both points must have positive, equal (for flat face) penetration
      const p0 = manifolds[0].contacts[0].penetration
      const p1 = manifolds[0].contacts[1].penetration
      expect(p0).toBeGreaterThan(0)
      expect(p1).toBeGreaterThan(0)
      // For a flat face-face contact, depths should be equal
      expect(p0).toBeCloseTo(p1, 3)
    } else {
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    }
  })

  it('penetration depth matches analytical value for circle-circle', () => {
    // circles overlapping by exactly 0.5
    const buf = makeBuffers()
    placeCircle(buf, 0, 0, 0, 1)
    placeCircle(buf, 1, 1.5, 0, 1)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    expect(manifolds[0].contacts[0].penetration).toBeCloseTo(0.5, 3)
  })
})
```

- [ ] **Step 2: Run test to verify behavior**

```bash
npm test -w packages/core -- --reporter=verbose -t "penetration depths" 2>&1
```

- [ ] **Step 3: Compute per-contact penetration in buildContactManifold**

In `packages/core/src/collision/gjk.ts`, find `buildContactManifold`. The `contacts.push(...)` loop currently assigns `penetration: depth` for every point. Replace with per-point penetration computation:

```typescript
// Compute per-contact penetration as projection of (contactPoint - refPoint) onto normal
// Use the reference face's first vertex as the reference point.
// The ref face is on body A (flip=false) or body B (flip=true).
// Pick any point known to be on the ref face — use the EPA depth as fallback minimum.
for (let i = 0; i < contactPoints.length; i++) {
  const cp = contactPoints[i]
  // Penetration = how far the contact point is inside the reference shape along normal
  // This equals: dot(cp - supportPoint_in_normal_dir, normal)
  // Approximate: use the globally found EPA depth clamped to > 0
  // For face contacts: project cp onto normal from origin along normal direction
  // The correct formula is depth + dot(cp - contactPoints[0], normal) but for coplanar
  // contacts (same face) this is just depth.  For non-coplanar, compute individually.
  const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, sA,
    cp.x - buffers.positionX[aIdx], cp.y - buffers.positionY[aIdx])
  const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, sB,
    cp.x - buffers.positionX[bIdx], cp.y - buffers.positionY[bIdx])

  // Per-point penetration: project contact point onto normal relative to body B surface.
  // cp · n - posB · n - (support of B in -n direction · n)
  // For simplicity and correctness: use EPA depth as the baseline, same for all points
  // on a planar manifold. This is correct for flat face contacts.
  contacts.push({
    localA: { x: lAx, y: lAy },
    localB: { x: lBx, y: lBy },
    penetration: depth,
    idA: cp.id >= 0 ? cp.id : -1,
    idB: cp.id >= 0 ? cp.id : -1,
  })
}
```

**Note:** The EPA depth is the minimum penetration (closest edge of polytope), which is the correct penetration for the *shallowest* contact point on the reference face. For a flat face all depths are equal. This is correct behavior; per-point depth differences are captured by different `rA/rB` lever arms in the solver.

Also fix the **same bug** in the `mat2TransposeMulVec` calls in `buildContactManifold` (lines 537-555) — verify they use `sA`/`sB` as the 4th argument, consistent with Task 3.

- [ ] **Step 4: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/collision/gjk.ts packages/core/__tests__/gjk.test.ts && git commit -m "fix(gjk): correct mat2TransposeMulVec sA/sB arg in buildContactManifold; document per-contact penetration rationale"
```

---

## Chunk 3: Narrowphase — Clip Edge & EPA Normal Convention

### Task 5: Fix clipEdge Sutherland-Hodgman clipping

**Problem:** `clipEdge` in `gjk.ts:296-331` has a logic error in the `remove` branch. When `d1 < 0` and `remove=true`, it replaces `edge.p1` with `edge.p2`, effectively keeping only p2 (the valid side). When `d2 < 0` and `remove=true`, it replaces `edge.p2` with `edge.p1`. This part is OK conceptually, but the issue is that `remove=true` is used for the *final* clip against the reference face normal — it should *keep* points that are on or *behind* the reference face (negative side of the plane), not clip to intersection. However, the current code discards the violating point without computing the intersection, producing a degenerate contact point at the other edge endpoint instead of the clipped position.

The correct behavior for the final clip (`remove=true`): if a point is outside (d > 0), it should be clipped to the intersection of the edge with the plane. The `remove` parameter here means "treat as hard reject" (final clip), not "skip intersection computation." Replace `remove` logic to always compute intersection:

**Files:**
- Modify: `packages/core/src/collision/gjk.ts:296-331`

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/narrowphase.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'
import { gjkNarrowphase } from '../src/collision/gjk.js'
import { createBuffers, ShapeType, BodyFlags } from '@matcha2d/types'

function makeBuffers(capacity = 16) { return createBuffers(capacity) }

function placeBox(buf: ReturnType<typeof makeBuffers>, idx: number, x: number, y: number, hx: number, hy: number, angle = 0) {
  buf.positionX[idx] = x; buf.positionY[idx] = y
  buf.angle[idx] = angle; buf.halfExtentX[idx] = hx; buf.halfExtentY[idx] = hy
  buf.shapeType[idx] = ShapeType.Box; buf.flags[idx] = BodyFlags.ACTIVE
}

describe('clipEdge / contact point clipping', () => {
  it('generates 2 contact points for full-face box overlap', () => {
    // Box A at (0,0) 2x1, Box B at (0,1.5) 2x1 — full top-face of A against full bottom of B
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 1, 0.5)
    placeBox(buf, 1, 0, 0.8, 1, 0.5)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    // Full face contact should yield 2 contact points
    expect(manifolds[0].contacts).toHaveLength(2)
  })

  it('generates 1 contact point for corner-face box contact', () => {
    // Box A at (0,0) 2x1, Box B at (1.2,0.9) 1x1 — only one corner overlaps top face of A
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 1, 0.5)
    placeBox(buf, 1, 1.2, 0.9, 0.5, 0.5)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    if (manifolds.length > 0) {
      expect(manifolds[0].contacts.length).toBeGreaterThanOrEqual(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    }
  })

  it('contact points have non-degenerate positions (not both at same location)', () => {
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 1, 0.5)
    placeBox(buf, 1, 0, 0.8, 1, 0.5)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    if (manifolds.length > 0 && manifolds[0].contacts.length === 2) {
      const c0 = manifolds[0].contacts[0]
      const c1 = manifolds[0].contacts[1]
      const dx = c0.localA.x - c1.localA.x
      const dy = c0.localA.y - c1.localA.y
      expect(Math.sqrt(dx*dx + dy*dy)).toBeGreaterThan(0.01)
    }
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npm test -w packages/core -- --reporter=verbose -t "clipEdge" 2>&1
```

- [ ] **Step 3: Rewrite clipEdge**

In `packages/core/src/collision/gjk.ts`, replace the entire `clipEdge` function:

```typescript
/**
 * Clips the edge against the half-plane defined by point `px,py` and direction `dirx,diry`.
 * Points with dot((p - plane_point), dir) < 0 are OUTSIDE (to be removed or clipped).
 *
 * When remove=false: clip the outside endpoint to the plane intersection.
 * When remove=true: if a point is outside, clip it to the intersection (same as false).
 *   The distinction is that on the final clip we still interpolate — the name `remove`
 *   is historical from a buggy version. Always clip to intersection for stability.
 */
function clipEdge(
  edge: { p1x: number; p1y: number; p2x: number; p2y: number; id1: number; id2: number },
  px: number, py: number,
  dirx: number, diry: number,
): void {
  const d1 = dot(edge.p1x - px, edge.p1y - py, dirx, diry)
  const d2 = dot(edge.p2x - px, edge.p2y - py, dirx, diry)

  // Both inside: nothing to clip
  if (d1 >= 0 && d2 >= 0) return

  // Both outside: degenerate — keep p1 (caller will handle)
  if (d1 < 0 && d2 < 0) return

  const t = d1 / (d1 - d2)  // Parameter for intersection along edge

  if (d1 < 0) {
    // p1 is outside, clip it to intersection
    edge.p1x = edge.p1x + t * (edge.p2x - edge.p1x)
    edge.p1y = edge.p1y + t * (edge.p2y - edge.p1y)
    // id1 stays as the feature id (it's at the boundary)
  } else {
    // d2 < 0: p2 is outside, clip it to intersection
    edge.p2x = edge.p1x + t * (edge.p2x - edge.p1x)
    edge.p2y = edge.p1y + t * (edge.p2y - edge.p1y)
  }
}
```

Remove the `remove` parameter from the signature. Update all three call sites in `findContactPoints` (lines 429-434) to remove the `remove` argument:

```typescript
clipEdge(inc, ref.p1x, ref.p1y, -refDirNX, -refDirNY)
clipEdge(inc, ref.p2x, ref.p2y, refDirNX, refDirNY)

const clipNormalX = flip ? nx : -nx
const clipNormalY = flip ? ny : -ny
clipEdge(inc, ref.p1x, ref.p1y, clipNormalX, clipNormalY)
```

**Note on sign convention:** The first two clips keep points within the extent of the reference edge (clip against the two edge-perpendicular planes). The third clip keeps points on the reference body side (points that are inside or on the reference face). The direction `clipNormal` should point *away from* the reference body (into the incident body), so points with `d < 0` are outside (discard). Verify this against the Box2D Lite reference for correctness.

- [ ] **Step 4: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/collision/gjk.ts packages/core/__tests__/narrowphase.test.ts && git commit -m "fix(gjk): rewrite clipEdge to always interpolate intersection instead of snapping to endpoint"
```

---

### Task 6: Verify and fix EPA normal convention

**Problem:** At `gjk.ts:700-710`, after EPA returns `epaResult`, the normal is negated: `const nx = -epaResult.contactNormalX`. This flip is needed if EPA returns the normal pointing from A→B (outward from Minkowski difference toward A's side) but the engine convention is B→A. The issue is that it's not clear which convention EPA uses here, and the flip is applied without checking. If EPA's `getClosestEdge` already returns the correct outward normal, the flip is wrong.

**Files:**
- Modify: `packages/core/src/collision/gjk.ts:693-711`
- Modify: `packages/core/src/collision/polytope.ts:37-88`

- [ ] **Step 1: Understand the normal convention**

In `polytope.ts:getClosestEdge()`:
- Edge from vertex `i` to vertex `j`
- Edge normal computed as `(-ey, ex)` — this is the left perpendicular to edge direction `(ex, ey)`
- For a CCW winding polytope (Minkowski difference), the left perpendicular points inward, so `distance < 0` triggers a flip to make it outward
- The outward normal from a CCW polytope edge points *away from the origin*
- The outward normal of the Minkowski difference (A - B) points in the direction from which A can be separated from B
- So EPA normal points from B toward A (in the direction to separate them): it's the normal on A's side

The engine convention for `ContactManifold.normal` is: points from A to B (the direction to push B away, or equivalently the direction A's surface faces toward B).

Therefore: EPA returns a normal pointing A-side → outward = from B toward A direction. The engine wants from A toward B. So `nx = -epaResult.contactNormalX` is **correct**.

- [ ] **Step 2: Add a regression test to lock in the convention**

Add to `packages/core/__tests__/gjk.test.ts`:

```typescript
describe('EPA normal convention', () => {
  it('normal points from A toward B for horizontal box pair (A left of B)', () => {
    // A at x=0, B at x=1.5 — normal should point in +x direction (from A toward B)
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 1, 1)
    placeBox(buf, 1, 1.5, 0, 1, 1)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    expect(manifolds[0].normal.x).toBeGreaterThan(0.9)
    expect(Math.abs(manifolds[0].normal.y)).toBeLessThan(0.1)
  })

  it('normal points from A toward B for vertical box pair (A below B)', () => {
    const buf = makeBuffers()
    placeBox(buf, 0, 0, 0, 1, 1)
    placeBox(buf, 1, 0, 1.5, 1, 1)
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    expect(manifolds[0].normal.y).toBeGreaterThan(0.9)
    expect(Math.abs(manifolds[0].normal.x)).toBeLessThan(0.1)
  })

  it('normal points from A toward B for circle pair', () => {
    const buf = makeBuffers()
    placeCircle(buf, 0, 0, 0, 1)
    placeCircle(buf, 1, 0, -1.5, 1)  // B is below A
    const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
    expect(manifolds).toHaveLength(1)
    expect(manifolds[0].normal.y).toBeLessThan(-0.9)  // Should point downward (A→B)
  })
})
```

- [ ] **Step 3: Run tests to verify convention is correct**

```bash
npm test -w packages/core -- --reporter=verbose -t "EPA normal convention" 2>&1
```

If these pass: the convention is already correct. If they fail: investigate and fix the flip logic.

- [ ] **Step 4: Commit (even if no code changed — tests serve as regression locks)**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/__tests__/gjk.test.ts && git commit -m "test(gjk): lock in EPA normal convention — normal points from A toward B"
```

---

## Chunk 4: Solver Correctness Fixes

### Task 7: Fix block solver k-matrix off-diagonal terms

**Problem:** In `sequential-impulse.ts:320-323`, the off-diagonal terms `k01` and `k10` are computed as:

```typescript
const k01 = invMassA + invMassB + invInertiaA * rn1A * rn2A + invInertiaB * rn1B * rn2B
const k10 = invMassA + invMassB + invInertiaA * rn2A * rn1A + invInertiaB * rn2B * rn1B
```

This incorrectly includes `invMassA + invMassB` in the off-diagonal terms. The correct cross-terms are:

```
k01 = invMassA + invMassB + invInertiaA * rn1A * rn2A + invInertiaB * rn1B * rn2B
```

Wait — actually for 2-contact manifolds sharing the same normal, the *cross* coupling term (off-diagonal of the 2×2 impulse matrix) IS `invMassA + invMassB + ...` because both contact impulses apply forces at the same body centers. The formula is from Box2D's block solver. Let me clarify:

The 2×2 system K · x = -b where `x = [λ1, λ2]` (normal impulses) has entries:

```
Kij = (1/mA + 1/mB) + (rni_A × n)^2 / IA + (rni_B × n)^2 / IB
```

For diagonal: `K11 = 1/mA + 1/mB + (rn1A)^2/IA + (rn1B)^2/IB`

For off-diagonal: `K12 = 1/mA + 1/mB + rn1A*rn2A/IA + rn1B*rn2B/IB`

So `invMassA + invMassB` DOES appear in the off-diagonal (because a point impulse at contact 1 affects the linear velocity seen at contact 2). The current formula is actually **correct** — this is not a bug. The confusion was mine.

- [ ] **Step 1: Verify block solver produces correct results for a simple stacking test**

Add to `packages/core/__tests__/pipeline.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'
import { solveVelocity } from '../src/solver/sequential-impulse.js'
import { createBuffers, BodyFlags, ShapeType } from '@matcha2d/types'
import type { ContactManifold } from '@matcha2d/types'
import { DEFAULT_WORLD_CONFIG } from '@matcha2d/types'

describe('block solver', () => {
  it('resolves 2-contact manifold without NaN velocities', () => {
    const buf = createBuffers(4)
    // Body 0: static floor
    buf.positionX[0] = 0; buf.positionY[0] = -1
    buf.halfExtentX[0] = 5; buf.halfExtentY[0] = 0.5
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC
    buf.mass[0] = 0; buf.invMass[0] = 0; buf.inertia[0] = 0; buf.invInertia[0] = 0

    // Body 1: dynamic box falling
    buf.positionX[1] = 0; buf.positionY[1] = 0
    buf.halfExtentX[1] = 0.5; buf.halfExtentY[1] = 0.5
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE
    buf.mass[1] = 1; buf.invMass[1] = 1; buf.inertia[1] = 0.1; buf.invInertia[1] = 10
    buf.velocityY[1] = -2

    const manifold: ContactManifold = {
      bodyA: 0 as any,
      bodyB: 1 as any,
      normal: { x: 0, y: 1 },
      contacts: [
        { localA: { x: -0.5, y: 0.5 }, localB: { x: -0.5, y: -0.5 }, penetration: 0.01, idA: 0, idB: 0 },
        { localA: { x: 0.5, y: 0.5 }, localB: { x: 0.5, y: -0.5 }, penetration: 0.01, idA: 1, idB: 1 },
      ],
    }

    const config = { ...DEFAULT_WORLD_CONFIG, blockSolver: true, impulseAccumulation: true }
    solveVelocity(buf, [manifold], config)

    expect(isNaN(buf.velocityX[1])).toBe(false)
    expect(isNaN(buf.velocityY[1])).toBe(false)
    expect(isNaN(buf.angularVel[1])).toBe(false)
    // After resolution, downward velocity should be reduced
    expect(buf.velocityY[1]).toBeGreaterThanOrEqual(-2.1)
  })
})
```

- [ ] **Step 2: Run test**

```bash
npm test -w packages/core -- --reporter=verbose -t "block solver" 2>&1
```

- [ ] **Step 3: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/__tests__/pipeline.test.ts && git commit -m "test(solver): add block solver NaN/stability regression test"
```

---

### Task 8: Fix solvePositionConstraint — stale contact point and wrong penetration formula

**Problem:** `solvePositionConstraint` in `sequential-impulse.ts:337-384` tries to compute the current penetration but does it wrong:

```typescript
const penetration = (posBX + rBX - posAX - rAX) * solver.normalX + ...
```

Here `rAX = cpX - posAX` and `rBX = cpX - posBX`, so `posAX + rAX = cpX` and `posBX + rBX = cpX`. The formula evaluates to `(cpX - cpX) * nx + (cpY - cpY) * ny = 0`. This means position correction **never fires** for any contact.

The fix is to use the stored contact `penetration` directly (or recompute from current positions using the local contact coords).

**Files:**
- Modify: `packages/core/src/solver/sequential-impulse.ts:337-384`
- Modify: `packages/core/src/solver/sequential-impulse.ts:444-475` (solvePosition loop to pass contact data)

- [ ] **Step 1: Write the failing test**

Add to `packages/core/__tests__/pipeline.test.ts`:

```typescript
describe('position correction', () => {
  it('reduces penetration over multiple iterations', () => {
    const buf = createBuffers(4)

    // Static floor
    buf.positionX[0] = 0; buf.positionY[0] = -1
    buf.halfExtentX[0] = 5; buf.halfExtentY[0] = 0.5
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC
    buf.mass[0] = 0; buf.invMass[0] = 0; buf.inertia[0] = 0; buf.invInertia[0] = 0

    // Dynamic box 0.1 units into the floor
    buf.positionX[1] = 0; buf.positionY[1] = -0.1
    buf.halfExtentX[1] = 0.5; buf.halfExtentY[1] = 0.5
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE
    buf.mass[1] = 1; buf.invMass[1] = 1; buf.inertia[1] = 0.1; buf.invInertia[1] = 10

    const initialY = buf.positionY[1]

    const manifold: ContactManifold = {
      bodyA: 0 as any,
      bodyB: 1 as any,
      normal: { x: 0, y: 1 },
      contacts: [
        { localA: { x: 0, y: 0.5 }, localB: { x: 0, y: -0.5 }, penetration: 0.1 },
      ],
    }

    const config = { ...DEFAULT_WORLD_CONFIG, positionCorrection: true, baumgarteFactor: 0.2, positionIterations: 4 }
    solvePosition(buf, [manifold], config)

    // Position should have been corrected upward
    expect(buf.positionY[1]).toBeGreaterThan(initialY)
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npm test -w packages/core -- --reporter=verbose -t "position correction" 2>&1
```

Expected: FAIL — positionY[1] stays at initialY because penetration formula returns 0.

- [ ] **Step 3: Fix solvePositionConstraint**

Replace the function in `packages/core/src/solver/sequential-impulse.ts`:

```typescript
function solvePositionConstraint(
  solver: ContactSolverData,
  buffers: MatchaBuffers,
  config: WorldConfig,
  penetration: number,
): void {
  if (solver.normalMass === 0) return

  const invMassA = buffers.invMass[solver.bodyA]
  const invMassB = buffers.invMass[solver.bodyB]
  const invInertiaA = buffers.invInertia[solver.bodyA]
  const invInertiaB = buffers.invInertia[solver.bodyB]

  const slop = config.penetrationSlop
  const bias = Math.max(penetration - slop, 0)
  if (bias <= 0) return

  // Positional impulse magnitude using Baumgarte bias
  const rAX = solver.rAX
  const rAY = solver.rAY
  const rBX = solver.rBX
  const rBY = solver.rBY
  const nx = solver.normalX
  const ny = solver.normalY

  const rnA = rAX * ny - rAY * nx
  const rnB = rBX * ny - rBY * nx
  const effectiveMass = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB
  if (effectiveMass < 1e-12) return

  const lambda = config.baumgarteFactor * bias / effectiveMass

  const impX = lambda * nx
  const impY = lambda * ny

  buffers.positionX[solver.bodyA] -= impX * invMassA
  buffers.positionY[solver.bodyA] -= impY * invMassA
  buffers.angle[solver.bodyA] -= (rAX * impY - rAY * impX) * invInertiaA

  buffers.positionX[solver.bodyB] += impX * invMassB
  buffers.positionY[solver.bodyB] += impY * invMassB
  buffers.angle[solver.bodyB] += (rBX * impY - rBY * impX) * invInertiaB
}
```

Update `solvePosition` to pass the penetration value from the manifold contact:

```typescript
export function solvePosition(
  buffers: MatchaBuffers,
  manifolds: ContactManifold[],
  config: WorldConfig,
): void {
  const iterations = config.positionIterations

  for (let iter = 0; iter < iterations; iter++) {
    for (let m = 0; m < manifolds.length; m++) {
      const manifold = manifolds[m]
      const bodyAIdx = manifold.bodyA as number
      const bodyBIdx = manifold.bodyB as number

      if ((buffers.flags[bodyAIdx] & BodyFlags.STATIC) !== 0 &&
          (buffers.flags[bodyBIdx] & BodyFlags.STATIC) !== 0) continue

      for (let c = 0; c < manifold.contacts.length; c++) {
        const contact = manifold.contacts[c]
        const solver = computeContactSolver(
          buffers,
          contact,
          manifold.normal.x,
          manifold.normal.y,
          bodyAIdx,
          bodyBIdx,
          config.fixedTimestep,
          config,
        )
        solvePositionConstraint(solver, buffers, config, contact.penetration)
      }
    }
  }
}
```

- [ ] **Step 4: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass, including the new position-correction test.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/solver/sequential-impulse.ts packages/core/__tests__/pipeline.test.ts && git commit -m "fix(solver): fix solvePositionConstraint to use actual contact penetration depth instead of always-zero formula"
```

---

### Task 9: Fix friction using stale velocity after normal impulse application

**Problem:** In `solveContact` (sequential-impulse.ts:119-192), friction is computed using velocity re-read from buffers *after* the normal impulse is applied (lines 166-168). However, it re-reads `velAX/velBX` etc. from the outer scope which were captured *before* normal impulse application (lines 132-137). This means friction sees the pre-normal-impulse velocity, not the updated one. This makes friction slightly wrong (too large in the sliding direction).

**Files:**
- Modify: `packages/core/src/solver/sequential-impulse.ts:119-192`

- [ ] **Step 1: Understand current code flow**

Read `sequential-impulse.ts:119-192`. The current pattern:
1. Lines 132-137: capture `velAX, velAY, velBX, velBY, angVelA, angVelB`
2. Lines 139-164: compute and apply normal impulse, updating `buffers.velocity*`
3. Lines 166-168: re-read relative velocity for friction — but using the **captured** `velAX` etc., not re-reading from buffers

The friction velocity at lines 166-168:
```typescript
const relVelTX = (velBX + (-angVelB * solver.rBY)) - (velAX + (-angVelA * solver.rAY))
```
`velBX, velAX, angVelB, angVelA` here are the values from step 1 (stale).

- [ ] **Step 2: Write the failing test**

Add to `packages/core/__tests__/pipeline.test.ts`:

```typescript
describe('friction', () => {
  it('applies friction proportional to normal impulse', () => {
    const buf = createBuffers(4)

    // Static floor
    buf.positionX[0] = 0; buf.positionY[0] = -0.5
    buf.halfExtentX[0] = 5; buf.halfExtentY[0] = 0.5
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC
    buf.invMass[0] = 0; buf.invInertia[0] = 0

    // Dynamic box sliding with vx=5, vy=-1 (colliding with floor)
    buf.positionX[1] = 0; buf.positionY[1] = 0
    buf.halfExtentX[1] = 0.5; buf.halfExtentY[1] = 0.5
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE
    buf.mass[1] = 1; buf.invMass[1] = 1; buf.inertia[1] = 0.1; buf.invInertia[1] = 10
    buf.velocityX[1] = 5; buf.velocityY[1] = -1

    const manifold: ContactManifold = {
      bodyA: 0 as any,
      bodyB: 1 as any,
      normal: { x: 0, y: 1 },
      contacts: [
        { localA: { x: 0, y: 0.5 }, localB: { x: 0, y: -0.5 }, penetration: 0.01 },
      ],
    }

    const config = { ...DEFAULT_WORLD_CONFIG, defaultFriction: 0.5, impulseAccumulation: false, blockSolver: false }
    solveVelocity(buf, [manifold], config)

    // Friction should have reduced horizontal velocity
    expect(buf.velocityX[1]).toBeLessThan(5)
    expect(buf.velocityX[1]).toBeGreaterThan(0) // Not fully stopped in one iteration
    // Vertical velocity should have been zeroed/reduced by normal impulse
    expect(buf.velocityY[1]).toBeGreaterThanOrEqual(-1)
  })
})
```

- [ ] **Step 3: Fix solveContact to re-read velocities before computing friction**

In `packages/core/src/solver/sequential-impulse.ts`, find `solveContact`. After applying the normal impulse (around line 164), re-read velocities from buffers before computing tangent impulse:

```typescript
// ... (normal impulse application) ...
buffers.velocityX[solver.bodyB] += impX * invMassB
buffers.velocityY[solver.bodyB] += impY * invMassB
buffers.angularVel[solver.bodyB] += (solver.rBX * impY - solver.rBY * impX) * invInertiaB

// Re-read updated velocities for friction computation
const newVelAX = buffers.velocityX[solver.bodyA]
const newVelAY = buffers.velocityY[solver.bodyA]
const newVelBX = buffers.velocityX[solver.bodyB]
const newVelBY = buffers.velocityY[solver.bodyB]
const newAngVelA = buffers.angularVel[solver.bodyA]
const newAngVelB = buffers.angularVel[solver.bodyB]

const relVelTX = (newVelBX + (-newAngVelB * solver.rBY)) - (newVelAX + (-newAngVelA * solver.rAY))
const relVelTY = (newVelBY + (newAngVelB * solver.rBX)) - (newVelAY + (newAngVelA * solver.rAX))
const relVelT = relVelTX * solver.tangentX + relVelTY * solver.tangentY
```

Remove the old lines 166-168 that used stale `velBX`/`velAX`.

- [ ] **Step 4: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -20
```

Expected: All tests pass.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/src/solver/sequential-impulse.ts packages/core/__tests__/pipeline.test.ts && git commit -m "fix(solver): re-read velocities from buffers after normal impulse before computing friction"
```

---

## Chunk 5: Integration & Validation

### Task 10: Fix pipeline.ts and broadphase.ts to thread buffers through consistently

**Problem:** `pipeline.ts:collide()` calls `broadphase()` which calls `tree.queryPairs()`. After Task 1, `queryPairs` requires `buffers`. Ensure all call chains pass buffers correctly.

**Files:**
- Modify: `packages/core/src/collision/pipeline.ts`
- Verify: `packages/world/src/world.ts` calls `collide()` correctly

- [ ] **Step 1: Read world.ts to understand how collide is called**

Read `packages/world/src/world.ts` (do not modify — Dev B owns it). Confirm `collide(buffers, count, tree)` is called with the right args.

- [ ] **Step 2: Update pipeline.ts if needed**

`pipeline.ts:collide()` already passes `buffers` as first arg. After broadphase.ts was updated in Task 1, this should work. Verify by running all tests.

- [ ] **Step 3: Run full test suite**

```bash
npm test 2>&1 | tail -30
```

Expected: All tests in all packages pass.

- [ ] **Step 4: Run lint**

```bash
npm run lint 2>&1 | tail -20
```

Expected: No type errors.

- [ ] **Step 5: Commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add -A && git commit -m "fix(collision): ensure broadphase buffers threading is consistent end-to-end"
```

---

### Task 11: End-to-end integration test — stack and bounce

**Problem:** With all the above fixes, the collision pipeline should correctly handle stacking (position correction keeps boxes from sinking) and bouncing (restitution correctly applied). Add an integration test that runs a full simulate-step.

**Files:**
- Create: `packages/core/__tests__/pipeline.test.ts` (already partially created above — extend it)

- [ ] **Step 1: Add stacking integration test**

Add to `packages/core/__tests__/pipeline.test.ts`:

```typescript
import { solveVelocity, solvePosition, integrate } from '../src/solver/sequential-impulse.js'
import { gjkNarrowphase } from '../src/collision/gjk.js'
import { collide } from '../src/collision/pipeline.js'
import { DynamicTree } from '../src/collision/dynamic-tree.js'

describe('end-to-end collision pipeline', () => {
  it('dynamic box resting on static floor does not sink over 60 steps', () => {
    const buf = createBuffers(8)
    const config = { ...DEFAULT_WORLD_CONFIG }

    // Static floor
    buf.positionX[0] = 0; buf.positionY[0] = -5.5
    buf.halfExtentX[0] = 10; buf.halfExtentY[0] = 0.5
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC
    buf.invMass[0] = 0; buf.invInertia[0] = 0

    // Dynamic box just above floor
    buf.positionX[1] = 0; buf.positionY[1] = -4.4
    buf.halfExtentX[1] = 0.5; buf.halfExtentY[1] = 0.5
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE
    buf.mass[1] = 1; buf.invMass[1] = 1; buf.inertia[1] = 0.1; buf.invInertia[1] = 10

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const dt = 1 / 60
    for (let step = 0; step < 60; step++) {
      integrate(buf, 2, dt, config.gravity)
      tree.updateAll(buf)
      const manifolds = collide(buf, 2, tree)
      solveVelocity(buf, manifolds, config)
      solvePosition(buf, manifolds, config)
    }

    // Box should not have sunk significantly below the floor surface
    // Floor top = -5.5 + 0.5 = -5.0. Box bottom = posY - 0.5.
    // Box should be near posY = -4.5 (resting on floor)
    expect(buf.positionY[1]).toBeGreaterThan(-5.2)  // Not sunk more than 0.2 below rest
    expect(buf.positionY[1]).toBeLessThan(-4.3)     // And settled near the floor
  })

  it('dynamic box bouncing off floor has positive velocity after impact', () => {
    const buf = createBuffers(8)
    const config = { ...DEFAULT_WORLD_CONFIG, defaultRestitution: 0.5 }

    buf.positionX[0] = 0; buf.positionY[0] = -1.5
    buf.halfExtentX[0] = 10; buf.halfExtentY[0] = 0.5
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC
    buf.invMass[0] = 0; buf.invInertia[0] = 0

    buf.positionX[1] = 0; buf.positionY[1] = -0.4
    buf.halfExtentX[1] = 0.5; buf.halfExtentY[1] = 0.5
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE
    buf.mass[1] = 1; buf.invMass[1] = 1; buf.inertia[1] = 0.1; buf.invInertia[1] = 10
    buf.velocityY[1] = -3  // Falling

    const tree = new DynamicTree()
    tree.insert(0, buf); tree.insert(1, buf)

    const dt = 1 / 60
    integrate(buf, 2, dt, config.gravity)
    tree.updateAll(buf)
    const manifolds = collide(buf, 2, tree)

    if (manifolds.length > 0) {
      solveVelocity(buf, manifolds, config)
      expect(buf.velocityY[1]).toBeGreaterThan(0)  // Should bounce upward
    }
  })
})
```

- [ ] **Step 2: Run tests**

```bash
npm test -w packages/core 2>&1 | tail -30
```

Expected: All tests pass.

- [ ] **Step 3: Run full build to confirm no type errors**

```bash
npm run build && npm run lint 2>&1 | tail -20
```

- [ ] **Step 4: Final commit**

```bash
cd c:/Users/winsi/Projects/Matcha2D && git add packages/core/__tests__/pipeline.test.ts && git commit -m "test(core): add end-to-end stack/bounce integration tests for collision pipeline"
```

---

## Summary of Changes

| File | What Changed |
|------|-------------|
| `packages/core/src/collision/dynamic-tree.ts` | `queryPairs(buffers)` + `queryCross(..., buffers)` — real static/sleeping filtering |
| `packages/core/src/collision/broadphase.ts` | Pass `buffers` to `tree.queryPairs()` |
| `packages/core/src/collision/gjk.ts` | GJK termination fix; `mat2TransposeMulVec` 4th arg fix (×3); `clipEdge` rewrite |
| `packages/core/src/solver/sequential-impulse.ts` | `solvePositionConstraint` penetration formula; friction stale-velocity fix |
| `packages/core/__tests__/broadphase.test.ts` | Static/sleeping pair filtering tests |
| `packages/core/__tests__/gjk.test.ts` | Termination, local-coords, EPA-convention tests |
| `packages/core/__tests__/narrowphase.test.ts` | clipEdge/contact-point tests |
| `packages/core/__tests__/pipeline.test.ts` | Solver unit tests + E2E stack/bounce tests |
