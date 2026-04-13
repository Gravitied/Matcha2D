---
title: "Broadphase (Dynamic AABB Tree / SAP)"
type: entity
tags: [collision, algorithm, dev-a, performance]
related: [collision-pipeline.md, narrowphase-gjk.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Broadphase

## Location

`packages/core/src/collision/broadphase.ts`  
`packages/core/src/collision/dynamic-tree.ts`

## What It Is

The broadphase culls down the O(n²) all-pairs collision check to a small set of candidate pairs using spatial acceleration structures. Only candidate pairs go to the (expensive) narrowphase.

## Implementations

### Dynamic AABB Tree (`dynamicTree`) — **Default**

Incremental AABB tree. Bodies are inserted/removed/updated as they move.

API:
- `insert(handle, aabb)` — add body
- `remove(handle)` — remove body
- `updateAll(buffers, count)` — refit all AABBs
- `queryPairs()` — returns all overlapping pairs

Location: `packages/core/src/collision/dynamic-tree.ts`

Best for: general-purpose, mixed dynamic/static scenes.

### Sort-and-Sweep (`sap`)

Projects AABBs onto an axis and sorts them. Sweeps the sorted list to find overlapping intervals.

Best for: mostly-static or highly coherent scenes.

### BVH (`bvh`)

Legacy alias.

## Dispatch

`broadphase(buffers, count, method?)` in `broadphase.ts` — dispatches to the selected implementation.

## Output

`CollisionPair[]` — pairs of `BodyHandle`s whose AABBs overlap.

## AABB Computation

`computeBodyAABB(buffers, handle)` in `packages/core/src/collision/aabb.ts` — computes a body's world-space AABB from its shape data.

## Related

- [[collision-pipeline]] — how broadphase fits in the pipeline
- [[narrowphase-gjk]] — next stage after broadphase
