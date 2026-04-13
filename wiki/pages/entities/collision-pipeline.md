---
title: "Collision Pipeline"
type: entity
tags: [collision, algorithm, dev-a]
related: [broadphase.md, narrowphase-gjk.md, sequential-impulse-solver.md, contact-tracker.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Collision Pipeline

## Location

`packages/core/src/collision/pipeline.ts`

## Pipeline Flow

```
Broadphase (DynamicTree / SAP)
  -> Narrowphase (GJK + EPA for all convex shapes; circle-circle analytical)
    -> Contact manifolds
      -> Sequential impulse solver (velocity + position)
```

## Entry Points

- `collide(buffers, count, broadphaseMethod?, narrowphaseMethod?) -> ContactManifold[]` — full pipeline
- `narrowphaseDispatch(buffers, pairs, method?) -> ContactManifold[]` — narrowphase only

## Broadphase Methods

| Method | Key | Notes |
|--------|-----|-------|
| Dynamic AABB Tree | `'dynamicTree'` | Default; incremental, best for general use |
| Sort-and-Sweep | `'sap'` | Alternative; good for mostly-static scenes |
| BVH | `'bvh'` | Legacy alias |

## Narrowphase

Single method: GJK + EPA (`'gjk'`). Circle-circle has an analytical shortcut that bypasses GJK.

## Outputs

`ContactManifold[]` — each manifold has the two body handles, contact points, normals, and penetration depths. Fed directly into the sequential impulse solver.

## Related

- [[broadphase]] — broadphase implementations
- [[narrowphase-gjk]] — GJK + EPA details
- [[sequential-impulse-solver]] — consumes manifolds
- [[contact-tracker]] — tracks begin/stay/end events from manifolds
