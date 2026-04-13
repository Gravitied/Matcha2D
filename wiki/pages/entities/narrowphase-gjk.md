---
title: "Narrowphase: GJK + EPA"
type: entity
tags: [collision, algorithm, dev-a]
related: [collision-pipeline.md, broadphase.md, sequential-impulse-solver.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Narrowphase: GJK + EPA

## Location

`packages/core/src/collision/gjk.ts`  
`packages/core/src/collision/narrowphase.ts`  
`packages/core/src/collision/simplex.ts`  
`packages/core/src/collision/polytope.ts`

## What It Is

The narrowphase determines exact contact information (penetration depth, contact normal, contact points) for each candidate pair from the broadphase.

## Algorithm

**GJK (Gilbert-Johnson-Keerthi)** — determines if two convex shapes intersect by iteratively building a simplex in Minkowski difference space.

**EPA (Expanding Polytope Algorithm)** — when GJK finds an intersection, EPA expands the polytope to extract the penetration depth and contact normal.

**Circle-circle analytical shortcut** — skips GJK/EPA entirely; just distance check on centers.

## Support Structures

- `simplex.ts` — GJK simplex (point, line, triangle)
- `polytope.ts` — EPA polytope

## Shape System

`packages/core/src/collision/shapes.ts`:
- `ShapeHandler` interface — `support(buffers, handle, dir)` function per shape type
- `registerShapeHandler(type, handler)` — extend with new shapes
- `getShapeHandler(type)` — lookup
- Built-in handlers: **Circle**, **Box**, **Polygon**

## Dispatch

`narrowphase(buffers, pairs, method?)` in `narrowphase.ts` — thin wrapper routing to `gjkNarrowphase`.

## Output

`ContactManifold[]` — penetration depth, contact normal, contact points for each colliding pair.

## Related

- [[collision-pipeline]] — context
- [[broadphase]] — feeds pairs into narrowphase
- [[sequential-impulse-solver]] — consumes manifolds
