---
title: "Math Primitives (Vec2, Mat2, AABB)"
type: entity
tags: [math, dev-a, performance]
related: [buffer-layout.md, broadphase.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Math Primitives

## Vec2 (`packages/core/src/math/vec2.ts`)

Flat-array operations — no Vec2 objects. All functions operate on typed array indices.

Signature convention: `(outX, outY, outIdx, aX, aY, aIdx, ...)`

Key functions: `vec2Add`, `vec2Dot`, `vec2Cross`, `vec2Normalize` + scalar helpers `dot`, `cross`, `length`.

## Mat2 (`packages/core/src/math/mat2.ts`)

2×2 rotation matrix.

Key functions:
- `mat2FromAngle(angle)` — construct rotation matrix
- `mat2MulVec(m, vx, vy)` — transform vector
- `mat2TransposeMulVec(m, vx, vy)` — inverse-transform vector

## AABB (`packages/core/src/collision/aabb.ts`)

Axis-Aligned Bounding Box.

Key functions:
- `aabbOverlap(a, b)` — boolean overlap test
- `aabbMerge(a, b)` — union AABB
- `aabbContains(outer, inner)` — containment test
- `aabbArea(a)` — area
- `aabbPerimeter(a)` — perimeter
- `computeBodyAABB(buffers, handle)` — compute world AABB from body shape data

## Types (`packages/types/src/`)

- `Vec2Readonly` — read-only `{ x, y }` math type
- `AABB` — `{ minX, minY, maxX, maxY }`
- `Transform` — position + rotation

## Performance Rule

Hot-path math uses flat typed arrays, not objects. Never allocate a `{x, y}` object in a tight loop.

## Related

- [[buffer-layout]] — arrays these functions operate on
- [[broadphase]] — uses AABB functions
