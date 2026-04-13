---
title: "Buffer Layout (SoA Typed Arrays)"
type: concept
tags: [data-structures, performance, architecture]
related: [architecture-overview.md, data-oriented-design.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Buffer Layout (SoA Typed Arrays)

## What It Is

All body data lives in flat `Float32Array`/`Uint8Array` buffers using a **Structure of Arrays (SoA)** layout. Bodies are identified by an index (`BodyHandle`) into these arrays.

Allocated with `createBuffers(capacity)`. Max capacity: **8192 bodies** per world (`MAX_BODIES`).

## The `MatchaBuffers` Interface

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

## Constants

- `MAX_BODIES` = 8192 — maximum bodies per world
- `MAX_VERTICES_PER_SHAPE` = 16 — max polygon vertices
- `shapeVerticesX/Y` size = `capacity × MAX_VERTICES_PER_SHAPE`

## Body & Shape Identification

- `BodyHandle` — branded `number` (index into buffer arrays). Cast with `as BodyHandle`.
- `BodyFlags` bitfield:
  - `ACTIVE (0x01)`
  - `STATIC (0x02)`
  - `SLEEPING (0x04)`
  - `SENSOR (0x08)`
- `BodyType` enum: `Dynamic (0)`, `Static (1)`, `Kinematic (2)`
- `ShapeType` enum: `Box (0)`, `Circle (1)`, `Polygon (2)`

## Why SoA?

- Cache performance: iterating all positions touches only `positionX`/`positionY` — no wasted cache lines from unrelated fields
- Future WASM shared memory compatibility — flat arrays map directly to linear memory
- Avoids GC pressure — no object per body

## Performance Rules

- Precompute `invMass` and `invInertia` at body creation, not per-frame
- No allocations in tight loops — pass output arrays + indices
- Vec2 array function signature: `(outX, outY, outIdx, aX, aY, aIdx, ...)`

## Related

- [[architecture-overview]] — overall design
- [[data-oriented-design]] — the concept behind SoA
- [[physics-backend-interface]] — how backends consume these buffers
