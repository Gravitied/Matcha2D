---
title: "Data-Oriented Design (DoD)"
type: concept
tags: [performance, data-structures, architecture]
related: [buffer-layout.md, architecture-overview.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Data-Oriented Design (DoD)

## What It Is

Data-Oriented Design is an approach where data layout is chosen for how the CPU accesses it, not for how humans conceptualize it. Instead of an array of body objects (AoS — Array of Structs), Matcha2D uses SoA (Struct of Arrays): separate typed arrays for each field.

## Why Matcha2D Uses DoD

- **Cache efficiency** — iterating positions only touches `positionX`/`positionY`. AoS would load every field of every body into cache even if only position is needed.
- **WASM compatibility** — flat arrays map directly to WebAssembly linear memory, enabling future shared-memory zero-copy interop.
- **SIMD potential** — contiguous same-type data is a prerequisite for SIMD vectorization.
- **GC pressure** — no per-body JS objects means the garbage collector has less work.

## Trade-offs

- Less intuitive to read than `body.positionX` — requires `buffers.positionX[handle]`
- Body "objects" are virtual — they exist only as an index (`BodyHandle`)
- More boilerplate when accessing multiple fields for a single body

## Pattern in Matcha2D

All body data in `MatchaBuffers`. `BodyHandle` is a branded `number` — just an array index. All physics functions take `(buffers, handle)` or `(buffers, count)` signatures, never body objects.

## Related

- [[buffer-layout]] — the concrete SoA layout
- [[architecture-overview]] — design decisions
