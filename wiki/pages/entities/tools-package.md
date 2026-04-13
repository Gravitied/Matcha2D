---
title: "Tools Package (@matcha2d/tools)"
type: entity
tags: [dev-b, api]
related: [world-class.md, buffer-layout.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Tools Package

## Location

`packages/tools/`

## What It Is

Serialization, profiling, and debug utilities for Matcha2D scenes.

## Exports

### Serialization

```typescript
serializeBuffers(buffers, count) -> Uint8Array  // save scene to bytes
deserializeBuffers(data) -> { buffers, count }  // restore scene
```

Use case: save/load physics scenes, snapshot testing, replay.

### Profiler

```typescript
class Profiler {
  // per-step timing for each pipeline stage
  broadphase: number   // ms
  narrowphase: number  // ms
  solver: number       // ms
  integrate: number    // ms
}
```

Wrap around `World.step()` calls to profile performance.

## Related

- [[world-class]] — what gets profiled
- [[buffer-layout]] — what gets serialized
