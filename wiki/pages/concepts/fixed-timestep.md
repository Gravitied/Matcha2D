---
title: "Fixed Timestep & Render Interpolation"
type: concept
tags: [architecture, performance, dev-b]
related: [architecture-overview.md, world-class.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Fixed Timestep & Render Interpolation

## What It Is

The physics simulation advances in fixed-size time steps regardless of frame rate. The render layer interpolates between physics states to produce smooth visuals.

## Pattern

```typescript
accumulator += deltaTime
while (accumulator >= fixedStep) {
  physicsStep(fixedStep)
  accumulator -= fixedStep
}
renderAlpha = accumulator / fixedStep  // 0..1 interpolation factor
```

- `fixedStep` defaults to `1/60` (60 Hz physics)
- `renderAlpha` is exposed on `World` for the renderer to interpolate body positions

## Why It Matters

- Deterministic physics regardless of frame rate
- Avoids spiral-of-death: if `deltaTime` is large, multiple steps fire but the loop is bounded
- Smooth rendering even when physics runs slower than the display refresh rate

## Implementation

`SimulationLoop` (`packages/world/src/simulation-loop.ts`) owns the accumulator logic. `World.step(dt)` delegates to it.

## Config

`WorldConfig.fixedTimestep` — default `1/60`.

## Related

- [[world-class]] — exposes `renderAlpha`
- [[world-config]] — `fixedTimestep` field
