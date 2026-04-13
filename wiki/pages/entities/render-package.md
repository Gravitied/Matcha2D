---
title: "Render Package (@matcha2d/render)"
type: entity
tags: [dev-b, api, architecture]
related: [world-class.md, buffer-layout.md, fixed-timestep.md]
sources: []
created: 2026-04-06
updated: 2026-04-06
---

# Render Package

## Location

`packages/render/`

## What It Is

Debug renderer for Matcha2D. Reads body buffers and draws shapes. Target: Canvas2D and WebGL.

## Interface

```typescript
interface IRenderer {
  begin(): void
  drawBodies(buffers, count, alpha): void
  end(): void
  destroy(): void
}
```

## Implementations

- `Canvas2DRenderer` — debug renderer, reads buffers, draws shapes using Canvas2D API

## Usage Pattern

```typescript
renderer.begin()
renderer.drawBodies(world.buffers, world.bodyCount, world.renderAlpha)
renderer.end()
```

`renderAlpha` (from `World`) is the interpolation factor — `0..1` position between last and next physics state.

## DOM Requirement

`tsconfig.json` for this package includes `"lib": ["ES2017", "DOM"]` since it uses browser Canvas APIs.

## Related

- [[world-class]] — provides `renderAlpha` and buffers
- [[buffer-layout]] — what `drawBodies` reads
- [[fixed-timestep]] — source of `renderAlpha`
