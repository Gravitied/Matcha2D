import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import { logFailureBundle, logWorldMetrics } from './test-debug.js'

describe('WASM lifecycle (concrete invariants)', () => {
  it('World.create completes and exposes zero bodies until bodies are added', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    expect(world.bodyCount).toBe(0)
    expect(world.colliderCount).toBe(0)
  })

  it('sequential World.create calls each yield an independent engine instance', async () => {
    const a = await World.create({ gravity: { x: 0, y: -1 }, fixedTimestep: 1 / 60 })
    const b = await World.create({ gravity: { x: 0, y: -2 }, fixedTimestep: 1 / 60 })
    expect(a.config.gravity.y).toBe(-1)
    expect(b.config.gravity.y).toBe(-2)
    expect(a.bodyCount).toBe(0)
    expect(b.bodyCount).toBe(0)
  })

  it('destroyBody removes exactly one dynamic body and colliders tied to it', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    const h0 = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(h0, { shape: 'ball', radius: 0.25 })
    const h1 = world.createBody({ type: 'dynamic', positionX: 2, positionY: 0 })
    world.addCollider(h1, { shape: 'cuboid', halfExtentX: 0.25, halfExtentY: 0.25 })

    expect(world.bodyCount).toBe(2)
    expect(world.colliderCount).toBe(2)

    world.destroyBody(h0)

    if (world.bodyCount !== 1 || world.colliderCount !== 1) {
      logFailureBundle(world, {
        testName: 'destroyBody removes one body',
        reason: `after destroyBody expected bodyCount=1 colliderCount=1, got bodyCount=${world.bodyCount} colliderCount=${world.colliderCount}`,
        hints: [
          'Verify remove_body in Rust removes attached colliders from BVH and collider count.',
          'Check for double-free or stale handle if counts are lower than expected.',
        ],
      })
      logWorldMetrics(world)
    }

    expect(world.bodyCount).toBe(1)
    expect(world.colliderCount).toBe(1)
  })

  it('step(0) does not advance render alpha past one fixed tick worth of debt', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    world.step(0)
    expect(world.renderAlpha).toBeGreaterThanOrEqual(0)
    expect(world.renderAlpha).toBeLessThan(1)
  })
})
