import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import {
  assertFinite2,
  logFailureBundle,
  snapshotBody,
  formatBodySnapshot,
} from './test-debug.js'

describe('API contracts (absolute expectations)', () => {
  it('setBodyPosition is read back exactly by getBodyPosition', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    const b = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(b, { shape: 'ball', radius: 0.5 })
    world.setBodyPosition(b, 1.25, -3.5)
    const p = world.getBodyPosition(b)
    expect(p[0]).toBe(1.25)
    expect(p[1]).toBe(-3.5)
  })

  it('setBodyVelocity is read back exactly by getBodyVelocity', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    const b = world.createBody({ type: 'kinematic', positionX: 0, positionY: 0 })
    world.addCollider(b, { shape: 'ball', radius: 0.5 })
    world.setBodyVelocity(b, -2, 7)
    const v = world.getBodyVelocity(b)
    expect(v[0]).toBe(-2)
    expect(v[1]).toBe(7)
  })

  it('impulse on static body does not change linear velocity (stays zero)', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    const floor = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(floor, { shape: 'cuboid', halfExtentX: 1, halfExtentY: 0.5 })
    const v0 = world.getBodyVelocity(floor)
    world.applyImpulse(floor, 1e6, 1e6)
    const v1 = world.getBodyVelocity(floor)
    expect(v0[0]).toBe(v1[0])
    expect(v0[1]).toBe(v1[1])
  })

  /**
   * Linear impulse superposition: Δv from (a+b) equals Δv from a plus Δv from b (same frame, no step).
   * Uses one cuboid so mass is fixed; avoids guessing restitution or contact.
   */
  it('applyImpulse superposes: I1+I2 at rest equals separate impulses summed', async () => {
    const world = await World.create({ gravity: { x: 0, y: 0 }, fixedTimestep: 1 / 60 })
    const a = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(a, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })
    const b = world.createBody({ type: 'dynamic', positionX: 10, positionY: 0 })
    world.addCollider(b, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })

    world.applyImpulse(a, 3, -2)
    world.applyImpulse(a, 5, 7)
    const va = world.getBodyVelocity(a)

    world.applyImpulse(b, 8, 5)
    const vb = world.getBodyVelocity(b)

    assertFinite2('superpose va', va, world, ['Compare apply_impulse order in Rust for dynamics.'])
    assertFinite2('superpose vb', vb, world, [])

    if (va[0] !== vb[0] || va[1] !== vb[1]) {
      logFailureBundle(world, {
        testName: 'impulse superposition',
        reason: `velocity mismatch: combined impulses on A vs single on B`,
        bodies: [snapshotBody(world, a, 'A (3,-2)+(5,7)'), snapshotBody(world, b, 'B (8,5)')],
        hints: [
          `A: ${formatBodySnapshot(snapshotBody(world, a, 'A'))}`,
          `B: ${formatBodySnapshot(snapshotBody(world, b, 'B'))}`,
          'If A ≠ B: impulse accumulation or mass recompute differs between bodies — inspect apply_impulse + recompute_body_mass.',
        ],
      })
    }

    expect(va[0]).toBe(vb[0])
    expect(va[1]).toBe(vb[1])
  })
})
