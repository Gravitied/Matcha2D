import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import {
  assertFinite2,
  assertFiniteScalar,
  logFailureBundle,
  snapshotBody,
} from './test-debug.js'

/**
 * Floor: static cuboid centered at (0, 0) with halfExtentY = 0.5 → top surface at y = +0.5.
 * Ball radius R: lowest valid center satisfies center_y - R >= surface_y - reported_penetration.
 */
describe('Collision pipeline (geometry + maxPenetration)', () => {
  it('ball dropped onto horizontal floor never crosses surface by more than maxPenetration', async () => {
    const R = 0.25
    const floorHalfY = 0.5
    const floorTop = 0 + floorHalfY

    const world = await World.create({
      gravity: { x: 0, y: -12 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
      defaultLinearDamping: 0.02,
      defaultRestitution: 0,
    })

    const floor = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(floor, {
      shape: 'cuboid',
      halfExtentX: 8,
      halfExtentY: floorHalfY,
      friction: 0.4,
      restitution: 0,
    })

    const ball = world.createBody({ type: 'dynamic', positionX: 0, positionY: 3 })
    world.addCollider(ball, { shape: 'ball', radius: R, friction: 0.3, restitution: 0 })

    const maxSteps = 900
    for (let step = 0; step < maxSteps; step++) {
      world.stepPhysicsOnce()

      const pen = world.maxPenetration
      assertFiniteScalar(`step ${step} maxPenetration`, pen, world, [
        'max_penetration() returned non-finite — inspect contact depth accumulation in Rust.',
      ])

      const [px, py] = world.getBodyPosition(ball)
      assertFinite2(`step ${step} position`, [px, py], world, [])

      const [vx, vy] = world.getBodyVelocity(ball)
      assertFinite2(`step ${step} velocity`, [vx, vy], world, [])

      const ballBottom = py - R
      /** Float32 + solver reporting slack — not a license to tunnel (see Rust `GEOM_SLACK`). */
      const GEOM_SLACK = 2e-4
      const allowed = floorTop - pen - GEOM_SLACK

      if (ballBottom < allowed) {
        logFailureBundle(world, {
          testName: 'floor tunnel / deep penetration',
          step,
          reason:
            `ball_bottom=${ballBottom} < floorTop - maxPen - eps (${allowed}); ` +
            `floorTop=${floorTop} R=${R} maxPenetration=${pen}`,
          bodies: [snapshotBody(world, ball, 'ball'), snapshotBody(world, floor, 'floor')],
          hints: [
            'If maxPen is small but ball_bottom is far below: broadphase/narrowphase missed contacts (swept AABB, TOI).',
            'If maxPen is huge: solver separation or position correction is failing — inspect PGS + Baumgarte.',
            'Compare debug_dump() overlap section with narrowphase outlines from getNarrowphaseColliderOutlines().',
          ],
        })
        throw new Error(`Step ${step}: geometric floor invariant violated`)
      }
    }

    expect(world.maxPenetration).toBeGreaterThanOrEqual(0)
  })
})
