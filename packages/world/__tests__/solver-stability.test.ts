/**
 * Solver / integration stability — catches velocity explosions using **engine contracts**
 * (`set_max_integrate_velocities`) plus bounded gravity-only velocity change while airborne.
 */
import { describe, it, expect } from 'vitest'
import type { BodyHandle } from '@matcha2d/types'
import { World } from '../src/world.js'
import {
  assertIntegrateVelocityCap,
  assertFinite2,
  assertFiniteScalar,
  logFailureBundle,
  snapshotBody,
} from './test-debug.js'

describe('Solver stability — integrate velocity caps (WASM)', () => {
  it('three-box stack: no body exceeds configured integrate caps over 120 steps', async () => {
    const maxLin = 200
    const maxAng = 100
    const world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      defaultLinearDamping: 0.02,
      defaultAngularDamping: 0.02,
    })
    world.engine.set_max_integrate_velocities(maxLin, maxAng)

    const floor = world.createBody({ type: 'static', positionX: 0, positionY: -0.5 })
    world.addCollider(floor, { shape: 'cuboid', halfExtentX: 10, halfExtentY: 0.5 })

    const dynamics: BodyHandle[] = []
    for (let i = 0; i < 3; i++) {
      const h = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0.5 + i })
      world.addCollider(h, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })
      dynamics.push(h)
    }

    const slack = 0.05
    for (let step = 0; step < 120; step++) {
      world.stepPhysicsOnce()
      assertIntegrateVelocityCap(world, dynamics, maxLin, maxAng, slack, step, 'stack integrate cap')
    }
  })

  it('20-box vertical stack: integrate caps + floor support + no vertical or lateral blow-up', async () => {
    const numBoxes = 20
    const half = 0.5
    const floorTop = 0
    /** Matches `PhysicsWorld::new` defaults — violations mean the engine broke its own post-integrate clamp. */
    const maxLin = 500
    const maxAng = 200
    const integrateSlack = 0.05
    /**
     * Tight column + many simultaneous contacts: first frames can sit slightly below the
     * `floorTop - maxPen` line by ~1e-4 purely from f32 + initial overlap resolution.
     * Still far smaller than real passthrough (O(0.1+) m).
     */
    const stackFloorSlack = 0.0012
    /** Initial highest box center ≈ 0.5 + (numBoxes − 1); allow modest bounce/settling, reject explosion. */
    const maxReasonableCenterY = half + (numBoxes - 1) + 22
    const maxReasonableAbsX = 2.5

    const world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      defaultLinearDamping: 0.02,
      defaultAngularDamping: 0.02,
      sleepVelocityThreshold: 100,
      sleepAngularVelocityThreshold: 100,
      sleepTimeThreshold: 100,
    })
    world.engine.set_max_integrate_velocities(maxLin, maxAng)

    const floor = world.createBody({ type: 'static', positionX: 0, positionY: -0.5 })
    world.addCollider(floor, { shape: 'cuboid', halfExtentX: 12, halfExtentY: 0.5 })

    const dynamics: BodyHandle[] = []
    for (let i = 0; i < numBoxes; i++) {
      const h = world.createBody({ type: 'dynamic', positionX: 0, positionY: half + i })
      world.addCollider(h, { shape: 'cuboid', halfExtentX: half, halfExtentY: half })
      dynamics.push(h)
    }

    const steps = 360
    for (let step = 0; step < steps; step++) {
      world.stepPhysicsOnce()

      assertIntegrateVelocityCap(
        world,
        dynamics,
        maxLin,
        maxAng,
        integrateSlack,
        step,
        '20-box stack integrate cap',
      )

      const pen = world.maxPenetration
      assertFiniteScalar(`step ${step} maxPenetration`, pen, world, [])

      let maxCenterY = -Infinity
      for (const h of dynamics) {
        const [x, y] = world.getBodyPosition(h)
        assertFinite2(`step ${step} pos`, [x, y], world, [])

        maxCenterY = Math.max(maxCenterY, y)

        if (Math.abs(x) > maxReasonableAbsX) {
          logFailureBundle(world, {
            testName: '20-box stack lateral',
            step,
            reason: `|x|=${Math.abs(x)} > ${maxReasonableAbsX} (lateral blow-up)`,
            bodies: dynamics.map((bh, j) => snapshotBody(world, bh, `box_${j}`)),
            hints: ['Friction/stack instability scattering bodies horizontally.'],
          })
          throw new Error('20-box stack lateral explosion')
        }

        const bottom = y - half
        const allowedBottom = floorTop - pen - stackFloorSlack
        if (bottom < allowedBottom) {
          logFailureBundle(world, {
            testName: '20-box stack passthrough / support loss',
            step,
            reason:
              `box bottom=${bottom} < floorTop - maxPen - slack (${allowedBottom}); ` +
              `floorTop=${floorTop} maxPen=${pen}`,
            bodies: dynamics.map((bh, j) => snapshotBody(world, bh, `box_${j}`)),
            hints: [
              'Lower boxes tunneling or global penetration under-reporting — inspect PGS + Baumgarte + internal iterations.',
            ],
          })
          throw new Error('20-box stack floor invariant')
        }

        if (y < -4) {
          logFailureBundle(world, {
            testName: '20-box stack fell through',
            step,
            reason: `box center y=${y} < -4`,
            bodies: [snapshotBody(world, h, 'fallen')],
            hints: ['Severe floor passthrough — same checks as thin-stack resting tests.'],
          })
          throw new Error('20-box stack fell through floor')
        }
      }

      if (maxCenterY > maxReasonableCenterY) {
        logFailureBundle(world, {
          testName: '20-box stack vertical explosion',
          step,
          reason: `max center y=${maxCenterY} > ${maxReasonableCenterY}`,
          bodies: dynamics.map((bh, j) => snapshotBody(world, bh, `box_${j}`)),
          hints: ['Velocity explosion or contact resolution launching the tower.'],
        })
        throw new Error('20-box stack vertical explosion')
      }
    }
  })

  it('high-speed head-on balls: post-step speeds stay inside integrate caps', async () => {
    const maxLin = 250
    const maxAng = 120
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })
    world.engine.set_max_integrate_velocities(maxLin, maxAng)

    const a = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(a, { shape: 'ball', radius: 0.35 })
    world.setBodyVelocity(a, 220, 0)

    const b = world.createBody({ type: 'dynamic', positionX: 28, positionY: 0 })
    world.addCollider(b, { shape: 'ball', radius: 0.35 })
    world.setBodyVelocity(b, -220, 0)

    const pair: BodyHandle[] = [a, b]
    const slack = 0.05
    for (let step = 0; step < 150; step++) {
      world.stepPhysicsOnce()
      assertIntegrateVelocityCap(world, pair, maxLin, maxAng, slack, step, 'head-on integrate cap')
    }
  })
})

describe('Solver stability — gravity-only Δv bound (no contact)', () => {
  it('while airborne and contact-free, per-step |Δvy - g·dt| is tiny (no phantom impulses)', async () => {
    const g = -9.81
    const dt = 1 / 60
    const world = await World.create({
      gravity: { x: 0, y: g },
      fixedTimestep: dt,
      defaultLinearDamping: 0,
      defaultAngularDamping: 0,
    })

    const ball = world.createBody({ type: 'dynamic', positionX: 0, positionY: 50 })
    world.addCollider(ball, { shape: 'ball', radius: 0.2 })

    let prevVy = world.getBodyVelocity(ball)[1]
    const maxDvErr = 1e-4

    for (let step = 0; step < 80; step++) {
      world.stepPhysicsOnce()

      if (world.contactPairCount > 0) {
        break
      }

      const pos = world.getBodyPosition(ball)
      assertFinite2(`step ${step} pos`, pos, world, [])
      const vy = world.getBodyVelocity(ball)[1]
      const dv = vy - prevVy
      const expectedDv = g * dt
      const err = Math.abs(dv - expectedDv)

      if (err > maxDvErr) {
        logFailureBundle(world, {
          testName: 'gravity-only dv',
          step,
          reason: `|Δvy - g·dt|=${err} > ${maxDvErr}; dv=${dv} expectedDv=${expectedDv} vy=${vy}`,
          bodies: [snapshotBody(world, ball, 'ball')],
          hints: [
            'Phantom impulse while airborne: solver contacts when pair count should be 0, or gravity applied twice.',
          ],
        })
        throw new Error('gravity-only step corrupted')
      }
      prevVy = vy
    }
  })
})
