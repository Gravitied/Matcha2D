import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import {
  assertFinite2,
  assertFiniteScalar,
  logFailureBundle,
  snapshotBody,
} from './test-debug.js'

describe('Physics invariants (no heuristics)', () => {
  it('single substep: gravity matches g·dt within float noise (no double-bitwise replay of WASM)', async () => {
    const g = -10
    const dt = 1 / 60
    const world = await World.create({
      gravity: { x: 0, y: g },
      fixedTimestep: dt,
      defaultLinearDamping: 0,
      defaultAngularDamping: 0,
    })
    const b = world.createBody({ type: 'dynamic', positionX: 0, positionY: 100 })
    world.addCollider(b, { shape: 'ball', radius: 0.25 })

    world.stepPhysicsOnce()
    const v = world.getBodyVelocity(b)
    assertFinite2('post-step velocity', v, world, [])

    const refVy = g * dt
    const err = Math.abs(v[1] - refVy)
    /** WASM uses `f32`; JS cannot reliably reconstruct the exact multiply order — bound rejects wrong g/dt. */
    const MAX_ERR = 1e-5
    if (v[0] !== 0 || err > MAX_ERR) {
      logFailureBundle(world, {
        testName: 'gravity single substep',
        reason: `expected vx=0 and |vy - (g*dt)| <= ${MAX_ERR}; vx=${v[0]} vy=${v[1]} refVy=${refVy} err=${err} engine_dt=${world.engine.get_dt()}`,
        bodies: [snapshotBody(world, b, 'dynamic')],
        hints: [
          'Inspect solver_body force_incr_linear and apply_force_increments.',
          'Confirm World.stepPhysicsOnce maps to exactly one PhysicsEngine.step().',
          'Compare to `cargo test gravity_first_substep` in packages/physics-rust (native exact f32).',
        ],
      })
    }

    expect(v[0]).toBe(0)
    expect(err).toBeLessThanOrEqual(MAX_ERR)
  })

  it('vertical free-fall: vy strictly decreases over 30 substeps (more negative each step)', async () => {
    const world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      defaultLinearDamping: 0,
      defaultAngularDamping: 0,
    })
    const b = world.createBody({ type: 'dynamic', positionX: 0, positionY: 500 })
    world.addCollider(b, { shape: 'ball', radius: 0.1 })

    let prevVy = world.getBodyVelocity(b)[1]
    for (let i = 0; i < 30; i++) {
      world.stepPhysicsOnce()
      const vy = world.getBodyVelocity(b)[1]
      assertFiniteScalar(`step ${i} vy`, vy, world, [])
      assertFinite2(`step ${i} pos`, world.getBodyPosition(b), world, [])

      if (!(vy < prevVy)) {
        logFailureBundle(world, {
          testName: 'free-fall vy monotonic',
          step: i,
          reason: `expected vy[${i}] < vy[${i - 1}] (more negative); got prev=${prevVy} curr=${vy}`,
          bodies: [snapshotBody(world, b, 'ball')],
          hints: [
            'If monotonicity breaks: check damping accidentally applied, gravity sign, or sleeping waking mid-loop.',
          ],
        })
        throw new Error(`vy not strictly decreasing at step ${i}`)
      }
      prevVy = vy
    }
  })

  it('kinematic with zero velocity does not move over substeps', async () => {
    const world = await World.create({
      gravity: { x: 0, y: -10 },
      fixedTimestep: 1 / 60,
    })
    const k = world.createBody({ type: 'kinematic', positionX: 3, positionY: 4 })
    world.addCollider(k, { shape: 'ball', radius: 0.5 })
    world.setBodyVelocity(k, 0, 0)

    const p0 = world.getBodyPosition(k)
    for (let i = 0; i < 20; i++) {
      world.stepPhysicsOnce()
    }
    const p1 = world.getBodyPosition(k)
    expect(p1[0]).toBe(p0[0])
    expect(p1[1]).toBe(p0[1])
  })
})
