/**
 * Mirrors `demo/collision.html` stack scene (pixel-scale arena + solver caps) so regressions in
 * dense resting stacks show up in CI. Uses `penetrationSlop` as the **contract**: raw `maxPen`
 * may sit near slop by design; we bound **excess** over slop after warmup.
 */
import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import { assertFiniteScalar, logFailureBundle } from './test-debug.js'

const W = 800
const H = 500

/** Same wall layout as `createWalls` in collision.html */
function addDemoWalls(
  world: World,
  friction: number,
  restitution: number,
): void {
  const wallThickness = 8
  const floorHx = W / 2
  const floorHy = wallThickness / 2
  const floor = world.createBody({
    type: 'static',
    positionX: W / 2,
    positionY: H - wallThickness / 2,
  })
  world.addCollider(floor, {
    shape: 'cuboid',
    halfExtentX: floorHx,
    halfExtentY: floorHy,
    friction,
    restitution,
  })
  const ceil = world.createBody({
    type: 'static',
    positionX: W / 2,
    positionY: wallThickness / 2,
  })
  world.addCollider(ceil, {
    shape: 'cuboid',
    halfExtentX: floorHx,
    halfExtentY: floorHy,
    friction,
    restitution,
  })
  const sideHy = H / 2
  const sideHx = wallThickness / 2
  const left = world.createBody({
    type: 'static',
    positionX: wallThickness / 2,
    positionY: H / 2,
  })
  world.addCollider(left, {
    shape: 'cuboid',
    halfExtentX: sideHx,
    halfExtentY: sideHy,
    friction,
    restitution,
  })
  const right = world.createBody({
    type: 'static',
    positionX: W - wallThickness / 2,
    positionY: H / 2,
  })
  world.addCollider(right, {
    shape: 'cuboid',
    halfExtentX: sideHx,
    halfExtentY: sideHy,
    friction,
    restitution,
  })
}

/** Same grid as `createStackScene` (boxes only). */
function addStackBodies(world: World, bodyCount: number, friction: number, restitution: number) {
  const boxSize = 20
  const cols = Math.min(Math.ceil(Math.sqrt(bodyCount * 2)), 8)
  const rows = Math.ceil(bodyCount / cols)
  let count = 0
  for (let r = 0; r < rows && count < bodyCount; r++) {
    for (let c = 0; c < cols && count < bodyCount; c++) {
      const x = W / 2 - (cols - 1) * boxSize + c * boxSize * 2
      const y = H - 20 - boxSize - r * boxSize * 2
      const body = world.createBody({
        type: 'dynamic',
        positionX: x,
        positionY: y,
        angle: 0,
      })
      const s = boxSize
      world.addCollider(body, {
        shape: 'cuboid',
        halfExtentX: s,
        halfExtentY: s,
        friction,
        restitution,
      })
      count++
    }
  }
}

describe('collision.html stack regression (50 bodies, pixel-scale)', () => {
  it('after warmup, max penetration excess over configured slop stays bounded (no runaway overlap)', async () => {
    const bodyCount = 50
    const friction = 0.3
    const restitution = 0
    const gravityY = 400

    const world = await World.create({
      gravity: { x: 0, y: gravityY },
      fixedTimestep: 1 / 60,
      maxSolverLinearVelocity: 700,
      maxSolverAngularVelocity: 150,
      maxCorrectiveVelocity: 280,
      penetrationSlop: 0.65,
      baumgarteFactor: 0.15,
      velocityIterations: 12,
      internalVelocityIterations: 10,
      sweptBroadphase: true,
      defaultRestitution: 0,
    })

    addDemoWalls(world, friction, restitution)
    addStackBodies(world, bodyCount, friction, restitution)

    const slop = world.config.penetrationSlop
    /** Allow some manifold depth above slop; tighten if engine improves stack convergence. */
    const maxExcessOverSlop = 0.95
    const warmupSteps = 180
    const totalSteps = 520

    let worstExcess = 0

    for (let step = 0; step < totalSteps; step++) {
      world.stepPhysicsOnce()
      const pen = world.maxPenetration
      assertFiniteScalar(`step ${step} maxPen`, pen, world, [])
      const excess = pen - slop

      if (step >= warmupSteps) {
        if (excess > worstExcess) {
          worstExcess = excess
        }
        if (excess > maxExcessOverSlop) {
          logFailureBundle(world, {
            testName: 'demo stack pen excess',
            step,
            reason: `pen=${pen} slop=${slop} excess=${excess} > cap ${maxExcessOverSlop}`,
            hints: [
              'Raise internalVelocityIterations / velocityIterations or lower penetrationSlop in demo — see demo/collision.html comments.',
              'If excess is huge: solver spring bias or maxCorrectiveVelocity limiting separation.',
            ],
          })
          throw new Error('stack penetration excess over slop')
        }
      }
    }

    expect(worstExcess).toBeLessThanOrEqual(maxExcessOverSlop)
  })
})
