/**
 * Passthrough / tunnel regressions — scenarios aligned with `PhysicsWorld` tests in
 * `packages/physics-rust/src/world/mod.rs`. Assertions are geometric or contact-order facts,
 * not loose “energy feels ok” checks.
 */
import { describe, it, expect } from 'vitest'
import { World } from '../src/world.js'
import {
  assertFinite2,
  assertFiniteScalar,
  logFailureBundle,
  snapshotBody,
} from './test-debug.js'

describe('Passthrough — thin wall & shelf (WASM)', () => {
  it('fast linear bullet must register wall contact within 60 steps (broadphase smoke)', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })
    const wall = world.createBody({ type: 'static', positionX: 5, positionY: 0 })
    world.addCollider(wall, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 2 })

    const bullet = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(bullet, { shape: 'ball', radius: 0.1 })
    world.setBodyVelocity(bullet, 100, 0)

    let maxX = 0
    let collided = false
    for (let step = 0; step < 60; step++) {
      world.stepPhysicsOnce()
      const px = world.getBodyPosition(bullet)[0]
      maxX = Math.max(maxX, px)
      if (world.contactPairCount > 0) {
        collided = true
        break
      }
    }
    if (!collided) {
      logFailureBundle(world, {
        testName: 'fast bullet contact',
        reason: `no contact in 60 steps; max bullet x=${maxX}`,
        bodies: [snapshotBody(world, bullet, 'bullet')],
        hints: ['Swept / discrete broadphase missed wall — inspect pair generation for fast movers.'],
      })
    }
    expect(collided).toBe(true)
  })

  it('ultrafast ball does not cross thin wall face (swept broadphase)', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
      defaultRestitution: 0,
    })

    const wallX = 40
    const wallHx = 0.04
    const wall = world.createBody({ type: 'static', positionX: wallX, positionY: 0 })
    const wallCol = world.addCollider(wall, {
      shape: 'cuboid',
      halfExtentX: wallHx,
      halfExtentY: 4,
      restitution: 0,
    })
    void wallCol

    const bullet = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    const ballCol = world.addCollider(bullet, { shape: 'ball', radius: 0.12, restitution: 0 })
    void ballCol
    world.setBodyVelocity(bullet, 480, 0)

    const wallFaceX = wallX - wallHx
    /** Same margin as native test: radius + face tolerance. */
    const LIMIT_X = wallFaceX + 0.14

    let collided = false
    for (let step = 0; step < 100; step++) {
      world.stepPhysicsOnce()
      const px = world.getBodyPosition(bullet)[0]
      assertFiniteScalar(`step ${step} px`, px, world, [])

      if (px >= LIMIT_X) {
        logFailureBundle(world, {
          testName: 'ultrafast ball thin wall',
          step,
          reason: `ball center x=${px} >= wall_face+margin (${LIMIT_X}); wallFaceX=${wallFaceX}`,
          bodies: [snapshotBody(world, bullet, 'bullet')],
          hints: [
            'Tunnel: verify swept AABB / TOI path for ball-vs-thin-box.',
            'Compare `contactPairCount` timeline — missing pairs before x breach means broadphase.',
          ],
        })
        throw new Error('ball crossed wall plane')
      }
      if (world.contactPairCount > 0) {
        collided = true
      }
    }
    expect(collided).toBe(true)
  })

  it('falling ball does not drop through hairline horizontal shelf', async () => {
    const world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
      defaultRestitution: 0,
    })

    const shelf = world.createBody({ type: 'static', positionX: 0, positionY: 0.99 })
    const shelfCol = world.addCollider(shelf, {
      shape: 'cuboid',
      halfExtentX: 6,
      halfExtentY: 0.01,
      restitution: 0,
    })
    void shelfCol

    const ball = world.createBody({ type: 'dynamic', positionX: 0, positionY: 12 })
    const ballC = world.addCollider(ball, { shape: 'ball', radius: 0.22, restitution: 0 })
    void ballC

    for (let step = 0; step < 240; step++) {
      world.stepPhysicsOnce()
      const py = world.getBodyPosition(ball)[1]
      assertFiniteScalar(`step ${step} py`, py, world, [])
      if (py <= 0.75) {
        logFailureBundle(world, {
          testName: 'thin shelf passthrough',
          step,
          reason: `ball center y=${py} <= 0.75 (native regression threshold)`,
          bodies: [snapshotBody(world, ball, 'ball')],
          hints: [
            'Thin shelf: narrowphase SAT / contact persistence vs substep integration.',
            'Check `maxPenetration` and `debug_dump` at failing step.',
          ],
        })
        throw new Error('ball fell through shelf')
      }
    }

    const finalY = world.getBodyPosition(ball)[1]
    expect(finalY).toBeGreaterThan(0.85)
    expect(finalY).toBeLessThan(2.5)
  })

  it('two dynamic balls head-on: strict separation until first contact pair', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })

    const a = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(a, { shape: 'ball', radius: 0.35 })
    world.setBodyVelocity(a, 220, 0)

    const b = world.createBody({ type: 'dynamic', positionX: 28, positionY: 0 })
    world.addCollider(b, { shape: 'ball', radius: 0.35 })
    world.setBodyVelocity(b, -220, 0)

    let everContact = false
    for (let step = 0; step < 150; step++) {
      world.stepPhysicsOnce()
      if (world.contactPairCount > 0) {
        everContact = true
      }
      const pa = world.getBodyPosition(a)[0]
      const pb = world.getBodyPosition(b)[0]
      assertFinite2(`step ${step} pa`, [pa, 0], world, [])
      assertFinite2(`step ${step} pb`, [pb, 0], world, [])

      if (!everContact && step < 40) {
        if (pa >= pb - 0.15) {
          logFailureBundle(world, {
            testName: 'head-on balls order',
            step,
            reason: `A crossed B separation band without contact: pa=${pa} pb=${pb}`,
            bodies: [snapshotBody(world, a, 'A'), snapshotBody(world, b, 'B')],
            hints: [
              'Dynamic–dynamic tunnel: compare swept pairs vs discrete broadphase.',
            ],
          })
          throw new Error('dynamic–dynamic tunnel ordering')
        }
      }
    }
    expect(everContact).toBe(true)
  })
})

describe('Passthrough — spinning box & resting contact (WASM)', () => {
  it('spinning cuboid hitting static cuboid: must collide and not deep-penetrate left of origin', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })

    const staticBody = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(staticBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })

    const dynamicBody = world.createBody({ type: 'dynamic', positionX: 3, positionY: 0 })
    world.addCollider(dynamicBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })
    world.setBodyVelocity(dynamicBody, -10, 0)
    world.setBodyAngularVelocity(dynamicBody, 15)

    let collisionDetected = false
    let passedThrough = false

    for (let step = 0; step < 120; step++) {
      world.stepPhysicsOnce()
      const pos = world.getBodyPosition(dynamicBody)
      const vel = world.getBodyVelocity(dynamicBody)
      assertFinite2(`step ${step} pos`, pos, world, [])
      assertFinite2(`step ${step} vel`, vel, world, [])

      const contactCount = world.contactPairCount
      if (!collisionDetected && contactCount > 0) {
        collisionDetected = true
      }

      if (pos[0] < -0.5 && vel[0] < -1) {
        passedThrough = true
        logFailureBundle(world, {
          testName: 'spinning cuboid passthrough signature',
          step,
          reason: `x=${pos[0]} < -0.5 and vx=${vel[0]} < -1 (native tunnel signature)`,
          bodies: [snapshotBody(world, dynamicBody, 'dynamic')],
          hints: ['SAT cuboid–cuboid with high ω — compare narrowphase manifolds.'],
        })
      }
      if (pos[0] < -1.0 && !passedThrough) {
        passedThrough = true
      }
    }

    expect(collisionDetected).toBe(true)
    expect(passedThrough).toBe(false)

    const finalVel = world.getBodyVelocity(dynamicBody)
    expect(finalVel[0]).toBeGreaterThan(-10)
  })

  it('45° spinning cuboid: contacts static and does not tunnel (vertex regime)', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })

    const staticBody = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(staticBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })

    const dynamicBody = world.createBody({ type: 'dynamic', positionX: 3, positionY: 0 })
    world.addCollider(dynamicBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })
    world.setBodyAngle(dynamicBody, Math.PI / 4)
    world.setBodyVelocity(dynamicBody, -10, 0)
    world.setBodyAngularVelocity(dynamicBody, -20)

    let collided = false
    let passedThrough = false
    for (let step = 0; step < 120; step++) {
      world.stepPhysicsOnce()
      const pos = world.getBodyPosition(dynamicBody)
      const vel = world.getBodyVelocity(dynamicBody)
      assertFinite2(`step ${step} pos`, pos, world, [])
      assertFinite2(`step ${step} vel`, vel, world, [])
      if (world.contactPairCount > 0) {
        collided = true
      }
      if (pos[0] < -0.5 && vel[0] < -1) {
        passedThrough = true
        logFailureBundle(world, {
          testName: 'spinning cuboid 45° passthrough',
          step,
          reason: `x=${pos[0]} vx=${vel[0]} tunnel signature`,
          bodies: [snapshotBody(world, dynamicBody, 'dyn')],
          hints: ['Rotated cuboid–cuboid SAT / contact reduction.'],
        })
      }
    }
    expect(collided).toBe(true)
    expect(passedThrough).toBe(false)
  })

  it('very high ω cuboid vs static: still collides, no tunnel signature', async () => {
    const world = await World.create({
      gravity: { x: 0, y: 0 },
      fixedTimestep: 1 / 60,
      sweptBroadphase: true,
    })

    const staticBody = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(staticBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })

    const dynamicBody = world.createBody({ type: 'dynamic', positionX: 3, positionY: 0 })
    world.addCollider(dynamicBody, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })
    world.setBodyVelocity(dynamicBody, -8, 0)
    world.setBodyAngularVelocity(dynamicBody, 60)

    let collided = false
    let passedThrough = false
    for (let step = 0; step < 120; step++) {
      world.stepPhysicsOnce()
      const pos = world.getBodyPosition(dynamicBody)
      const vel = world.getBodyVelocity(dynamicBody)
      assertFinite2(`step ${step} pos`, pos, world, [])
      assertFinite2(`step ${step} vel`, vel, world, [])
      if (world.contactPairCount > 0) {
        collided = true
      }
      if (pos[0] < -0.5 && vel[0] < -1) {
        passedThrough = true
        logFailureBundle(world, {
          testName: 'extreme spin passthrough',
          step,
          reason: `x=${pos[0]} vx=${vel[0]} tunnel signature`,
          bodies: [snapshotBody(world, dynamicBody, 'dyn')],
          hints: ['High ω broadphase expansion and cuboid–cuboid contacts.'],
        })
      }
    }
    expect(collided).toBe(true)
    expect(passedThrough).toBe(false)
  })

  it('resting box on floor: no sinking through floor plane over sustained contact', async () => {
    const world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      sleepVelocityThreshold: 100,
      sleepAngularVelocityThreshold: 100,
      sleepTimeThreshold: 100,
    })

    const floor = world.createBody({ type: 'static', positionX: 0, positionY: -2 })
    world.addCollider(floor, { shape: 'cuboid', halfExtentX: 10, halfExtentY: 0.5 })

    const box = world.createBody({ type: 'dynamic', positionX: 0, positionY: 0 })
    world.addCollider(box, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5 })

    const floorTop = -2 + 0.5
    const halfY = 0.5

    for (let step = 0; step < 120; step++) {
      world.stepPhysicsOnce()
      const pos = world.getBodyPosition(box)
      assertFinite2(`step ${step} pos`, pos, world, [])

      if (pos[1] < -5) {
        logFailureBundle(world, {
          testName: 'resting box floor',
          step,
          reason: `box center y=${pos[1]} < -5 (fell through)`,
          bodies: [snapshotBody(world, box, 'box')],
          hints: ['Position correction / sleep interaction with floor.'],
        })
        throw new Error('box fell through floor')
      }

      const boxBottom = pos[1] - halfY
      const pen = world.maxPenetration
      assertFiniteScalar(`step ${step} pen`, pen, world, [])
      const allowedBottom = floorTop - pen - 2e-4
      if (boxBottom < allowedBottom) {
        logFailureBundle(world, {
          testName: 'resting box penetration',
          step,
          reason: `box_bottom=${boxBottom} < floorTop - maxPen - slack (${allowedBottom})`,
          bodies: [snapshotBody(world, box, 'box')],
          hints: ['Creep / bias insufficient — compare Baumgarte and internal iterations.'],
        })
        throw new Error('box sank below floor contact')
      }
    }

    const vy = world.getBodyVelocity(box)[1]
    expect(Math.abs(vy)).toBeLessThan(5)
  })
})
