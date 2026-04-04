import { describe, it, expect, beforeAll } from 'vitest'
import { World } from '../src/world.js'

interface StepDiagnostic {
  step: number
  body0: {
    positionX: number
    positionY: number
    velocityX: number
    velocityY: number
    angle: number
    angularVelocity: number
  }
  body1: {
    positionX: number
    positionY: number
    velocityX: number
    velocityY: number
    angle: number
    angularVelocity: number
  }
  contactPairCount: number
  contactPointCount: number
  maxPenetration: number
  firstContactNormal: [number, number]
}

describe('Two Cuboid Collision Diagnostics', () => {
  let diagnostics: StepDiagnostic[]
  let body0: ReturnType<typeof world.createBody>
  let body1: ReturnType<typeof world.createBody>
  let world: World

  beforeAll(async () => {
    world = await World.create({
      gravity: { x: 0, y: -9.81 },
      fixedTimestep: 1 / 60,
      velocityIterations: 8,
      positionIterations: 3,
      sleepVelocityThreshold: 0.05,
      sleepAngularVelocityThreshold: 0.1,
      sleepTimeThreshold: 1.0,
    })

    // Body 0: static ground platform at y=0
    body0 = world.createBody({ type: 'static', positionX: 0, positionY: 0 })
    world.addCollider(body0, { shape: 'cuboid', halfExtentX: 5, halfExtentY: 0.5, friction: 0.3 })

    // Body 1: dynamic cube falling from y=5
    body1 = world.createBody({ type: 'dynamic', positionX: 0, positionY: 5 })
    world.addCollider(body1, { shape: 'cuboid', halfExtentX: 0.5, halfExtentY: 0.5, friction: 0.3, restitution: 0.0 })

    diagnostics = []

    // Run 60 steps total
    for (let step = 0; step < 60; step++) {
      world.step(1 / 60)

      const pos0 = world.getBodyPosition(body0)
      const vel0 = world.getBodyVelocity(body0)
      const angle0 = world.getBodyAngle(body0)
      const angVel0 = world.getBodyAngularVelocity(body0)

      const pos1 = world.getBodyPosition(body1)
      const vel1 = world.getBodyVelocity(body1)
      const angle1 = world.getBodyAngle(body1)
      const angVel1 = world.getBodyAngularVelocity(body1)

      diagnostics.push({
        step,
        body0: {
          positionX: pos0[0],
          positionY: pos0[1],
          velocityX: vel0[0],
          velocityY: vel0[1],
          angle: angle0,
          angularVelocity: angVel0,
        },
        body1: {
          positionX: pos1[0],
          positionY: pos1[1],
          velocityX: vel1[0],
          velocityY: vel1[1],
          angle: angle1,
          angularVelocity: angVel1,
        },
        contactPairCount: world.contactPairCount,
        contactPointCount: world.contactPointCount,
        maxPenetration: world.maxPenetration,
        firstContactNormal: world.firstContactNormal,
      })
    }
  })

  it('should produce 60 diagnostic entries', () => {
    expect(diagnostics.length).toBe(60)
  })

  it('should have body0 (static) position unchanged throughout', () => {
    for (const d of diagnostics) {
      expect(d.body0.positionX).toBe(0)
      expect(d.body0.positionY).toBe(0)
      expect(d.body0.velocityX).toBe(0)
      expect(d.body0.velocityY).toBe(0)
      expect(d.body0.angle).toBe(0)
      expect(d.body0.angularVelocity).toBe(0)
    }
  })

  it('should have body1 (dynamic) fall due to gravity before collision', () => {
    // Steps 0-9: body1 should be falling (negative velocityY, decreasing positionY)
    const preCollisionSteps = diagnostics.filter(d => d.step < 10)
    for (const d of preCollisionSteps) {
      expect(d.body1.velocityY).toBeLessThan(0)
    }
    // Position should decrease over first few steps
    expect(diagnostics[0].body1.positionY).toBeGreaterThan(diagnostics[5].body1.positionY)
  })

  it('should detect collision after body1 reaches ground', () => {
    // Find first step with contact
    const firstContactStep = diagnostics.find(d => d.contactPairCount > 0)
    expect(firstContactStep).toBeDefined()
    expect(firstContactStep!.step).toBeGreaterThanOrEqual(10)
  })

  it('should have body1 bounce after collision with ground', () => {
    // After collision, body1 should bounce upward (positive velocityY)
    const postCollisionSteps = diagnostics.filter(d => d.step > 54)
    for (const d of postCollisionSteps) {
      expect(d.body1.velocityY).toBeGreaterThan(0)
    }
  })

  it('should have body1 position increase after bounce', () => {
    // After bounce, position should be increasing
    const bounceStep = diagnostics[54]
    const laterStep = diagnostics[59]
    expect(laterStep.body1.positionY).toBeGreaterThan(bounceStep.body1.positionY)
  })

  it('should have no significant rotation for axis-aligned cuboids', () => {
    for (const d of diagnostics) {
      expect(Math.abs(d.body0.angle)).toBeLessThan(0.01)
      expect(Math.abs(d.body1.angle)).toBeLessThan(0.01)
      expect(Math.abs(d.body0.angularVelocity)).toBeLessThan(0.01)
      expect(Math.abs(d.body1.angularVelocity)).toBeLessThan(0.1)
    }
  })

  it('should return all diagnostics data', () => {
    // Verify diagnostics structure
    for (const d of diagnostics) {
      expect(d).toHaveProperty('step')
      expect(d).toHaveProperty('body0')
      expect(d).toHaveProperty('body1')
      expect(d).toHaveProperty('contactPairCount')
      expect(d).toHaveProperty('contactPointCount')
      expect(d).toHaveProperty('maxPenetration')
      expect(d).toHaveProperty('firstContactNormal')
      expect(d.body0).toHaveProperty('positionX')
      expect(d.body0).toHaveProperty('positionY')
      expect(d.body0).toHaveProperty('velocityX')
      expect(d.body0).toHaveProperty('velocityY')
      expect(d.body0).toHaveProperty('angle')
      expect(d.body0).toHaveProperty('angularVelocity')
      expect(d.body1).toHaveProperty('positionX')
      expect(d.body1).toHaveProperty('positionY')
      expect(d.body1).toHaveProperty('velocityX')
      expect(d.body1).toHaveProperty('velocityY')
      expect(d.body1).toHaveProperty('angle')
      expect(d.body1).toHaveProperty('angularVelocity')
    }
  })

  it('should log diagnostics table for debugging', () => {
    console.log('\n=== Collision Diagnostics Table ===')
    console.log('Step | Body0 Pos(x,y) | Body0 Vel(x,y) | Body0 Angle | Body0 AngVel | Body1 Pos(x,y) | Body1 Vel(x,y) | Body1 Angle | Body1 AngVel | Contacts | Penetration')
    console.log('-'.repeat(180))
    for (const d of diagnostics) {
      const line = `${d.step.toString().padStart(4)} | ` +
        `(${d.body0.positionX.toFixed(3)}, ${d.body0.positionY.toFixed(3)}) | ` +
        `(${d.body0.velocityX.toFixed(3)}, ${d.body0.velocityY.toFixed(3)}) | ` +
        `${d.body0.angle.toFixed(4)} | ${d.body0.angularVelocity.toFixed(4)} | ` +
        `(${d.body1.positionX.toFixed(3)}, ${d.body1.positionY.toFixed(3)}) | ` +
        `(${d.body1.velocityX.toFixed(3)}, ${d.body1.velocityY.toFixed(3)}) | ` +
        `${d.body1.angle.toFixed(4)} | ${d.body1.angularVelocity.toFixed(4)} | ` +
        `${d.contactPairCount} | ${d.maxPenetration.toFixed(4)}`
      console.log(line)
    }
    console.log('='.repeat(180))
  })
})
