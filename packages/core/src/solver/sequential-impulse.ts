import type { ContactManifold, ContactPoint, MatchaBuffers, WorldConfig } from '@matcha2d/types'
import { BodyFlags } from '@matcha2d/types'

interface ContactSolverData {
  bodyA: number
  bodyB: number
  normalX: number
  normalY: number
  tangentX: number
  tangentY: number
  rAX: number
  rAY: number
  rBX: number
  rBY: number
  normalMass: number
  tangentMass: number
  friction: number
  restitution: number
  bias: number
  normalImpulse: number
  tangentImpulse: number
  pointX: number
  pointY: number
}

interface BlockSolverData {
  solver1: ContactSolverData
  solver2: ContactSolverData
  k: [number, number, number, number]
  b: [number, number]
  impulses: [number, number]
}

function computeContactSolver(
  buffers: MatchaBuffers,
  contact: ContactPoint,
  normalX: number,
  normalY: number,
  bodyAIdx: number,
  bodyBIdx: number,
  dt: number,
  config: WorldConfig,
): ContactSolverData {
  const invMassA = buffers.invMass[bodyAIdx]
  const invMassB = buffers.invMass[bodyBIdx]
  const invInertiaA = buffers.invInertia[bodyAIdx]
  const invInertiaB = buffers.invInertia[bodyBIdx]

  const angleA = buffers.angle[bodyAIdx]
  const angleB = buffers.angle[bodyBIdx]
  const cosA = Math.cos(angleA)
  const sinA = Math.sin(angleA)
  const cosB = Math.cos(angleB)
  const sinB = Math.sin(angleB)

  const cpX = cosA * contact.localA.x - sinA * contact.localA.y + buffers.positionX[bodyAIdx]
  const cpY = sinA * contact.localA.x + cosA * contact.localA.y + buffers.positionY[bodyAIdx]

  const rAX = cpX - buffers.positionX[bodyAIdx]
  const rAY = cpY - buffers.positionY[bodyAIdx]
  const rBX = cpX - buffers.positionX[bodyBIdx]
  const rBY = cpY - buffers.positionY[bodyBIdx]

  const tangentX = -normalY
  const tangentY = normalX

  const rnA = rAX * normalY - rAY * normalX
  const rnB = rBX * normalY - rBY * normalX
  const normalMass = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB

  const rtA = rAX * tangentY - rAY * tangentX
  const rtB = rBX * tangentY - rBY * tangentX
  const tangentMass = invMassA + invMassB + invInertiaA * rtA * rtA + invInertiaB * rtB * rtB

  const velAX = buffers.velocityX[bodyAIdx]
  const velAY = buffers.velocityY[bodyAIdx]
  const velBX = buffers.velocityX[bodyBIdx]
  const velBY = buffers.velocityY[bodyBIdx]
  const angVelA = buffers.angularVel[bodyAIdx]
  const angVelB = buffers.angularVel[bodyBIdx]

  const relVelNormalX = (velBX + (-angVelB * rBY)) - (velAX + (-angVelA * rAY))
  const relVelNormalY = (velBY + (angVelB * rBX)) - (velAY + (angVelA * rAX))
  const relVelNormal = relVelNormalX * normalX + relVelNormalY * normalY

  const restitution = config.defaultRestitution
  let bias = 0

  if (config.positionCorrection) {
    const penetration = Math.max(contact.penetration - config.penetrationSlop, 0)
    bias = -(config.positionCorrectionBeta / dt) * penetration
  }

  bias += restitution * Math.min(relVelNormal + config.restitutionSlop, 0)

  return {
    bodyA: bodyAIdx,
    bodyB: bodyBIdx,
    normalX,
    normalY,
    tangentX,
    tangentY,
    rAX,
    rAY,
    rBX,
    rBY,
    normalMass: normalMass > 1e-10 ? 1 / normalMass : 0,
    tangentMass: tangentMass > 1e-10 ? 1 / tangentMass : 0,
    friction: config.defaultFriction,
    restitution,
    bias,
    normalImpulse: 0,
    tangentImpulse: 0,
    pointX: cpX,
    pointY: cpY,
  }
}

function solveContact(
  solver: ContactSolverData,
  buffers: MatchaBuffers,
  normalContact?: ContactSolverData,
  config?: WorldConfig,
): void {
  if (solver.normalMass === 0) return

  const invMassA = buffers.invMass[solver.bodyA]
  const invMassB = buffers.invMass[solver.bodyB]
  const invInertiaA = buffers.invInertia[solver.bodyA]
  const invInertiaB = buffers.invInertia[solver.bodyB]

  const velAX = buffers.velocityX[solver.bodyA]
  const velAY = buffers.velocityY[solver.bodyA]
  const velBX = buffers.velocityX[solver.bodyB]
  const velBY = buffers.velocityY[solver.bodyB]
  const angVelA = buffers.angularVel[solver.bodyA]
  const angVelB = buffers.angularVel[solver.bodyB]

  const relVelNX = (velBX + (-angVelB * solver.rBY)) - (velAX + (-angVelA * solver.rAY))
  const relVelNY = (velBY + (angVelB * solver.rBX)) - (velAY + (angVelA * solver.rAX))
  const relVelN = relVelNX * solver.normalX + relVelNY * solver.normalY

  const lambda = solver.normalMass * (-(relVelN + solver.bias))

  const oldImpulseN = solver.normalImpulse
  if (config?.impulseAccumulation) {
    solver.normalImpulse = Math.max(0, solver.normalImpulse + lambda)
  } else {
    solver.normalImpulse = Math.max(0, lambda)
  }
  const dLambdaN = config?.impulseAccumulation
    ? solver.normalImpulse - oldImpulseN
    : solver.normalImpulse

  const impX = dLambdaN * solver.normalX
  const impY = dLambdaN * solver.normalY

  buffers.velocityX[solver.bodyA] -= impX * invMassA
  buffers.velocityY[solver.bodyA] -= impY * invMassA
  buffers.angularVel[solver.bodyA] -= (solver.rAX * impY - solver.rAY * impX) * invInertiaA

  buffers.velocityX[solver.bodyB] += impX * invMassB
  buffers.velocityY[solver.bodyB] += impY * invMassB
  buffers.angularVel[solver.bodyB] += (solver.rBX * impY - solver.rBY * impX) * invInertiaB

  const newVelAX = buffers.velocityX[solver.bodyA]
  const newVelAY = buffers.velocityY[solver.bodyA]
  const newVelBX = buffers.velocityX[solver.bodyB]
  const newVelBY = buffers.velocityY[solver.bodyB]
  const newAngVelA = buffers.angularVel[solver.bodyA]
  const newAngVelB = buffers.angularVel[solver.bodyB]

  const relVelTX = (newVelBX + (-newAngVelB * solver.rBY)) - (newVelAX + (-newAngVelA * solver.rAY))
  const relVelTY = (newVelBY + (newAngVelB * solver.rBX)) - (newVelAY + (newAngVelA * solver.rAX))
  const relVelT = relVelTX * solver.tangentX + relVelTY * solver.tangentY

  const maxFriction = solver.friction * (normalContact?.normalImpulse ?? solver.normalImpulse)
  const lambdaT = solver.tangentMass * (-relVelT)
  const oldImpulseT = solver.tangentImpulse
  if (config?.impulseAccumulation) {
    solver.tangentImpulse = Math.max(-maxFriction, Math.min(maxFriction, solver.tangentImpulse + lambdaT))
  } else {
    solver.tangentImpulse = Math.max(-maxFriction, Math.min(maxFriction, lambdaT))
  }
  const dLambdaT = config?.impulseAccumulation
    ? solver.tangentImpulse - oldImpulseT
    : solver.tangentImpulse

  const impTX = dLambdaT * solver.tangentX
  const impTY = dLambdaT * solver.tangentY

  buffers.velocityX[solver.bodyA] -= impTX * invMassA
  buffers.velocityY[solver.bodyA] -= impTY * invMassA
  buffers.angularVel[solver.bodyA] -= (solver.rAX * impTY - solver.rAY * impTX) * invInertiaA

  buffers.velocityX[solver.bodyB] += impTX * invMassB
  buffers.velocityY[solver.bodyB] += impTY * invMassB
  buffers.angularVel[solver.bodyB] += (solver.rBX * impTY - solver.rBY * impTX) * invInertiaB
}

function solveBlock(
  block: BlockSolverData,
  buffers: MatchaBuffers,
): void {
  const { solver1: s1, solver2: s2, k, b } = block

  const velAX = buffers.velocityX[s1.bodyA]
  const velAY = buffers.velocityY[s1.bodyA]
  const velBX = buffers.velocityX[s1.bodyB]
  const velBY = buffers.velocityY[s1.bodyB]
  const angVelA = buffers.angularVel[s1.bodyA]
  const angVelB = buffers.angularVel[s1.bodyB]

  const rv1NX = (velBX + (-angVelB * s1.rBY)) - (velAX + (-angVelA * s1.rAY))
  const rv1NY = (velBY + (angVelB * s1.rBX)) - (velAY + (angVelA * s1.rAX))
  const relVelN1 = rv1NX * s1.normalX + rv1NY * s1.normalY

  const rv2NX = (velBX + (-angVelB * s2.rBY)) - (velAX + (-angVelA * s2.rAY))
  const rv2NY = (velBY + (angVelB * s2.rBX)) - (velAY + (angVelA * s2.rAX))
  const relVelN2 = rv2NX * s2.normalX + rv2NY * s2.normalY

  const b0 = relVelN1 + b[0]
  const b1 = relVelN2 + b[1]

  const aX = s1.normalImpulse
  const aY = s2.normalImpulse

  const bPrimeX = b0 - (k[0] * aX + k[1] * aY)
  const bPrimeY = b1 - (k[2] * aX + k[3] * aY)

  const det = k[0] * k[3] - k[1] * k[2]
  if (Math.abs(det) > 1e-12) {
    const x1 = (-bPrimeX * k[3] + bPrimeY * k[1]) / det
    const x2 = (bPrimeX * k[2] - bPrimeY * k[0]) / det
    if (x1 >= 0 && x2 >= 0) {
      applyBlockImpulses(s1, s2, x1, x2, buffers)
      return
    }
  }

  if (k[0] > 1e-12) {
    const x1 = -bPrimeX / k[0]
    if (x1 >= 0) {
      const lambda2 = k[2] * x1 + bPrimeY
      if (lambda2 <= 0) {
        applyBlockImpulses(s1, s2, x1, 0, buffers)
        return
      }
    }
  }

  if (k[3] > 1e-12) {
    const x2 = -bPrimeY / k[3]
    if (x2 >= 0) {
      const lambda1 = k[1] * x2 + bPrimeX
      if (lambda1 <= 0) {
        applyBlockImpulses(s1, s2, 0, x2, buffers)
        return
      }
    }
  }

  applyBlockImpulses(s1, s2, 0, 0, buffers)
}

function applyBlockImpulses(
  s1: ContactSolverData,
  s2: ContactSolverData,
  lambda1: number,
  lambda2: number,
  buffers: MatchaBuffers,
): void {
  const bodyA = s1.bodyA
  const bodyB = s1.bodyB
  const invMassA = buffers.invMass[bodyA]
  const invMassB = buffers.invMass[bodyB]
  const invInertiaA = buffers.invInertia[bodyA]
  const invInertiaB = buffers.invInertia[bodyB]

  const impX = (lambda1 * s1.normalX + lambda2 * s2.normalX)
  const impY = (lambda1 * s1.normalY + lambda2 * s2.normalY)

  buffers.velocityX[bodyA] -= impX * invMassA
  buffers.velocityY[bodyA] -= impY * invMassA
  buffers.angularVel[bodyA] -= (
    lambda1 * (s1.rAX * s1.normalY - s1.rAY * s1.normalX) +
    lambda2 * (s2.rAX * s2.normalY - s2.rAY * s2.normalX)
  ) * invInertiaA

  buffers.velocityX[bodyB] += impX * invMassB
  buffers.velocityY[bodyB] += impY * invMassB
  buffers.angularVel[bodyB] += (
    lambda1 * (s1.rBX * s1.normalY - s1.rBY * s1.normalX) +
    lambda2 * (s2.rBX * s2.normalY - s2.rBY * s2.normalX)
  ) * invInertiaB

  s1.normalImpulse = lambda1
  s2.normalImpulse = lambda2
}

function buildBlockSolver(
  manifold: ContactManifold,
  buffers: MatchaBuffers,
  dt: number,
  config: WorldConfig,
): BlockSolverData | null {
  if (manifold.contacts.length !== 2) return null

  const bodyAIdx = manifold.bodyA as number
  const bodyBIdx = manifold.bodyB as number
  const nx = manifold.normal.x
  const ny = manifold.normal.y

  const s1 = computeContactSolver(buffers, manifold.contacts[0], nx, ny, bodyAIdx, bodyBIdx, dt, config)
  const s2 = computeContactSolver(buffers, manifold.contacts[1], nx, ny, bodyAIdx, bodyBIdx, dt, config)

  const rn1A = s1.rAX * ny - s1.rAY * nx
  const rn1B = s1.rBX * ny - s1.rBY * nx
  const rn2A = s2.rAX * ny - s2.rAY * nx
  const rn2B = s2.rBX * ny - s2.rBY * nx

  const invMassA = buffers.invMass[bodyAIdx]
  const invMassB = buffers.invMass[bodyBIdx]
  const invInertiaA = buffers.invInertia[bodyAIdx]
  const invInertiaB = buffers.invInertia[bodyBIdx]

  const k00 = invMassA + invMassB + invInertiaA * rn1A * rn1A + invInertiaB * rn1B * rn1B
  const k01 = invMassA + invMassB + invInertiaA * rn1A * rn2A + invInertiaB * rn1B * rn2B
  const k10 = invMassA + invMassB + invInertiaA * rn2A * rn1A + invInertiaB * rn2B * rn1B
  const k11 = invMassA + invMassB + invInertiaA * rn2A * rn2A + invInertiaB * rn2B * rn2B

  const b0 = -(config.baumgarteFactor / dt) * Math.max(manifold.contacts[0].penetration - config.penetrationSlop, 0)
  const b1 = -(config.baumgarteFactor / dt) * Math.max(manifold.contacts[1].penetration - config.penetrationSlop, 0)

  return {
    solver1: s1,
    solver2: s2,
    k: [k00, k01, k10, k11],
    b: [b0, b1],
    impulses: [0, 0],
  }
}

function solvePositionConstraint(
  solver: ContactSolverData,
  buffers: MatchaBuffers,
  config: WorldConfig,
  penetration: number,
): void {
  if (solver.normalMass === 0) return

  const invMassA = buffers.invMass[solver.bodyA]
  const invMassB = buffers.invMass[solver.bodyB]
  const invInertiaA = buffers.invInertia[solver.bodyA]
  const invInertiaB = buffers.invInertia[solver.bodyB]

  const slop = config.penetrationSlop
  const bias = Math.max(penetration - slop, 0)
  if (bias <= 0) return

  const rAX = solver.rAX
  const rAY = solver.rAY
  const rBX = solver.rBX
  const rBY = solver.rBY
  const nx = solver.normalX
  const ny = solver.normalY

  const rnA = rAX * ny - rAY * nx
  const rnB = rBX * ny - rBY * nx
  const effectiveMass = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB
  if (effectiveMass < 1e-12) return

  const lambda = config.baumgarteFactor * bias / effectiveMass

  const impX = lambda * nx
  const impY = lambda * ny

  buffers.positionX[solver.bodyA] -= impX * invMassA
  buffers.positionY[solver.bodyA] -= impY * invMassA
  buffers.angle[solver.bodyA] -= (rAX * impY - rAY * impX) * invInertiaA

  buffers.positionX[solver.bodyB] += impX * invMassB
  buffers.positionY[solver.bodyB] += impY * invMassB
  buffers.angle[solver.bodyB] += (rBX * impY - rBY * impX) * invInertiaB
}

export function solveVelocity(
  buffers: MatchaBuffers,
  manifolds: ContactManifold[],
  config: WorldConfig,
): void {
  const dt = config.fixedTimestep
  const iterations = config.velocityIterations

  const contactSolvers: ContactSolverData[][] = []
  const blockSolvers: (BlockSolverData | null)[] = []

  for (let m = 0; m < manifolds.length; m++) {
    const manifold = manifolds[m]
    const bodyAIdx = manifold.bodyA as number
    const bodyBIdx = manifold.bodyB as number

    if ((buffers.flags[bodyAIdx] & BodyFlags.SLEEPING) !== 0 &&
        (buffers.flags[bodyBIdx] & BodyFlags.SLEEPING) !== 0) continue

    const solvers: ContactSolverData[] = []
    for (let c = 0; c < manifold.contacts.length; c++) {
      const solver = computeContactSolver(
        buffers,
        manifold.contacts[c],
        manifold.normal.x,
        manifold.normal.y,
        bodyAIdx,
        bodyBIdx,
        dt,
        config,
      )
      solvers.push(solver)
    }
    contactSolvers.push(solvers)

    if (config.blockSolver && manifold.contacts.length === 2) {
      blockSolvers.push(buildBlockSolver(manifold, buffers, dt, config))
    } else {
      blockSolvers.push(null)
    }
  }

  for (let iter = 0; iter < iterations; iter++) {
    for (let m = 0; m < contactSolvers.length; m++) {
      const solvers = contactSolvers[m]
      const block = blockSolvers[m]

      if (block) {
        solveBlock(block, buffers)
      } else {
        for (let c = 0; c < solvers.length; c++) {
          solveContact(solvers[c], buffers, undefined, config)
        }
      }
    }
  }
}

export function solvePosition(
  buffers: MatchaBuffers,
  manifolds: ContactManifold[],
  config: WorldConfig,
): void {
  const iterations = config.positionIterations

  for (let iter = 0; iter < iterations; iter++) {
    for (let m = 0; m < manifolds.length; m++) {
      const manifold = manifolds[m]
      const bodyAIdx = manifold.bodyA as number
      const bodyBIdx = manifold.bodyB as number

      if ((buffers.flags[bodyAIdx] & BodyFlags.STATIC) !== 0 &&
          (buffers.flags[bodyBIdx] & BodyFlags.STATIC) !== 0) continue

      for (let c = 0; c < manifold.contacts.length; c++) {
        const solver = computeContactSolver(
          buffers,
          manifold.contacts[c],
          manifold.normal.x,
          manifold.normal.y,
          bodyAIdx,
          bodyBIdx,
          config.fixedTimestep,
          config,
        )
        solvePositionConstraint(solver, buffers, config, manifold.contacts[c].penetration)
      }
    }
  }
}

export function integrate(
  buffers: MatchaBuffers,
  count: number,
  dt: number,
  gravity: { x: number; y: number },
): void {
  for (let i = 0; i < count; i++) {
    if ((buffers.flags[i] & BodyFlags.ACTIVE) === 0) continue
    if ((buffers.flags[i] & BodyFlags.STATIC) !== 0) continue
    if ((buffers.flags[i] & BodyFlags.SLEEPING) !== 0) continue

    const invMass = buffers.invMass[i]
    if (invMass === 0) continue

    const ax = gravity.x
    const ay = gravity.y

    buffers.velocityX[i] += ax * dt
    buffers.velocityY[i] += ay * dt

    buffers.positionX[i] += buffers.velocityX[i] * dt
    buffers.positionY[i] += buffers.velocityY[i] * dt

    buffers.angle[i] += buffers.angularVel[i] * dt
  }
}
