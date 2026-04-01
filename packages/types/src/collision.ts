import type { BodyHandle } from './body.js'
import type { Vec2Readonly } from './math.js'

/** A single contact point between two bodies. */
export interface ContactPoint {
  localA: Vec2Readonly
  localB: Vec2Readonly
  penetration: number
}

/** Result of narrowphase collision between two bodies. */
export interface ContactManifold {
  bodyA: BodyHandle
  bodyB: BodyHandle
  normal: Vec2Readonly
  contacts: ContactPoint[]
}

/** A pair flagged by broadphase for narrowphase testing. */
export interface CollisionPair {
  a: BodyHandle
  b: BodyHandle
}
