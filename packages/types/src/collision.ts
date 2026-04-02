import type { BodyHandle } from './body.js'
import type { Vec2Readonly } from './math.js'

/** Shape types supported by the collision system. */
export const ShapeType = {
  Box: 0,
  Circle: 1,
  Polygon: 2,
} as const

export type ShapeType = (typeof ShapeType)[keyof typeof ShapeType]

/** A single contact point between two bodies. */
export interface ContactPoint {
  localA: Vec2Readonly
  localB: Vec2Readonly
  penetration: number
  /** Feature ID for contact persistence (vertex index or -1 for circles). */
  idA?: number
  idB?: number
}

/** Result of narrowphase collision between two bodies. */
export interface ContactManifold {
  bodyA: BodyHandle
  bodyB: BodyHandle
  normal: Vec2Readonly
  contacts: ContactPoint[]
  /** Accumulated normal impulses for warm starting (persisted across frames). */
  accumulatedImpulses?: number[]
  /** Whether this manifold persisted from the previous frame. */
  persistent?: boolean
}

/** A pair flagged by broadphase for narrowphase testing. */
export interface CollisionPair {
  a: BodyHandle
  b: BodyHandle
}

/** Contact info exposed to collision callbacks. */
export interface ContactInfo {
  bodyA: BodyHandle
  bodyB: BodyHandle
  contactNormal: { x: number; y: number }
  contactPoints: Array<{ x: number; y: number }>
  totalImpulse: number
}

/** Collision callback listener. */
export interface CollisionCallbacks {
  /** Called when two bodies first start touching. */
  onBeginContact?(info: ContactInfo): void
  /** Called every frame while two bodies remain in contact. */
  onStayContact?(info: ContactInfo): void
  /** Called when two bodies stop touching. */
  onEndContact?(bodyA: BodyHandle, bodyB: BodyHandle): void
  /** Called for sensor/trigger bodies (no physical response). */
  onTriggerEnter?(bodyA: BodyHandle, bodyB: BodyHandle): void
}
