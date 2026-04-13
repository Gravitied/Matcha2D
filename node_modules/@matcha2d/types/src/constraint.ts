import type { BodyHandle } from './body.js'
import type { Vec2Readonly } from './math.js'

export const JointType = {
  Distance: 0,
  Revolute: 1,
  Prismatic: 2,
  Weld: 3,
} as const

export type JointType = (typeof JointType)[keyof typeof JointType]

export interface ConstraintDef {
  type: JointType
  bodyA: BodyHandle
  bodyB: BodyHandle
  anchorA: Vec2Readonly
  anchorB: Vec2Readonly
}
