export { createBuffers, MAX_BODIES } from './buffers.js'
export type { MatchaBuffers } from './buffers.js'

export { BodyFlags, BodyType } from './body.js'
export type { BodyHandle } from './body.js'

export type { Vec2Readonly, AABB, Transform } from './math.js'

export type {
  ContactPoint,
  ContactManifold,
  CollisionPair,
} from './collision.js'

export { JointType } from './constraint.js'
export type { ConstraintDef } from './constraint.js'

export { DEFAULT_WORLD_CONFIG } from './config.js'
export type { WorldConfig } from './config.js'

export type { PhysicsBackend } from './backend.js'
