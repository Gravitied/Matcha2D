export { createBuffers, MAX_BODIES, MAX_VERTICES_PER_SHAPE } from './buffers.js'
export type { MatchaBuffers } from './buffers.js'

export { BodyFlags, BodyType } from './body.js'
export type { BodyHandle } from './body.js'

export type { Vec2Readonly, AABB, Transform } from './math.js'

export { ShapeType } from './collision.js'
export type {
  ContactPoint,
  ContactManifold,
  CollisionPair,
  ContactInfo,
  CollisionCallbacks,
} from './collision.js'

export { JointType } from './constraint.js'
export type { ConstraintDef } from './constraint.js'

export { DEFAULT_WORLD_CONFIG } from './config.js'
export type { WorldConfig, BroadphaseMethod, NarrowphaseMethod } from './config.js'

export type { PhysicsBackend } from './backend.js'
