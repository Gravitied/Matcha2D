export interface WorldConfig {
  gravity: { x: number; y: number }
  fixedTimestep: number
  velocityIterations: number
  positionIterations: number
  /** Position correction bias factor. Very small for pixel-scale physics (0.001-0.01). Higher = more aggressive. */
  baumgarteFactor: number
  /** Maximum velocity for position correction (caps per-step correction). */
  maxCorrectiveVelocity: number
  /** Penetration slop: allowed penetration depth before correction kicks in. */
  penetrationSlop: number
  /** Enable warm starting (reuse impulses from previous frame). */
  warmStarting: boolean
  /** Enable block solver for 2-contact manifolds (reduces jitter). */
  blockSolver: boolean
  /** Default friction coefficient when not specified per-body. */
  defaultFriction: number
  /** Default restitution (bounciness) coefficient when not specified per-body. */
  defaultRestitution: number
  /** Restitution slop: velocity threshold below which restitution is ignored. */
  restitutionSlop: number
  /** Enable swept AABB broadphase to catch fast-moving objects (CCD-lite). */
  sweptBroadphase: boolean
  /** Linear velocity threshold (m/s) below which a body is a candidate for sleeping. */
  sleepVelocityThreshold: number
  /** Angular velocity threshold (rad/s) below which a body is a candidate for sleeping. */
  sleepAngularVelocityThreshold: number
  /** Time (seconds) a body must be below velocity thresholds before it sleeps. */
  sleepTimeThreshold: number
}

export const DEFAULT_WORLD_CONFIG: Readonly<WorldConfig> = {
  gravity: { x: 0, y: -9.81 },
  fixedTimestep: 1 / 60,
  velocityIterations: 8,
  positionIterations: 3,
  baumgarteFactor: 0.01,
  maxCorrectiveVelocity: 50.0,
  penetrationSlop: 0.02,
  warmStarting: true,
  blockSolver: true,
  defaultFriction: 0.3,
  defaultRestitution: 0.0,
  restitutionSlop: 1.0,
  sweptBroadphase: true,
  sleepVelocityThreshold: 0.05,
  sleepAngularVelocityThreshold: 0.1,
  sleepTimeThreshold: 1.0,
}
