export type BroadphaseMethod = 'sap' | 'bvh' | 'dynamicTree'
export type NarrowphaseMethod = 'gjk'

export interface WorldConfig {
  gravity: { x: number; y: number }
  fixedTimestep: number
  velocityIterations: number
  positionIterations: number
  maxBodies: number
  sleepEnabled: boolean
  sleepTimeThreshold: number
  sleepVelocityThreshold: number
  /** Broadphase algorithm. 'sap' = Sort-and-Sweep. 'dynamicTree' = incremental AABB tree (default). */
  broadphaseMethod: BroadphaseMethod
  /** Narrowphase algorithm. 'gjk' = GJK + EPA (only option). */
  narrowphaseMethod: NarrowphaseMethod
  /** Position correction bias factor (0.01-0.2). Higher = more aggressive correction. */
  baumgarteFactor: number
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
  /** Enable impulse accumulation (warm starting). */
  impulseAccumulation: boolean
  /** Position correction mode. */
  positionCorrection: boolean
  /** Position correction beta factor. */
  positionCorrectionBeta: number
  /** Warm start distance threshold — skip if contact points moved further. */
  warmStartThreshold: number
}

export const DEFAULT_WORLD_CONFIG: Readonly<WorldConfig> = {
  gravity: { x: 0, y: -9.81 },
  fixedTimestep: 1 / 60,
  velocityIterations: 10,
  positionIterations: 4,
  maxBodies: 8192,
  sleepEnabled: true,
  sleepTimeThreshold: 0.5,
  sleepVelocityThreshold: 0.01,
  broadphaseMethod: 'dynamicTree',
  narrowphaseMethod: 'gjk',
  baumgarteFactor: 0.2,
  penetrationSlop: 0.005,
  warmStarting: true,
  blockSolver: true,
  defaultFriction: 0.3,
  defaultRestitution: 0.0,
  restitutionSlop: 0.01,
  impulseAccumulation: true,
  positionCorrection: true,
  positionCorrectionBeta: 0.2,
  warmStartThreshold: 0.01,
}
