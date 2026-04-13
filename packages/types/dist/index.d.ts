/** Readonly 2D vector (for function signatures). */
interface Vec2Readonly {
    readonly x: number;
    readonly y: number;
}
/** Axis-Aligned Bounding Box. */
interface AABB {
    minX: number;
    minY: number;
    maxX: number;
    maxY: number;
}
/** Rigid transform: position + rotation angle (radians). */
interface Transform {
    x: number;
    y: number;
    angle: number;
}

/**
 * A BodyHandle is an opaque index into the MatchaBuffers arrays.
 * Branded type prevents accidental misuse of raw numbers.
 */
type BodyHandle = number & {
    readonly __brand: 'BodyHandle';
};
/** Bit flags stored in MatchaBuffers.flags */
declare const BodyFlags: {
    readonly ACTIVE: 1;
    readonly STATIC: 2;
    readonly SLEEPING: 4;
    readonly SENSOR: 8;
};
declare const BodyType: {
    readonly Dynamic: 0;
    readonly Static: 1;
    readonly Kinematic: 2;
};
type BodyType = (typeof BodyType)[keyof typeof BodyType];

/** Maximum vertices per convex polygon shape. */
declare const MAX_VERTICES_PER_SHAPE = 16;
/** Flat SoA buffers shared between physics core and world management. */
interface MatchaBuffers {
    positionX: Float32Array;
    positionY: Float32Array;
    velocityX: Float32Array;
    velocityY: Float32Array;
    angle: Float32Array;
    angularVel: Float32Array;
    mass: Float32Array;
    invMass: Float32Array;
    inertia: Float32Array;
    invInertia: Float32Array;
    flags: Uint8Array;
    /** AABB half-extents along X — set at body creation, used by broadphase. */
    halfExtentX: Float32Array;
    /** AABB half-extents along Y — set at body creation, used by broadphase. */
    halfExtentY: Float32Array;
    /** Shape type per body: 0=Box, 1=Circle, 2=Polygon. */
    shapeType: Uint8Array;
    /** Circle radius (used when shapeType == Circle). */
    shapeRadius: Float32Array;
    /** Number of vertices (used when shapeType == Polygon, 0 otherwise). */
    shapeVertexCount: Uint8Array;
    /** Polygon vertices X (capacity * MAX_VERTICES_PER_SHAPE, local space). */
    shapeVerticesX: Float32Array;
    /** Polygon vertices Y (capacity * MAX_VERTICES_PER_SHAPE, local space). */
    shapeVerticesY: Float32Array;
}
/** Maximum bodies supported in one world. */
declare const MAX_BODIES = 8192;
/** Allocate a fresh set of buffers for `capacity` bodies. */
declare function createBuffers(capacity?: number): MatchaBuffers;

/** Shape types supported by the collision system. */
declare const ShapeType: {
    readonly Box: 0;
    readonly Circle: 1;
    readonly Polygon: 2;
};
type ShapeType = (typeof ShapeType)[keyof typeof ShapeType];
/** A single contact point between two bodies. */
interface ContactPoint {
    localA: Vec2Readonly;
    localB: Vec2Readonly;
    penetration: number;
    /** Feature ID for contact persistence (vertex index or -1 for circles). */
    idA?: number;
    idB?: number;
}
/** Result of narrowphase collision between two bodies. */
interface ContactManifold {
    bodyA: BodyHandle;
    bodyB: BodyHandle;
    normal: Vec2Readonly;
    contacts: ContactPoint[];
    /** Accumulated normal impulses for warm starting (persisted across frames). */
    accumulatedImpulses?: number[];
    /** Whether this manifold persisted from the previous frame. */
    persistent?: boolean;
}
/** A pair flagged by broadphase for narrowphase testing. */
interface CollisionPair {
    a: BodyHandle;
    b: BodyHandle;
}
/** Contact info exposed to collision callbacks. */
interface ContactInfo {
    bodyA: BodyHandle;
    bodyB: BodyHandle;
    contactNormal: {
        x: number;
        y: number;
    };
    contactPoints: Array<{
        x: number;
        y: number;
    }>;
    totalImpulse: number;
}
/** Collision callback listener. */
interface CollisionCallbacks {
    /** Called when two bodies first start touching. */
    onBeginContact?(info: ContactInfo): void;
    /** Called every frame while two bodies remain in contact. */
    onStayContact?(info: ContactInfo): void;
    /** Called when two bodies stop touching. */
    onEndContact?(bodyA: BodyHandle, bodyB: BodyHandle): void;
    /** Called for sensor/trigger bodies (no physical response). */
    onTriggerEnter?(bodyA: BodyHandle, bodyB: BodyHandle): void;
}

declare const JointType: {
    readonly Distance: 0;
    readonly Revolute: 1;
    readonly Prismatic: 2;
    readonly Weld: 3;
};
type JointType = (typeof JointType)[keyof typeof JointType];
interface ConstraintDef {
    type: JointType;
    bodyA: BodyHandle;
    bodyB: BodyHandle;
    anchorA: Vec2Readonly;
    anchorB: Vec2Readonly;
}

interface WorldConfig {
    gravity: {
        x: number;
        y: number;
    };
    fixedTimestep: number;
    /** Outer solver substeps per `engine.step()` (integration between passes). */
    velocityIterations: number;
    /** PGS (Gauss–Seidel) iterations per substep; higher improves contact/friction convergence. */
    internalVelocityIterations: number;
    /** @deprecated Ignored by the WASM solver (stabilization pass removed). Kept for saved configs / API stability. */
    positionIterations: number;
    /** Position correction bias factor (0.0–1.0). 0.05 is conservative; 0.2 is Box2D's default for meter-scale physics. */
    baumgarteFactor: number;
    /**
     * Maximum velocity magnitude for spring penetration bias (`f32::INFINITY` / `Number.POSITIVE_INFINITY` = uncapped).
     */
    maxCorrectiveVelocity: number;
    /** Penetration slop: allowed penetration depth before correction kicks in. */
    penetrationSlop: number;
    /** @deprecated Ignored by the WASM solver (per-substep impulse reset). Kept for saved configs / API stability. */
    warmStarting: boolean;
    /** Enable block solver for 2-contact manifolds (reduces jitter). */
    blockSolver: boolean;
    /** Default friction coefficient when not specified per-body. */
    defaultFriction: number;
    /** Default restitution (bounciness) coefficient when not specified per-body. */
    defaultRestitution: number;
    /** Restitution slop: velocity threshold below which restitution is ignored. */
    restitutionSlop: number;
    /** Default linear damping coefficient (energy dissipation). 0 = no damping, 1 = heavy damping. */
    defaultLinearDamping: number;
    /** Default angular damping coefficient. */
    defaultAngularDamping: number;
    /** Enable swept AABB broadphase to catch fast-moving objects (CCD-lite). */
    sweptBroadphase: boolean;
    /** Linear velocity threshold (m/s) below which a body is a candidate for sleeping. */
    sleepVelocityThreshold: number;
    /** Angular velocity threshold (rad/s) below which a body is a candidate for sleeping. */
    sleepAngularVelocityThreshold: number;
    /** Time (seconds) a body must be below velocity thresholds before it sleeps. */
    sleepTimeThreshold: number;
    /**
     * Optional post-PGS clamp on |linear velocity| for bodies in contact (`Infinity` disables).
     * Prefer raising substeps / `pgsRelaxation` tuning over low caps.
     */
    maxSolverLinearVelocity: number;
    /** Optional post-PGS clamp on |angular velocity| (`Infinity` disables). */
    maxSolverAngularVelocity: number;
    /** PGS impulse under-relaxation in `(0, 1]` — principal stability knob instead of velocity clamps. */
    pgsRelaxation: number;
    /**
     * Optional cap on |linear velocity| before substep integration (`Infinity` = none).
     * Does not replace CCD / TOI for tunneling.
     */
    maxIntegrateLinearVelocity: number;
    /** Optional cap on |angular velocity| before substep integration (`Infinity` = none). */
    maxIntegrateAngularVelocity: number;
    /** Optional axis-aligned bounds (meters); dynamic bodies are clamped after each step. Pass null to disable. */
    worldBounds: {
        minX: number;
        minY: number;
        maxX: number;
        maxY: number;
    } | null;
}
declare const DEFAULT_WORLD_CONFIG: Readonly<WorldConfig>;

/**
 * One collider outline in **world space** (meters, Y-up), matching Rust narrowphase
 * `collider_world_transform` + local shape geometry.
 */
interface NarrowphaseColliderOutline {
    colliderHandle: number;
    /** Parent rigid body handle, or `null` if unparented */
    bodyHandle: number | null;
    shapeKind: 'ball' | 'cuboid' | 'polygon';
    sensor: boolean;
    /** Loop vertices in world meters; winding matches engine (CCW for poly/cuboid in collider space) */
    vertices: Array<{
        x: number;
        y: number;
    }>;
}
/**
 * Parse WASM `PhysicsEngine.debug_narrowphase_outlines()` flat buffer.
 *
 * Layout: `[entry_count,` then per collider:
 * `collider_handle, body_handle, shape_kind, flags, point_count,` then `point_count` (x,y) pairs.
 * - `shape_kind`: 0 ball, 1 cuboid, 2 polygon
 * - `flags`: 1 = sensor
 * - `body_handle`: NaN means no parent
 *
 * **Note:** handles are encoded as f32; values above ~2^24 may round. Typical Matcha handles are safe.
 */
declare function parseNarrowphaseOutlinesFlat(data: number[] | Float32Array): NarrowphaseColliderOutline[];

export { type AABB, BodyFlags, type BodyHandle, BodyType, type CollisionCallbacks, type CollisionPair, type ConstraintDef, type ContactInfo, type ContactManifold, type ContactPoint, DEFAULT_WORLD_CONFIG, JointType, MAX_BODIES, MAX_VERTICES_PER_SHAPE, type MatchaBuffers, type NarrowphaseColliderOutline, ShapeType, type Transform, type Vec2Readonly, type WorldConfig, createBuffers, parseNarrowphaseOutlinesFlat };
