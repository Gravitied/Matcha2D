import { WorldConfig, BodyHandle, CollisionCallbacks, NarrowphaseColliderOutline } from '@matcha2d/types';
export { NarrowphaseColliderOutline, parseNarrowphaseOutlinesFlat } from '@matcha2d/types';
import { PhysicsEngine } from '@matcha2d/physics-rust';

interface BodyDef {
    positionX?: number;
    positionY?: number;
    velocityX?: number;
    velocityY?: number;
    angle?: number;
    type?: 'dynamic' | 'static' | 'kinematic';
    /** Collision group bitfield (which groups this body belongs to). Default: all bits set. */
    collisionGroups?: number;
    /** Collision mask bitfield (which groups this body can collide with). Default: all bits set. */
    collisionMask?: number;
}
interface ColliderDef {
    shape: 'ball' | 'cuboid' | 'polygon';
    radius?: number;
    halfExtentX?: number;
    halfExtentY?: number;
    vertices?: Array<{
        x: number;
        y: number;
    }>;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}
declare class World {
    readonly config: WorldConfig;
    private _engine;
    private _accumulator;
    private _bodyHandles;
    private _nextBodyId;
    private constructor();
    /**
     * Create a World with the WASM physics engine.
     * This is async because the WASM module must be loaded first.
     */
    static create(config?: Partial<WorldConfig>): Promise<World>;
    /** Create a new body with the given properties. */
    createBody(def?: BodyDef): BodyHandle;
    /** Destroy a body. */
    destroyBody(handle: BodyHandle): void;
    /** Add a collider to a body. */
    addCollider(body: BodyHandle, def: ColliderDef): number;
    /** Set body position. */
    setBodyPosition(handle: BodyHandle, x: number, y: number): void;
    /** Set body velocity. */
    setBodyVelocity(handle: BodyHandle, vx: number, vy: number): void;
    /** Set body angle. */
    setBodyAngle(handle: BodyHandle, angle: number): void;
    /** Set gravity (affects all dynamic bodies). */
    setGravity(x: number, y: number): void;
    /**
     * Axis-aligned world bounds (meters). Dynamic bodies are nudged back inside after each step.
     * Pass `null` to disable clamping.
     */
    setWorldBounds(bounds: {
        minX: number;
        minY: number;
        maxX: number;
        maxY: number;
    } | null): void;
    /** Set collision groups (bitfield) for a body. */
    setCollisionGroups(handle: BodyHandle, groups: number): void;
    /** Set collision mask (who this body collides with) for a body. */
    setCollisionMask(handle: BodyHandle, mask: number): void;
    /** Wake a body from sleep so it participates in physics. */
    wakeBody(handle: BodyHandle): void;
    /** Force a body to sleep (stop simulating). */
    sleepBody(handle: BodyHandle): void;
    /** Check if a body is currently sleeping. */
    isBodySleeping(handle: BodyHandle): boolean;
    /**
     * Mass-weighted push-out for a dynamic body using narrowphase overlaps (spawn / layout helper).
     * Safe to call multiple times; pairs of dynamics may need several round-robins.
     */
    resolveOverlapsForBody(handle: BodyHandle): void;
    /** Get body position as [x, y]. */
    getBodyPosition(handle: BodyHandle): [number, number];
    /** Get body velocity as [vx, vy]. */
    getBodyVelocity(handle: BodyHandle): [number, number];
    /** Get body angle in radians. */
    getBodyAngle(handle: BodyHandle): number;
    /** Set body angular velocity. */
    setBodyAngularVelocity(handle: BodyHandle, angvel: number): void;
    /** Get body angular velocity in rad/s. */
    getBodyAngularVelocity(handle: BodyHandle): number;
    /** Apply an impulse to a body. */
    applyImpulse(handle: BodyHandle, fx: number, fy: number): void;
    /**
     * Advance exactly one fixed physics tick (`PhysicsEngine.step()`), ignoring the frame accumulator.
     * Intended for tests and deterministic bisect (e.g. hunting WASM `unreachable`).
     */
    stepPhysicsOnce(): void;
    /** Step the simulation by the given delta time. */
    step(dt: number): void;
    /** Get the interpolation alpha for rendering. */
    get renderAlpha(): number;
    /** Get all body positions X coordinates. */
    getPositionsX(): Float32Array;
    /** Get all body positions Y coordinates. */
    getPositionsY(): Float32Array;
    /** Get all body angles. */
    getAngles(): Float32Array;
    /** Get the number of bodies. */
    get bodyCount(): number;
    /** Get the number of colliders. */
    get colliderCount(): number;
    /** Set collision callbacks (placeholder for future contact event support). */
    setCollisionCallbacks(_callbacks: CollisionCallbacks | null): void;
    /** Access the underlying physics engine (for advanced use). */
    get engine(): PhysicsEngine;
    /** Initialize WebGPU for hybrid GPU stepping. */
    initGpu(): Promise<void>;
    /** Enable or disable GPU acceleration. Requires `initGpu()` to be called first. */
    setGpuAccelerationEnabled(enabled: boolean): void;
    /** Number of active contact pairs this frame. */
    get contactPairCount(): number;
    /** Total contact points across all pairs this frame. */
    get contactPointCount(): number;
    /** Maximum penetration depth across all contacts this frame. */
    get maxPenetration(): number;
    /** Contact normal of the first active pair (or [0,0] if none). */
    get firstContactNormal(): [number, number];
    /** Full debug dump string for the current frame state. */
    get debugDump(): string;
    /**
     * Collider outlines in world space — same geometry/transform as Rust narrowphase SAT.
     * Use for debug overlay; compare to visual proxies that use body COM + guessed size.
     */
    getNarrowphaseColliderOutlines(): NarrowphaseColliderOutline[];
}

export { type BodyDef, type ColliderDef, World };
