import type { WorldConfig, BodyHandle, CollisionCallbacks } from '@matcha2d/types'
import { DEFAULT_WORLD_CONFIG } from '@matcha2d/types'
import init, { PhysicsEngine } from '@matcha2d/physics-rust'

export interface BodyDef {
  positionX?: number
  positionY?: number
  velocityX?: number
  velocityY?: number
  angle?: number
  type?: 'dynamic' | 'static' | 'kinematic'
  /** Collision group bitfield (which groups this body belongs to). Default: all bits set. */
  collisionGroups?: number
  /** Collision mask bitfield (which groups this body can collide with). Default: all bits set. */
  collisionMask?: number
}

export interface ColliderDef {
  shape: 'ball' | 'cuboid' | 'polygon'
  radius?: number
  halfExtentX?: number
  halfExtentY?: number
  vertices?: Array<{ x: number; y: number }>
  friction?: number
  restitution?: number
  sensor?: boolean
}

export class World {
  readonly config: WorldConfig
  private _engine: PhysicsEngine
  private _accumulator = 0
  private _bodyHandles: Map<number, BodyHandle> = new Map()
  private _nextBodyId = 0

  private constructor(engine: PhysicsEngine, config: WorldConfig) {
    this._engine = engine
    this.config = config
  }

  /**
   * Create a World with the WASM physics engine.
   * This is async because the WASM module must be loaded first.
   */
  static async create(config: Partial<WorldConfig> = {}): Promise<World> {
    await init()
    const merged = { ...DEFAULT_WORLD_CONFIG, ...config }
    const engine = new PhysicsEngine()
    engine.set_gravity(merged.gravity.x, merged.gravity.y)
    engine.set_dt(merged.fixedTimestep)
    engine.set_swept_broadphase(merged.sweptBroadphase)
    engine.set_sleep_thresholds(
      merged.sleepVelocityThreshold,
      merged.sleepAngularVelocityThreshold,
      merged.sleepTimeThreshold,
    )
    engine.set_solver_config(
      merged.velocityIterations,
      1,
      merged.positionIterations,
      merged.warmStarting ? 1.0 : 0.0,
      merged.blockSolver,
    )
    engine.set_position_correction(merged.baumgarteFactor, merged.maxCorrectiveVelocity ?? 10.0)
    return new World(engine, merged)
  }

  /** Create a new body with the given properties. */
  createBody(def: BodyDef = {}): BodyHandle {
    const x = def.positionX ?? 0
    const y = def.positionY ?? 0
    const bodyType = def.type ?? 'dynamic'

    let handle: number
    switch (bodyType) {
      case 'static':
        handle = this._engine.create_static_body(x, y)
        break
      case 'kinematic':
        handle = this._engine.create_kinematic_body(x, y)
        break
      default:
        handle = this._engine.create_dynamic_body(x, y)
        break
    }

    if (def.velocityX !== undefined || def.velocityY !== undefined) {
      this._engine.set_body_velocity(handle, def.velocityX ?? 0, def.velocityY ?? 0)
    }
    if (def.angle !== undefined) {
      this._engine.set_body_angle(handle, def.angle)
    }
    if (def.collisionGroups !== undefined) {
      this._engine.set_body_collision_groups(handle, def.collisionGroups)
    }
    if (def.collisionMask !== undefined) {
      this._engine.set_body_collision_mask(handle, def.collisionMask)
    }

    return handle as unknown as BodyHandle
  }

  /** Destroy a body. */
  destroyBody(handle: BodyHandle): void {
    this._engine.remove_body(handle as unknown as number)
  }

  /** Add a collider to a body. */
  addCollider(body: BodyHandle, def: ColliderDef): number {
    const bodyHandle = body as unknown as number

    let colliderHandle: number
    switch (def.shape) {
      case 'ball':
        colliderHandle = this._engine.create_ball_collider(def.radius ?? 0.5, bodyHandle)
        break
      case 'cuboid':
        colliderHandle = this._engine.create_box_collider(
          def.halfExtentX ?? 0.5,
          def.halfExtentY ?? 0.5,
          bodyHandle,
        )
        break
      case 'polygon':
        if (def.vertices && def.vertices.length > 0) {
          const vx = new Float32Array(def.vertices.map(v => v.x))
          const vy = new Float32Array(def.vertices.map(v => v.y))
          colliderHandle = this._engine.create_polygon_collider(vx, vy, bodyHandle)
        } else {
          // Fallback to cuboid
          colliderHandle = this._engine.create_box_collider(0.5, 0.5, bodyHandle)
        }
        break
      default:
        colliderHandle = this._engine.create_ball_collider(0.5, bodyHandle)
        break
    }

    if (def.friction !== undefined) {
      this._engine.set_collider_friction(colliderHandle, def.friction)
    }
    if (def.restitution !== undefined) {
      this._engine.set_collider_restitution(colliderHandle, def.restitution)
    }
    if (def.sensor !== undefined) {
      this._engine.set_collider_sensor(colliderHandle, def.sensor)
    }

    return colliderHandle
  }

  /** Set body position. */
  setBodyPosition(handle: BodyHandle, x: number, y: number): void {
    this._engine.set_body_position(handle as unknown as number, x, y)
  }

  /** Set body velocity. */
  setBodyVelocity(handle: BodyHandle, vx: number, vy: number): void {
    this._engine.set_body_velocity(handle as unknown as number, vx, vy)
  }

  /** Set body angle. */
  setBodyAngle(handle: BodyHandle, angle: number): void {
    this._engine.set_body_angle(handle as unknown as number, angle)
  }

  /** Set gravity (affects all dynamic bodies). */
  setGravity(x: number, y: number): void {
    this._engine.set_gravity(x, y)
    this.config.gravity = { x, y }
  }

  /** Set collision groups (bitfield) for a body. */
  setCollisionGroups(handle: BodyHandle, groups: number): void {
    this._engine.set_body_collision_groups(handle as unknown as number, groups)
  }

  /** Set collision mask (who this body collides with) for a body. */
  setCollisionMask(handle: BodyHandle, mask: number): void {
    this._engine.set_body_collision_mask(handle as unknown as number, mask)
  }

  /** Wake a body from sleep so it participates in physics. */
  wakeBody(handle: BodyHandle): void {
    this._engine.wake_body(handle as unknown as number)
  }

  /** Force a body to sleep (stop simulating). */
  sleepBody(handle: BodyHandle): void {
    this._engine.sleep_body(handle as unknown as number)
  }

  /** Check if a body is currently sleeping. */
  isBodySleeping(handle: BodyHandle): boolean {
    return this._engine.is_body_sleeping(handle as unknown as number)
  }

  /** Get body position as [x, y]. */
  getBodyPosition(handle: BodyHandle): [number, number] {
    const pos = this._engine.get_body_position(handle as unknown as number)
    return [pos[0], pos[1]]
  }

  /** Get body velocity as [vx, vy]. */
  getBodyVelocity(handle: BodyHandle): [number, number] {
    const vel = this._engine.get_body_velocity(handle as unknown as number)
    return [vel[0], vel[1]]
  }

  /** Get body angle in radians. */
  getBodyAngle(handle: BodyHandle): number {
    return this._engine.get_body_angle(handle as unknown as number)
  }

  /** Set body angular velocity. */
  setBodyAngularVelocity(handle: BodyHandle, angvel: number): void {
    this._engine.set_body_angular_velocity(handle as unknown as number, angvel)
  }

  /** Get body angular velocity in rad/s. */
  getBodyAngularVelocity(handle: BodyHandle): number {
    return this._engine.get_body_angular_velocity(handle as unknown as number)
  }

  /** Apply an impulse to a body. */
  applyImpulse(handle: BodyHandle, fx: number, fy: number): void {
    this._engine.apply_impulse(handle as unknown as number, fx, fy)
  }

  /** Step the simulation by the given delta time. */
  step(dt: number): void {
    this._accumulator += dt

    while (this._accumulator >= this.config.fixedTimestep) {
      this._engine.step()
      this._accumulator -= this.config.fixedTimestep
    }
  }

  /** Get the interpolation alpha for rendering. */
  get renderAlpha(): number {
    return this._accumulator / this.config.fixedTimestep
  }

  /** Get all body positions X coordinates. */
  getPositionsX(): Float32Array {
    return this._engine.get_positions_x()
  }

  /** Get all body positions Y coordinates. */
  getPositionsY(): Float32Array {
    return this._engine.get_positions_y()
  }

  /** Get all body angles. */
  getAngles(): Float32Array {
    return this._engine.get_angles()
  }

  /** Get the number of bodies. */
  get bodyCount(): number {
    return this._engine.body_count()
  }

  /** Get the number of colliders. */
  get colliderCount(): number {
    return this._engine.collider_count()
  }

  /** Set collision callbacks (placeholder for future contact event support). */
  setCollisionCallbacks(_callbacks: CollisionCallbacks | null): void {
    // Contact events will be implemented when the Rust contact tracker
    // exports begin/stay/end contact data
  }

  /** Access the underlying physics engine (for advanced use). */
  get engine(): PhysicsEngine {
    return this._engine
  }

  // ========== Diagnostics ==========

  /** Number of active contact pairs this frame. */
  get contactPairCount(): number {
    return this._engine.contact_pair_count()
  }

  /** Total contact points across all pairs this frame. */
  get contactPointCount(): number {
    return this._engine.contact_point_count()
  }

  /** Maximum penetration depth across all contacts this frame. */
  get maxPenetration(): number {
    return this._engine.max_penetration()
  }

  /** Contact normal of the first active pair (or [0,0] if none). */
  get firstContactNormal(): [number, number] {
    const n = this._engine.first_contact_normal()
    return [n[0], n[1]]
  }

  /** Full debug dump string for the current frame state. */
  get debugDump(): string {
    return this._engine.debug_dump()
  }
}
