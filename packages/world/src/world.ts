import type { MatchaBuffers, WorldConfig, PhysicsBackend, BodyHandle, ShapeType, CollisionCallbacks } from '@matcha2d/types'
import { createBuffers, DEFAULT_WORLD_CONFIG, BodyFlags, BodyType } from '@matcha2d/types'
import { integrate, solveVelocity, solvePosition, collide, DynamicTree, ContactTracker } from '@matcha2d/core'

export interface BodyDef {
  positionX?: number
  positionY?: number
  velocityX?: number
  velocityY?: number
  angle?: number
  angularVel?: number
  type?: BodyType
  shapeType?: ShapeType
  halfExtentX?: number
  halfExtentY?: number
  radius?: number
  mass?: number
  friction?: number
  restitution?: number
}

export class World {
  readonly config: WorldConfig
  readonly buffers: MatchaBuffers
  private _bodyCount = 0
  private _tree: DynamicTree
  private _contactTracker: ContactTracker
  private _accumulator = 0
  private _callbacks: CollisionCallbacks | null = null

  constructor(config: Partial<WorldConfig> = {}) {
    this.config = { ...DEFAULT_WORLD_CONFIG, ...config }
    this.buffers = createBuffers(this.config.maxBodies)
    this._tree = new DynamicTree()
    this._contactTracker = new ContactTracker()
  }

  get bodyCount(): number {
    return this._bodyCount
  }

  /** Create a new body with the given properties. */
  createBody(def: BodyDef = {}): BodyHandle {
    const handle = this._bodyCount as BodyHandle
    const idx = this._bodyCount

    this.buffers.positionX[idx] = def.positionX ?? 0
    this.buffers.positionY[idx] = def.positionY ?? 0
    this.buffers.velocityX[idx] = def.velocityX ?? 0
    this.buffers.velocityY[idx] = def.velocityY ?? 0
    this.buffers.angle[idx] = def.angle ?? 0
    this.buffers.angularVel[idx] = def.angularVel ?? 0
    this.buffers.shapeType[idx] = def.shapeType ?? 0

    if (def.halfExtentX !== undefined) this.buffers.halfExtentX[idx] = def.halfExtentX
    if (def.halfExtentY !== undefined) this.buffers.halfExtentY[idx] = def.halfExtentY
    if (def.radius !== undefined) this.buffers.shapeRadius[idx] = def.radius

    const type = def.type ?? BodyType.Dynamic
    const mass = def.mass ?? (type === BodyType.Static ? 0 : 1)
    const invMass = type === BodyType.Static ? 0 : (mass > 0 ? 1 / mass : 0)

    this.buffers.mass[idx] = mass
    this.buffers.invMass[idx] = invMass

    const hx = def.halfExtentX ?? (def.shapeType === 0 ? 0.5 : 0)
    const hy = def.halfExtentY ?? (def.shapeType === 0 ? 0.5 : 0)
    const radius = def.radius ?? (def.shapeType === 1 ? 0.5 : 0)

    if (type === BodyType.Static) {
      this.buffers.inertia[idx] = 0
      this.buffers.invInertia[idx] = 0
    } else if (def.shapeType === 1) {
      const inertia = 0.5 * mass * radius * radius
      this.buffers.inertia[idx] = inertia
      this.buffers.invInertia[idx] = invMass > 0 ? 1 / inertia : 0
    } else {
      const inertia = (mass * (hx * hx + hy * hy)) / 3
      this.buffers.inertia[idx] = inertia
      this.buffers.invInertia[idx] = invMass > 0 ? 1 / inertia : 0
    }

    let flags = BodyFlags.ACTIVE
    if (type === BodyType.Static) flags |= BodyFlags.STATIC
    this.buffers.flags[idx] = flags

    this._tree.insert(idx, this.buffers)
    this._bodyCount++

    return handle
  }

  /** Destroy a body and remove it from the tree. */
  destroyBody(handle: BodyHandle): void {
    const idx = handle as number
    if (idx >= this._bodyCount) return

    this._tree.remove(idx)
    this.buffers.flags[idx] = 0
  }

  /** Set collision callbacks for begin/stay/end contact events. */
  setCollisionCallbacks(callbacks: CollisionCallbacks | null): void {
    this._callbacks = callbacks
    this._contactTracker.setCallbacks(callbacks)
  }

  /**
   * Step the simulation by the given delta time.
   * Uses a fixed-timestep accumulator with interpolation alpha.
   */
  step(dt: number): void {
    this._accumulator += dt

    while (this._accumulator >= this.config.fixedTimestep) {
      this._physicsStep(this.config.fixedTimestep)
      this._accumulator -= this.config.fixedTimestep
    }
  }

  /** Get the interpolation alpha for rendering. */
  get renderAlpha(): number {
    return this._accumulator / this.config.fixedTimestep
  }

  private _physicsStep(dt: number): void {
    integrate(this.buffers, this._bodyCount, dt, this.config.gravity)

    this._tree.updateAll(this.buffers)

    const manifolds = collide(
      this.buffers,
      this._bodyCount,
      this._tree,
      this.config.broadphaseMethod,
      this.config.narrowphaseMethod,
    )

    solveVelocity(this.buffers, manifolds, this.config)
    solvePosition(this.buffers, manifolds, this.config)

    this._contactTracker.update(manifolds, this.config)
  }
}
