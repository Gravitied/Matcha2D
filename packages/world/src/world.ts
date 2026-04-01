import type { MatchaBuffers, WorldConfig, PhysicsBackend, BodyHandle } from '@matcha2d/types'
import { createBuffers, DEFAULT_WORLD_CONFIG } from '@matcha2d/types'

/**
 * The main World class — owns bodies, runs the simulation loop.
 * TODO: Dev B implements this.
 */
export class World {
  readonly config: WorldConfig
  readonly buffers: MatchaBuffers
  private _bodyCount = 0

  constructor(config: Partial<WorldConfig> = {}) {
    this.config = { ...DEFAULT_WORLD_CONFIG, ...config }
    this.buffers = createBuffers(this.config.maxBodies)
  }

  get bodyCount(): number {
    return this._bodyCount
  }

  /** Create a new dynamic body. Returns its handle. */
  createBody(): BodyHandle {
    const handle = this._bodyCount++ as BodyHandle
    return handle
  }
}
