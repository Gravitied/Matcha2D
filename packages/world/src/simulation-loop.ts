import type { WorldConfig } from '@matcha2d/types'

/**
 * Fixed-timestep simulation loop with interpolation.
 * TODO: Dev B implements accumulator-based stepping.
 *
 * Usage pattern:
 *   accumulator += deltaTime
 *   while (accumulator >= fixedStep) {
 *     physicsStep(fixedStep)
 *     accumulator -= fixedStep
 *   }
 *   renderAlpha = accumulator / fixedStep
 */
export class SimulationLoop {
  private _accumulator = 0

  constructor(readonly config: WorldConfig) {}

  get accumulator(): number {
    return this._accumulator
  }

  /** Returns the interpolation alpha for rendering. */
  get renderAlpha(): number {
    return this._accumulator / this.config.fixedTimestep
  }
}
