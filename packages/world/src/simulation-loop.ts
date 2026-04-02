import type { WorldConfig } from '@matcha2d/types'
import type { World } from './world.js'

/**
 * Fixed-timestep simulation loop with interpolation.
 *
 * Usage pattern:
 *   loop.update(world, deltaTime)
 *   render(world, loop.renderAlpha)
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

  /**
   * Step the simulation by the given delta time.
   * Accumulates time and runs fixed-timestep physics steps.
   */
  update(world: World, deltaTime: number): void {
    this._accumulator += deltaTime

    while (this._accumulator >= this.config.fixedTimestep) {
      world.step(this.config.fixedTimestep)
      this._accumulator -= this.config.fixedTimestep
    }
  }

  /** Reset the accumulator. */
  reset(): void {
    this._accumulator = 0
  }
}
