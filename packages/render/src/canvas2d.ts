import type { MatchaBuffers } from '@matcha2d/types'
import type { IRenderer } from './renderer.js'

/**
 * Canvas2D debug renderer.
 * TODO: Dev B implements — draws circles/boxes at body positions for debugging.
 */
export class Canvas2DRenderer implements IRenderer {
  constructor(private readonly _ctx: CanvasRenderingContext2D) {}

  begin(): void {
    this._ctx.clearRect(0, 0, this._ctx.canvas.width, this._ctx.canvas.height)
  }

  drawBodies(_buffers: MatchaBuffers, _count: number, _alpha: number): void {
    // stub
  }

  end(): void {
    // stub — Canvas2D auto-flushes
  }

  destroy(): void {
    // stub
  }
}
