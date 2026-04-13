import type { MatchaBuffers } from '@matcha2d/types'

/**
 * Abstract renderer interface.
 * Dev B implements Canvas2D and WebGL backends.
 */
export interface IRenderer {
  /** Begin a new frame. */
  begin(): void

  /** Draw all bodies from the buffer data. */
  drawBodies(buffers: MatchaBuffers, count: number, alpha: number): void

  /** End the frame and flush to screen. */
  end(): void

  /** Clean up resources. */
  destroy(): void
}
