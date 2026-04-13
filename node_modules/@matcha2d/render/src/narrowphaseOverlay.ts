import type { NarrowphaseColliderOutline } from '@matcha2d/types'

export interface DrawNarrowphaseCanvasOptions {
  /** Pixels per physics meter (same as demo `PIXELS_PER_METER`; use `1` if physics units are already pixels) */
  pixelsPerMeter: number
  /** Canvas height in pixels — used when `physicsYUp` is true */
  canvasHeight: number
  /**
   * If true (default), world space is Y-up (Matcha demos in meters) → map with `H - y * ppm`.
   * If false, world Y matches canvas Y-down (e.g. `collision.html` pixel space) → `y * ppm` only.
   */
  physicsYUp?: boolean
  strokeSolid?: string
  strokeSensor?: string
  /** Stroke width in **canvas pixels** */
  lineWidth?: number
}

/**
 * Draw narrowphase outlines on a **raw** canvas context (Y-down, pixels).
 * With default `physicsYUp: true`, matches `demo/compare.html` (`toCanvasX` / `toCanvasY`).
 */
export function drawNarrowphaseColliderOutlinesCanvas(
  ctx: CanvasRenderingContext2D,
  outlines: NarrowphaseColliderOutline[],
  opts: DrawNarrowphaseCanvasOptions,
): void {
  const ppm = opts.pixelsPerMeter
  const H = opts.canvasHeight
  const physicsYUp = opts.physicsYUp !== false
  const strokeSolid = opts.strokeSolid ?? 'rgba(0, 255, 140, 0.95)'
  const strokeSensor = opts.strokeSensor ?? 'rgba(255, 210, 60, 0.9)'
  const lw = opts.lineWidth ?? 1.75
  const toCx = (x: number) => x * ppm
  const toCy = (y: number) => (physicsYUp ? H - y * ppm : y * ppm)

  for (const o of outlines) {
    if (o.vertices.length === 0) continue
    ctx.strokeStyle = o.sensor ? strokeSensor : strokeSolid
    ctx.lineWidth = lw
    ctx.beginPath()
    for (let v = 0; v < o.vertices.length; v++) {
      const wx = toCx(o.vertices[v].x)
      const wy = toCy(o.vertices[v].y)
      if (v === 0) ctx.moveTo(wx, wy)
      else ctx.lineTo(wx, wy)
    }
    ctx.closePath()
    ctx.stroke()
  }
}
