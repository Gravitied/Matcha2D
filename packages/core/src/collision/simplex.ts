/**
 * GJK Simplex management.
 * Tracks up to 3 vertices in the Minkowski difference space and computes
 * the closest point on the simplex to the origin.
 *
 * Adapted from Sopiro/Physics simplex.ts for SoA buffers.
 */

export interface ClosestResult {
  resultX: number
  resultY: number
  contributors: number[]
}

export class Simplex {
  public verticesX: number[] = []
  public verticesY: number[] = []

  get count(): number {
    return this.verticesX.length
  }

  clear(): void {
    this.verticesX = []
    this.verticesY = []
  }

  addVertex(x: number, y: number): void {
    if (this.count >= 3) throw new Error('Simplex can have at most 3 vertices')
    this.verticesX.push(x)
    this.verticesY.push(y)
  }

  containsVertex(x: number, y: number, epsilon = 1e-10): boolean {
    for (let i = 0; i < this.count; i++) {
      const dx = this.verticesX[i] - x
      const dy = this.verticesY[i] - y
      if (dx * dx + dy * dy < epsilon) return true
    }
    return false
  }

  shrink(indices: number[]): void {
    const newX: number[] = []
    const newY: number[] = []
    for (let i = 0; i < indices.length; i++) {
      newX.push(this.verticesX[indices[i]])
      newY.push(this.verticesY[indices[i]])
    }
    this.verticesX = newX
    this.verticesY = newY
  }

  /**
   * Returns the closest point on this simplex to the origin (qx=0, qy=0).
   * Also returns which vertices contributed to the closest point.
   */
  getClosest(qx: number, qy: number): ClosestResult {
    switch (this.count) {
      case 1:
        return { resultX: this.verticesX[0], resultY: this.verticesY[0], contributors: [0] }

      case 2: {
        const ax = this.verticesX[0]
        const ay = this.verticesY[0]
        const bx = this.verticesX[1]
        const by = this.verticesY[1]
        const w = getUV(ax, ay, bx, by, qx, qy)

        if (w.v <= 0) {
          return { resultX: ax, resultY: ay, contributors: [0] }
        } else if (w.v >= 1) {
          return { resultX: bx, resultY: by, contributors: [1] }
        } else {
          const rx = ax + w.v * (bx - ax)
          const ry = ay + w.v * (by - ay)
          return { resultX: rx, resultY: ry, contributors: [0, 1] }
        }
      }

      case 3: {
        const ax = this.verticesX[0]
        const ay = this.verticesY[0]
        const bx = this.verticesX[1]
        const by = this.verticesY[1]
        const cx = this.verticesX[2]
        const cy = this.verticesY[2]

        const wab = getUV(ax, ay, bx, by, qx, qy)
        const wbc = getUV(bx, by, cx, cy, qx, qy)
        const wca = getUV(cx, cy, ax, ay, qx, qy)

        if (wca.u <= 0 && wab.v <= 0) {
          return { resultX: ax, resultY: ay, contributors: [0] }
        } else if (wab.u <= 0 && wbc.v <= 0) {
          return { resultX: bx, resultY: by, contributors: [1] }
        } else if (wbc.u <= 0 && wca.v <= 0) {
          return { resultX: cx, resultY: cy, contributors: [2] }
        }

        const area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

        const u = (bx - qx) * (cy - qy) - (cx - qx) * (by - qy)
        const v = (cx - qx) * (ay - qy) - (ax - qx) * (cy - qy)
        const w = (ax - qx) * (by - qy) - (bx - qx) * (ay - qy)

        if (wab.u > 0 && wab.v > 0 && w * area <= 0) {
          const rx = ax + wab.u * (bx - ax)
          const ry = ay + wab.v * (by - ay)
          return {
            resultX: rx, resultY: ry,
            contributors: area !== 0 ? [0, 1] : [0, 1, 2],
          }
        } else if (wbc.u > 0 && wbc.v > 0 && u * area <= 0) {
          const rx = bx + wbc.u * (cx - bx)
          const ry = by + wbc.v * (cy - by)
          return {
            resultX: rx, resultY: ry,
            contributors: area !== 0 ? [1, 2] : [0, 1, 2],
          }
        } else if (wca.u > 0 && wca.v > 0 && v * area <= 0) {
          const rx = cx + wca.u * (ax - cx)
          const ry = cy + wca.v * (ay - cy)
          return {
            resultX: rx, resultY: ry,
            contributors: area !== 0 ? [2, 0] : [0, 1, 2],
          }
        } else {
          return { resultX: qx, resultY: qy, contributors: [] }
        }
      }

      default:
        throw new Error('Simplex contains more than 3 vertices')
    }
  }
}

/**
 * Barycentric coordinates: returns u, v such that
 * the closest point on segment AB to Q is A + u*(B-A) + v*(B-A)
 * Actually returns u for A, v for B where u+v=1.
 */
function getUV(
  ax: number, ay: number,
  bx: number, by: number,
  qx: number, qy: number,
): { u: number; v: number } {
  const abx = bx - ax
  const aby = by - ay
  const aqx = qx - ax
  const aqy = qy - ay

  const abLenSq = abx * abx + aby * aby
  if (abLenSq < 1e-20) {
    return { u: 1, v: 0 }
  }

  const v = (aqx * abx + aqy * aby) / abLenSq
  const u = 1 - v
  return { u, v }
}
