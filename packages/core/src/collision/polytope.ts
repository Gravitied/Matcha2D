import type { Simplex } from './simplex.js'

/**
 * EPA Polytope for penetration depth computation.
 * Initialized from a GJK simplex (triangle) and expanded iteratively
 * to find the closest edge to the origin in the Minkowski difference.
 *
 * Adapted from Sopiro/Physics polytope.ts for SoA buffers.
 */

export interface ClosestEdgeInfo {
  index: number
  distance: number
  normalX: number
  normalY: number
}

export class Polytope {
  public verticesX: number[]
  public verticesY: number[]

  constructor(simplex: Simplex) {
    if (simplex.count !== 3) throw new Error('Input simplex must be a triangle')

    this.verticesX = [simplex.verticesX[0], simplex.verticesX[1], simplex.verticesX[2]]
    this.verticesY = [simplex.verticesY[0], simplex.verticesY[1], simplex.verticesY[2]]
  }

  get count(): number {
    return this.verticesX.length
  }

  /**
   * Returns the edge closest to the origin.
   * For each edge, computes the outward normal and the signed distance
   * from the origin to the edge's supporting line.
   */
  getClosestEdge(): ClosestEdgeInfo {
    let minIndex = 0
    let minDistance = Infinity
    let minNormalX = 0
    let minNormalY = 0

    for (let i = 0; i < this.count; i++) {
      const j = (i + 1) % this.count

      const vx = this.verticesX[i]
      const vy = this.verticesY[i]
      const wx = this.verticesX[j]
      const wy = this.verticesY[j]

      const ex = wx - vx
      const ey = wy - vy

      // Edge normal (perpendicular to edge), pointing toward origin.
      let nx = ey
      let ny = -ex
      const len = Math.sqrt(nx * nx + ny * ny)
      if (len < 1e-20) continue

      nx /= len
      ny /= len

      // Distance from origin to edge's supporting line.
      let distance = nx * vx + ny * vy

      // Ensure distance is positive (normal points toward origin).
      if (distance < 0) {
        distance = -distance
        nx = -nx
        ny = -ny
      }

      if (distance < minDistance) {
        minDistance = distance
        minNormalX = nx
        minNormalY = ny
        minIndex = i
      }
    }

    return {
      index: minIndex,
      distance: minDistance,
      normalX: minNormalX,
      normalY: minNormalY,
    }
  }
}
