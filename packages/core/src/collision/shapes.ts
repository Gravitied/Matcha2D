import type { MatchaBuffers } from '@matcha2d/types'
import { ShapeType, MAX_VERTICES_PER_SHAPE } from '@matcha2d/types'
import { mat2FromAngle } from '../math/mat2.js'
import { dot, length } from '../math/vec2.js'

/**
 * Interface that any shape type must satisfy to integrate with the collision
 * pipeline.  Register custom shapes with `registerShapeHandler`.
 */
export interface ShapeHandler {
  /**
   * Compute the world-space tight AABB for body `idx`.
   * Returns [minX, minY, maxX, maxY].
   */
  getAABB(idx: number, buffers: MatchaBuffers): [number, number, number, number]

  /**
   * Write world-space vertices into `outX`/`outY` and return the vertex count.
   * For shapes without a polygonal hull (e.g. circles) return 0.
   */
  getWorldVertices(
    idx: number,
    buffers: MatchaBuffers,
    outX: number[],
    outY: number[],
  ): number

  /**
   * Return the GJK support point in direction `(dx, dy)`.
   * The direction does not need to be normalized.
   */
  getSupportPoint(
    idx: number,
    buffers: MatchaBuffers,
    dx: number,
    dy: number,
  ): [number, number]
}

// ── Registry ────────────────────────────────────────────────────────────────

const registry = new Map<number, ShapeHandler>()

/** Register a handler for a custom (or built-in) shape type number. */
export function registerShapeHandler(type: number, handler: ShapeHandler): void {
  registry.set(type, handler)
}

/** Retrieve the handler registered for a shape type, if any. */
export function getShapeHandler(type: number): ShapeHandler | undefined {
  return registry.get(type)
}

// ── Built-in handlers ────────────────────────────────────────────────────────

const circleHandler: ShapeHandler = {
  getAABB(idx, buffers) {
    const r = buffers.shapeRadius[idx]
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    return [px - r, py - r, px + r, py + r]
  },
  getWorldVertices(_idx, _buffers, _outX, _outY) {
    return 0
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    const r = buffers.shapeRadius[idx]
    const len = length(dx, dy)
    if (len < 1e-10) return [px + r, py]
    return [px + (dx / len) * r, py + (dy / len) * r]
  },
}

const boxHandler: ShapeHandler = {
  getAABB(idx, buffers) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const lx = [-hx, hx, hx, -hx]
    const ly = [-hy, -hy, hy, hy]
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity
    for (let i = 0; i < 4; i++) {
      const wx = c * lx[i] + negS * ly[i] + px
      const wy = s * lx[i] + c * ly[i] + py
      if (wx < minX) minX = wx
      if (wx > maxX) maxX = wx
      if (wy < minY) minY = wy
      if (wy > maxY) maxY = wy
    }
    return [minX, minY, maxX, maxY]
  },
  getWorldVertices(idx, buffers, outX, outY) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    const lx = [-hx, hx, hx, -hx]
    const ly = [-hy, -hy, hy, hy]
    for (let i = 0; i < 4; i++) {
      outX[i] = c * lx[i] + negS * ly[i] + px
      outY[i] = s * lx[i] + c * ly[i] + py
    }
    return 4
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    const lx = [-hx, hx, hx, -hx]
    const ly = [-hy, -hy, hy, hy]
    let maxD = -Infinity
    let bestX = px, bestY = py
    for (let i = 0; i < 4; i++) {
      const wx = c * lx[i] + negS * ly[i] + px
      const wy = s * lx[i] + c * ly[i] + py
      const d = dot(wx, wy, dx, dy)
      if (d > maxD) { maxD = d; bestX = wx; bestY = wy }
    }
    return [bestX, bestY]
  },
}

const polygonHandler: ShapeHandler = {
  getAABB(idx, buffers) {
    const count = buffers.shapeVertexCount[idx]
    const base = idx * MAX_VERTICES_PER_SHAPE
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i]
      const ly = buffers.shapeVerticesY[base + i]
      const wx = c * lx + negS * ly + px
      const wy = s * lx + c * ly + py
      if (wx < minX) minX = wx
      if (wx > maxX) maxX = wx
      if (wy < minY) minY = wy
      if (wy > maxY) maxY = wy
    }
    if (count === 0) {
      const hx = buffers.halfExtentX[idx]
      const hy = buffers.halfExtentY[idx]
      return [px - hx, py - hy, px + hx, py + hy]
    }
    return [minX, minY, maxX, maxY]
  },
  getWorldVertices(idx, buffers, outX, outY) {
    const count = buffers.shapeVertexCount[idx]
    const base = idx * MAX_VERTICES_PER_SHAPE
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i]
      const ly = buffers.shapeVerticesY[base + i]
      outX[i] = c * lx + negS * ly + px
      outY[i] = s * lx + c * ly + py
    }
    return count
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const count = buffers.shapeVertexCount[idx]
    const base = idx * MAX_VERTICES_PER_SHAPE
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const px = buffers.positionX[idx]
    const py = buffers.positionY[idx]
    let maxD = -Infinity
    let bestX = px, bestY = py
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i]
      const ly = buffers.shapeVerticesY[base + i]
      const wx = c * lx + negS * ly + px
      const wy = s * lx + c * ly + py
      const d = dot(wx, wy, dx, dy)
      if (d > maxD) { maxD = d; bestX = wx; bestY = wy }
    }
    return [bestX, bestY]
  },
}

registerShapeHandler(ShapeType.Circle, circleHandler)
registerShapeHandler(ShapeType.Box, boxHandler)
registerShapeHandler(ShapeType.Polygon, polygonHandler)
