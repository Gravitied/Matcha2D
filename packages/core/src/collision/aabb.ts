import type { AABB, MatchaBuffers } from '@matcha2d/types'
import { ShapeType, MAX_VERTICES_PER_SHAPE } from '@matcha2d/types'
import { mat2FromAngle } from '../math/mat2.js'

export function aabbOverlap(a: AABB, b: AABB): boolean {
  return a.minX <= b.maxX && a.maxX >= b.minX
      && a.minY <= b.maxY && a.maxY >= b.minY
}

export function aabbMerge(a: AABB, b: AABB): AABB {
  return {
    minX: Math.min(a.minX, b.minX),
    minY: Math.min(a.minY, b.minY),
    maxX: Math.max(a.maxX, b.maxX),
    maxY: Math.max(a.maxY, b.maxY),
  }
}

export function aabbContains(outer: AABB, inner: AABB): boolean {
  return outer.minX <= inner.minX && outer.maxX >= inner.maxX
      && outer.minY <= inner.minY && outer.maxY >= inner.maxY
}

export function aabbArea(a: AABB): number {
  return (a.maxX - a.minX) * (a.maxY - a.minY)
}

export function aabbPerimeter(a: AABB): number {
  return 2 * ((a.maxX - a.minX) + (a.maxY - a.minY))
}

/**
 * Compute the world-space tight AABB for body `idx`.
 * Returns [minX, minY, maxX, maxY].
 * Correctly accounts for rotation — rotated boxes/polygons produce a larger
 * AABB than their rest-pose half-extents suggest.
 */
export function computeBodyAABB(
  idx: number,
  buffers: MatchaBuffers,
): [number, number, number, number] {
  const px = buffers.positionX[idx]
  const py = buffers.positionY[idx]
  const type = buffers.shapeType[idx]

  if (type === ShapeType.Circle) {
    const r = buffers.shapeRadius[idx]
    return [px - r, py - r, px + r, py + r]
  }

  if (type === ShapeType.Box) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    // Rotate all 4 corners and take world min/max
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity
    const lx = [-hx, hx, hx, -hx]
    const ly = [-hy, -hy, hy, hy]
    for (let i = 0; i < 4; i++) {
      const wx = c * lx[i] + negS * ly[i] + px
      const wy = s * lx[i] + c * ly[i] + py
      if (wx < minX) minX = wx
      if (wx > maxX) maxX = wx
      if (wy < minY) minY = wy
      if (wy > maxY) maxY = wy
    }
    return [minX, minY, maxX, maxY]
  }

  // Polygon: rotate all local vertices and take world min/max
  const count = buffers.shapeVertexCount[idx]
  const base = idx * MAX_VERTICES_PER_SHAPE
  const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
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
  // Fallback for zero-vertex polygon (degenerate)
  if (count === 0) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    return [px - hx, py - hy, px + hx, py + hy]
  }
  return [minX, minY, maxX, maxY]
}
