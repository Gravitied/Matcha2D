import type { ContactManifold, ContactPoint, MatchaBuffers } from '@matcha2d/types'
import { BodyFlags, ShapeType, MAX_VERTICES_PER_SHAPE } from '@matcha2d/types'
import { mat2FromAngle, mat2TransposeMulVec } from '../math/mat2.js'
import { dot, length } from '../math/vec2.js'
import { getShapeHandler } from './shapes.js'
import { Simplex } from './simplex.js'
import { Polytope } from './polytope.js'

const GJK_MAX_ITERATIONS = 30
const GJK_TOLERANCE = 1e-10
const EPA_MAX_ITERATIONS = 30
const EPA_TOLERANCE = 1e-4
const TANGENT_MIN_LENGTH = 0.01
const CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH

interface SupportResult {
  x: number
  y: number
  index: number
}

function getShapeVertices(
  idx: number,
  buffers: MatchaBuffers,
  outX: number[],
  outY: number[],
): number {
  const type = buffers.shapeType[idx]

  if (type === ShapeType.Box) {
    const hx = buffers.halfExtentX[idx]
    const hy = buffers.halfExtentY[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const bx = buffers.positionX[idx]
    const by = buffers.positionY[idx]
    const lx = [-hx, hx, hx, -hx]
    const ly = [-hy, -hy, hy, hy]
    for (let i = 0; i < 4; i++) {
      outX[i] = c * lx[i] + negS * ly[i] + bx
      outY[i] = s * lx[i] + c * ly[i] + by
    }
    return 4
  }

  if (type === ShapeType.Circle) {
    return 0
  }

  if (type === ShapeType.Polygon) {
    const count = buffers.shapeVertexCount[idx]
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx])
    const bx = buffers.positionX[idx]
    const by = buffers.positionY[idx]
    const base = idx * MAX_VERTICES_PER_SHAPE
    for (let i = 0; i < count; i++) {
      const lvx = buffers.shapeVerticesX[base + i]
      const lvy = buffers.shapeVerticesY[base + i]
      outX[i] = c * lvx + negS * lvy + bx
      outY[i] = s * lvx + c * lvy + by
    }
    return count
  }

  const handler = getShapeHandler(type)
  if (handler) return handler.getWorldVertices(idx, buffers, outX, outY)
  return 0
}

function supportCircle(
  cx: number, cy: number, radius: number,
  dx: number, dy: number,
): SupportResult {
  const len = length(dx, dy)
  if (len < 1e-10) return { x: cx + radius, y: cy, index: -1 }
  return { x: cx + (dx / len) * radius, y: cy + (dy / len) * radius, index: -1 }
}

function supportPolygon(
  vertsX: number[], vertsY: number[], count: number,
  dx: number, dy: number,
): SupportResult {
  let maxDot = -Infinity
  let bestX = 0
  let bestY = 0
  let bestIdx = 0
  for (let i = 0; i < count; i++) {
    const d = dot(vertsX[i], vertsY[i], dx, dy)
    if (d > maxDot) {
      maxDot = d
      bestX = vertsX[i]
      bestY = vertsY[i]
      bestIdx = i
    }
  }
  return { x: bestX, y: bestY, index: bestIdx }
}

function minkowskiSupport(
  vertsAX: number[], vertsAY: number[], countA: number, typeA: number,
  vertsBX: number[], vertsBY: number[], countB: number, typeB: number,
  aIdx: number, bIdx: number,
  buffers: MatchaBuffers,
  dx: number, dy: number,
): [number, number] {
  let saX: number, saY: number
  if (typeA === ShapeType.Circle) {
    const sr = supportCircle(
      buffers.positionX[aIdx], buffers.positionY[aIdx],
      buffers.shapeRadius[aIdx], dx, dy,
    )
    saX = sr.x
    saY = sr.y
  } else {
    const handlerA = typeA !== ShapeType.Box && typeA !== ShapeType.Polygon
      ? getShapeHandler(typeA) : undefined
    if (handlerA) {
      [saX, saY] = handlerA.getSupportPoint(aIdx, buffers, dx, dy)
    } else {
      const sr = supportPolygon(vertsAX, vertsAY, countA, dx, dy)
      saX = sr.x
      saY = sr.y
    }
  }

  let sbX: number, sbY: number
  if (typeB === ShapeType.Circle) {
    const sr = supportCircle(
      buffers.positionX[bIdx], buffers.positionY[bIdx],
      buffers.shapeRadius[bIdx], -dx, -dy,
    )
    sbX = sr.x
    sbY = sr.y
  } else {
    const handlerB = typeB !== ShapeType.Box && typeB !== ShapeType.Polygon
      ? getShapeHandler(typeB) : undefined
    if (handlerB) {
      [sbX, sbY] = handlerB.getSupportPoint(bIdx, buffers, -dx, -dy)
    } else {
      const sr = supportPolygon(vertsBX, vertsBY, countB, -dx, -dy)
      sbX = sr.x
      sbY = sr.y
    }
  }

  return [saX - sbX, saY - sbY]
}

interface GJKResult {
  collided: boolean
  simplex: Simplex
}

function gjk(
  vertsAX: number[], vertsAY: number[], countA: number, typeA: number,
  vertsBX: number[], vertsBY: number[], countB: number, typeB: number,
  aIdx: number, bIdx: number,
  buffers: MatchaBuffers,
): GJKResult {
  const simplex = new Simplex()

  const dx = buffers.positionX[aIdx] - buffers.positionX[bIdx]
  const dy = buffers.positionY[aIdx] - buffers.positionY[bIdx]
  let dirX: number, dirY: number
  if (dx * dx + dy * dy < 1e-10) {
    dirX = 1
    dirY = 0
  } else {
    dirX = dx
    dirY = dy
  }

  const [sx, sy] = minkowskiSupport(
    vertsAX, vertsAY, countA, typeA,
    vertsBX, vertsBY, countB, typeB,
    aIdx, bIdx, buffers,
    dirX, dirY,
  )
  simplex.addVertex(sx, sy)

  dirX = -sx
  dirY = -sy

  for (let k = 0; k < GJK_MAX_ITERATIONS; k++) {
    const closest = simplex.getClosest(0, 0)
    const distSq = closest.resultX * closest.resultX + closest.resultY * closest.resultY

    if (distSq < GJK_TOLERANCE) {
      return { collided: true, simplex }
    }

    if (simplex.count !== 1) {
      simplex.shrink(closest.contributors)
    }

    dirX = -closest.resultX
    dirY = -closest.resultY

    const [nx, ny] = minkowskiSupport(
      vertsAX, vertsAY, countA, typeA,
      vertsBX, vertsBY, countB, typeB,
      aIdx, bIdx, buffers,
      dirX, dirY,
    )

    const dirLen = length(dirX, dirY)
    if (dirLen < 1e-10) {
      return { collided: true, simplex }
    }

    const closestDist = Math.sqrt(distSq)

    const supportProj = (nx * dirX + ny * dirY) / dirLen
    const closestProj = closestDist

    if (closestProj - supportProj < GJK_TOLERANCE) {
      return { collided: false, simplex }
    }

    if (simplex.containsVertex(nx, ny)) {
      return { collided: false, simplex }
    }

    simplex.addVertex(nx, ny)
  }

  return { collided: true, simplex }
}

interface EPAResult {
  penetrationDepth: number
  contactNormalX: number
  contactNormalY: number
}

function epa(
  simplex: Simplex,
  vertsAX: number[], vertsAY: number[], countA: number, typeA: number,
  vertsBX: number[], vertsBY: number[], countB: number, typeB: number,
  aIdx: number, bIdx: number,
  buffers: MatchaBuffers,
): EPAResult | null {
  let poly: Polytope
  try {
    poly = new Polytope(simplex)
  } catch {
    return null
  }

  let closestEdge = poly.getClosestEdge()

  for (let i = 0; i < EPA_MAX_ITERATIONS; i++) {
    const [sx, sy] = minkowskiSupport(
      vertsAX, vertsAY, countA, typeA,
      vertsBX, vertsBY, countB, typeB,
      aIdx, bIdx, buffers,
      closestEdge.normalX, closestEdge.normalY,
    )

    const newDistance = dot(sx, sy, closestEdge.normalX, closestEdge.normalY)

    if (Math.abs(closestEdge.distance - newDistance) > EPA_TOLERANCE) {
      // Check if support point is already in polytope (prevents degenerate expansion)
      let alreadyPresent = false
      for (let v = 0; v < poly.count; v++) {
        const dvx = sx - poly.verticesX[v]
        const dvy = sy - poly.verticesY[v]
        if (dvx * dvx + dvy * dvy < 1e-10) {
          alreadyPresent = true
          break
        }
      }
      if (alreadyPresent) break

      const insertIdx = closestEdge.index + 1
      poly.verticesX.splice(insertIdx, 0, sx)
      poly.verticesY.splice(insertIdx, 0, sy)
      closestEdge = poly.getClosestEdge()
    } else {
      break
    }
  }

  return {
    penetrationDepth: closestEdge.distance,
    contactNormalX: closestEdge.normalX,
    contactNormalY: closestEdge.normalY,
  }
}

interface ContactPointData {
  x: number
  y: number
  id: number
}

function clipEdge(
  edge: { p1x: number; p1y: number; p2x: number; p2y: number; id1: number; id2: number },
  px: number, py: number,
  dirx: number, diry: number,
  remove: boolean,
): void {
  const d1 = dot(edge.p1x - px, edge.p1y - py, dirx, diry)
  const d2 = dot(edge.p2x - px, edge.p2y - py, dirx, diry)

  if (d1 >= 0 && d2 >= 0) return

  const per = Math.abs(d1) + Math.abs(d2)
  if (per < 1e-10) return

  if (d1 < 0) {
    if (remove) {
      edge.p1x = edge.p2x
      edge.p1y = edge.p2y
      edge.id1 = edge.id2
    } else {
      const t = -d1 / per
      edge.p1x = edge.p1x + (edge.p2x - edge.p1x) * t
      edge.p1y = edge.p1y + (edge.p2y - edge.p1y) * t
    }
  } else if (d2 < 0) {
    if (remove) {
      edge.p2x = edge.p1x
      edge.p2y = edge.p1y
      edge.id2 = edge.id1
    } else {
      const t = -d2 / per
      edge.p2x = edge.p2x + (edge.p1x - edge.p2x) * t
      edge.p2y = edge.p2y + (edge.p1y - edge.p2y) * t
    }
  }
}

function findFarthestEdge(
  bodyIdx: number, buffers: MatchaBuffers,
  vertsX: number[], vertsY: number[], count: number, type: number,
  dirx: number, diry: number,
): { p1x: number; p1y: number; p2x: number; p2y: number; id1: number; id2: number } {
  if (type === ShapeType.Circle) {
    const bx = buffers.positionX[bodyIdx]
    const by = buffers.positionY[bodyIdx]
    const radius = buffers.shapeRadius[bodyIdx]
    const len = length(dirx, diry)
    const nx = len < 1e-10 ? 1 : dirx / len
    const ny = len < 1e-10 ? 0 : diry / len
    const cx = bx + nx * radius
    const cy = by + ny * radius
    const tangentX = -diry * TANGENT_MIN_LENGTH
    const tangentY = dirx * TANGENT_MIN_LENGTH
    return { p1x: cx, p1y: cy, p2x: cx + tangentX, p2y: cy + tangentY, id1: -1, id2: -1 }
  }

  const bx = buffers.positionX[bodyIdx]
  const by = buffers.positionY[bodyIdx]

  const sr = supportPolygon(vertsX, vertsY, count, dirx, diry)
  const currX = sr.x
  const currY = sr.y
  const idx = sr.index

  const prevIdx = (idx - 1 + count) % count
  const nextIdx = (idx + 1) % count

  const e1x = currX - vertsX[prevIdx]
  const e1y = currY - vertsY[prevIdx]
  const e1Len = length(e1x, e1y)
  const e1dot = e1Len < 1e-10 ? 0 : (e1x * dirx + e1y * diry) / e1Len

  const e2x = currX - vertsX[nextIdx]
  const e2y = currY - vertsY[nextIdx]
  const e2Len = length(e2x, e2y)
  const e2dot = e2Len < 1e-10 ? 0 : (e2x * dirx + e2y * diry) / e2Len

  const usePrev = Math.abs(e1dot) <= Math.abs(e2dot)

  if (usePrev) {
    return { p1x: vertsX[prevIdx], p1y: vertsY[prevIdx], p2x: currX, p2y: currY, id1: prevIdx, id2: idx }
  } else {
    return { p1x: currX, p1y: currY, p2x: vertsX[nextIdx], p2y: vertsY[nextIdx], id1: idx, id2: nextIdx }
  }
}

function findContactPoints(
  nx: number, ny: number,
  aIdx: number, buffers: MatchaBuffers,
  vertsAX: number[], vertsAY: number[], countA: number, typeA: number,
  bIdx: number,
  vertsBX: number[], vertsBY: number[], countB: number, typeB: number,
): ContactPointData[] {
  if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
    const cax = buffers.positionX[aIdx]
    const cay = buffers.positionY[aIdx]
    const cbx = buffers.positionX[bIdx]
    const cby = buffers.positionY[bIdx]
    const ra = buffers.shapeRadius[aIdx]
    return [{ x: cax + nx * ra, y: cay + ny * ra, id: -1 }]
  }

  const edgeA = findFarthestEdge(aIdx, buffers, vertsAX, vertsAY, countA, typeA, nx, ny)
  const edgeB = findFarthestEdge(bIdx, buffers, vertsBX, vertsBY, countB, typeB, -nx, -ny)

  let ref = edgeA
  let inc = edgeB
  let flip = false

  const aEdgeLen = length(edgeA.p2x - edgeA.p1x, edgeA.p2y - edgeA.p1y)
  const bEdgeLen = length(edgeB.p2x - edgeB.p1x, edgeB.p2y - edgeB.p1y)
  if (aEdgeLen < 1e-10 && bEdgeLen < 1e-10) {
    return [{ x: edgeA.p1x, y: edgeA.p1y, id: edgeA.id1 }]
  }

  const aPerp = aEdgeLen < 1e-10 ? 1 : Math.abs(dot(edgeA.p2x - edgeA.p1x, edgeA.p2y - edgeA.p1y, nx, ny)) / aEdgeLen
  const bPerp = bEdgeLen < 1e-10 ? 1 : Math.abs(dot(edgeB.p2x - edgeB.p1x, edgeB.p2y - edgeB.p1y, nx, ny)) / bEdgeLen

  if (aPerp >= bPerp) {
    ref = edgeB
    inc = edgeA
    flip = true
  }

  const refDirX = ref.p2x - ref.p1x
  const refDirY = ref.p2y - ref.p1y
  const refDirLen = length(refDirX, refDirY)
  if (refDirLen < 1e-10) {
    return [{ x: ref.p1x, y: ref.p1y, id: ref.id1 }]
  }
  const refDirNX = refDirX / refDirLen
  const refDirNY = refDirY / refDirLen

  clipEdge(inc, ref.p1x, ref.p1y, -refDirNX, -refDirNY, false)
  clipEdge(inc, ref.p2x, ref.p2y, refDirNX, refDirNY, false)

  const clipNormalX = flip ? nx : -nx
  const clipNormalY = flip ? ny : -ny
  clipEdge(inc, ref.p1x, ref.p1y, clipNormalX, clipNormalY, true)

  const incLen = length(inc.p2x - inc.p1x, inc.p2y - inc.p1y)

  if (incLen <= CONTACT_MERGE_THRESHOLD) {
    return [{ x: inc.p1x, y: inc.p1y, id: inc.id1 }]
  }

  if (incLen < 1e-10) {
    return [{ x: inc.p1x, y: inc.p1y, id: inc.id1 }]
  }

  return [
    { x: inc.p1x, y: inc.p1y, id: inc.id1 },
    { x: inc.p2x, y: inc.p2y, id: inc.id2 },
  ]
}

function circleCircleContact(
  aIdx: number, bIdx: number, buffers: MatchaBuffers,
): ContactManifold | null {
  const dx = buffers.positionX[bIdx] - buffers.positionX[aIdx]
  const dy = buffers.positionY[bIdx] - buffers.positionY[aIdx]
  const distSq = dx * dx + dy * dy
  const radiusSum = buffers.shapeRadius[aIdx] + buffers.shapeRadius[bIdx]

  if (distSq >= radiusSum * radiusSum) return null

  let contactNormalX: number
  let contactNormalY: number
  let penetration: number
  let contactX: number
  let contactY: number

  if (distSq < 1e-10) {
    contactNormalX = 1
    contactNormalY = 0
    penetration = radiusSum
    contactX = buffers.positionX[aIdx]
    contactY = buffers.positionY[aIdx]
  } else {
    const dist = Math.sqrt(distSq)
    contactNormalX = dx / dist
    contactNormalY = dy / dist
    penetration = radiusSum - dist
    contactX = buffers.positionX[aIdx] + contactNormalX * buffers.shapeRadius[aIdx]
    contactY = buffers.positionY[aIdx] + contactNormalY * buffers.shapeRadius[aIdx]
  }

  const [cA, negSA, sA] = mat2FromAngle(buffers.angle[aIdx])
  const [cB, negSB, sB] = mat2FromAngle(buffers.angle[bIdx])

  const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, cA,
    contactX - buffers.positionX[aIdx], contactY - buffers.positionY[aIdx])
  const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, cB,
    contactX - buffers.positionX[bIdx], contactY - buffers.positionY[bIdx])

  return {
    bodyA: aIdx as any,
    bodyB: bIdx as any,
    normal: { x: contactNormalX, y: contactNormalY },
    contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration, idA: -1, idB: -1 }],
  }
}

function buildContactManifold(
  aIdx: number, bIdx: number, buffers: MatchaBuffers,
  vertsAX: number[], vertsAY: number[], countA: number, typeA: number,
  vertsBX: number[], vertsBY: number[], countB: number, typeB: number,
  depth: number, nx: number, ny: number,
): ContactManifold | null {
  const contactPoints = findContactPoints(
    nx, ny,
    aIdx, buffers,
    vertsAX, vertsAY, countA, typeA,
    bIdx,
    vertsBX, vertsBY, countB, typeB,
  )

  if (contactPoints.length === 0) {
    const bodyAX = buffers.positionX[aIdx]
    const bodyAY = buffers.positionY[aIdx]
    const bodyBX = buffers.positionX[bIdx]
    const bodyBY = buffers.positionY[bIdx]
    const contactX = bodyAX + nx * depth * 0.5
    const contactY = bodyAY + ny * depth * 0.5

    const [cA, negSA, sA] = mat2FromAngle(buffers.angle[aIdx])
    const [cB, negSB, sB] = mat2FromAngle(buffers.angle[bIdx])

    const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, cA,
      contactX - bodyAX, contactY - bodyAY)
    const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, cB,
      contactX - bodyBX, contactY - bodyBY)

    return {
      bodyA: aIdx as any,
      bodyB: bIdx as any,
      normal: { x: nx, y: ny },
      contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration: depth, idA: -1, idB: -1 }],
    }
  }

  const [cA, negSA, sA] = mat2FromAngle(buffers.angle[aIdx])
  const [cB, negSB, sB] = mat2FromAngle(buffers.angle[bIdx])

  const contacts: ContactPoint[] = []
  for (let i = 0; i < contactPoints.length; i++) {
    const cp = contactPoints[i]
    const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, cA,
      cp.x - buffers.positionX[aIdx], cp.y - buffers.positionY[aIdx])
    const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, cB,
      cp.x - buffers.positionX[bIdx], cp.y - buffers.positionY[bIdx])

    contacts.push({
      localA: { x: lAx, y: lAy },
      localB: { x: lBx, y: lBy },
      penetration: depth,
      idA: cp.id >= 0 ? cp.id : -1,
      idB: cp.id >= 0 ? cp.id : -1,
    })
  }

  return {
    bodyA: aIdx as any,
    bodyB: bIdx as any,
    normal: { x: nx, y: ny },
    contacts,
  }
}

export function gjkNarrowphase(
  buffers: MatchaBuffers,
  pairs: Array<{ a: number; b: number }>,
): ContactManifold[] {
  const manifolds: ContactManifold[] = []
  const vertsAX = new Array<number>(MAX_VERTICES_PER_SHAPE)
  const vertsAY = new Array<number>(MAX_VERTICES_PER_SHAPE)
  const vertsBX = new Array<number>(MAX_VERTICES_PER_SHAPE)
  const vertsBY = new Array<number>(MAX_VERTICES_PER_SHAPE)

  for (let i = 0; i < pairs.length; i++) {
    const a = pairs[i].a
    const b = pairs[i].b

    if ((buffers.flags[a] & BodyFlags.ACTIVE) === 0 ||
        (buffers.flags[b] & BodyFlags.ACTIVE) === 0) continue
    if ((buffers.flags[a] & BodyFlags.SLEEPING) !== 0 &&
        (buffers.flags[b] & BodyFlags.SLEEPING) !== 0) continue

    const typeA = buffers.shapeType[a]
    const typeB = buffers.shapeType[b]

    if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
      const m = circleCircleContact(a, b, buffers)
      if (m) manifolds.push(m)
      continue
    }

    const countA = getShapeVertices(a, buffers, vertsAX, vertsAY)
    const countB = getShapeVertices(b, buffers, vertsBX, vertsBY)

    const gjkResult = gjk(
      vertsAX, vertsAY, countA, typeA,
      vertsBX, vertsBY, countB, typeB,
      a, b, buffers,
    )

    if (!gjkResult.collided) continue

    let simplex = gjkResult.simplex

    if (simplex.count === 1) {
      const vx = simplex.verticesX[0]
      const vy = simplex.verticesY[0]
      const dirToOriginX = -vx
      const dirToOriginY = -vy
      const dirLen = length(dirToOriginX, dirToOriginY)
      if (dirLen < 1e-10) {
        const perpX = 0
        const perpY = 1
        const [sx, sy] = minkowskiSupport(
          vertsAX, vertsAY, countA, typeA,
          vertsBX, vertsBY, countB, typeB,
          a, b, buffers,
          perpX, perpY,
        )
        if (Math.abs(sx - vx) > 1e-10 || Math.abs(sy - vy) > 1e-10) {
          simplex.addVertex(sx, sy)
        }
      } else {
        const ndx = dirToOriginX / dirLen
        const ndy = dirToOriginY / dirLen
        const [sx, sy] = minkowskiSupport(
          vertsAX, vertsAY, countA, typeA,
          vertsBX, vertsBY, countB, typeB,
          a, b, buffers,
          ndx, ndy,
        )
        if (Math.abs(sx - vx) > 1e-10 || Math.abs(sy - vy) > 1e-10) {
          simplex.addVertex(sx, sy)
        }
      }
    }

    if (simplex.count === 2) {
      const ex = simplex.verticesX[1] - simplex.verticesX[0]
      const ey = simplex.verticesY[1] - simplex.verticesY[0]
      const eLen = length(ex, ey)
      const perpX = eLen < 1e-10 ? 0 : -ey / eLen
      const perpY = eLen < 1e-10 ? 1 : ex / eLen
      const midX = (simplex.verticesX[0] + simplex.verticesX[1]) * 0.5
      const midY = (simplex.verticesY[0] + simplex.verticesY[1]) * 0.5
      const toOriginX = -midX
      const toOriginY = -midY
      const dotProd = toOriginX * perpX + toOriginY * perpY
      const searchX = dotProd >= 0 ? perpX : -perpX
      const searchY = dotProd >= 0 ? perpY : -perpY
      const [sx, sy] = minkowskiSupport(
        vertsAX, vertsAY, countA, typeA,
        vertsBX, vertsBY, countB, typeB,
        a, b, buffers,
        searchX, searchY,
      )
      if (!simplex.containsVertex(sx, sy)) {
        simplex.addVertex(sx, sy)
      }
    }

    if (simplex.count < 3) continue

    let abx = simplex.verticesX[1] - simplex.verticesX[0]
    let aby = simplex.verticesY[1] - simplex.verticesY[0]
    let acx = simplex.verticesX[2] - simplex.verticesX[0]
    let acy = simplex.verticesY[2] - simplex.verticesY[0]
    let crossMag = abx * acy - aby * acx

    if (Math.abs(crossMag) < 1e-10) {
      const perpX = -aby
      const perpY = abx
      const perpLen = length(perpX, perpY)
      const searchX = perpLen < 1e-10 ? 1 : perpX / perpLen
      const searchY = perpLen < 1e-10 ? 0 : perpY / perpLen
      const [sx, sy] = minkowskiSupport(
        vertsAX, vertsAY, countA, typeA,
        vertsBX, vertsBY, countB, typeB,
        a, b, buffers,
        searchX, searchY,
      )
      if (!simplex.containsVertex(sx, sy)) {
        simplex.verticesX[2] = sx
        simplex.verticesY[2] = sy
        acx = sx - simplex.verticesX[0]
        acy = sy - simplex.verticesY[0]
        crossMag = abx * acy - aby * acx
      }
      if (Math.abs(crossMag) < 1e-10) continue
    }

    const epaResult = epa(
      simplex,
      vertsAX, vertsAY, countA, typeA,
      vertsBX, vertsBY, countB, typeB,
      a, b, buffers,
    )

    if (epaResult && epaResult.penetrationDepth > 1e-6) {
      const nx = -epaResult.contactNormalX
      const ny = -epaResult.contactNormalY
      const manifold = buildContactManifold(
        a, b, buffers,
        vertsAX, vertsAY, countA, typeA,
        vertsBX, vertsBY, countB, typeB,
        epaResult.penetrationDepth,
        nx, ny,
      )
      if (manifold) { manifolds.push(manifold); continue }
    }

    // EPA failed or returned zero depth — create a simple contact
    const dx = buffers.positionX[b] - buffers.positionX[a]
    const dy = buffers.positionY[b] - buffers.positionY[a]
    const len = length(dx, dy)
    const cnx = len < 1e-10 ? 1 : dx / len
    const cny = len < 1e-10 ? 0 : dy / len

    // Estimate penetration from shape overlap
    let penetration = 0.01
    if (typeA === ShapeType.Box && typeB === ShapeType.Box) {
      const penX = (buffers.halfExtentX[a] + buffers.halfExtentX[b]) - Math.abs(dx)
      const penY = (buffers.halfExtentY[a] + buffers.halfExtentY[b]) - Math.abs(dy)
      penetration = Math.min(Math.max(penX, 0.001), Math.max(penY, 0.001))
    } else if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
      penetration = buffers.shapeRadius[a] + buffers.shapeRadius[b] - len
    } else {
      penetration = Math.max(0.01, len * 0.1)
    }

    const contactX = buffers.positionX[a] + cnx * penetration * 0.5
    const contactY = buffers.positionY[a] + cny * penetration * 0.5

    const [cA, negSA, sA] = mat2FromAngle(buffers.angle[a])
    const [cB, negSB, sB] = mat2FromAngle(buffers.angle[b])

    const [lAx, lAy] = mat2TransposeMulVec(cA, negSA, sA, cA,
      contactX - buffers.positionX[a], contactY - buffers.positionY[a])
    const [lBx, lBy] = mat2TransposeMulVec(cB, negSB, sB, cB,
      contactX - buffers.positionX[b], contactY - buffers.positionY[b])

    manifolds.push({
      bodyA: a as any,
      bodyB: b as any,
      normal: { x: cnx, y: cny },
      contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration, idA: -1, idB: -1 }],
    })
  }

  return manifolds
}
