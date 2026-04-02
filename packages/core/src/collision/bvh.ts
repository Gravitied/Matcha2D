import type { BodyHandle, CollisionPair, MatchaBuffers } from '@matcha2d/types'
import { BodyFlags } from '@matcha2d/types'
import { computeBodyAABB } from './aabb.js'

/** Internal node for the AABB BVH tree. */
interface BVHNode {
  minX: number; minY: number; maxX: number; maxY: number
  /** Index of left child in the nodes array, or -1 if this is a leaf. */
  left: number
  /** Index of right child in the nodes array, or -1 if this is a leaf. */
  right: number
  /** Body index if this is a leaf node, -1 if internal. */
  body: number
}

/**
 * Emit all candidate pairs between two subtrees.
 * Prunes when their AABBs do not overlap, skips static-static pairs at leaves.
 */
function queryCross(
  nodes: BVHNode[],
  a: number,
  b: number,
  flags: Uint8Array,
  pairs: CollisionPair[],
): void {
  const na = nodes[a], nb = nodes[b]
  // Early exit: AABBs don't overlap.
  if (na.maxX < nb.minX || nb.maxX < na.minX ||
      na.maxY < nb.minY || nb.maxY < na.minY) return

  if (na.body !== -1 && nb.body !== -1) {
    // Both leaves — emit pair unless both are static.
    const i = na.body, j = nb.body
    if (!((flags[i] & BodyFlags.STATIC) && (flags[j] & BodyFlags.STATIC))) {
      pairs.push({ a: i as BodyHandle, b: j as BodyHandle })
    }
    return
  }

  // Split the larger internal node to balance recursion depth.
  if (na.body !== -1) {
    // na is a leaf; split nb.
    queryCross(nodes, a, nb.left,  flags, pairs)
    queryCross(nodes, a, nb.right, flags, pairs)
  } else if (nb.body !== -1) {
    // nb is a leaf; split na.
    queryCross(nodes, na.left,  b, flags, pairs)
    queryCross(nodes, na.right, b, flags, pairs)
  } else {
    // Both internal; split the one with larger surface area.
    const areaA = (na.maxX - na.minX) * (na.maxY - na.minY)
    const areaB = (nb.maxX - nb.minX) * (nb.maxY - nb.minY)
    if (areaA >= areaB) {
      queryCross(nodes, na.left,  b, flags, pairs)
      queryCross(nodes, na.right, b, flags, pairs)
    } else {
      queryCross(nodes, a, nb.left,  flags, pairs)
      queryCross(nodes, a, nb.right, flags, pairs)
    }
  }
}

/**
 * Emit all candidate pairs within a subtree (self-pairs).
 * Recurses into children, then cross-tests left vs right.
 */
function querySelf(
  nodes: BVHNode[],
  node: number,
  flags: Uint8Array,
  pairs: CollisionPair[],
): void {
  const n = nodes[node]
  if (n.body !== -1) return  // Leaf — no self-pairs.
  querySelf(nodes, n.left,  flags, pairs)
  querySelf(nodes, n.right, flags, pairs)
  queryCross(nodes, n.left, n.right, flags, pairs)
}

/**
 * BVH broadphase: builds a top-down AABB tree over all active bodies each
 * frame, then traverses it with dual-subtree pair queries.
 *
 * Complexity: O(n log n) build, O(n log n) expected query for coherent scenes.
 * Better than SAP when bodies are densely clustered in space.
 */
export function broadphaseBVH(
  buffers: MatchaBuffers,
  count: number,
): CollisionPair[] {
  const { flags } = buffers

  const bodies: number[] = []
  for (let i = 0; i < count; i++) {
    if (flags[i] & BodyFlags.ACTIVE) bodies.push(i)
  }
  if (bodies.length < 2) return []

  const nodes: BVHNode[] = []
  buildNodeBVH(nodes, bodies, 0, bodies.length, buffers)

  const pairs: CollisionPair[] = []
  querySelf(nodes, 0, flags, pairs)
  return pairs
}

/** Recursively build BVH using correct per-body world-space AABBs. */
function buildNodeBVH(
  nodes: BVHNode[],
  bodies: number[],
  start: number,
  end: number,
  buffers: MatchaBuffers,
): number {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity
  for (let i = start; i < end; i++) {
    const [bMinX, bMinY, bMaxX, bMaxY] = computeBodyAABB(bodies[i], buffers)
    if (bMinX < minX) minX = bMinX
    if (bMinY < minY) minY = bMinY
    if (bMaxX > maxX) maxX = bMaxX
    if (bMaxY > maxY) maxY = bMaxY
  }

  const idx = nodes.length
  nodes.push({ minX, minY, maxX, maxY, left: -1, right: -1, body: -1 })

  if (end - start === 1) {
    nodes[idx].body = bodies[start]
    return idx
  }

  const mid = start + Math.floor((end - start) / 2)
  const rangeX = maxX - minX
  const rangeY = maxY - minY

  // Compute AABB centers for sorting (not body positions, which can differ for rotated shapes).
  const centerX = new Float64Array(end - start)
  const centerY = new Float64Array(end - start)
  for (let i = start; i < end; i++) {
    const [bMinX, bMinY, bMaxX, bMaxY] = computeBodyAABB(bodies[i], buffers)
    centerX[i - start] = (bMinX + bMaxX) * 0.5
    centerY[i - start] = (bMinY + bMaxY) * 0.5
  }

  if (rangeX >= rangeY) {
    const slice = bodies.slice(start, end).sort((a, b) => centerX[a - start] - centerX[b - start])
    for (let i = 0; i < slice.length; i++) bodies[start + i] = slice[i]
  } else {
    const slice = bodies.slice(start, end).sort((a, b) => centerY[a - start] - centerY[b - start])
    for (let i = 0; i < slice.length; i++) bodies[start + i] = slice[i]
  }

  nodes[idx].left  = buildNodeBVH(nodes, bodies, start, mid, buffers)
  nodes[idx].right = buildNodeBVH(nodes, bodies, mid,   end, buffers)
  return idx
}
