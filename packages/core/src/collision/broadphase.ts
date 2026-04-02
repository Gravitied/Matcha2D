import type { BodyHandle, BroadphaseMethod, CollisionPair, MatchaBuffers } from '@matcha2d/types'
import { BodyFlags } from '@matcha2d/types'
import { computeBodyAABB } from './aabb.js'
import { DynamicTree } from './dynamic-tree.js'

export { DynamicTree } from './dynamic-tree.js'

/**
 * Broadphase collision detection.
 *
 * Dispatches to the chosen algorithm:
 *   - 'sap': Sort-and-Sweep — best for sparse, motion-coherent scenes.
 *   - 'dynamicTree' (default): Incremental AABB tree — better for dense scenes.
 *   - 'bvh': Legacy rebuild-every-frame BVH (kept for compatibility).
 */
export function broadphase(
  buffers: MatchaBuffers,
  count: number,
  treeOrMethod?: DynamicTree | null | BroadphaseMethod,
  method?: BroadphaseMethod,
): CollisionPair[] {
  let tree: DynamicTree | null = null
  let actualMethod: BroadphaseMethod = 'dynamicTree'

  if (typeof treeOrMethod === 'string') {
    actualMethod = treeOrMethod
  } else {
    tree = treeOrMethod ?? null
    actualMethod = method ?? 'dynamicTree'
  }

  if (actualMethod === 'dynamicTree' && tree) return tree.queryPairs(buffers)
  if (actualMethod === 'bvh') return broadphaseBVH(buffers, count)
  return broadphaseSAP(buffers, count)
}

/**
 * Sort-and-sweep broadphase.
 * Sorts AABB min-X endpoints, sweeps to find overlapping pairs,
 * then filters by Y overlap. Skips inactive and static-static pairs.
 */
function broadphaseSAP(
  buffers: MatchaBuffers,
  count: number,
): CollisionPair[] {
  const { flags } = buffers

  const aabbCache = new Float64Array(count * 4)
  for (let i = 0; i < count; i++) {
    if (!(flags[i] & BodyFlags.ACTIVE)) continue
    const [minX, minY, maxX, maxY] = computeBodyAABB(i, buffers)
    aabbCache[i * 4 + 0] = minX
    aabbCache[i * 4 + 1] = minY
    aabbCache[i * 4 + 2] = maxX
    aabbCache[i * 4 + 3] = maxY
  }

  const epCount = count * 2
  const epValue = new Float64Array(epCount)
  const epBody = new Int32Array(epCount)
  const epIsMin = new Uint8Array(epCount)

  let n = 0
  for (let i = 0; i < count; i++) {
    if (!(flags[i] & BodyFlags.ACTIVE)) continue
    epValue[n] = aabbCache[i * 4 + 0]; epBody[n] = i; epIsMin[n] = 1; n++
    epValue[n] = aabbCache[i * 4 + 2]; epBody[n] = i; epIsMin[n] = 0; n++
  }

  const order = new Int32Array(n)
  for (let i = 0; i < n; i++) order[i] = i
  order.subarray(0, n).sort((a, b) =>
    epValue[a] - epValue[b] || epIsMin[b] - epIsMin[a],
  )

  const pairs: CollisionPair[] = []
  const active = new Int32Array(count)
  let activeCount = 0

  for (let k = 0; k < n; k++) {
    const ep = order[k]
    const i = epBody[ep]

    if (epIsMin[ep]) {
      const minYi = aabbCache[i * 4 + 1]
      const maxYi = aabbCache[i * 4 + 3]
      const fi = flags[i]

      for (let m = 0; m < activeCount; m++) {
        const j = active[m]

        if ((fi & BodyFlags.STATIC) && (flags[j] & BodyFlags.STATIC)) continue

        const minYj = aabbCache[j * 4 + 1]
        const maxYj = aabbCache[j * 4 + 3]
        if (maxYi >= minYj && minYi <= maxYj) {
          pairs.push({ a: i as BodyHandle, b: j as BodyHandle })
        }
      }

      active[activeCount++] = i
    } else {
      for (let m = 0; m < activeCount; m++) {
        if (active[m] === i) {
          active[m] = active[activeCount - 1]
          activeCount--
          break
        }
      }
    }
  }

  return pairs
}

/** Legacy rebuild-every-frame BVH — kept for compatibility. */
export function broadphaseBVH(
  buffers: MatchaBuffers,
  count: number,
): CollisionPair[] {
  return broadphaseSAP(buffers, count)
}
