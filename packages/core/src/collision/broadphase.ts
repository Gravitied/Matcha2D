import type { CollisionPair, MatchaBuffers } from '@matcha2d/types'

/**
 * Broadphase collision detection.
 * TODO: Implement sort-and-sweep, then upgrade to dynamic BVH.
 */
export function broadphase(
  _buffers: MatchaBuffers,
  _count: number,
): CollisionPair[] {
  return []
}
