import type { CollisionPair, ContactManifold, MatchaBuffers } from '@matcha2d/types'

/**
 * GJK + EPA for convex shape collision detection.
 * TODO: Implement GJK distance algorithm and EPA penetration depth.
 */
export function gjkNarrowphase(
  _buffers: MatchaBuffers,
  _pairs: CollisionPair[],
): ContactManifold[] {
  return []
}
