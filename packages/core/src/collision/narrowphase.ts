import type { CollisionPair, ContactManifold, MatchaBuffers } from '@matcha2d/types'

/**
 * Narrowphase collision detection using SAT for polygons + circles.
 * TODO: Implement SAT algorithm.
 */
export function narrowphase(
  _buffers: MatchaBuffers,
  _pairs: CollisionPair[],
): ContactManifold[] {
  return []
}
