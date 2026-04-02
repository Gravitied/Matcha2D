import type { CollisionPair, ContactManifold, MatchaBuffers } from '@matcha2d/types'
import { gjkNarrowphase } from './gjk.js'

/**
 * Narrowphase collision detection.
 *
 * Uses GJK+EPA for all shape combinations.
 * Circle-circle uses an analytical shortcut for performance.
 */
export function narrowphase(
  buffers: MatchaBuffers,
  pairs: CollisionPair[],
): ContactManifold[] {
  return gjkNarrowphase(buffers, pairs)
}
