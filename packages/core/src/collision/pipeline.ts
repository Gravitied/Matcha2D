import type {
  BroadphaseMethod,
  CollisionPair,
  ContactManifold,
  MatchaBuffers,
  NarrowphaseMethod,
} from '@matcha2d/types'
import { broadphase } from './broadphase.js'
import { gjkNarrowphase } from './gjk.js'
import type { DynamicTree } from './dynamic-tree.js'

/**
 * Collision pipeline: broadphase → narrowphase.
 *
 * Users can customize the broadphase method:
 *   - broadphaseMethod: 'sap', 'dynamicTree' (default), or 'bvh'
 *
 * Narrowphase always uses GJK+EPA for robust collision detection.
 */
export function collide(
  buffers: MatchaBuffers,
  count: number,
  treeOrMethod?: DynamicTree | null | BroadphaseMethod,
  broadphaseMethod?: BroadphaseMethod,
  _narrowphaseMethod?: NarrowphaseMethod,
): ContactManifold[] {
  const pairs = broadphase(buffers, count, treeOrMethod, broadphaseMethod)
  if (pairs.length === 0) return []

  return gjkNarrowphase(buffers, pairs)
}

/**
 * Run narrowphase on pre-filtered candidate pairs.
 * Use this when you already have pairs from a custom broadphase.
 */
export function narrowphaseDispatch(
  buffers: MatchaBuffers,
  pairs: CollisionPair[],
  _method: NarrowphaseMethod = 'gjk',
): ContactManifold[] {
  if (pairs.length === 0) return []
  return gjkNarrowphase(buffers, pairs)
}
