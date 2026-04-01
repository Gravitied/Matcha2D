import type { MatchaBuffers } from './buffers.js'
import type { CollisionPair, ContactManifold } from './collision.js'
import type { WorldConfig } from './config.js'

/**
 * The contract between the world (Dev B) and physics core (Dev A).
 * Dev B calls these methods; Dev A implements them.
 */
export interface PhysicsBackend {
  /** Run broadphase on current positions, return candidate pairs. */
  broadphase(buffers: MatchaBuffers, count: number): CollisionPair[]

  /** Run narrowphase on candidate pairs, return contact manifolds. */
  narrowphase(buffers: MatchaBuffers, pairs: CollisionPair[]): ContactManifold[]

  /** Solve velocity constraints (contacts + joints). */
  solveVelocity(
    buffers: MatchaBuffers,
    manifolds: ContactManifold[],
    config: WorldConfig,
  ): void

  /** Integrate positions from velocities. */
  integrate(
    buffers: MatchaBuffers,
    count: number,
    dt: number,
    gravity: { x: number; y: number },
  ): void

  /** Solve position constraints (penetration correction). */
  solvePosition(
    buffers: MatchaBuffers,
    manifolds: ContactManifold[],
    config: WorldConfig,
  ): void
}
