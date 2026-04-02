import type { BodyHandle, CollisionCallbacks, ContactInfo, ContactManifold, WorldConfig } from '@matcha2d/types'

/**
 * Tracks contact state across frames and fires collision callbacks.
 *
 * Compares new manifolds against previous frame to detect:
 * - Begin contact: new pair not seen before
 * - Stay contact: pair that existed last frame
 * - End contact: pair that existed last frame but not this frame
 */
export class ContactTracker {
  private callbacks: CollisionCallbacks | null = null
  private activePairs = new Map<number, ContactInfo>()
  private prevKeys = new Set<number>()
  private currKeys = new Set<number>()

  setCallbacks(callbacks: CollisionCallbacks | null): void {
    this.callbacks = callbacks
  }

  /**
   * Update contact state and fire callbacks.
   * Call this after solveVelocity, before storing manifolds for next frame.
   */
  update(manifolds: ContactManifold[], _config: WorldConfig): void {
    if (!this.callbacks) return

    this.prevKeys = new Set(this.currKeys)
    this.currKeys.clear()

    for (let m = 0; m < manifolds.length; m++) {
      const manifold = manifolds[m]
      const key = makePairKey(manifold.bodyA as number, manifold.bodyB as number)
      this.currKeys.add(key)

      const totalImpulse = manifold.accumulatedImpulses
        ? manifold.accumulatedImpulses.reduce((sum, v) => sum + v, 0)
        : 0

      const contactPoints: Array<{ x: number; y: number }> = []
      for (let c = 0; c < manifold.contacts.length; c++) {
        const cp = manifold.contacts[c]
        contactPoints.push({ x: cp.localA.x, y: cp.localA.y })
      }

      const info: ContactInfo = {
        bodyA: manifold.bodyA,
        bodyB: manifold.bodyB,
        contactNormal: { x: manifold.normal.x, y: manifold.normal.y },
        contactPoints,
        totalImpulse,
      }

      if (!this.prevKeys.has(key)) {
        this.callbacks.onBeginContact?.(info)
      } else {
        this.callbacks.onStayContact?.(info)
      }

      this.activePairs.set(key, info)
    }

    for (const key of this.prevKeys) {
      if (!this.currKeys.has(key)) {
        const info = this.activePairs.get(key)
        if (info) {
          this.callbacks.onEndContact?.(info.bodyA, info.bodyB)
          this.activePairs.delete(key)
        }
      }
    }
  }

  clear(): void {
    this.activePairs.clear()
    this.prevKeys.clear()
    this.currKeys.clear()
  }
}

/** Generate a unique key for a body pair (order-independent). */
function makePairKey(a: number, b: number): number {
  if (a < b) return (a * 73856093) ^ (b * 19349663)
  return (b * 73856093) ^ (a * 19349663)
}
