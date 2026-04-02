/**
 * WASM-based PhysicsBackend implementation for Matcha2D.
 *
 * Wraps the Emscripten-compiled Box2D WASM module behind the standard
 * PhysicsBackend interface. The WASM backend takes a unified approach:
 * Box2D's b2World_Step internally handles broadphase → narrowphase →
 * solve → integrate, so individual methods are thin wrappers or no-ops.
 */

import type {
  BroadphaseMethod,
  CollisionPair,
  ContactManifold,
  MatchaBuffers,
  NarrowphaseMethod,
  PhysicsBackend,
  WorldConfig,
} from '@matcha2d/types'
import { loadWasmModule } from './WasmModule.js'
import type { WasmInstance } from './WasmModule.js'

export class WasmPhysicsBackend implements PhysicsBackend {
  private wasm: WasmInstance | null = null
  private worldHandle: number = 0
  private initialized = false

  /**
   * Initialize the WASM backend.
   *
   * Loads the WASM module, creates a Box2D world with the given gravity,
   * and allocates internal data structures for up to maxBodies bodies.
   */
  async init(config: WorldConfig): Promise<void> {
    if (this.initialized) return

    this.wasm = await loadWasmModule()
    const maxBodies = config.maxBodies ?? 8192
    this.worldHandle = this.wasm.init(
      config.gravity.x,
      config.gravity.y,
      maxBodies,
    )

    if (this.worldHandle < 0) {
      throw new Error(
        `WASM backend: b2_init failed (handle=${this.worldHandle}). ` +
        'Check that the WASM module was built correctly.',
      )
    }

    this.initialized = true
  }

  /**
   * Run broadphase collision detection.
   *
   * In WASM mode, broadphase is handled internally by Box2D during
   * b2_step(). This method returns an empty array — use collide()
   * instead for the full pipeline.
   */
  broadphase(
    _buffers: MatchaBuffers,
    _count: number,
    _method?: BroadphaseMethod,
  ): CollisionPair[] {
    // WASM handles broadphase internally during step().
    // Return empty — callers should use collide() instead.
    return []
  }

  /**
   * Run narrowphase collision detection on candidate pairs.
   *
   * In WASM mode, narrowphase is handled internally by Box2D during
   * b2_step(). This method returns an empty array — use collide()
   * instead for the full pipeline.
   */
  narrowphase(
    _buffers: MatchaBuffers,
    _pairs: CollisionPair[],
    _method?: NarrowphaseMethod,
  ): ContactManifold[] {
    // WASM handles narrowphase internally during step().
    // Return empty — callers should use collide() instead.
    return []
  }

  /**
   * Run the full collision pipeline.
   *
   * This is the primary entry point for the WASM backend. It:
   * 1. Syncs body data (position, velocity, mass, etc.) to WASM
   * 2. Syncs shape data (boxes, circles, polygons) to WASM
   * 3. Advances the simulation by one time step
   * 4. Reads updated body state back from WASM
   * 5. Extracts contact manifolds from the simulation
   *
   * The method and narrowMethod parameters are ignored in WASM mode
   * since Box2D uses its own internal algorithms.
   */
  collide(
    buffers: MatchaBuffers,
    count: number,
    _broadMethod?: BroadphaseMethod,
    _narrowMethod?: NarrowphaseMethod,
  ): ContactManifold[] {
    if (!this.initialized || !this.wasm) {
      throw new Error(
        'WASM backend not initialized. Call init() before collide().',
      )
    }

    // Push current body state into WASM memory
    this.wasm.syncBodies(this.worldHandle, buffers, count)

    // Push shape data into WASM memory
    this.wasm.syncShapes(this.worldHandle, buffers, count)

    // Run one simulation step (Box2D handles broadphase → narrowphase → solve → integrate)
    this.wasm.step(this.worldHandle, 1 / 60, 4)

    // Read updated body state back from WASM
    this.wasm.readBodies(this.worldHandle, buffers, count)

    // Extract contact manifolds
    return this.wasm.getContacts(this.worldHandle)
  }

  /**
   * Solve velocity constraints.
   *
   * In WASM mode, velocity solving is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  solveVelocity(
    _buffers: MatchaBuffers,
    _manifolds: ContactManifold[],
    _config: WorldConfig,
  ): void {
    // Handled internally by Box2D during step().
  }

  /**
   * Integrate positions from velocities.
   *
   * In WASM mode, integration is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  integrate(
    _buffers: MatchaBuffers,
    _count: number,
    _dt: number,
    _gravity: { x: number; y: number },
  ): void {
    // Handled internally by Box2D during step().
  }

  /**
   * Solve position constraints (penetration correction).
   *
   * In WASM mode, position solving is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  solvePosition(
    _buffers: MatchaBuffers,
    _manifolds: ContactManifold[],
    _config: WorldConfig,
  ): void {
    // Handled internally by Box2D during step().
  }

  /**
   * Clean up WASM resources.
   *
   * Destroys the Box2D world and frees all WASM-allocated memory.
   */
  dispose(): void {
    if (this.wasm) {
      if (this.worldHandle >= 0) {
        this.wasm.destroy(this.worldHandle)
      }
      this.wasm.dispose()
      this.wasm = null
    }
    this.initialized = false
    this.worldHandle = 0
  }
}
