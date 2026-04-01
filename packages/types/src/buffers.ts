/** Flat SoA buffers shared between physics core and world management. */
export interface MatchaBuffers {
  positionX: Float32Array
  positionY: Float32Array
  velocityX: Float32Array
  velocityY: Float32Array
  angle: Float32Array
  angularVel: Float32Array
  mass: Float32Array
  invMass: Float32Array
  inertia: Float32Array
  invInertia: Float32Array
  flags: Uint8Array
}

/** Maximum bodies supported in one world. */
export const MAX_BODIES = 8192

/** Allocate a fresh set of buffers for `capacity` bodies. */
export function createBuffers(capacity: number = MAX_BODIES): MatchaBuffers {
  return {
    positionX: new Float32Array(capacity),
    positionY: new Float32Array(capacity),
    velocityX: new Float32Array(capacity),
    velocityY: new Float32Array(capacity),
    angle: new Float32Array(capacity),
    angularVel: new Float32Array(capacity),
    mass: new Float32Array(capacity),
    invMass: new Float32Array(capacity),
    inertia: new Float32Array(capacity),
    invInertia: new Float32Array(capacity),
    flags: new Uint8Array(capacity),
  }
}
