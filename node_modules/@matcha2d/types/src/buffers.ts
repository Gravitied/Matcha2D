/** Maximum vertices per convex polygon shape. */
export const MAX_VERTICES_PER_SHAPE = 16

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
  /** AABB half-extents along X — set at body creation, used by broadphase. */
  halfExtentX: Float32Array
  /** AABB half-extents along Y — set at body creation, used by broadphase. */
  halfExtentY: Float32Array
  /** Shape type per body: 0=Box, 1=Circle, 2=Polygon. */
  shapeType: Uint8Array
  /** Circle radius (used when shapeType == Circle). */
  shapeRadius: Float32Array
  /** Number of vertices (used when shapeType == Polygon, 0 otherwise). */
  shapeVertexCount: Uint8Array
  /** Polygon vertices X (capacity * MAX_VERTICES_PER_SHAPE, local space). */
  shapeVerticesX: Float32Array
  /** Polygon vertices Y (capacity * MAX_VERTICES_PER_SHAPE, local space). */
  shapeVerticesY: Float32Array
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
    halfExtentX: new Float32Array(capacity),
    halfExtentY: new Float32Array(capacity),
    shapeType: new Uint8Array(capacity),
    shapeRadius: new Float32Array(capacity),
    shapeVertexCount: new Uint8Array(capacity),
    shapeVerticesX: new Float32Array(capacity * MAX_VERTICES_PER_SHAPE),
    shapeVerticesY: new Float32Array(capacity * MAX_VERTICES_PER_SHAPE),
  }
}
