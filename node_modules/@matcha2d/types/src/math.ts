/** Readonly 2D vector (for function signatures). */
export interface Vec2Readonly {
  readonly x: number
  readonly y: number
}

/** Axis-Aligned Bounding Box. */
export interface AABB {
  minX: number
  minY: number
  maxX: number
  maxY: number
}

/** Rigid transform: position + rotation angle (radians). */
export interface Transform {
  x: number
  y: number
  angle: number
}
