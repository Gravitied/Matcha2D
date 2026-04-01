/**
 * 2x2 rotation matrix operations.
 * A rotation by angle θ is represented as [cos θ, -sin θ, sin θ, cos θ].
 */

export function mat2FromAngle(angle: number): [number, number, number, number] {
  const c = Math.cos(angle)
  const s = Math.sin(angle)
  return [c, -s, s, c]
}

/** Multiply a 2x2 matrix by a vector: [m00*x + m01*y, m10*x + m11*y] */
export function mat2MulVec(
  m00: number, m01: number, m10: number, m11: number,
  x: number, y: number,
): [number, number] {
  return [m00 * x + m01 * y, m10 * x + m11 * y]
}

/** Transpose-multiply (inverse rotation): [m00*x + m10*y, m01*x + m11*y] */
export function mat2TransposeMulVec(
  m00: number, m01: number, m10: number, m11: number,
  x: number, y: number,
): [number, number] {
  return [m00 * x + m10 * y, m01 * x + m11 * y]
}
