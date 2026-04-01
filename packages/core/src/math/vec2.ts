/**
 * Vec2 operations on flat Float32Array buffers.
 * Array+index signatures avoid allocation in hot loops.
 */

export function vec2Set(
  outX: Float32Array, outY: Float32Array, idx: number,
  x: number, y: number,
): void {
  outX[idx] = x
  outY[idx] = y
}

export function vec2Add(
  outX: Float32Array, outY: Float32Array, outIdx: number,
  aX: Float32Array, aY: Float32Array, aIdx: number,
  bX: Float32Array, bY: Float32Array, bIdx: number,
): void {
  outX[outIdx] = aX[aIdx] + bX[bIdx]
  outY[outIdx] = aY[aIdx] + bY[bIdx]
}

export function vec2Sub(
  outX: Float32Array, outY: Float32Array, outIdx: number,
  aX: Float32Array, aY: Float32Array, aIdx: number,
  bX: Float32Array, bY: Float32Array, bIdx: number,
): void {
  outX[outIdx] = aX[aIdx] - bX[bIdx]
  outY[outIdx] = aY[aIdx] - bY[bIdx]
}

export function vec2Scale(
  outX: Float32Array, outY: Float32Array, outIdx: number,
  aX: Float32Array, aY: Float32Array, aIdx: number,
  s: number,
): void {
  outX[outIdx] = aX[aIdx] * s
  outY[outIdx] = aY[aIdx] * s
}

export function vec2Dot(
  aX: Float32Array, aY: Float32Array, aIdx: number,
  bX: Float32Array, bY: Float32Array, bIdx: number,
): number {
  return aX[aIdx] * bX[bIdx] + aY[aIdx] * bY[bIdx]
}

export function vec2Cross(
  aX: Float32Array, aY: Float32Array, aIdx: number,
  bX: Float32Array, bY: Float32Array, bIdx: number,
): number {
  return aX[aIdx] * bY[bIdx] - aY[aIdx] * bX[bIdx]
}

export function vec2LengthSq(
  x: Float32Array, y: Float32Array, idx: number,
): number {
  return x[idx] * x[idx] + y[idx] * y[idx]
}

export function vec2Length(
  x: Float32Array, y: Float32Array, idx: number,
): number {
  return Math.sqrt(x[idx] * x[idx] + y[idx] * y[idx])
}

export function vec2Normalize(
  outX: Float32Array, outY: Float32Array, outIdx: number,
  aX: Float32Array, aY: Float32Array, aIdx: number,
): void {
  const len = Math.sqrt(aX[aIdx] * aX[aIdx] + aY[aIdx] * aY[aIdx])
  if (len > 0) {
    const invLen = 1 / len
    outX[outIdx] = aX[aIdx] * invLen
    outY[outIdx] = aY[aIdx] * invLen
  } else {
    outX[outIdx] = 0
    outY[outIdx] = 0
  }
}

export function vec2DistanceSq(
  aX: Float32Array, aY: Float32Array, aIdx: number,
  bX: Float32Array, bY: Float32Array, bIdx: number,
): number {
  const dx = aX[aIdx] - bX[bIdx]
  const dy = aY[aIdx] - bY[bIdx]
  return dx * dx + dy * dy
}

// --- Scalar convenience helpers ---

export function dot(ax: number, ay: number, bx: number, by: number): number {
  return ax * bx + ay * by
}

export function cross(ax: number, ay: number, bx: number, by: number): number {
  return ax * by - ay * bx
}

export function lengthSq(x: number, y: number): number {
  return x * x + y * y
}

export function length(x: number, y: number): number {
  return Math.sqrt(x * x + y * y)
}
