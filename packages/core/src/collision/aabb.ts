import type { AABB } from '@matcha2d/types'

export function aabbOverlap(a: AABB, b: AABB): boolean {
  return a.minX <= b.maxX && a.maxX >= b.minX
      && a.minY <= b.maxY && a.maxY >= b.minY
}

export function aabbMerge(a: AABB, b: AABB): AABB {
  return {
    minX: Math.min(a.minX, b.minX),
    minY: Math.min(a.minY, b.minY),
    maxX: Math.max(a.maxX, b.maxX),
    maxY: Math.max(a.maxY, b.maxY),
  }
}

export function aabbContains(outer: AABB, inner: AABB): boolean {
  return outer.minX <= inner.minX && outer.maxX >= inner.maxX
      && outer.minY <= inner.minY && outer.maxY >= inner.maxY
}

export function aabbArea(a: AABB): number {
  return (a.maxX - a.minX) * (a.maxY - a.minY)
}

export function aabbPerimeter(a: AABB): number {
  return 2 * ((a.maxX - a.minX) + (a.maxY - a.minY))
}
