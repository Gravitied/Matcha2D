import { describe, it, expect } from 'vitest'
import { aabbOverlap, aabbMerge, aabbContains, aabbArea, aabbPerimeter } from '../src/collision/aabb.js'

describe('AABB operations', () => {
  const a = { minX: 0, minY: 0, maxX: 2, maxY: 2 }
  const b = { minX: 1, minY: 1, maxX: 3, maxY: 3 }
  const c = { minX: 5, minY: 5, maxX: 6, maxY: 6 }

  it('detects overlapping AABBs', () => {
    expect(aabbOverlap(a, b)).toBe(true)
  })

  it('detects non-overlapping AABBs', () => {
    expect(aabbOverlap(a, c)).toBe(false)
  })

  it('merges two AABBs', () => {
    const merged = aabbMerge(a, b)
    expect(merged).toEqual({ minX: 0, minY: 0, maxX: 3, maxY: 3 })
  })

  it('checks containment', () => {
    const outer = { minX: 0, minY: 0, maxX: 10, maxY: 10 }
    expect(aabbContains(outer, a)).toBe(true)
    expect(aabbContains(a, outer)).toBe(false)
  })

  it('computes area', () => {
    expect(aabbArea(a)).toBe(4)
  })

  it('computes perimeter', () => {
    expect(aabbPerimeter(a)).toBe(8)
  })
})
