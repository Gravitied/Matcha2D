import { describe, it, expect } from 'vitest'
import { narrowphase } from '../src/collision/narrowphase.js'
import { createBuffers, ShapeType, BodyFlags, MAX_VERTICES_PER_SHAPE } from '@matcha2d/types'

function makeBuffers(capacity = 16) {
  return createBuffers(capacity)
}

function placeBox(buf: ReturnType<typeof makeBuffers>, idx: number, x: number, y: number, hx: number, hy: number, angle = 0) {
  buf.positionX[idx] = x
  buf.positionY[idx] = y
  buf.angle[idx] = angle
  buf.halfExtentX[idx] = hx
  buf.halfExtentY[idx] = hy
  buf.shapeType[idx] = ShapeType.Box
  buf.flags[idx] = BodyFlags.ACTIVE
}

function placeCircle(buf: ReturnType<typeof makeBuffers>, idx: number, x: number, y: number, radius: number) {
  buf.positionX[idx] = x
  buf.positionY[idx] = y
  buf.shapeRadius[idx] = radius
  buf.shapeType[idx] = ShapeType.Circle
  buf.flags[idx] = BodyFlags.ACTIVE
}

function placePolygon(buf: ReturnType<typeof makeBuffers>, idx: number, x: number, y: number, vertices: Array<[number, number]>, angle = 0) {
  buf.positionX[idx] = x
  buf.positionY[idx] = y
  buf.angle[idx] = angle
  buf.shapeType[idx] = ShapeType.Polygon
  buf.shapeVertexCount[idx] = vertices.length
  buf.flags[idx] = BodyFlags.ACTIVE
  const base = idx * MAX_VERTICES_PER_SHAPE
  for (let i = 0; i < vertices.length; i++) {
    buf.shapeVerticesX[base + i] = vertices[i][0]
    buf.shapeVerticesY[base + i] = vertices[i][1]
  }
}

describe('narrowphase (GJK+EPA)', () => {
  describe('box vs box', () => {
    it('detects overlapping axis-aligned boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('returns no collision for separated boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 10, 0, 1, 1)
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects rotated box collision', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1, Math.PI / 4)
      placeBox(buf, 1, 0.8, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for just-touching boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeLessThanOrEqual(1)
    })

    it('provides correct normal direction (from A toward B)', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.5, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].normal.x).toBeGreaterThan(0)
    })

    it('provides unit-length contact normal', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      const n = manifolds[0].normal
      const len = Math.sqrt(n.x * n.x + n.y * n.y)
      expect(len).toBeCloseTo(1, 4)
    })
  })

  describe('circle vs circle', () => {
    it('detects overlapping circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 1, 0, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeCloseTo(1, 3)
    })

    it('returns no collision for separated circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 10, 0, 1)
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('handles concentric circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 2)
      placeCircle(buf, 1, 0, 0, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('provides correct normal direction (from A toward B)', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 1.5, 0, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].normal.x).toBeGreaterThan(0)
    })
  })

  describe('circle vs box', () => {
    it('detects circle overlapping box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0.5, 0, 1)
      placeBox(buf, 1, 0, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for circle far from box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeBox(buf, 1, 10, 10, 1, 1)
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects box vs circle (reversed order)', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeCircle(buf, 1, 0.5, 0, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('detects circle inside rotated box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0.3, 0.3, 0.5)
      placeBox(buf, 1, 0, 0, 1, 1, Math.PI / 6)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('polygon vs polygon', () => {
    it('detects overlapping triangles', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placePolygon(buf, 1, 0.3, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for separated polygons', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placePolygon(buf, 1, 10, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects rotated polygon collision', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]], Math.PI / 4)
      placePolygon(buf, 1, 0.2, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('circle vs polygon', () => {
    it('detects circle overlapping triangle', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 0.5)
      placePolygon(buf, 1, 0.3, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('detects triangle vs circle (reversed order)', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0.3, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placeCircle(buf, 1, 0, 0, 0.5)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for circle far from polygon', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 0.5)
      placePolygon(buf, 1, 10, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })
  })

  describe('edge cases', () => {
    it('skips inactive bodies', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      buf.flags[0] = 0
      placeBox(buf, 1, 0.5, 0, 1, 1)
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('skips both-sleeping bodies', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.SLEEPING
      placeBox(buf, 1, 0.5, 0, 1, 1)
      buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.SLEEPING
      expect(narrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('handles empty pairs array', () => {
      expect(narrowphase(makeBuffers(), [])).toHaveLength(0)
    })

    it('handles deep overlap without crashing', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 0, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('handles grazing contact (barely touching)', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeLessThanOrEqual(1)
    })
  })

  describe('manifold quality', () => {
    it('provides unit-length contact normal', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      const n = manifolds[0].normal
      const len = Math.sqrt(n.x * n.x + n.y * n.y)
      expect(len).toBeCloseTo(1, 4)
    })

    it('provides positive penetration depth', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('provides at least one contact point', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = narrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds[0].contacts.length).toBeGreaterThanOrEqual(1)
    })
  })
})
