import { describe, it, expect } from 'vitest'
import { gjkNarrowphase } from '../src/collision/gjk.js'
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

describe('GJK narrowphase', () => {
  describe('box vs box', () => {
    it('detects overlapping axis-aligned boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('returns no collision for separated boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 10, 0, 1, 1)
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects rotated box collision', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1, Math.PI / 4)
      placeBox(buf, 1, 0.8, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('provides valid contact normal pointing from A to B', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.5, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].normal.x).toBeGreaterThan(0)
    })

    it('sets correct body handles', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].bodyA).toBe(0)
      expect(manifolds[0].bodyB).toBe(1)
    })

    it('handles identical overlapping boxes', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 0, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('circle vs circle', () => {
    it('detects overlapping circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 1, 0, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeCloseTo(1, 3)
    })

    it('returns no collision for separated circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 10, 0, 1)
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('handles concentric circles', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 2)
      placeCircle(buf, 1, 0, 0, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('provides correct normal direction (from A center toward B)', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeCircle(buf, 1, 1.5, 0, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].normal.x).toBeGreaterThan(0)
    })
  })

  describe('circle vs box', () => {
    it('detects circle overlapping box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0.5, 0, 1)
      placeBox(buf, 1, 0, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for circle far from box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0, 0, 1)
      placeBox(buf, 1, 10, 10, 1, 1)
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects circle inside rotated box', () => {
      const buf = makeBuffers()
      placeCircle(buf, 0, 0.3, 0.3, 0.5)
      placeBox(buf, 1, 0, 0, 1, 1, Math.PI / 6)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('polygon vs polygon', () => {
    it('detects overlapping triangles', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placePolygon(buf, 1, 0.3, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for separated polygons', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placePolygon(buf, 1, 10, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('detects rotated polygon collision', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]], Math.PI / 4)
      placePolygon(buf, 1, 0.2, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('handles pentagon vs pentagon overlap', () => {
      const pentagon: Array<[number, number]> = []
      for (let i = 0; i < 5; i++) {
        const a = (i / 5) * Math.PI * 2
        pentagon.push([Math.cos(a) * 0.8, Math.sin(a) * 0.8])
      }
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, pentagon)
      placePolygon(buf, 1, 0.5, 0, pentagon)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('handles hexagon vs hexagon overlap', () => {
      const hexagon: Array<[number, number]> = []
      for (let i = 0; i < 6; i++) {
        const a = (i / 6) * Math.PI * 2
        hexagon.push([Math.cos(a), Math.sin(a)])
      }
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, hexagon)
      placePolygon(buf, 1, 0.8, 0, hexagon)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('polygon vs circle', () => {
    it('detects polygon overlapping circle', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placeCircle(buf, 1, 0.2, 0, 0.5)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('returns no collision for polygon far from circle', () => {
      const buf = makeBuffers()
      placePolygon(buf, 0, 0, 0, [[-0.5, -0.5], [0.5, -0.5], [0, 0.5]])
      placeCircle(buf, 1, 10, 0, 0.5)
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })
  })

  describe('edge cases', () => {
    it('skips inactive bodies', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      buf.flags[0] = 0
      placeBox(buf, 1, 0.5, 0, 1, 1)
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('skips both-sleeping bodies', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.SLEEPING
      placeBox(buf, 1, 0.5, 0, 1, 1)
      buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.SLEEPING
      expect(gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])).toHaveLength(0)
    })

    it('handles empty pairs array', () => {
      expect(gjkNarrowphase(makeBuffers(), [])).toHaveLength(0)
    })

    it('handles deep overlap without crashing', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 0, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
    })

    it('handles grazing contact (barely touching)', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeLessThanOrEqual(1)
    })
  })

  describe('GJK edge cases — termination correctness', () => {
    it('detects collision for two boxes where centers are close but shapes just overlap', () => {
      // Box A: 1x1 at origin. Box B: 1x1 at x=0.95 — they overlap by 0.05 (half-extents 0.5 each)
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 0.5, 0.5)
      placeBox(buf, 1, 0.95, 0, 0.5, 0.5)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
      expect(manifolds[0].contacts[0].penetration).toBeLessThan(0.15)
    })

    it('does not detect collision for two separated boxes (gap=0.01)', () => {
      // Box A: half-extent 0.5 at x=0. Box B: half-extent 0.5 at x=1.01 — gap of 0.01
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 0.5, 0.5)
      placeBox(buf, 1, 1.01, 0, 0.5, 0.5)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(0)
    })

    it('fallback penetration picks minimum SAT axis for overlapping box pair', () => {
      // Box A: half-extents (2, 2), Box B: half-extents (2, 2), offset (3.5, 0)
      // X-axis overlap = (2+2) - 3.5 = 0.5, Y-axis overlap = (2+2) - 0 = 4.0
      // SAT minimum axis = X = 0.5 — correct penetration should be < 1.0
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 2, 2)
      placeBox(buf, 1, 3.5, 0, 2, 2)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds).toHaveLength(1)
      // X-axis overlap is 0.5, Y-axis overlap is 4.0 — SAT should pick X = 0.5
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
      expect(manifolds[0].contacts[0].penetration).toBeLessThan(1.0)
    })
  })

  describe('manifold quality', () => {
    it('provides unit-length contact normal', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
      const n = manifolds[0].normal
      const len = Math.sqrt(n.x * n.x + n.y * n.y)
      expect(len).toBeCloseTo(1, 4)
    })

    it('provides positive penetration depth', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
      expect(manifolds[0].contacts[0].penetration).toBeGreaterThan(0)
    })

    it('provides at least one contact point', () => {
      const buf = makeBuffers()
      placeBox(buf, 0, 0, 0, 1, 1)
      placeBox(buf, 1, 1.2, 0, 1, 1)
      const manifolds = gjkNarrowphase(buf, [{ a: 0 as any, b: 1 as any }])
      expect(manifolds.length).toBeGreaterThanOrEqual(1)
      expect(manifolds[0].contacts.length).toBeGreaterThanOrEqual(1)
    })
  })
})
