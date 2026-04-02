import { describe, it, expect } from 'vitest'
import { broadphase, broadphaseBVH, DynamicTree } from '../src/collision/broadphase.js'
import { createBuffers, BodyFlags, ShapeType } from '@matcha2d/types'

function makeBuffers(bodies: Array<{
  x: number; y: number; hx: number; hy: number; flags?: number
}>) {
  const buf = createBuffers(bodies.length)
  for (let i = 0; i < bodies.length; i++) {
    const b = bodies[i]
    buf.positionX[i] = b.x
    buf.positionY[i] = b.y
    buf.halfExtentX[i] = b.hx
    buf.halfExtentY[i] = b.hy
    buf.flags[i] = b.flags ?? BodyFlags.ACTIVE
  }
  return buf
}

function hasPair(pairs: { a: number; b: number }[], a: number, b: number) {
  return pairs.some(p => (p.a === a && p.b === b) || (p.a === b && p.b === a))
}

describe('broadphase — sort-and-sweep', () => {
  it('returns no pairs for a single body', () => {
    const buf = makeBuffers([{ x: 0, y: 0, hx: 1, hy: 1 }])
    expect(broadphase(buf, 1, 'sap')).toHaveLength(0)
  })

  it('returns no pairs when bodies are fully separated', () => {
    const buf = makeBuffers([
      { x: 0,  y: 0, hx: 1, hy: 1 },
      { x: 10, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphase(buf, 2, 'sap')).toHaveLength(0)
  })

  it('detects an overlapping pair', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    const pairs = broadphase(buf, 2, 'sap')
    expect(pairs).toHaveLength(1)
    expect(hasPair(pairs, 0, 1)).toBe(true)
  })

  it('separates on Y even when X intervals overlap', () => {
    const buf = makeBuffers([
      { x: 0, y: 0,  hx: 1, hy: 1 },
      { x: 0, y: 10, hx: 1, hy: 1 },
    ])
    expect(broadphase(buf, 2, 'sap')).toHaveLength(0)
  })

  it('skips inactive bodies', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1, flags: 0 },
      { x: 0, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphase(buf, 2, 'sap')).toHaveLength(0)
  })

  it('skips static-static pairs', () => {
    const staticFlags = BodyFlags.ACTIVE | BodyFlags.STATIC
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
    ])
    expect(broadphase(buf, 2, 'sap')).toHaveLength(0)
  })

  it('includes dynamic-static pairs', () => {
    const staticFlags = BodyFlags.ACTIVE | BodyFlags.STATIC
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
    ])
    const pairs = broadphase(buf, 2, 'sap')
    expect(pairs).toHaveLength(1)
    expect(hasPair(pairs, 0, 1)).toBe(true)
  })

  it('handles three bodies with two overlapping pairs', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1.5, hy: 1 },
      { x: 2, y: 0, hx: 1.5, hy: 1 },
      { x: 4, y: 0, hx: 1.5, hy: 1 },
    ])
    const pairs = broadphase(buf, 3, 'sap')
    expect(hasPair(pairs, 0, 1)).toBe(true)
    expect(hasPair(pairs, 1, 2)).toBe(true)
    expect(hasPair(pairs, 0, 2)).toBe(false)
  })

  it('detects touching edges as overlapping', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 2, y: 0, hx: 1, hy: 1 },
    ])
    const pairs = broadphase(buf, 2, 'sap')
    expect(pairs).toHaveLength(1)
  })
})

describe('broadphase — BVH', () => {
  it('returns no pairs for a single body', () => {
    const buf = makeBuffers([{ x: 0, y: 0, hx: 1, hy: 1 }])
    expect(broadphaseBVH(buf, 1)).toHaveLength(0)
  })

  it('returns no pairs when bodies are fully separated', () => {
    const buf = makeBuffers([
      { x: 0,  y: 0, hx: 1, hy: 1 },
      { x: 10, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphaseBVH(buf, 2)).toHaveLength(0)
  })

  it('detects an overlapping pair', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    const pairs = broadphaseBVH(buf, 2)
    expect(pairs).toHaveLength(1)
    expect(hasPair(pairs, 0, 1)).toBe(true)
  })

  it('separates on Y even when X intervals overlap', () => {
    const buf = makeBuffers([
      { x: 0, y: 0,  hx: 1, hy: 1 },
      { x: 0, y: 10, hx: 1, hy: 1 },
    ])
    expect(broadphaseBVH(buf, 2)).toHaveLength(0)
  })

  it('skips inactive bodies', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1, flags: 0 },
      { x: 0, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphaseBVH(buf, 2)).toHaveLength(0)
  })

  it('skips static-static pairs', () => {
    const staticFlags = BodyFlags.ACTIVE | BodyFlags.STATIC
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
    ])
    expect(broadphaseBVH(buf, 2)).toHaveLength(0)
  })

  it('includes dynamic-static pairs', () => {
    const staticFlags = BodyFlags.ACTIVE | BodyFlags.STATIC
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 0, y: 0, hx: 1, hy: 1, flags: staticFlags },
    ])
    const pairs = broadphaseBVH(buf, 2)
    expect(pairs).toHaveLength(1)
    expect(hasPair(pairs, 0, 1)).toBe(true)
  })

  it('handles three bodies with two overlapping pairs', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1.5, hy: 1 },
      { x: 2, y: 0, hx: 1.5, hy: 1 },
      { x: 4, y: 0, hx: 1.5, hy: 1 },
    ])
    const pairs = broadphaseBVH(buf, 3)
    expect(hasPair(pairs, 0, 1)).toBe(true)
    expect(hasPair(pairs, 1, 2)).toBe(true)
    expect(hasPair(pairs, 0, 2)).toBe(false)
  })

  it('produces the same pairs as SAP for a random scene', () => {
    const bodies = [
      { x: 0,  y: 0,  hx: 1, hy: 1 },
      { x: 1,  y: 0,  hx: 1, hy: 1 },
      { x: 5,  y: 5,  hx: 2, hy: 2 },
      { x: 6,  y: 6,  hx: 2, hy: 2 },
      { x: 20, y: 0,  hx: 1, hy: 1 },
      { x: 20, y: 0,  hx: 1, hy: 1 },
      { x: 0,  y: 20, hx: 1, hy: 1 },
      { x: 0,  y: 20, hx: 1, hy: 1 },
    ]
    const buf = makeBuffers(bodies)
    const sapPairs = broadphase(buf, bodies.length, 'sap')
    const bvhPairs = broadphaseBVH(buf, bodies.length)

    expect(bvhPairs).toHaveLength(sapPairs.length)
    for (const p of sapPairs) {
      expect(hasPair(bvhPairs, p.a, p.b)).toBe(true)
    }
  })
})

describe('broadphase — dynamic tree', () => {
  it('returns no pairs for a single body', () => {
    const buf = makeBuffers([{ x: 0, y: 0, hx: 1, hy: 1 }])
    const tree = new DynamicTree()
    tree.insert(0, buf)
    expect(tree.queryPairs(buf)).toHaveLength(0)
  })

  it('detects an overlapping pair', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)
    const pairs = tree.queryPairs(buf)
    expect(pairs.length).toBeGreaterThanOrEqual(1)
  })

  it('removes bodies correctly', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)
    tree.remove(0)
    expect(tree.queryPairs(buf)).toHaveLength(0)
  })

  it('updates bodies when they move', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 10, y: 0, hx: 1, hy: 1 },
    ])
    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)
    expect(tree.queryPairs(buf)).toHaveLength(0)

    buf.positionX[1] = 1
    tree.update(1, buf)
    expect(tree.queryPairs(buf).length).toBeGreaterThanOrEqual(1)
  })
})

describe('broadphase — dispatch via method param', () => {
  it("routes to SAP when method is 'sap'", () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphase(buf, 2, 'sap')).toHaveLength(1)
  })

  it("routes to BVH when method is 'bvh'", () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    expect(broadphase(buf, 2, 'bvh')).toHaveLength(1)
  })

  it('works with dynamic tree', () => {
    const buf = makeBuffers([
      { x: 0, y: 0, hx: 1, hy: 1 },
      { x: 1, y: 0, hx: 1, hy: 1 },
    ])
    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)
    expect(broadphase(buf, 2, tree, 'dynamicTree').length).toBeGreaterThanOrEqual(1)
  })
})

describe('DynamicTree', () => {
  it('does not emit static-static pairs', () => {
    const buf = createBuffers(8)
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC

    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.STATIC

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(0)
  })

  it('emits dynamic-static pairs', () => {
    const buf = createBuffers(8)
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.STATIC

    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(1)
  })

  it('does not emit sleeping-sleeping pairs', () => {
    const buf = createBuffers(8)
    buf.positionX[0] = 0; buf.positionY[0] = 0
    buf.halfExtentX[0] = 1; buf.halfExtentY[0] = 1
    buf.shapeType[0] = ShapeType.Box
    buf.flags[0] = BodyFlags.ACTIVE | BodyFlags.SLEEPING

    buf.positionX[1] = 0.5; buf.positionY[1] = 0
    buf.halfExtentX[1] = 1; buf.halfExtentY[1] = 1
    buf.shapeType[1] = ShapeType.Box
    buf.flags[1] = BodyFlags.ACTIVE | BodyFlags.SLEEPING

    const tree = new DynamicTree()
    tree.insert(0, buf)
    tree.insert(1, buf)

    const pairs = tree.queryPairs(buf)
    expect(pairs).toHaveLength(0)
  })
})
