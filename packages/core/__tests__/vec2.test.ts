import { describe, it, expect } from 'vitest'
import {
  vec2Set, vec2Add, vec2Sub, vec2Scale,
  vec2Dot, vec2Cross, vec2LengthSq, vec2Length,
  vec2Normalize, vec2DistanceSq,
  dot, cross,
} from '../src/math/vec2.js'

describe('vec2 flat-array operations', () => {
  it('vec2Set writes values at index', () => {
    const x = new Float32Array(4)
    const y = new Float32Array(4)
    vec2Set(x, y, 2, 3.5, -1.0)
    expect(x[2]).toBe(3.5)
    expect(y[2]).toBe(-1.0)
  })

  it('vec2Add sums two vectors', () => {
    const ax = new Float32Array([1, 2])
    const ay = new Float32Array([3, 4])
    const bx = new Float32Array([5, 6])
    const by = new Float32Array([7, 8])
    const ox = new Float32Array(2)
    const oy = new Float32Array(2)

    vec2Add(ox, oy, 0, ax, ay, 0, bx, by, 0)
    expect(ox[0]).toBe(6)
    expect(oy[0]).toBe(10)

    vec2Add(ox, oy, 1, ax, ay, 1, bx, by, 1)
    expect(ox[1]).toBe(8)
    expect(oy[1]).toBe(12)
  })

  it('vec2Sub subtracts two vectors', () => {
    const ax = new Float32Array([5])
    const ay = new Float32Array([7])
    const bx = new Float32Array([2])
    const by = new Float32Array([3])
    const ox = new Float32Array(1)
    const oy = new Float32Array(1)

    vec2Sub(ox, oy, 0, ax, ay, 0, bx, by, 0)
    expect(ox[0]).toBe(3)
    expect(oy[0]).toBe(4)
  })

  it('vec2Scale multiplies by scalar', () => {
    const ax = new Float32Array([3])
    const ay = new Float32Array([4])
    const ox = new Float32Array(1)
    const oy = new Float32Array(1)

    vec2Scale(ox, oy, 0, ax, ay, 0, 2)
    expect(ox[0]).toBe(6)
    expect(oy[0]).toBe(8)
  })

  it('vec2Dot computes dot product', () => {
    const ax = new Float32Array([1])
    const ay = new Float32Array([2])
    const bx = new Float32Array([3])
    const by = new Float32Array([4])

    expect(vec2Dot(ax, ay, 0, bx, by, 0)).toBe(11)
  })

  it('vec2Cross computes 2D cross product', () => {
    const ax = new Float32Array([1])
    const ay = new Float32Array([0])
    const bx = new Float32Array([0])
    const by = new Float32Array([1])

    expect(vec2Cross(ax, ay, 0, bx, by, 0)).toBe(1)
  })

  it('vec2LengthSq and vec2Length', () => {
    const x = new Float32Array([3])
    const y = new Float32Array([4])

    expect(vec2LengthSq(x, y, 0)).toBe(25)
    expect(vec2Length(x, y, 0)).toBe(5)
  })

  it('vec2Normalize produces unit vector', () => {
    const ax = new Float32Array([3])
    const ay = new Float32Array([4])
    const ox = new Float32Array(1)
    const oy = new Float32Array(1)

    vec2Normalize(ox, oy, 0, ax, ay, 0)
    expect(ox[0]).toBeCloseTo(0.6)
    expect(oy[0]).toBeCloseTo(0.8)
  })

  it('vec2Normalize handles zero vector', () => {
    const ax = new Float32Array([0])
    const ay = new Float32Array([0])
    const ox = new Float32Array(1)
    const oy = new Float32Array(1)

    vec2Normalize(ox, oy, 0, ax, ay, 0)
    expect(ox[0]).toBe(0)
    expect(oy[0]).toBe(0)
  })

  it('vec2DistanceSq computes squared distance', () => {
    const ax = new Float32Array([1])
    const ay = new Float32Array([2])
    const bx = new Float32Array([4])
    const by = new Float32Array([6])

    expect(vec2DistanceSq(ax, ay, 0, bx, by, 0)).toBe(25)
  })
})

describe('scalar helpers', () => {
  it('dot product', () => {
    expect(dot(1, 2, 3, 4)).toBe(11)
  })

  it('cross product', () => {
    expect(cross(1, 0, 0, 1)).toBe(1)
  })
})
