/**
 * One collider outline in **world space** (meters, Y-up), matching Rust narrowphase
 * `collider_world_transform` + local shape geometry.
 */
export interface NarrowphaseColliderOutline {
  colliderHandle: number
  /** Parent rigid body handle, or `null` if unparented */
  bodyHandle: number | null
  shapeKind: 'ball' | 'cuboid' | 'polygon'
  sensor: boolean
  /** Loop vertices in world meters; winding matches engine (CCW for poly/cuboid in collider space) */
  vertices: Array<{ x: number; y: number }>
}

/**
 * Parse WASM `PhysicsEngine.debug_narrowphase_outlines()` flat buffer.
 *
 * Layout: `[entry_count,` then per collider:
 * `collider_handle, body_handle, shape_kind, flags, point_count,` then `point_count` (x,y) pairs.
 * - `shape_kind`: 0 ball, 1 cuboid, 2 polygon
 * - `flags`: 1 = sensor
 * - `body_handle`: NaN means no parent
 *
 * **Note:** handles are encoded as f32; values above ~2^24 may round. Typical Matcha handles are safe.
 */
export function parseNarrowphaseOutlinesFlat(data: number[] | Float32Array): NarrowphaseColliderOutline[] {
  const f = data instanceof Float32Array ? data : Float32Array.from(data)
  if (f.length < 1) return []
  const entryCount = Math.floor(f[0])
  let i = 1
  const out: NarrowphaseColliderOutline[] = []
  for (let e = 0; e < entryCount; e++) {
    if (i + 5 > f.length) break
    const colliderHandle = Math.round(f[i++])
    const bodyRaw = f[i++]
    const bodyHandle = Number.isFinite(bodyRaw) ? Math.round(bodyRaw) : null
    const shapeKindNum = f[i++]
    const flags = f[i++]
    const pointCount = Math.floor(f[i++])
    const need = pointCount * 2
    if (pointCount < 0 || i + need > f.length) break
    const vertices: Array<{ x: number; y: number }> = []
    for (let p = 0; p < pointCount; p++) {
      vertices.push({ x: f[i++], y: f[i++] })
    }
    const shapeKind: NarrowphaseColliderOutline['shapeKind'] =
      shapeKindNum === 0 ? 'ball' : shapeKindNum === 1 ? 'cuboid' : 'polygon'
    out.push({
      colliderHandle,
      bodyHandle,
      shapeKind,
      sensor: flags !== 0,
      vertices,
    })
  }
  return out
}
