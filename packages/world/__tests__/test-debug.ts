/**
 * Bulletproof test diagnostics: structured logs + failure bundles so a red test
 * points to a concrete next debugging step (no vague "expected truthy").
 */
import type { BodyHandle } from '@matcha2d/types'
import type { World } from '../src/world.js'

export interface BodySnapshot {
  handle: string
  position: [number, number]
  velocity: [number, number]
  angle: number
  angularVelocity: number
}

export function snapshotBody(world: World, handle: BodyHandle, label: string): BodySnapshot {
  return {
    handle: label,
    position: [...world.getBodyPosition(handle)] as [number, number],
    velocity: [...world.getBodyVelocity(handle)] as [number, number],
    angle: world.getBodyAngle(handle),
    angularVelocity: world.getBodyAngularVelocity(handle),
  }
}

export function formatBodySnapshot(s: BodySnapshot): string {
  return (
    `${s.handle}: pos=(${s.position[0]}, ${s.position[1]}) vel=(${s.velocity[0]}, ${s.velocity[1]}) ` +
    `ang=${s.angle} ω=${s.angularVelocity}`
  )
}

/** Full engine-facing dump (Rust narrowphase + solver state). */
export function formatWorldDump(world: World): string {
  return world.debugDump
}

export function logWorldBanner(title: string, extra?: Record<string, unknown>): void {
  const line = '='.repeat(72)
  console.error(`\n${line}\n[Matcha2D test] ${title}\n${line}`)
  if (extra) {
    for (const [k, v] of Object.entries(extra)) {
      console.error(`  ${k}: ${typeof v === 'object' ? JSON.stringify(v) : String(v)}`)
    }
  }
}

export function logWorldMetrics(world: World, step?: number): void {
  const prefix = step === undefined ? '' : `step=${step} `
  console.error(
    `${prefix}bodyCount=${world.bodyCount} colliderCount=${world.colliderCount} ` +
      `contactPairs=${world.contactPairCount} contactPoints=${world.contactPointCount} ` +
      `maxPenetration=${world.maxPenetration} renderAlpha=${world.renderAlpha}`,
  )
}

export function logBodySnapshots(label: string, snapshots: BodySnapshot[]): void {
  console.error(`\n--- ${label} ---`)
  for (const s of snapshots) {
    console.error(formatBodySnapshot(s))
  }
}

/**
 * When an invariant fails, call this before `expect.fail` / `throw` so CI logs
 * contain everything needed to bisect (WASM vs TS vs scene).
 */
export function logFailureBundle(
  world: World,
  ctx: {
    testName: string
    step?: number
    reason: string
    bodies?: BodySnapshot[]
    hints: string[]
  },
): void {
  logWorldBanner(`FAIL: ${ctx.testName}`, { reason: ctx.reason, step: ctx.step })
  logWorldMetrics(world, ctx.step)
  if (ctx.bodies?.length) {
    logBodySnapshots('Bodies', ctx.bodies)
  }
  console.error('\n--- Rust debug_dump() ---\n' + formatWorldDump(world))
  console.error('\n--- What to do next ---')
  for (const h of ctx.hints) {
    console.error(`  • ${h}`)
  }
  console.error('')
}

export function assertFinite2(name: string, v: [number, number], world: World, hints: string[]): void {
  if (!Number.isFinite(v[0]) || !Number.isFinite(v[1])) {
    logFailureBundle(world, {
      testName: name,
      reason: `Non-finite vector component in ${name}: (${v[0]}, ${v[1]})`,
      hints: [
        'Inspect Rust panic hook / wasm `unreachable` in recent engine changes.',
        'Re-run with `vitest run --reporter=verbose` and capture full stderr.',
        ...hints,
      ],
    })
    throw new Error(`[${name}] expected finite (x,y), got (${v[0]}, ${v[1]})`)
  }
}

export function assertFiniteScalar(name: string, x: number, world: World, hints: string[]): void {
  if (!Number.isFinite(x)) {
    logFailureBundle(world, {
      testName: name,
      reason: `Non-finite scalar in ${name}: ${x}`,
      hints: ['Check angular velocity integration and collision responses.', ...hints],
    })
    throw new Error(`[${name}] expected finite scalar, got ${x}`)
  }
}

export function linSpeed(world: World, handle: BodyHandle): number {
  const [vx, vy] = world.getBodyVelocity(handle)
  return Math.hypot(vx, vy)
}

export function maxLinSpeedOver(world: World, handles: BodyHandle[]): number {
  let m = 0
  for (const h of handles) {
    m = Math.max(m, linSpeed(world, h))
  }
  return m
}

export function maxAbsAngularSpeed(world: World, handles: BodyHandle[]): number {
  let m = 0
  for (const h of handles) {
    m = Math.max(m, Math.abs(world.getBodyAngularVelocity(h)))
  }
  return m
}

/**
 * Engine contract: after `set_max_integrate_velocities(maxLin, maxAng)`, no dynamic body should
 * exceed those magnitudes post-step (integration clamps in `solver_body::integrate_positions`).
 * `slack` is only for float noise at the clamp boundary — not a physics fudge factor.
 */
export function assertIntegrateVelocityCap(
  world: World,
  handles: BodyHandle[],
  maxLin: number,
  maxAng: number,
  slack: number,
  step: number,
  testName: string,
): void {
  const maxS = maxLinSpeedOver(world, handles)
  const maxW = maxAbsAngularSpeed(world, handles)
  if (maxS > maxLin + slack || maxW > maxAng + slack) {
    const snaps = handles.map((h, i) => snapshotBody(world, h, `body_${i}`))
    logFailureBundle(world, {
      testName,
      step,
      reason: `integrate cap violated: max|v|=${maxS} (cap ${maxLin}), max|ω|=${maxW} (cap ${maxAng}), slack=${slack}`,
      bodies: snaps,
      hints: [
        'Search `integrate_positions` / `max_integrate_linear_velocity` — clamp skipped or overwritten after solve.',
        'Check for NaN sanitization masking explosions (vel reset to zero then next frame spikes).',
      ],
    })
    throw new Error(`${testName}: velocity above integrate cap at step ${step}`)
  }
}
