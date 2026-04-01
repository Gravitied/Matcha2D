/**
 * A BodyHandle is an opaque index into the MatchaBuffers arrays.
 * Branded type prevents accidental misuse of raw numbers.
 */
export type BodyHandle = number & { readonly __brand: 'BodyHandle' }

/** Bit flags stored in MatchaBuffers.flags */
export const BodyFlags = {
  ACTIVE: 0b0000_0001,
  STATIC: 0b0000_0010,
  SLEEPING: 0b0000_0100,
  SENSOR: 0b0000_1000,
} as const

export const BodyType = {
  Dynamic: 0,
  Static: 1,
  Kinematic: 2,
} as const

export type BodyType = (typeof BodyType)[keyof typeof BodyType]
