import type { MatchaBuffers } from '@matcha2d/types'

/**
 * Scene serialization — save/load world state.
 * TODO: Dev B implements JSON and binary serialization.
 */
export function serializeBuffers(_buffers: MatchaBuffers, _count: number): ArrayBuffer {
  return new ArrayBuffer(0)
}

export function deserializeBuffers(_data: ArrayBuffer): { buffers: MatchaBuffers; count: number } | null {
  return null
}
