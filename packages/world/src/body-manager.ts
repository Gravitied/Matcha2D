import type { MatchaBuffers, BodyHandle } from '@matcha2d/types'

/**
 * Manages flat-array body storage: allocation, deletion, compaction.
 * TODO: Dev B implements free-list allocation, defragmentation.
 */
export class BodyManager {
  private _count = 0

  constructor(readonly buffers: MatchaBuffers) {}

  get count(): number {
    return this._count
  }

  allocate(): BodyHandle {
    return this._count++ as BodyHandle
  }
}
