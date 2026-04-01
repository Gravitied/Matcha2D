/**
 * Performance profiler for physics step timing.
 * TODO: Dev B implements — tracks broadphase, narrowphase, solver, integrate times.
 */
export class Profiler {
  private _marks = new Map<string, number>()

  begin(label: string): void {
    this._marks.set(label, performance.now())
  }

  end(label: string): number {
    const start = this._marks.get(label)
    if (start === undefined) return 0
    const elapsed = performance.now() - start
    this._marks.delete(label)
    return elapsed
  }
}
