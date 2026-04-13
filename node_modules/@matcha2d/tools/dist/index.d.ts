import { MatchaBuffers } from '@matcha2d/types';

/**
 * Scene serialization — save/load world state.
 * TODO: Dev B implements JSON and binary serialization.
 */
declare function serializeBuffers(_buffers: MatchaBuffers, _count: number): ArrayBuffer;
declare function deserializeBuffers(_data: ArrayBuffer): {
    buffers: MatchaBuffers;
    count: number;
} | null;

/**
 * Performance profiler for physics step timing.
 * TODO: Dev B implements — tracks broadphase, narrowphase, solver, integrate times.
 */
declare class Profiler {
    private _marks;
    begin(label: string): void;
    end(label: string): number;
}

export { Profiler, deserializeBuffers, serializeBuffers };
