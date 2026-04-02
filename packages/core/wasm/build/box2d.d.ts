/**
 * Type declarations for the Emscripten-generated Box2D WASM glue module.
 *
 * This file provides types for the JS glue file at build/box2d.js.
 * When the WASM build runs, Emscripten overwrites box2d.js but this
 * .d.ts file remains to provide type information.
 */

export interface EmscriptenModule {
  HEAPF32: Float32Array
  HEAPU8: Uint8Array
  HEAP32: Int32Array
  cwrap(
    name: string,
    returnType: string | null,
    argTypes: string[],
  ): (...args: unknown[]) => unknown
  _malloc(size: number): number
  _free(ptr: number): void
}

export type EmscriptenModuleFactory = () => Promise<EmscriptenModule>

declare const _default: EmscriptenModuleFactory
export default _default
