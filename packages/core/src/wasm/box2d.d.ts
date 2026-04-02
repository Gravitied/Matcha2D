/**
 * Type declarations for the Emscripten-generated Box2D WASM glue module.
 *
 * The actual JS file is built at packages/core/wasm/build/box2d.js
 * by the Emscripten build pipeline (MODULARIZE=1, EXPORT_ES6=1).
 * This file exists at build time but not during development.
 */

// Ambient module declaration for the Emscripten glue file.
// The relative path matches the import in WasmModule.ts.
declare module '../../wasm/build/box2d.js' {
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

  const factory: EmscriptenModuleFactory
  export default factory
}
