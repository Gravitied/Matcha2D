/**
 * Public API for the WASM physics backend.
 *
 * Re-exports the WASM module loader and the PhysicsBackend implementation
 * that wraps the Emscripten-compiled Box2D engine.
 */

export { WasmPhysicsBackend } from './WasmPhysicsBackend.js'
export { loadWasmModule } from './WasmModule.js'
export type { WasmInstance } from './WasmModule.js'
