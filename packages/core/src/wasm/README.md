# WASM Physics Core

This directory will contain Rust → WASM bindings for the physics core.

The Rust port will replace the TypeScript implementations in `../math/`, `../collision/`, and `../solver/` with WASM equivalents that operate on shared memory buffers.

## Build Prerequisites (future)
- Rust toolchain with `wasm32-unknown-unknown` target
- `wasm-pack` for generating JS/TS bindings
