# WASM Physics Core

Box2D compiled to WASM via Emscripten, with TypeScript wrappers that implement the `PhysicsBackend` interface.

## Architecture

```
World (TS)
  └─ WasmPhysicsBackend (TS) — implements PhysicsBackend
       └─ WasmModule (TS) — typed wrappers around C bridge
            └─ box2d.wasm — Emscripten-compiled Box2D
                 └─ wasm_bridge.c — C API (SoA ↔ Box2D)
                      └─ box2d/ — Box2D 3.x C source
```

## Directory Structure

```
src/wasm/
  ├── WasmModule.ts          — WASM loader, memory management, typed cwrap
  ├── WasmPhysicsBackend.ts  — PhysicsBackend implementation
  ├── index.ts               — Public exports
  └── box2d.d.ts             — Type declarations for Emscripten glue

wasm/
  ├── CMakeLists.txt         — Emscripten build configuration
  ├── build.sh               — Unix build script
  ├── build.ps1              — Windows build script
  ├── wasm_bridge.h          — C API header (8 exported functions)
  ├── wasm_bridge.c          — C bridge implementation
  └── build/
       ├── box2d.js          — Emscripten JS glue (generated)
       └── box2d.wasm        — WASM binary (generated)
```

## Build Prerequisites

- [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) installed and activated
  ```bash
  git clone https://github.com/emscripten-core/emsdk.git
  cd emsdk && ./emsdk install latest && ./emsdk activate latest
  source ./emsdk_env.sh  # or emsdk_env.bat on Windows
  ```

## Building the WASM Module

### Unix / macOS
```bash
cd packages/core/wasm
./build.sh
```

### Windows (PowerShell)
```powershell
cd packages/core/wasm
.\build.ps1
```

This produces:
- `wasm/build/box2d.js` — Emscripten JS glue module (ESM, modularized)
- `wasm/build/box2d.wasm` — WASM binary with SIMD128 support

## TypeScript API

```typescript
import { WasmPhysicsBackend, loadWasmModule } from '@matcha2d/core/wasm'

// Option 1: Use via PhysicsBackend interface
const backend = new WasmPhysicsBackend()
await backend.init({ gravity: { x: 0, y: -9.81 }, ...config })
const contacts = backend.collide(buffers, count)
backend.dispose()

// Option 2: Use WASM module directly
const wasm = await loadWasmModule()
const world = wasm.init(0, -9.81, 8192)
wasm.syncBodies(world, buffers, count)
wasm.step(world, 1/60, 4)
wasm.readBodies(world, buffers, count)
const manifolds = wasm.getContacts(world)
wasm.destroy(world)
wasm.dispose()
```

## C Bridge API

The WASM module exposes these functions (from `wasm_bridge.h`):

| Function | Purpose |
|----------|---------|
| `b2_init(gx, gy, max)` | Create Box2D world, return handle |
| `b2_destroy(handle)` | Destroy world, free memory |
| `b2_sync_bodies(handle, ...)` | Push SoA body data to WASM |
| `b2_sync_shapes(handle, ...)` | Push SoA shape data to WASM |
| `b2_step(handle, dt, steps)` | Advance simulation |
| `b2_read_bodies(handle, ...)` | Read updated state from WASM |
| `b2_get_contact_count(handle)` | Get number of active contacts |
| `b2_get_contacts(handle, ...)` | Read contact manifold data |

## Design Notes

- **Unified step**: Box2D's `b2World_Step` handles broadphase → narrowphase → solve → integrate internally. The WASM backend's `collide()` method is the primary entry point; individual methods (`broadphase`, `narrowphase`, `solveVelocity`, `integrate`, `solvePosition`) are no-ops.
- **Memory management**: SoA buffers are copied to WASM heap via `malloc` + `HEAPF32.set()`, then freed after each call. The `dispose()` method cleans up the world and any persistent allocations.
- **SoA → AoS translation**: The C bridge (`wasm_bridge.c`) translates Matcha2D's Structure-of-Arrays format to Box2D's object-per-body model on each sync/read cycle.
