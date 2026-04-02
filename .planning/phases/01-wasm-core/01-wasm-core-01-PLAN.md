---
phase: 01-wasm-core
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - packages/core/wasm/CMakeLists.txt
  - packages/core/wasm/wasm_bridge.c
  - packages/core/wasm/wasm_bridge.h
  - packages/core/wasm/build.sh
autonomous: true
requirements:
  - WASM-01
  - WASM-02
must_haves:
  truths:
    - "Box2D C source compiles to a .wasm file via Emscripten"
    - "WASM module exposes a C API that accepts SoA buffer pointers"
    - "Build script produces .wasm + .js glue in packages/core/dist/wasm/"
  artifacts:
    - path: "packages/core/wasm/CMakeLists.txt"
      provides: "Emscripten build config for Box2D + bridge"
      contains: "emscripten_add_link_options"
    - path: "packages/core/wasm/wasm_bridge.c"
      provides: "C API bridging SoA buffers to Box2D internals"
      exports: ["b2_init", "b2_step", "b2_collide"]
    - path: "packages/core/wasm/build.sh"
      provides: "One-command build script"
      contains: "emcmake cmake"
  key_links:
    - from: "packages/core/wasm/wasm_bridge.c"
      to: "box2d-main/src/*.c"
      via: "#include"
      pattern: "#include.*box2d"
---

<objective>
Set up Emscripten build pipeline for Box2D and create a C API bridge that accepts SoA-style buffer pointers from JavaScript.

Purpose: Compile Box2D's 58 C source files into a WASM module with an API surface that the TypeScript wrapper can call. The bridge layer translates Matcha2D's SoA buffer format into Box2D's internal body/shape representation.

Output: CMakeLists.txt, wasm_bridge.c/h, build.sh — all producing a .wasm + .js glue file.
</objective>

<execution_context>
@C:/Users/winsi/.config/opencode/get-shit-done/workflows/execute-plan.md
@C:/Users/winsi/.config/opencode/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/ROADMAP.md
@.planning/STATE.md
@packages/types/src/buffers.ts
@packages/types/src/backend.ts
@packages/core/src/solver/sequential-impulse.ts
@packages/core/src/collision/broadphase.ts
@box2d-main/CMakeLists.txt
@box2d-main/src/CMakeLists.txt
@box2d-main/build_emscripten.sh
@box2d-main/include/box2d/box2d.h
@box2d-main/include/box2d/types.h
@box2d-main/include/box2d/collision.h
@box2d-main/src/physics_world.c
@box2d-main/src/solver.c
@box2d-main/src/contact_solver.c
@box2d-main/src/broad_phase.c
@box2d-main/src/distance.c

# Key Box2D types from the public API:
# b2WorldId, b2BodyId, b2ShapeId — opaque handles
# b2Vec2 — {float x, float y}
# b2BodyDef — position, rotation, type, etc.
# b2ShapeDef — density, friction, restitution, filter
# b2Circle, b2Polygon, b2Capsule, b2Segment — shape geometry
# b2World_Step(worldId, timeStep, subStepCount) — main simulation step
# b2CreateWorld, b2DestroyWorld — world lifecycle
# b2CreateBody, b2DestroyBody — body lifecycle
# b2CreateCircleShape, b2CreatePolygonShape, etc. — shape creation

# Matcha2D buffer format (SoA — Structure of Arrays):
# positionX/Y, velocityX/Y, angle, angularVel, mass, invMass, inertia, invInertia, flags
# halfExtentX/Y, shapeType, shapeRadius, shapeVertexCount, shapeVerticesX/Y
# All indexed by body index (0..capacity-1)

# The bridge must:
# 1. Accept SoA buffer pointers from JS via WASM linear memory
# 2. Create/update Box2D bodies and shapes from buffer data
# 3. Step the Box2D world
# 4. Read results back into the SoA buffers
</context>

<tasks>

<task type="auto">
  <name>Task 1: Create Emscripten build configuration for Box2D</name>
  <files>packages/core/wasm/CMakeLists.txt, packages/core/wasm/build.sh</files>
  <action>
    Create `packages/core/wasm/CMakeLists.txt` that:
    1. Adds Box2D source files from `../../../box2d-main/src/` (all 58 .c files listed in box2d-main/src/CMakeLists.txt)
    2. Sets include directories to `../../../box2d-main/include/`
    3. Configures for Emscripten: set C_STANDARD 17, enable SIMD128 if available, disable BOX2D_VALIDATE for WASM size, disable BOX2D_SAMPLES and BOX2D_UNIT_TESTS
    4. Compiles the bridge file `wasm_bridge.c` alongside Box2D sources
    5. Outputs a shared library (.so/.wasm) with EXPORTED_FUNCTIONS and EXPORTED_RUNTIME_METHODS
    6. Uses `-s ALLOW_MEMORY_GROWTH=1`, `-s MODULARIZE=1`, `-s EXPORT_ES6=1`, `-s ENVIRONMENT='web'`

    Create `packages/core/wasm/build.sh` (bash script) that:
    1. Checks for emcc availability, prints helpful error if missing
    2. Creates build directory `packages/core/wasm/build/`
    3. Runs `emcmake cmake -DCMAKE_BUILD_TYPE=Release ..`
    4. Runs `cmake --build .`
    5. Copies output .wasm and .js to `packages/core/dist/wasm/`

    Also create `packages/core/wasm/build.ps1` (PowerShell equivalent for Windows).

    Reference box2d-main/src/CMakeLists.txt for the full list of source files. Do NOT modify any files inside box2d-main/.
  </action>
  <verify>
    <automated>Test: ls packages/core/wasm/CMakeLists.txt packages/core/wasm/build.sh packages/core/wasm/build.ps1 — all three files exist and contain expected content</automated>
  </verify>
  <done>CMakeLists.txt lists all 58 Box2D source files + wasm_bridge.c, build scripts exist for bash and PowerShell, no files in box2d-main/ were modified</done>
</task>

<task type="auto">
  <name>Task 2: Write the C WASM bridge API</name>
  <files>packages/core/wasm/wasm_bridge.h, packages/core/wasm/wasm_bridge.c</files>
  <action>
    Create `wasm_bridge.h` and `wasm_bridge.c` that expose these EMSCRIPTEN_KEEPALIVE functions:

    **World management:**
    - `int b2_init(float gravityX, float gravityY, int maxBodies)` — creates a Box2D world, returns world handle as int
    - `void b2_destroy(int worldHandle)` — destroys the world

    **Body/shape sync from SoA buffers (called each frame before step):**
    - `void b2_sync_bodies(int worldHandle, float* posX, float* posY, float* velX, float* velY, float* angle, float* angVel, float* mass, float* invMass, float* inertia, float* invInertia, uint8_t* flags, int count)` — reads SoA buffers, creates/updates Box2D bodies. For new bodies (flag ACTIVE, not yet created), call b2CreateBody. For existing bodies, update position/velocity. Track which JS body index maps to which Box2D body ID using a simple int array.
    - `void b2_sync_shapes(int worldHandle, float* posX, float* posY, float* angle, float* halfExtentX, float* halfExtentY, uint8_t* shapeType, float* shapeRadius, uint8_t* shapeVertexCount, float* shapeVerticesX, float* shapeVerticesY, int count)` — creates/updates Box2D shapes (circles, boxes, polygons) on the corresponding bodies.

    **Simulation step:**
    - `void b2_step(int worldHandle, float dt, int subSteps)` — calls b2World_Step

    **Read results back into SoA buffers (called after step):**
    - `void b2_read_bodies(int worldHandle, float* posX, float* posY, float* velX, float* velY, float* angle, float* angVel, int count)` — reads Box2D body state back into SoA buffers

    **Collision query:**
    - `int b2_get_contact_count(int worldHandle)` — returns number of active contacts
    - `void b2_get_contacts(int worldHandle, int* bodyA, int* bodyB, float* nx, float* ny, float* px, float* py, float* penetration, int maxContacts)` — fills arrays with contact data

    Implementation approach:
    - Use a simple fixed-size array `b2BodyId js_to_b2[MAX_BODIES]` to map JS body indices to Box2D body IDs
    - On sync, iterate through all active bodies, create Box2D bodies if they don't exist yet, update transforms/velocities for existing ones
    - On read, iterate through Box2D bodies and write back to the SoA buffer positions
    - Keep it simple: recreate shapes each sync if shape data changed (acceptable for initial version)
    - Use b2DefaultWorldDef(), b2DefaultBodyDef(), b2DefaultShapeDef() for sensible defaults
    - Include `#include "box2d/box2d.h"` from the Box2D include path

    Do NOT modify any files inside box2d-main/.
  </action>
  <verify>
    <automated>Test: ls packages/core/wasm/wasm_bridge.h packages/core/wasm/wasm_bridge.c — both exist, header declares all 7 functions with EMSCRIPTEN_KEEPALIVE, .c file includes box2d/box2d.h and emscripten.h</automated>
  </verify>
  <done>Bridge header declares all 7 EMSCRIPTEN_KEEPALIVE functions, implementation file has stubs or full implementations that compile against Box2D headers, no files in box2d-main/ were modified</done>
</task>

</tasks>

<verification>
- CMakeLists.txt correctly references all Box2D source files from box2d-main/src/
- Build scripts (bash + PowerShell) are executable and reference correct paths
- wasm_bridge.h declares all required functions with EMSCRIPTEN_KEEPALIVE
- wasm_bridge.c implements the bridge logic without modifying box2d-main/
- No files inside box2d-main/ directory are modified (reference only)
</verification>

<success_criteria>
- Build configuration files exist and are syntactically valid CMake
- C bridge API header and implementation exist with all 7 required functions
- All Box2D source files referenced in CMakeLists.txt (58 files from box2d-main/src/)
- Build scripts handle both Unix and Windows environments
- Zero modifications to box2d-main/ directory
</success_criteria>

<output>
After completion, create `.planning/phases/01-wasm-core/01-wasm-core-01-SUMMARY.md`
</output>
