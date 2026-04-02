# Matcha2D WASM Bridge — Emscripten build configuration
# Compiles Box2D C sources + wasm_bridge into a single .wasm module

cmake_minimum_required(VERSION 3.22)
project(matcha2d_wasm LANGUAGES C)

# ── Emscripten-specific settings ──────────────────────────────────────────────
if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "This CMakeLists.txt is for Emscripten only. Use emcmake cmake.")
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED YES)
set(CMAKE_C_EXTENSIONS YES)

# ── Box2D source files (from box2d-main/src/) ─────────────────────────────────
set(BOX2D_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../box2d-main/src")
set(BOX2D_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../box2d-main/include")

set(BOX2D_SOURCES
  ${BOX2D_SRC_DIR}/aabb.c
  ${BOX2D_SRC_DIR}/arena_allocator.c
  ${BOX2D_SRC_DIR}/array.c
  ${BOX2D_SRC_DIR}/bitset.c
  ${BOX2D_SRC_DIR}/body.c
  ${BOX2D_SRC_DIR}/broad_phase.c
  ${BOX2D_SRC_DIR}/constraint_graph.c
  ${BOX2D_SRC_DIR}/contact.c
  ${BOX2D_SRC_DIR}/contact_solver.c
  ${BOX2D_SRC_DIR}/core.c
  ${BOX2D_SRC_DIR}/distance.c
  ${BOX2D_SRC_DIR}/distance_joint.c
  ${BOX2D_SRC_DIR}/dynamic_tree.c
  ${BOX2D_SRC_DIR}/geometry.c
  ${BOX2D_SRC_DIR}/hull.c
  ${BOX2D_SRC_DIR}/id_pool.c
  ${BOX2D_SRC_DIR}/island.c
  ${BOX2D_SRC_DIR}/joint.c
  ${BOX2D_SRC_DIR}/manifold.c
  ${BOX2D_SRC_DIR}/math_functions.c
  ${BOX2D_SRC_DIR}/motor_joint.c
  ${BOX2D_SRC_DIR}/mover.c
  ${BOX2D_SRC_DIR}/physics_world.c
  ${BOX2D_SRC_DIR}/prismatic_joint.c
  ${BOX2D_SRC_DIR}/revolute_joint.c
  ${BOX2D_SRC_DIR}/sensor.c
  ${BOX2D_SRC_DIR}/shape.c
  ${BOX2D_SRC_DIR}/solver.c
  ${BOX2D_SRC_DIR}/solver_set.c
  ${BOX2D_SRC_DIR}/table.c
  ${BOX2D_SRC_DIR}/timer.c
  ${BOX2D_SRC_DIR}/types.c
  ${BOX2D_SRC_DIR}/weld_joint.c
  ${BOX2D_SRC_DIR}/wheel_joint.c
)

# ── Bridge source ──────────────────────────────────────────────────────────────
set(BRIDGE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_bridge.c
)

# ── Combined target ────────────────────────────────────────────────────────────
set(ALL_SOURCES ${BOX2D_SOURCES} ${BRIDGE_SOURCES})

# ── Emscripten link options ────────────────────────────────────────────────────
set(EXPORTED_FUNCTIONS
  "_b2_init",
  "_b2_destroy",
  "_b2_sync_bodies",
  "_b2_sync_shapes",
  "_b2_step",
  "_b2_read_bodies",
  "_b2_get_contact_count",
  "_b2_get_contacts"
)

set(EXPORTED_RUNTIME_METHODS
  "ccall",
  "cwrap",
  "HEAPF32",
  "HEAPU8",
  "HEAP32"
)

set(EM_LINK_FLAGS
  "-s ALLOW_MEMORY_GROWTH=1"
  "-s MODULARIZE=1"
  "-s EXPORT_ES6=1"
  "-s ENVIRONMENT='web'"
  "-s EXPORTED_FUNCTIONS=${EXPORTED_FUNCTIONS}"
  "-s EXPORTED_RUNTIME_METHODS=${EXPORTED_RUNTIME_METHODS}"
  "-s SINGLE_FILE=0"
  "-s WASM=1"
  "-O3"
)

# Disable Box2D validation for WASM size
add_compile_definitions(BOX2D_VALIDATE=0)

# ── Build the WASM module ─────────────────────────────────────────────────────
add_executable(matcha2d_wasm ${ALL_SOURCES})

target_include_directories(matcha2d_wasm
  PRIVATE
    ${BOX2D_INC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(matcha2d_wasm PRIVATE
  -msimd128
  -msse2
)

set_target_properties(matcha2d_wasm PROPERTIES
  SUFFIX ".js"
  LINK_FLAGS "${EM_LINK_FLAGS}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build"
)

message(STATUS "Matcha2D WASM: ${CMAKE_C_COMPILER_ID}")
message(STATUS "Box2D sources: ${CMAKE_CURRENT_SOURCE_DIR}/../../../box2d-main/src/")
message(STATUS "Bridge source: ${CMAKE_CURRENT_SOURCE_DIR}/wasm_bridge.c")
