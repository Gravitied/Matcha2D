// Math
export {
  vec2Set, vec2Add, vec2Sub, vec2Scale,
  vec2Dot, vec2Cross, vec2LengthSq, vec2Length,
  vec2Normalize, vec2DistanceSq,
  dot, cross, lengthSq, length,
} from './math/vec2.js'
export { mat2FromAngle, mat2MulVec, mat2TransposeMulVec } from './math/mat2.js'

// Collision
export { aabbOverlap, aabbMerge, aabbContains, aabbArea, aabbPerimeter, computeBodyAABB } from './collision/aabb.js'
export { broadphase, broadphaseBVH, DynamicTree } from './collision/broadphase.js'
export { narrowphase } from './collision/narrowphase.js'
export { gjkNarrowphase } from './collision/gjk.js'
export { collide, narrowphaseDispatch } from './collision/pipeline.js'
export { registerShapeHandler, getShapeHandler } from './collision/shapes.js'
export type { ShapeHandler } from './collision/shapes.js'
export { ContactTracker } from './collision/contact-tracker.js'
export { Simplex } from './collision/simplex.js'
export { Polytope } from './collision/polytope.js'

// Solver
export { solveVelocity, solvePosition, integrate } from './solver/sequential-impulse.js'

// WASM
export { WasmPhysicsBackend } from './wasm/WasmPhysicsBackend.js'
export { loadWasmModule } from './wasm/WasmModule.js'
export type { WasmInstance } from './wasm/WasmModule.js'
