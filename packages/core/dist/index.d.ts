import { AABB, MatchaBuffers, CollisionPair, BroadphaseMethod, ContactManifold, NarrowphaseMethod, CollisionCallbacks, WorldConfig, PhysicsBackend } from '@matcha2d/types';

/**
 * Vec2 operations on flat Float32Array buffers.
 * Array+index signatures avoid allocation in hot loops.
 */
declare function vec2Set(outX: Float32Array, outY: Float32Array, idx: number, x: number, y: number): void;
declare function vec2Add(outX: Float32Array, outY: Float32Array, outIdx: number, aX: Float32Array, aY: Float32Array, aIdx: number, bX: Float32Array, bY: Float32Array, bIdx: number): void;
declare function vec2Sub(outX: Float32Array, outY: Float32Array, outIdx: number, aX: Float32Array, aY: Float32Array, aIdx: number, bX: Float32Array, bY: Float32Array, bIdx: number): void;
declare function vec2Scale(outX: Float32Array, outY: Float32Array, outIdx: number, aX: Float32Array, aY: Float32Array, aIdx: number, s: number): void;
declare function vec2Dot(aX: Float32Array, aY: Float32Array, aIdx: number, bX: Float32Array, bY: Float32Array, bIdx: number): number;
declare function vec2Cross(aX: Float32Array, aY: Float32Array, aIdx: number, bX: Float32Array, bY: Float32Array, bIdx: number): number;
declare function vec2LengthSq(x: Float32Array, y: Float32Array, idx: number): number;
declare function vec2Length(x: Float32Array, y: Float32Array, idx: number): number;
declare function vec2Normalize(outX: Float32Array, outY: Float32Array, outIdx: number, aX: Float32Array, aY: Float32Array, aIdx: number): void;
declare function vec2DistanceSq(aX: Float32Array, aY: Float32Array, aIdx: number, bX: Float32Array, bY: Float32Array, bIdx: number): number;
declare function dot(ax: number, ay: number, bx: number, by: number): number;
declare function cross(ax: number, ay: number, bx: number, by: number): number;
declare function lengthSq(x: number, y: number): number;
declare function length(x: number, y: number): number;

/**
 * 2x2 rotation matrix operations.
 * A rotation by angle θ is represented as [cos θ, -sin θ, sin θ, cos θ].
 */
declare function mat2FromAngle(angle: number): [number, number, number, number];
/** Multiply a 2x2 matrix by a vector: [m00*x + m01*y, m10*x + m11*y] */
declare function mat2MulVec(m00: number, m01: number, m10: number, m11: number, x: number, y: number): [number, number];
/** Transpose-multiply (inverse rotation): [m00*x + m10*y, m01*x + m11*y] */
declare function mat2TransposeMulVec(m00: number, m01: number, m10: number, m11: number, x: number, y: number): [number, number];

declare function aabbOverlap(a: AABB, b: AABB): boolean;
declare function aabbMerge(a: AABB, b: AABB): AABB;
declare function aabbContains(outer: AABB, inner: AABB): boolean;
declare function aabbArea(a: AABB): number;
declare function aabbPerimeter(a: AABB): number;
/**
 * Compute the world-space tight AABB for body `idx`.
 * Returns [minX, minY, maxX, maxY].
 * Correctly accounts for rotation — rotated boxes/polygons produce a larger
 * AABB than their rest-pose half-extents suggest.
 */
declare function computeBodyAABB(idx: number, buffers: MatchaBuffers): [number, number, number, number];

/**
 * Incremental dynamic AABB tree for broadphase collision detection.
 *
 * Unlike the previous rebuild-every-frame BVH, this tree persists across frames
 * and supports O(log n) insert/remove with automatic tree rotations for balance.
 *
 * Algorithm adapted from Sopiro/Physics with adaptations for SoA buffers.
 */
declare class DynamicTree {
    private nodes;
    private nodeID;
    private root;
    /** Fat AABB margin for static bodies to reduce reinsertions. */
    aabbMargin: number;
    /** Maps body index → tree node index for O(1) lookup. */
    private bodyToNode;
    reset(): void;
    /**
     * Insert a body into the tree.
     * Creates a fat AABB for static bodies, finds the best sibling via cost-based
     * descent, then walks up refitting ancestors with tree rotations.
     */
    insert(bodyIdx: number, buffers: MatchaBuffers): number;
    /**
     * Remove a body from the tree.
     * Replaces the node with its sibling, updates parent links, and refits ancestors.
     */
    remove(bodyIdx: number): void;
    /**
     * Update a body's AABB in the tree.
     * If the AABB has moved outside the fat margin, remove and reinsert.
     */
    update(bodyIdx: number, buffers: MatchaBuffers): void;
    /** Update all bodies in the tree. */
    updateAll(buffers: MatchaBuffers): void;
    /**
     * Query all overlapping body pairs.
     * Uses dual-subtree traversal: self-pairs within subtrees + cross-pairs between siblings.
     * Filters out static-static and sleeping-sleeping pairs.
     */
    queryPairs(buffers: MatchaBuffers): CollisionPair[];
    /** Query all bodies whose AABBs overlap a region. */
    queryRegion(minX: number, minY: number, maxX: number, maxY: number): number[];
    /** Find the best sibling for a new leaf via cost-based descent (BFS with priority queue). */
    private findBestSibling;
    /** Compute the area of the union of two nodes' AABBs. */
    private unionArea;
    /** Refit AABBs up the tree from a given node, applying rotations for balance. */
    private refitAncestors;
    /**
     * Tree rotation heuristic for balance.
     * Tries 4 swap types and picks the one with lowest area cost.
     * Adapted from Sopiro/Physics AABBTree.rotate().
     */
    private rotate;
    /** Refit a single node's AABB from its children. */
    private refitNode;
    /** Dual-subtree traversal to find all overlapping leaf pairs. */
    private queryCross;
    /** Generate a unique key for a pair of node indices using Szudzik's bijective pairing. */
    private pairKey;
    /** Allocate a new node from the pool. */
    private allocateNode;
    /** Free a node (mark for reuse). */
    private freeNode;
    /** Get the number of bodies in the tree. */
    get bodyCount(): number;
    /** Get the total cost (sum of all node AABB areas) — useful for debugging. */
    get cost(): number;
}

/**
 * Broadphase collision detection.
 *
 * Dispatches to the chosen algorithm:
 *   - 'sap': Sort-and-Sweep — best for sparse, motion-coherent scenes.
 *   - 'dynamicTree' (default): Incremental AABB tree — better for dense scenes.
 *   - 'bvh': Legacy rebuild-every-frame BVH (kept for compatibility).
 */
declare function broadphase(buffers: MatchaBuffers, count: number, treeOrMethod?: DynamicTree | null | BroadphaseMethod, method?: BroadphaseMethod): CollisionPair[];
/** Legacy rebuild-every-frame BVH — kept for compatibility. */
declare function broadphaseBVH(buffers: MatchaBuffers, count: number): CollisionPair[];

/**
 * Narrowphase collision detection.
 *
 * Uses GJK+EPA for all shape combinations.
 * Circle-circle uses an analytical shortcut for performance.
 */
declare function narrowphase(buffers: MatchaBuffers, pairs: CollisionPair[]): ContactManifold[];

declare function gjkNarrowphase(buffers: MatchaBuffers, pairs: Array<{
    a: number;
    b: number;
}>): ContactManifold[];

/**
 * Collision pipeline: broadphase → narrowphase.
 *
 * Users can customize the broadphase method:
 *   - broadphaseMethod: 'sap', 'dynamicTree' (default), or 'bvh'
 *
 * Narrowphase always uses GJK+EPA for robust collision detection.
 */
declare function collide(buffers: MatchaBuffers, count: number, treeOrMethod?: DynamicTree | null | BroadphaseMethod, broadphaseMethod?: BroadphaseMethod, _narrowphaseMethod?: NarrowphaseMethod): ContactManifold[];
/**
 * Run narrowphase on pre-filtered candidate pairs.
 * Use this when you already have pairs from a custom broadphase.
 */
declare function narrowphaseDispatch(buffers: MatchaBuffers, pairs: CollisionPair[], _method?: NarrowphaseMethod): ContactManifold[];

/**
 * Interface that any shape type must satisfy to integrate with the collision
 * pipeline.  Register custom shapes with `registerShapeHandler`.
 */
interface ShapeHandler {
    /**
     * Compute the world-space tight AABB for body `idx`.
     * Returns [minX, minY, maxX, maxY].
     */
    getAABB(idx: number, buffers: MatchaBuffers): [number, number, number, number];
    /**
     * Write world-space vertices into `outX`/`outY` and return the vertex count.
     * For shapes without a polygonal hull (e.g. circles) return 0.
     */
    getWorldVertices(idx: number, buffers: MatchaBuffers, outX: number[], outY: number[]): number;
    /**
     * Return the GJK support point in direction `(dx, dy)`.
     * The direction does not need to be normalized.
     */
    getSupportPoint(idx: number, buffers: MatchaBuffers, dx: number, dy: number): [number, number];
}
/** Register a handler for a custom (or built-in) shape type number. */
declare function registerShapeHandler(type: number, handler: ShapeHandler): void;
/** Retrieve the handler registered for a shape type, if any. */
declare function getShapeHandler(type: number): ShapeHandler | undefined;

/**
 * Tracks contact state across frames and fires collision callbacks.
 *
 * Compares new manifolds against previous frame to detect:
 * - Begin contact: new pair not seen before
 * - Stay contact: pair that existed last frame
 * - End contact: pair that existed last frame but not this frame
 */
declare class ContactTracker {
    private callbacks;
    private activePairs;
    private prevKeys;
    private currKeys;
    setCallbacks(callbacks: CollisionCallbacks | null): void;
    /**
     * Update contact state and fire callbacks.
     * Call this after solveVelocity, before storing manifolds for next frame.
     */
    update(manifolds: ContactManifold[], _config: WorldConfig): void;
    clear(): void;
}

/**
 * GJK Simplex management.
 * Tracks up to 3 vertices in the Minkowski difference space and computes
 * the closest point on the simplex to the origin.
 *
 * Adapted from Sopiro/Physics simplex.ts for SoA buffers.
 */
interface ClosestResult {
    resultX: number;
    resultY: number;
    contributors: number[];
}
declare class Simplex {
    verticesX: number[];
    verticesY: number[];
    get count(): number;
    clear(): void;
    addVertex(x: number, y: number): void;
    containsVertex(x: number, y: number, epsilon?: number): boolean;
    shrink(indices: number[]): void;
    /**
     * Returns the closest point on this simplex to the origin (qx=0, qy=0).
     * Also returns which vertices contributed to the closest point.
     */
    getClosest(qx: number, qy: number): ClosestResult;
}

/**
 * EPA Polytope for penetration depth computation.
 * Initialized from a GJK simplex (triangle) and expanded iteratively
 * to find the closest edge to the origin in the Minkowski difference.
 *
 * Adapted from Sopiro/Physics polytope.ts for SoA buffers.
 */
interface ClosestEdgeInfo {
    index: number;
    distance: number;
    normalX: number;
    normalY: number;
}
declare class Polytope {
    verticesX: number[];
    verticesY: number[];
    constructor(simplex: Simplex);
    get count(): number;
    /**
     * Returns the edge closest to the origin.
     * For each edge, computes the outward normal and the signed distance
     * from the origin to the edge's supporting line.
     */
    getClosestEdge(): ClosestEdgeInfo;
}

declare function solveVelocity(buffers: MatchaBuffers, manifolds: ContactManifold[], config: WorldConfig): void;
declare function solvePosition(buffers: MatchaBuffers, manifolds: ContactManifold[], config: WorldConfig): void;
declare function integrate(buffers: MatchaBuffers, count: number, dt: number, gravity: {
    x: number;
    y: number;
}): void;

/**
 * WASM-based PhysicsBackend implementation for Matcha2D.
 *
 * Wraps the Emscripten-compiled Box2D WASM module behind the standard
 * PhysicsBackend interface. The WASM backend takes a unified approach:
 * Box2D's b2World_Step internally handles broadphase → narrowphase →
 * solve → integrate, so individual methods are thin wrappers or no-ops.
 */

declare class WasmPhysicsBackend implements PhysicsBackend {
    private wasm;
    private worldHandle;
    private initialized;
    private dt;
    private subSteps;
    /**
     * Initialize the WASM backend.
     *
     * Loads the WASM module, creates a Box2D world with the given gravity,
     * and allocates internal data structures for up to maxBodies bodies.
     *
     * @param config - World configuration including fixedTimestep and other settings
     * @param subSteps - Number of sub-steps per simulation step (default: 4)
     */
    init(config: WorldConfig, subSteps?: number): Promise<void>;
    /**
     * Run broadphase collision detection.
     *
     * In WASM mode, broadphase is handled internally by Box2D during
     * b2_step(). This method returns an empty array — use collide()
     * instead for the full pipeline.
     */
    broadphase(_buffers: MatchaBuffers, _count: number, _method?: BroadphaseMethod): CollisionPair[];
    /**
     * Run narrowphase collision detection on candidate pairs.
     *
     * In WASM mode, narrowphase is handled internally by Box2D during
     * b2_step(). This method returns an empty array — use collide()
     * instead for the full pipeline.
     */
    narrowphase(_buffers: MatchaBuffers, _pairs: CollisionPair[], _method?: NarrowphaseMethod): ContactManifold[];
    /**
     * Run the full collision pipeline.
     *
     * This is the primary entry point for the WASM backend. It:
     * 1. Syncs body data (position, velocity, mass, etc.) to WASM
     * 2. Syncs shape data (boxes, circles, polygons) to WASM
     * 3. Advances the simulation by one time step
     * 4. Reads updated body state back from WASM
     * 5. Extracts contact manifolds from the simulation
     *
     * The method and narrowMethod parameters are ignored in WASM mode
     * since Box2D uses its own internal algorithms.
     */
    collide(buffers: MatchaBuffers, count: number, _broadMethod?: BroadphaseMethod, _narrowMethod?: NarrowphaseMethod): ContactManifold[];
    /**
     * Solve velocity constraints.
     *
     * In WASM mode, velocity solving is handled internally by Box2D
     * during b2_step(). This is a no-op.
     */
    solveVelocity(_buffers: MatchaBuffers, _manifolds: ContactManifold[], _config: WorldConfig): void;
    /**
     * Integrate positions from velocities.
     *
     * In WASM mode, integration is handled internally by Box2D
     * during b2_step(). This is a no-op.
     */
    integrate(_buffers: MatchaBuffers, _count: number, _dt: number, _gravity: {
        x: number;
        y: number;
    }): void;
    /**
     * Solve position constraints (penetration correction).
     *
     * In WASM mode, position solving is handled internally by Box2D
     * during b2_step(). This is a no-op.
     */
    solvePosition(_buffers: MatchaBuffers, _manifolds: ContactManifold[], _config: WorldConfig): void;
    /**
     * Clean up WASM resources.
     *
     * Destroys the Box2D world and frees all WASM-allocated memory.
     */
    dispose(): void;
}

/**
 * WASM module loader for Matcha2D.
 *
 * Loads the Emscripten-compiled Box2D WASM module and provides typed
 * TypeScript wrapper methods around the C bridge API.
 *
 * The Emscripten module is built with MODULARIZE=1 + EXPORT_ES6=1,
 * producing a default export function that returns Promise<Module>.
 */

/**
 * Typed wrapper around the WASM C bridge functions.
 *
 * Each method corresponds to a WASM_EXPORT function from wasm_bridge.h:
 * b2_init, b2_destroy, b2_sync_bodies, b2_sync_shapes, b2_step,
 * b2_read_bodies, b2_get_contacts.
 */
interface WasmInstance {
    /** Create a Box2D world. Returns world handle or -1 on failure. */
    init(gravityX: number, gravityY: number, maxBodies: number): number;
    /** Destroy a Box2D world. */
    destroy(worldHandle: number): void;
    /** Push SoA body data into WASM linear memory for Box2D. */
    syncBodies(worldHandle: number, buffers: MatchaBuffers, count: number): void;
    /** Push SoA shape data into WASM linear memory for Box2D. */
    syncShapes(worldHandle: number, buffers: MatchaBuffers, count: number): void;
    /** Advance simulation by one time step. */
    step(worldHandle: number, dt: number, subSteps: number): void;
    /** Read updated body state from WASM back into JS SoA buffers. */
    readBodies(worldHandle: number, buffers: MatchaBuffers, count: number): void;
    /** Read contact manifolds from the WASM world. */
    getContacts(worldHandle: number): ContactManifold[];
    /** Free all WASM-allocated memory. */
    dispose(): void;
}
/**
 * Load the WASM module and return a typed WasmInstance.
 *
 * The returned instance wraps raw C bridge calls with clean TypeScript
 * signatures. SoA buffers are copied to/from WASM linear memory
 * using HEAPF32/HEAPU8 views.
 */
declare function loadWasmModule(): Promise<WasmInstance>;

export { ContactTracker, DynamicTree, Polytope, type ShapeHandler, Simplex, type WasmInstance, WasmPhysicsBackend, aabbArea, aabbContains, aabbMerge, aabbOverlap, aabbPerimeter, broadphase, broadphaseBVH, collide, computeBodyAABB, cross, dot, getShapeHandler, gjkNarrowphase, integrate, length, lengthSq, loadWasmModule, mat2FromAngle, mat2MulVec, mat2TransposeMulVec, narrowphase, narrowphaseDispatch, registerShapeHandler, solvePosition, solveVelocity, vec2Add, vec2Cross, vec2DistanceSq, vec2Dot, vec2Length, vec2LengthSq, vec2Normalize, vec2Scale, vec2Set, vec2Sub };
