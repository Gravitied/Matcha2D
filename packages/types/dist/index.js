// src/body.ts
var BodyFlags = {
  ACTIVE: 1,
  STATIC: 2,
  SLEEPING: 4,
  SENSOR: 8
};
var BodyType = {
  Dynamic: 0,
  Static: 1,
  Kinematic: 2
};

// src/buffers.ts
var MAX_VERTICES_PER_SHAPE = 16;
var MAX_BODIES = 8192;
function createBuffers(capacity = MAX_BODIES) {
  return {
    positionX: new Float32Array(capacity),
    positionY: new Float32Array(capacity),
    velocityX: new Float32Array(capacity),
    velocityY: new Float32Array(capacity),
    angle: new Float32Array(capacity),
    angularVel: new Float32Array(capacity),
    mass: new Float32Array(capacity),
    invMass: new Float32Array(capacity),
    inertia: new Float32Array(capacity),
    invInertia: new Float32Array(capacity),
    flags: new Uint8Array(capacity),
    halfExtentX: new Float32Array(capacity),
    halfExtentY: new Float32Array(capacity),
    shapeType: new Uint8Array(capacity),
    shapeRadius: new Float32Array(capacity),
    shapeVertexCount: new Uint8Array(capacity),
    shapeVerticesX: new Float32Array(capacity * MAX_VERTICES_PER_SHAPE),
    shapeVerticesY: new Float32Array(capacity * MAX_VERTICES_PER_SHAPE)
  };
}

// src/collision.ts
var ShapeType = {
  Box: 0,
  Circle: 1,
  Polygon: 2
};

// src/constraint.ts
var JointType = {
  Distance: 0,
  Revolute: 1,
  Prismatic: 2,
  Weld: 3
};

// src/config.ts
var DEFAULT_WORLD_CONFIG = {
  gravity: { x: 0, y: -9.81 },
  fixedTimestep: 1 / 60,
  velocityIterations: 12,
  internalVelocityIterations: 8,
  positionIterations: 6,
  baumgarteFactor: 0.2,
  maxCorrectiveVelocity: Number.POSITIVE_INFINITY,
  penetrationSlop: 0.02,
  warmStarting: true,
  blockSolver: true,
  defaultFriction: 0.3,
  defaultRestitution: 0,
  restitutionSlop: 1,
  defaultLinearDamping: 0.01,
  defaultAngularDamping: 0.01,
  sweptBroadphase: true,
  sleepVelocityThreshold: 0.25,
  sleepAngularVelocityThreshold: 0.5,
  sleepTimeThreshold: 1,
  maxSolverLinearVelocity: Number.POSITIVE_INFINITY,
  maxSolverAngularVelocity: Number.POSITIVE_INFINITY,
  pgsRelaxation: 0.88,
  maxIntegrateLinearVelocity: Number.POSITIVE_INFINITY,
  maxIntegrateAngularVelocity: Number.POSITIVE_INFINITY,
  worldBounds: null
};

// src/narrowphaseDebug.ts
function parseNarrowphaseOutlinesFlat(data) {
  const f = data instanceof Float32Array ? data : Float32Array.from(data);
  if (f.length < 1) return [];
  const entryCount = Math.floor(f[0]);
  let i = 1;
  const out = [];
  for (let e = 0; e < entryCount; e++) {
    if (i + 5 > f.length) break;
    const colliderHandle = Math.round(f[i++]);
    const bodyRaw = f[i++];
    const bodyHandle = Number.isFinite(bodyRaw) ? Math.round(bodyRaw) : null;
    const shapeKindNum = f[i++];
    const flags = f[i++];
    const pointCount = Math.floor(f[i++]);
    const need = pointCount * 2;
    if (pointCount < 0 || i + need > f.length) break;
    const vertices = [];
    for (let p = 0; p < pointCount; p++) {
      vertices.push({ x: f[i++], y: f[i++] });
    }
    const shapeKind = shapeKindNum === 0 ? "ball" : shapeKindNum === 1 ? "cuboid" : "polygon";
    out.push({
      colliderHandle,
      bodyHandle,
      shapeKind,
      sensor: flags !== 0,
      vertices
    });
  }
  return out;
}

export { BodyFlags, BodyType, DEFAULT_WORLD_CONFIG, JointType, MAX_BODIES, MAX_VERTICES_PER_SHAPE, ShapeType, createBuffers, parseNarrowphaseOutlinesFlat };
//# sourceMappingURL=index.js.map
//# sourceMappingURL=index.js.map