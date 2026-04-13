import { DEFAULT_WORLD_CONFIG, parseNarrowphaseOutlinesFlat } from '@matcha2d/types';
export { parseNarrowphaseOutlinesFlat } from '@matcha2d/types';
import init, { PhysicsEngine } from '@matcha2d/physics-rust';

var __defProp = Object.defineProperty;
var __getOwnPropSymbols = Object.getOwnPropertySymbols;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __propIsEnum = Object.prototype.propertyIsEnumerable;
var __defNormalProp = (obj, key, value) => key in obj ? __defProp(obj, key, { enumerable: true, configurable: true, writable: true, value }) : obj[key] = value;
var __spreadValues = (a, b) => {
  for (var prop in b || (b = {}))
    if (__hasOwnProp.call(b, prop))
      __defNormalProp(a, prop, b[prop]);
  if (__getOwnPropSymbols)
    for (var prop of __getOwnPropSymbols(b)) {
      if (__propIsEnum.call(b, prop))
        __defNormalProp(a, prop, b[prop]);
    }
  return a;
};
var World = class _World {
  constructor(engine, config) {
    this._accumulator = 0;
    this._bodyHandles = /* @__PURE__ */ new Map();
    this._nextBodyId = 0;
    this._engine = engine;
    this.config = config;
  }
  /**
   * Create a World with the WASM physics engine.
   * This is async because the WASM module must be loaded first.
   */
  static async create(config = {}) {
    await init();
    const merged = __spreadValues(__spreadValues({}, DEFAULT_WORLD_CONFIG), config);
    const engine = new PhysicsEngine();
    engine.set_gravity(merged.gravity.x, merged.gravity.y);
    engine.set_dt(merged.fixedTimestep);
    engine.set_swept_broadphase(merged.sweptBroadphase);
    engine.set_sleep_thresholds(
      merged.sleepVelocityThreshold,
      merged.sleepAngularVelocityThreshold,
      merged.sleepTimeThreshold
    );
    engine.set_solver_config(
      merged.velocityIterations,
      merged.internalVelocityIterations,
      merged.blockSolver
    );
    engine.set_position_correction(merged.baumgarteFactor, merged.maxCorrectiveVelocity);
    engine.set_penetration_slop(merged.penetrationSlop);
    engine.set_max_solver_velocities(
      merged.maxSolverLinearVelocity,
      merged.maxSolverAngularVelocity
    );
    engine.set_pgs_relaxation(merged.pgsRelaxation);
    engine.set_max_integrate_velocities(
      merged.maxIntegrateLinearVelocity,
      merged.maxIntegrateAngularVelocity
    );
    if (merged.worldBounds) {
      const b = merged.worldBounds;
      engine.set_world_bounds(b.minX, b.minY, b.maxX, b.maxY);
    } else {
      engine.set_world_bounds(Number.NaN, 0, 0, 0);
    }
    return new _World(engine, merged);
  }
  /** Create a new body with the given properties. */
  createBody(def = {}) {
    var _a, _b, _c, _d, _e;
    const x = (_a = def.positionX) != null ? _a : 0;
    const y = (_b = def.positionY) != null ? _b : 0;
    const bodyType = (_c = def.type) != null ? _c : "dynamic";
    let handle;
    switch (bodyType) {
      case "static":
        handle = this._engine.create_static_body(x, y);
        break;
      case "kinematic":
        handle = this._engine.create_kinematic_body(x, y);
        break;
      default:
        handle = this._engine.create_dynamic_body(x, y);
        break;
    }
    if (def.velocityX !== void 0 || def.velocityY !== void 0) {
      this._engine.set_body_velocity(handle, (_d = def.velocityX) != null ? _d : 0, (_e = def.velocityY) != null ? _e : 0);
    }
    if (def.angle !== void 0) {
      this._engine.set_body_angle(handle, def.angle);
    }
    if (def.collisionGroups !== void 0) {
      this._engine.set_body_collision_groups(handle, def.collisionGroups);
    }
    if (def.collisionMask !== void 0) {
      this._engine.set_body_collision_mask(handle, def.collisionMask);
    }
    if (bodyType === "dynamic") {
      this._engine.set_body_linear_damping(handle, this.config.defaultLinearDamping);
      this._engine.set_body_angular_damping(handle, this.config.defaultAngularDamping);
    }
    return handle;
  }
  /** Destroy a body. */
  destroyBody(handle) {
    this._engine.remove_body(handle);
  }
  /** Add a collider to a body. */
  addCollider(body, def) {
    var _a, _b, _c;
    const bodyHandle = body;
    let colliderHandle;
    switch (def.shape) {
      case "ball":
        colliderHandle = this._engine.create_ball_collider((_a = def.radius) != null ? _a : 0.5, bodyHandle);
        break;
      case "cuboid":
        colliderHandle = this._engine.create_box_collider(
          (_b = def.halfExtentX) != null ? _b : 0.5,
          (_c = def.halfExtentY) != null ? _c : 0.5,
          bodyHandle
        );
        break;
      case "polygon":
        if (def.vertices && def.vertices.length > 0) {
          const vx = new Float32Array(def.vertices.map((v) => v.x));
          const vy = new Float32Array(def.vertices.map((v) => v.y));
          colliderHandle = this._engine.create_polygon_collider(vx, vy, bodyHandle);
        } else {
          colliderHandle = this._engine.create_box_collider(0.5, 0.5, bodyHandle);
        }
        break;
      default:
        colliderHandle = this._engine.create_ball_collider(0.5, bodyHandle);
        break;
    }
    if (def.friction !== void 0) {
      this._engine.set_collider_friction(colliderHandle, def.friction);
    }
    if (def.restitution !== void 0) {
      this._engine.set_collider_restitution(colliderHandle, def.restitution);
    }
    if (def.sensor !== void 0) {
      this._engine.set_collider_sensor(colliderHandle, def.sensor);
    }
    return colliderHandle;
  }
  /** Set body position. */
  setBodyPosition(handle, x, y) {
    this._engine.set_body_position(handle, x, y);
  }
  /** Set body velocity. */
  setBodyVelocity(handle, vx, vy) {
    this._engine.set_body_velocity(handle, vx, vy);
  }
  /** Set body angle. */
  setBodyAngle(handle, angle) {
    this._engine.set_body_angle(handle, angle);
  }
  /** Set gravity (affects all dynamic bodies). */
  setGravity(x, y) {
    this._engine.set_gravity(x, y);
    this.config.gravity = { x, y };
  }
  /**
   * Axis-aligned world bounds (meters). Dynamic bodies are nudged back inside after each step.
   * Pass `null` to disable clamping.
   */
  setWorldBounds(bounds) {
    if (bounds) {
      this._engine.set_world_bounds(bounds.minX, bounds.minY, bounds.maxX, bounds.maxY);
      this.config.worldBounds = bounds;
    } else {
      this._engine.set_world_bounds(Number.NaN, 0, 0, 0);
      this.config.worldBounds = null;
    }
  }
  /** Set collision groups (bitfield) for a body. */
  setCollisionGroups(handle, groups) {
    this._engine.set_body_collision_groups(handle, groups);
  }
  /** Set collision mask (who this body collides with) for a body. */
  setCollisionMask(handle, mask) {
    this._engine.set_body_collision_mask(handle, mask);
  }
  /** Wake a body from sleep so it participates in physics. */
  wakeBody(handle) {
    this._engine.wake_body(handle);
  }
  /** Force a body to sleep (stop simulating). */
  sleepBody(handle) {
    this._engine.sleep_body(handle);
  }
  /** Check if a body is currently sleeping. */
  isBodySleeping(handle) {
    return this._engine.is_body_sleeping(handle);
  }
  /**
   * Mass-weighted push-out for a dynamic body using narrowphase overlaps (spawn / layout helper).
   * Safe to call multiple times; pairs of dynamics may need several round-robins.
   */
  resolveOverlapsForBody(handle) {
    this._engine.resolve_overlaps_for_body(handle);
  }
  /** Get body position as [x, y]. */
  getBodyPosition(handle) {
    const pos = this._engine.get_body_position(handle);
    return [pos[0], pos[1]];
  }
  /** Get body velocity as [vx, vy]. */
  getBodyVelocity(handle) {
    const vel = this._engine.get_body_velocity(handle);
    return [vel[0], vel[1]];
  }
  /** Get body angle in radians. */
  getBodyAngle(handle) {
    return this._engine.get_body_angle(handle);
  }
  /** Set body angular velocity. */
  setBodyAngularVelocity(handle, angvel) {
    this._engine.set_body_angular_velocity(handle, angvel);
  }
  /** Get body angular velocity in rad/s. */
  getBodyAngularVelocity(handle) {
    return this._engine.get_body_angular_velocity(handle);
  }
  /** Apply an impulse to a body. */
  applyImpulse(handle, fx, fy) {
    this._engine.apply_impulse(handle, fx, fy);
  }
  /**
   * Advance exactly one fixed physics tick (`PhysicsEngine.step()`), ignoring the frame accumulator.
   * Intended for tests and deterministic bisect (e.g. hunting WASM `unreachable`).
   */
  stepPhysicsOnce() {
    this._engine.step();
  }
  /** Step the simulation by the given delta time. */
  step(dt) {
    this._accumulator += dt;
    if (this._accumulator > 0.2) {
      this._accumulator = 0.2;
    }
    while (this._accumulator >= this.config.fixedTimestep) {
      this._engine.step();
      this._accumulator -= this.config.fixedTimestep;
    }
  }
  /** Get the interpolation alpha for rendering. */
  get renderAlpha() {
    return this._accumulator / this.config.fixedTimestep;
  }
  /** Get all body positions X coordinates. */
  getPositionsX() {
    return this._engine.get_positions_x();
  }
  /** Get all body positions Y coordinates. */
  getPositionsY() {
    return this._engine.get_positions_y();
  }
  /** Get all body angles. */
  getAngles() {
    return this._engine.get_angles();
  }
  /** Get the number of bodies. */
  get bodyCount() {
    return this._engine.body_count();
  }
  /** Get the number of colliders. */
  get colliderCount() {
    return this._engine.collider_count();
  }
  /** Set collision callbacks (placeholder for future contact event support). */
  setCollisionCallbacks(_callbacks) {
  }
  /** Access the underlying physics engine (for advanced use). */
  get engine() {
    return this._engine;
  }
  /** Initialize WebGPU for hybrid GPU stepping. */
  async initGpu() {
    await this._engine.init_gpu();
  }
  /** Enable or disable GPU acceleration. Requires `initGpu()` to be called first. */
  setGpuAccelerationEnabled(enabled) {
    this._engine.setGpuAccelerationEnabled(enabled);
  }
  // ========== Diagnostics ==========
  /** Number of active contact pairs this frame. */
  get contactPairCount() {
    return this._engine.contact_pair_count();
  }
  /** Total contact points across all pairs this frame. */
  get contactPointCount() {
    return this._engine.contact_point_count();
  }
  /** Maximum penetration depth across all contacts this frame. */
  get maxPenetration() {
    return this._engine.max_penetration();
  }
  /** Contact normal of the first active pair (or [0,0] if none). */
  get firstContactNormal() {
    const n = this._engine.first_contact_normal();
    return [n[0], n[1]];
  }
  /** Full debug dump string for the current frame state. */
  get debugDump() {
    return this._engine.debug_dump();
  }
  /**
   * Collider outlines in world space — same geometry/transform as Rust narrowphase SAT.
   * Use for debug overlay; compare to visual proxies that use body COM + guessed size.
   */
  getNarrowphaseColliderOutlines() {
    const raw = this._engine.debug_narrowphase_outlines();
    return parseNarrowphaseOutlinesFlat(raw);
  }
};

export { World };
//# sourceMappingURL=index.js.map
//# sourceMappingURL=index.js.map