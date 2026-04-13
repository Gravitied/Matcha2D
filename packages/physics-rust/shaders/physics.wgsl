// Matcha2D GPU physics — narrowphase (ball/ball, ball/box, box/box), Jacobi-style contact resolve, integration.
// Limits must match `gpu::types` in Rust.

const MAX_CONTACTS: u32 = 2u;
const SCALE: f32 = 1000000.0;
const PI: f32 = 3.141592653589793;

struct SimParams {
    dt: f32,
    gravity_x: f32,
    gravity_y: f32,
    pair_count: u32,
    body_count: u32,
    collider_count: u32,
    solver_iterations: u32,
    substeps: u32,
    penetration_slop: f32,
    max_corrective_velocity: f32,
    _pad1: f32,
    _pad2: f32,
}

struct Body {
    position: vec2<f32>,
    angle: f32,
    angvel: f32,
    linvel: vec2<f32>,
    inv_mass: f32,
    inv_inertia: f32,
    body_type: u32,
    flags: u32,
    linear_damping: f32,
    angular_damping: f32,
    gravity_scale: f32,
    _pad_b0: f32,
    _pad_b1: f32,
    _pad_b2: f32,
}

struct Collider {
    shape_kind: u32,
    body_index: u32,
    local_pos: vec2<f32>,
    local_sin: f32,
    local_cos: f32,
    friction: f32,
    restitution: f32,
    collider_flags: u32,
    param0: f32,
    param1: f32,
    poly_vert_start: u32,
    poly_vert_count: u32,
    _p0: u32,
    _p1: u32,
    _p2: u32,
}

struct ContactPoint {
    local_a: vec2<f32>,
    local_b: vec2<f32>,
    penetration: f32,
    _pad: f32,
}

struct Manifold {
    valid: u32,
    collider_a: u32,
    collider_b: u32,
    sensor_pair: u32,
    normal: vec2<f32>,
    tangent: vec2<f32>,
    penetration: f32,
    contact_count: u32,
    // Pad so `c0` starts at byte offset 64. `dist` = dist_offset + dot(world_b - world_a, normal).
    dist_offset: f32,
    _man_pad1: u32,
    _man_pad2: u32,
    _man_pad3: u32,
    _man_pad4: u32,
    _man_pad5: u32,
    c0: ContactPoint,
    c1: ContactPoint,
    _stride_pad: vec4<u32>,
}

@group(0) @binding(0) var<uniform> params: SimParams;
@group(0) @binding(1) var<storage, read_write> bodies: array<Body>;
@group(0) @binding(2) var<storage, read> colliders: array<Collider>;
@group(0) @binding(3) var<storage, read> poly_vertices: array<f32>;
@group(0) @binding(4) var<storage, read> pairs: array<u32>;
@group(0) @binding(5) var<storage, read_write> manifolds: array<Manifold>;
@group(0) @binding(6) var<storage, read_write> body_atomics: array<atomic<i32>>;

fn rot_mul(sin_c: f32, cos_c: f32, v: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(cos_c * v.x - sin_c * v.y, sin_c * v.x + cos_c * v.y);
}

/// Inverse rotate vector by angle (sa, ca) — undoes `rot_mul(sa, ca, _)`.
fn inv_rot_mul(sa: f32, ca: f32, v: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(ca * v.x + sa * v.y, -sa * v.x + ca * v.y);
}

fn world_rot_body(body: Body) -> vec2<f32> {
    return vec2<f32>(sin(body.angle), cos(body.angle));
}

fn collider_world_transform(body: Body, col: Collider) -> mat2x2<f32> {
    let br = world_rot_body(body);
    let sa = br.x;
    let ca = br.y;
    let sin_t = sa * col.local_cos + ca * col.local_sin;
    let cos_t = ca * col.local_cos - sa * col.local_sin;
    // Columns are basis vectors for shape in world space (for box axes)
    let ex = rot_mul(sin_t, cos_t, vec2<f32>(1.0, 0.0));
    let ey = rot_mul(sin_t, cos_t, vec2<f32>(0.0, 1.0));
    return mat2x2<f32>(ex, ey);
}

fn collider_world_pos(body: Body, col: Collider) -> vec2<f32> {
    let br = world_rot_body(body);
    let sa = br.x;
    let ca = br.y;
    let world_local = rot_mul(col.local_sin, col.local_cos, col.local_pos);
    return body.position + rot_mul(sa, ca, world_local);
}

/// World position of a point in collider-local space (`local_pt` relative to collider origin).
fn contact_world_point(body: Body, col: Collider, local_pt: vec2<f32>) -> vec2<f32> {
    let br = world_rot_body(body);
    let sa = br.x;
    let ca = br.y;
    let p_col = rot_mul(col.local_sin, col.local_cos, col.local_pos + local_pt);
    return body.position + rot_mul(sa, ca, p_col);
}

fn is_dynamic(body: Body) -> bool {
    return body.body_type == 0u && (body.flags & 1u) == 0u;
}

fn is_static_or_kinematic(body: Body) -> bool {
    return body.body_type != 0u || (body.flags & 1u) != 0u;
}

// ---------- integrate forces + semi-implicit position ----------
@compute @workgroup_size(64)
fn integrate_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= params.body_count) {
        return;
    }
    var b = bodies[gid.x];
    let dt = params.dt;
    // Kinematic: integrate pose from velocity, no gravity/damping.
    if (b.body_type == 2u) {
        if ((b.flags & 1u) != 0u) {
            return;
        }
        b.position += b.linvel * dt;
        b.angle += b.angvel * dt;
        bodies[gid.x] = b;
        return;
    }
    if (b.body_type != 0u) {
        return;
    }
    if ((b.flags & 1u) != 0u) {
        return;
    }
    let damp_l = clamp(1.0 - b.linear_damping * dt, 0.0, 1.0);
    let damp_a = clamp(1.0 - b.angular_damping * dt, 0.0, 1.0);
    b.linvel *= damp_l;
    b.angvel *= damp_a;
    b.linvel += vec2<f32>(params.gravity_x, params.gravity_y) * b.gravity_scale * dt;
    b.position += b.linvel * dt;
    b.angle += b.angvel * dt;
    bodies[gid.x] = b;
}

// ---------- clear atomics (4 i32 per body: dx, dy, dang, count) ----------
@compute @workgroup_size(64)
fn clear_atomics_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= params.body_count) {
        return;
    }
    let base = gid.x * 4u;
    atomicStore(&body_atomics[base + 0u], 0);
    atomicStore(&body_atomics[base + 1u], 0);
    atomicStore(&body_atomics[base + 2u], 0);
    atomicStore(&body_atomics[base + 3u], 0);
}

fn project_point_on_obb(p: vec2<f32>, center: vec2<f32>, axis_x: vec2<f32>, axis_y: vec2<f32>, hx: f32, hy: f32) -> vec2<f32> {
    let d = p - center;
    let x = clamp(dot(d, axis_x), -hx, hx);
    let y = clamp(dot(d, axis_y), -hy, hy);
    return center + axis_x * x + axis_y * y;
}

/// Stable unit normal when centers coincide (matches CPU `stable_normal_for_coincident_positions` idea).
fn stable_normal_coincident_positions(a: vec2<f32>, b: vec2<f32>) -> vec2<f32> {
    let h = sin(dot(a, vec2<f32>(12.9898, 78.233)) + dot(b, vec2<f32>(37.719, 45.1643))) * 43758.5453;
    let t = fract(h) * 6.28318530718;
    return vec2<f32>(cos(t), sin(t));
}

fn ball_ball(ca: Collider, cb: Collider, ba: Body, bb: Body, out: ptr<function, Manifold>) {
    let pa = collider_world_pos(ba, ca);
    let pb = collider_world_pos(bb, cb);
    let ra = ca.param0;
    let rb = cb.param0;
    let d = pb - pa;
    let dist = length(d);
    let pen = ra + rb - dist;
    if (pen <= 0.0) {
        (*out).valid = 0u;
        return;
    }
    var n = stable_normal_coincident_positions(pa, pb);
    if (dist > 1e-5) {
        n = d / dist;
    }
    (*out).valid = 1u;
    (*out).normal = n;
    (*out).tangent = vec2<f32>(-n.y, n.x);
    (*out).penetration = pen;
    (*out).contact_count = 1u;
    (*out).c0.local_a = vec2<f32>(0.0, 0.0);
    (*out).c0.local_b = vec2<f32>(0.0, 0.0);
    (*out).c0.penetration = pen;
}

fn ball_box(ball: Collider, bx: Collider, b_ball: Body, b_box: Body, out: ptr<function, Manifold>) {
    let center = collider_world_pos(b_box, bx);
    let m = collider_world_transform(b_box, bx);
    let axis_x = m[0];
    let axis_y = m[1];
    let hx = bx.param0;
    let hy = bx.param1;
    let pc = collider_world_pos(b_ball, ball);
    let closest = project_point_on_obb(pc, center, axis_x, axis_y, hx, hy);
    let d = pc - closest;
    let dist = length(d);
    let r = ball.param0;
    let pen = r - dist;
    if (pen <= 0.0) {
        (*out).valid = 0u;
        return;
    }
    var n = stable_normal_coincident_positions(pc, center);
    if (dist > 1e-5) {
        n = d / dist;
    }
    (*out).valid = 1u;
    (*out).normal = n;
    (*out).tangent = vec2<f32>(-n.y, n.x);
    (*out).penetration = pen;
    (*out).contact_count = 1u;
    (*out).c0.local_a = vec2<f32>(0.0, 0.0);
    let sin_t = sin(b_box.angle) * bx.local_cos + cos(b_box.angle) * bx.local_sin;
    let cos_t = cos(b_box.angle) * bx.local_cos - sin(b_box.angle) * bx.local_sin;
    (*out).c0.local_b = inv_rot_mul(sin_t, cos_t, closest - center);
    (*out).c0.penetration = pen;
}

fn sat_box_box(ca: Collider, cb: Collider, ba: Body, bb: Body, out: ptr<function, Manifold>) {
    let ca_c = collider_world_pos(ba, ca);
    let cb_c = collider_world_pos(bb, cb);
    let ma = collider_world_transform(ba, ca);
    let mb = collider_world_transform(bb, cb);
    let ax_a = ma[0];
    let ay_a = ma[1];
    let ax_b = mb[0];
    let ay_b = mb[1];
    let hax = ca.param0;
    let hay = ca.param1;
    let hbx = cb.param0;
    let hby = cb.param1;

    var min_overlap = 1e30;
    var best_n = vec2<f32>(1.0, 0.0);

    let axes = array<vec2<f32>, 4>(ax_a, ay_a, ax_b, ay_b);
    for (var i = 0u; i < 4u; i++) {
        let axis = axes[i];
        let len_a = length(axis);
        if (len_a < 1e-8) {
            continue;
        }
        let n = axis / len_a;
        let c = dot(cb_c - ca_c, n);
        let ra = abs(dot(ax_a, n)) * hax + abs(dot(ay_a, n)) * hay;
        let rb = abs(dot(ax_b, n)) * hbx + abs(dot(ay_b, n)) * hby;
        let overlap = ra + rb - abs(c);
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_n = select(n, -n, c < 0.0);
        }
        if (overlap < 0.0) {
            (*out).valid = 0u;
            return;
        }
    }

    if (min_overlap <= 0.0) {
        (*out).valid = 0u;
        return;
    }

    (*out).valid = 1u;
    (*out).normal = best_n;
    (*out).tangent = vec2<f32>(-best_n.y, best_n.x);
    (*out).penetration = min_overlap;
    (*out).contact_count = 1u;
    let mid = (ca_c + cb_c) * 0.5;
    let sin_a = sin(ba.angle) * ca.local_cos + cos(ba.angle) * ca.local_sin;
    let cos_a = cos(ba.angle) * ca.local_cos - sin(ba.angle) * ca.local_sin;
    let sin_b = sin(bb.angle) * cb.local_cos + cos(bb.angle) * cb.local_sin;
    let cos_b = cos(bb.angle) * cb.local_cos - sin(bb.angle) * cb.local_sin;
    (*out).c0.local_a = inv_rot_mul(sin_a, cos_a, mid - ca_c);
    (*out).c0.local_b = inv_rot_mul(sin_b, cos_b, mid - cb_c);
    (*out).c0.penetration = min_overlap;
}

@compute @workgroup_size(64)
fn narrowphase_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= params.pair_count) {
        return;
    }
    let ia = pairs[gid.x * 2u + 0u];
    let ib = pairs[gid.x * 2u + 1u];
    if (ia >= params.collider_count || ib >= params.collider_count) {
        manifolds[gid.x].valid = 0u;
        return;
    }
    let ca = colliders[ia];
    let cb = colliders[ib];
    if (ca.body_index >= params.body_count || cb.body_index >= params.body_count) {
        manifolds[gid.x].valid = 0u;
        return;
    }
    let ba = bodies[ca.body_index];
    let bb = bodies[cb.body_index];

    var m: Manifold;
    m.valid = 0u;
    m.collider_a = ia;
    m.collider_b = ib;
    m.sensor_pair = select(0u, 1u, (ca.collider_flags & 1u) != 0u && (cb.collider_flags & 1u) != 0u);
    m.contact_count = 0u;
    m.penetration = 0.0;
    m.normal = vec2<f32>(0.0, 0.0);
    m.tangent = vec2<f32>(0.0, 0.0);
    m.dist_offset = 0.0;
    m._man_pad1 = 0u;
    m._man_pad2 = 0u;
    m._man_pad3 = 0u;
    m._man_pad4 = 0u;
    m._man_pad5 = 0u;
    m.c0.penetration = 0.0;
    m.c1.penetration = 0.0;

    let ka = ca.shape_kind;
    let kb = cb.shape_kind;
    // 0 ball, 1 cuboid, 2 poly (treated as ball fallback using param0 as radius)
    if (ka == 0u && kb == 0u) {
        ball_ball(ca, cb, ba, bb, &m);
    } else if (ka == 0u && kb == 1u) {
        ball_box(ca, cb, ba, bb, &m);
    } else if (ka == 1u && kb == 0u) {
        ball_box(cb, ca, bb, ba, &m);
    } else if (ka == 1u && kb == 1u) {
        sat_box_box(ca, cb, ba, bb, &m);
    } else if (ka == 2u || kb == 2u) {
        // Convex polygon: conservative sphere — `param0` must hold effective radius on CPU upload.
        var sa = ca;
        var sb = cb;
        sa.shape_kind = 0u;
        sb.shape_kind = 0u;
        ball_ball(sa, sb, ba, bb, &m);
        m.collider_a = ia;
        m.collider_b = ib;
    } else {
        m.valid = 0u;
    }

    if (m.valid == 1u) {
        let world_a = contact_world_point(ba, ca, m.c0.local_a);
        let world_b = contact_world_point(bb, cb, m.c0.local_b);
        let initial_gap = dot(world_b - world_a, m.normal);
        m.dist_offset = -m.penetration - initial_gap;
    }

    manifolds[gid.x] = m;
}

fn add_body_delta(idx: u32, dpx: f32, dpy: f32, dang: f32) {
    let base = idx * 4u;
    atomicAdd(&body_atomics[base + 0u], i32(round(dpx * SCALE)));
    atomicAdd(&body_atomics[base + 1u], i32(round(dpy * SCALE)));
    atomicAdd(&body_atomics[base + 2u], i32(round(dang * SCALE)));
    atomicAdd(&body_atomics[base + 3u], 1i);
}

@compute @workgroup_size(64)
fn solve_contacts_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= params.pair_count) {
        return;
    }
    let man = manifolds[gid.x];
    if (man.valid == 0u) {
        return;
    }
    let ca = colliders[man.collider_a];
    let cb = colliders[man.collider_b];
    if ((ca.collider_flags & 1u) != 0u || (cb.collider_flags & 1u) != 0u) {
        return;
    }
    let ba = bodies[ca.body_index];
    let bb = bodies[cb.body_index];

    let n = man.normal;

    let world_a = contact_world_point(ba, ca, man.c0.local_a);
    let world_b = contact_world_point(bb, cb, man.c0.local_b);
    let current_gap = dot(world_b - world_a, n);
    let dist = man.dist_offset + current_gap;
    let current_pen = max(-dist, 0.0);
    let pen = max(current_pen - params.penetration_slop, 0.0);
    if (pen <= 0.0) {
        return;
    }

    let r_a = world_a - ba.position;
    let r_b = world_b - bb.position;

    let rn_a = r_a.x * n.y - r_a.y * n.x;
    let rn_b = r_b.x * n.y - r_b.y * n.x;

    var w = 0.0;
    if (is_dynamic(ba)) {
        w += ba.inv_mass + rn_a * rn_a * ba.inv_inertia;
    }
    if (is_dynamic(bb)) {
        w += bb.inv_mass + rn_b * rn_b * bb.inv_inertia;
    }
    if (w <= 1e-12) {
        return;
    }

    // `max_corrective_velocity` is packed finite from CPU (`f32::INFINITY` → huge sentinel).
    var lambda = pen / w;
    let cap = params.max_corrective_velocity * params.dt;
    if (cap > 0.0 && lambda > cap) {
        lambda = cap;
    }
    let corr = n * lambda;

    if (is_dynamic(ba)) {
        add_body_delta(ca.body_index, -corr.x * ba.inv_mass, -corr.y * ba.inv_mass, -rn_a * lambda * ba.inv_inertia);
    }
    if (is_dynamic(bb)) {
        add_body_delta(cb.body_index, corr.x * bb.inv_mass, corr.y * bb.inv_mass, rn_b * lambda * bb.inv_inertia);
    }
}

@compute @workgroup_size(64)
fn apply_deltas_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= params.body_count) {
        return;
    }
    var b = bodies[gid.x];
    if (!is_dynamic(b)) {
        return;
    }
    let base = gid.x * 4u;
    let c = atomicLoad(&body_atomics[base + 3u]);
    if (c <= 0) {
        return;
    }
    let fc = f32(c);
    let dx = f32(atomicLoad(&body_atomics[base + 0u])) / (SCALE * fc);
    let dy = f32(atomicLoad(&body_atomics[base + 1u])) / (SCALE * fc);
    let da = f32(atomicLoad(&body_atomics[base + 2u])) / (SCALE * fc);
    b.position += vec2<f32>(dx, dy);
    b.angle += da;
    if (params.dt > 0.0) {
        b.linvel += vec2<f32>(dx, dy) / params.dt;
        b.angvel += da / params.dt;
    }
    bodies[gid.x] = b;
    atomicStore(&body_atomics[base + 0u], 0);
    atomicStore(&body_atomics[base + 1u], 0);
    atomicStore(&body_atomics[base + 2u], 0);
    atomicStore(&body_atomics[base + 3u], 0);
}
