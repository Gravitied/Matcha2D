use crate::body::RigidBodySet;
use crate::collider::{Collider, ColliderHandle, ColliderSet, Shape};
use crate::contact::ContactManifold;
use crate::math::{Rot, Vec2};

/// Dispatch narrowphase collision detection for a pair of colliders
pub fn detect_collision(
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    handle_a: ColliderHandle,
    handle_b: ColliderHandle,
) -> Option<ContactManifold> {
    let col_a = colliders.get(handle_a)?;
    let col_b = colliders.get(handle_b)?;

    // Skip sensor-sensor pairs
    if col_a.is_sensor() && col_b.is_sensor() {
        return None;
    }

    // Get world-space transforms
    let (pos_a, rot_a) = get_world_transform(bodies, col_a);
    let (pos_b, rot_b) = get_world_transform(bodies, col_b);

    // Dispatch based on shape types
    match (&col_a.shape, &col_b.shape) {
        (Shape::Ball { radius: ra }, Shape::Ball { radius: rb }) => {
            sat_ball_ball(pos_a, *ra, pos_b, *rb)
        }
        (Shape::Ball { radius }, Shape::Cuboid { hx, hy }) => {
            sat_ball_cuboid(pos_a, *radius, pos_b, rot_b, *hx, *hy)
        }
        (Shape::Cuboid { hx, hy }, Shape::Ball { radius }) => {
            sat_ball_cuboid(pos_b, *radius, pos_a, rot_a, *hx, *hy).map(|m| flip_manifold(m))
        }
        (Shape::Ball { radius }, Shape::Polygon { vertices }) => {
            sat_ball_polygon(pos_a, *radius, pos_b, rot_b, vertices)
        }
        (Shape::Polygon { vertices }, Shape::Ball { radius }) => {
            sat_ball_polygon(pos_b, *radius, pos_a, rot_a, vertices).map(|m| flip_manifold(m))
        }
        (Shape::Cuboid { hx: hxa, hy: hya }, Shape::Cuboid { hx: hxb, hy: hyb }) => {
            sat_cuboid_cuboid(pos_a, rot_a, *hxa, *hya, pos_b, rot_b, *hxb, *hyb)
        }
        (Shape::Cuboid { hx, hy }, Shape::Polygon { vertices }) => {
            let verts_a = cuboid_vertices(*hx, *hy);
            sat_polygon_polygon(pos_a, rot_a, &verts_a, pos_b, rot_b, vertices)
        }
        (Shape::Polygon { vertices }, Shape::Cuboid { hx, hy }) => {
            let verts_b = cuboid_vertices(*hx, *hy);
            sat_polygon_polygon(pos_a, rot_a, vertices, pos_b, rot_b, &verts_b)
        }
        (Shape::Polygon { vertices: va }, Shape::Polygon { vertices: vb }) => {
            sat_polygon_polygon(pos_a, rot_a, va, pos_b, rot_b, vb)
        }
    }
}

fn get_world_transform(bodies: &RigidBodySet, collider: &Collider) -> (Vec2, Rot) {
    if let Some(parent_handle) = collider.parent {
        if let Some(body) = bodies.get(parent_handle) {
            let world_pos = body.pos + body.rot.mul_vec(collider.local_pos);
            let world_rot = body.rot.mul_rot(collider.local_rot);
            return (world_pos, world_rot);
        }
    }
    (collider.local_pos, collider.local_rot)
}

fn flip_manifold(mut m: ContactManifold) -> ContactManifold {
    m.normal = -m.normal;
    for cp in &mut m.contacts {
        std::mem::swap(&mut cp.local_a, &mut cp.local_b);
    }
    m
}

// ========== SAT: Ball-Ball ==========

pub fn sat_ball_ball(
    pos_a: Vec2,
    radius_a: f32,
    pos_b: Vec2,
    radius_b: f32,
) -> Option<ContactManifold> {
    let delta = pos_b - pos_a;
    let dist_sq = delta.length_sq();
    let total_r = radius_a + radius_b;

    if dist_sq >= total_r * total_r {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > f32::EPSILON {
        delta * (1.0 / dist)
    } else {
        Vec2::new(1.0, 0.0)
    };

    let penetration = total_r - dist;
    let world_contact = pos_a + normal * (radius_a - penetration * 0.5);

    let local_a = world_contact - pos_a;
    let local_b = world_contact - pos_b;

    Some(ContactManifold::new(normal).with_point(local_a, local_b, penetration))
}

// ========== SAT: Ball-Cuboid ==========

pub fn sat_ball_cuboid(
    ball_pos: Vec2,
    ball_r: f32,
    box_pos: Vec2,
    box_rot: Rot,
    hx: f32,
    hy: f32,
) -> Option<ContactManifold> {
    // Transform ball center to box local space
    let local_ball = box_rot.inv_mul_vec(ball_pos - box_pos);

    // Find closest point on box to ball center (in local space)
    let closest = Vec2::new(local_ball.x.clamp(-hx, hx), local_ball.y.clamp(-hy, hy));

    let delta = local_ball - closest;
    let dist_sq = delta.length_sq();

    if dist_sq >= ball_r * ball_r {
        return None;
    }

    let dist = dist_sq.sqrt();
    // Normal points from A (ball) to B (box) so the solver pushes ball away from box
    let local_normal = if dist > f32::EPSILON {
        -delta * (1.0 / dist)
    } else {
        // Ball center is inside the box — push out through nearest face.
        // Normal must still point from A (ball) toward B (box wall), i.e., toward exit.
        let dx = if local_ball.x >= 0.0 {
            hx - local_ball.x
        } else {
            hx + local_ball.x
        };
        let dy = if local_ball.y >= 0.0 {
            hy - local_ball.y
        } else {
            hy + local_ball.y
        };

        if dx < dy {
            Vec2::new(if local_ball.x >= 0.0 { -1.0 } else { 1.0 }, 0.0)
        } else {
            Vec2::new(0.0, if local_ball.y >= 0.0 { -1.0 } else { 1.0 })
        }
    };

    let penetration = if dist > f32::EPSILON {
        ball_r - dist
    } else {
        // Inside: need to push ball center past the face + ball radius
        let dx = if local_ball.x >= 0.0 {
            hx - local_ball.x
        } else {
            hx + local_ball.x
        };
        let dy = if local_ball.y >= 0.0 {
            hy - local_ball.y
        } else {
            hy + local_ball.y
        };
        ball_r + dx.min(dy)
    };
    let world_normal = box_rot.mul_vec(local_normal);
    // Contact point on ball surface facing the box (normal from A→B, so contact is at ball + n*r)
    let world_contact = ball_pos + world_normal * (ball_r - penetration * 0.5);

    let local_a = world_contact - ball_pos;
    let local_b = world_contact - box_pos;

    Some(ContactManifold::new(world_normal).with_point(local_a, local_b, penetration))
}

// ========== SAT: Ball-Polygon ==========

pub fn sat_ball_polygon(
    ball_pos: Vec2,
    ball_r: f32,
    poly_pos: Vec2,
    poly_rot: Rot,
    poly_verts: &[Vec2],
) -> Option<ContactManifold> {
    if poly_verts.is_empty() {
        return None;
    }

    // Transform ball center to polygon local space
    let local_ball = poly_rot.inv_mul_vec(ball_pos - poly_pos);

    // Find closest point on polygon edges
    let n = poly_verts.len();
    let mut min_dist_sq = f32::MAX;
    let mut closest_point = Vec2::zero();
    let mut inside = true;

    for i in 0..n {
        let j = (i + 1) % n;
        let edge = poly_verts[j] - poly_verts[i];
        let to_ball = local_ball - poly_verts[i];
        let t = to_ball.dot(edge) / edge.length_sq();
        let t_clamped = t.clamp(0.0, 1.0);
        let point = poly_verts[i] + edge * t_clamped;
        let dist_sq = local_ball.distance_sq(point);

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_point = point;
        }

        // Check winding direction to determine inside/outside
        if edge.perp().dot(to_ball) < 0.0 {
            inside = false;
        }
    }

    let dist = min_dist_sq.sqrt();

    if inside {
        // Ball center is inside polygon — push out through nearest edge.
        // Normal from A (ball) to B (polygon) = from ball center toward nearest edge.
        let penetration = ball_r + dist;
        let delta = closest_point - local_ball;
        let local_normal = if delta.length_sq() > f32::EPSILON * f32::EPSILON {
            delta * (1.0 / delta.length())
        } else {
            // Ball center is exactly on or extremely close to an edge.
            // Use the edge normal as fallback (like ball-cuboid does).
            let mut best_edge_normal = Vec2::new(1.0, 0.0);
            let mut min_dot = f32::MAX;
            for i in 0..n {
                let j = (i + 1) % n;
                let edge = poly_verts[j] - poly_verts[i];
                let edge_normal = edge.perp().normalize();
                let to_center = local_ball - poly_verts[i];
                let d = edge_normal.dot(to_center);
                if d.abs() < min_dot {
                    min_dot = d.abs();
                    best_edge_normal = if d >= 0.0 { edge_normal } else { -edge_normal };
                }
            }
            best_edge_normal
        };
        let world_normal = poly_rot.mul_vec(local_normal);
        let world_contact = ball_pos + world_normal * ball_r;
        let local_a = world_contact - ball_pos;
        let local_b = world_contact - poly_pos;
        Some(ContactManifold::new(world_normal).with_point(local_a, local_b, penetration))
    } else if dist < ball_r {
        let penetration = ball_r - dist;
        // Normal from A (ball) to B (polygon) = from ball center toward polygon edge
        let delta = closest_point - local_ball;
        let local_normal = if delta.length_sq() > f32::EPSILON * f32::EPSILON {
            delta * (1.0 / delta.length())
        } else {
            Vec2::new(1.0, 0.0)
        };
        let world_normal = poly_rot.mul_vec(local_normal);
        let world_contact = ball_pos + world_normal * (ball_r - penetration * 0.5);
        let local_a = world_contact - ball_pos;
        let local_b = world_contact - poly_pos;
        Some(ContactManifold::new(world_normal).with_point(local_a, local_b, penetration))
    } else {
        None
    }
}

// ========== SAT: Cuboid-Cuboid (Separating Axis Theorem) ==========

pub fn sat_cuboid_cuboid(
    pos_a: Vec2,
    rot_a: Rot,
    hxa: f32,
    hya: f32,
    pos_b: Vec2,
    rot_b: Rot,
    hxb: f32,
    hyb: f32,
) -> Option<ContactManifold> {
    let verts_a = cuboid_vertices(hxa, hya);
    let verts_b = cuboid_vertices(hxb, hyb);
    sat_polygon_polygon(pos_a, rot_a, &verts_a, pos_b, rot_b, &verts_b)
}

// ========== SAT: Polygon-Polygon ==========

pub fn sat_polygon_polygon(
    pos_a: Vec2,
    rot_a: Rot,
    verts_a: &[Vec2],
    pos_b: Vec2,
    rot_b: Rot,
    verts_b: &[Vec2],
) -> Option<ContactManifold> {
    if verts_a.is_empty() || verts_b.is_empty() {
        return None;
    }

    let mut min_penetration = f32::MAX;
    let mut best_axis = Vec2::zero();
    let mut best_from_a = true;

    // Test axes from A
    let na = verts_a.len();
    for i in 0..na {
        let j = (i + 1) % na;
        let world_edge = rot_a.mul_vec(verts_a[j] - verts_a[i]);
        let axis = world_edge.perp().normalize();

        let (min_a, max_a) = project_polygon(pos_a, rot_a, verts_a, axis);
        let (min_b, max_b) = project_polygon(pos_b, rot_b, verts_b, axis);

        let overlap = (max_a - min_b).min(max_b - min_a);
        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_penetration {
            min_penetration = overlap;
            best_axis = axis;
            best_from_a = true;
        }
    }

    // Test axes from B
    let nb = verts_b.len();
    for i in 0..nb {
        let j = (i + 1) % nb;
        let world_edge = rot_b.mul_vec(verts_b[j] - verts_b[i]);
        let axis = world_edge.perp().normalize();

        let (min_a, max_a) = project_polygon(pos_a, rot_a, verts_a, axis);
        let (min_b, max_b) = project_polygon(pos_b, rot_b, verts_b, axis);

        let overlap = (max_a - min_b).min(max_b - min_a);
        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_penetration {
            min_penetration = overlap;
            best_axis = axis;
            best_from_a = false;
        }
    }

    // Ensure normal points from A to B
    let d = pos_b - pos_a;
    let flipped = d.dot(best_axis) < 0.0;
    if flipped {
        best_axis = -best_axis;
    }

    // Reference face should have outward normal aligned with best_axis.
    // If the normal was flipped, swap reference/incident to maintain consistency.
    let use_a_as_ref = if flipped { !best_from_a } else { best_from_a };

    // Find contact points via edge clipping
    let (ref_verts, ref_pos, ref_rot, inc_verts, inc_pos, inc_rot) = if use_a_as_ref {
        (verts_a, pos_a, rot_a, verts_b, pos_b, rot_b)
    } else {
        (verts_b, pos_b, rot_b, verts_a, pos_a, rot_a)
    };

    let contacts = if use_a_as_ref {
        compute_contact_points(
            ref_pos, ref_rot, ref_verts, inc_pos, inc_rot, inc_verts, best_axis,
        )
    } else {
        // When B is reference, the reference face's outward normal aligns with -best_axis.
        // Since compute_contact_points uses min_dot (inward normal least aligned with passed normal),
        // we need to pass -best_axis so it selects the face whose inward normal is least aligned
        // with -best_axis (= most aligned with best_axis = outward normal aligned with -best_axis).
        compute_contact_points(
            ref_pos, ref_rot, ref_verts, inc_pos, inc_rot, inc_verts, -best_axis,
        )
    };

    let mut manifold = ContactManifold::new(best_axis);
    for (la, lb, pen) in contacts {
        // compute_contact_points returns (ref_frame, inc_frame).
        // When ref=B, swap so manifold always stores (A_frame, B_frame).
        if use_a_as_ref {
            manifold = manifold.with_point(la, lb, pen);
        } else {
            manifold = manifold.with_point(lb, la, pen);
        }
    }

    if manifold.contacts.is_empty() {
        // Fallback: use deepest penetrating vertices from both polygons
        let mut deepest_a = 0;
        let mut deepest_pen_a = -f32::MAX;
        for (i, v) in verts_a.iter().enumerate() {
            let world_v = pos_a + rot_a.mul_vec(*v);
            let proj = world_v.dot(best_axis);
            let pen = project_polygon(pos_b, rot_b, verts_b, best_axis).1 - proj;
            if pen > deepest_pen_a {
                deepest_pen_a = pen;
                deepest_a = i;
            }
        }

        let mut deepest_b = 0;
        let mut deepest_pen_b = -f32::MAX;
        for (i, v) in verts_b.iter().enumerate() {
            let world_v = pos_b + rot_b.mul_vec(*v);
            let proj = world_v.dot(-best_axis);
            let pen = project_polygon(pos_a, rot_a, verts_a, -best_axis).1 - proj;
            if pen > deepest_pen_b {
                deepest_pen_b = pen;
                deepest_b = i;
            }
        }

        // Add deepest vertex from A
        let world_va = pos_a + rot_a.mul_vec(verts_a[deepest_a]);
        let local_a = verts_a[deepest_a];
        let local_b = rot_b.inv_mul_vec(world_va - pos_b);
        manifold = manifold.with_point(local_a, local_b, deepest_pen_a.max(0.0));

        // Add deepest vertex from B if it's a different contact
        if deepest_pen_b > 0.0 && verts_a.len() + verts_b.len() > 3 {
            let world_vb = pos_b + rot_b.mul_vec(verts_b[deepest_b]);
            let local_b2 = verts_b[deepest_b];
            let local_a2 = rot_a.inv_mul_vec(world_vb - pos_a);
            manifold = manifold.with_point(local_a2, local_b2, deepest_pen_b.max(0.0));
        }
    }

    Some(manifold)
}

fn project_polygon(pos: Vec2, rot: Rot, verts: &[Vec2], axis: Vec2) -> (f32, f32) {
    let mut min = f32::MAX;
    let mut max = f32::MIN;
    for v in verts {
        let world_v = pos + rot.mul_vec(*v);
        let proj = world_v.dot(axis);
        min = min.min(proj);
        max = max.max(proj);
    }
    (min, max)
}

fn cuboid_vertices(hx: f32, hy: f32) -> [Vec2; 4] {
    [
        Vec2::new(-hx, -hy),
        Vec2::new(hx, -hy),
        Vec2::new(hx, hy),
        Vec2::new(-hx, hy),
    ]
}

/// Compute contact points via reference/incident edge clipping
fn compute_contact_points(
    ref_pos: Vec2,
    ref_rot: Rot,
    ref_verts: &[Vec2],
    inc_pos: Vec2,
    inc_rot: Rot,
    inc_verts: &[Vec2],
    normal: Vec2,
) -> Vec<(Vec2, Vec2, f32)> {
    // Find the reference face (edge most aligned with normal).
    // edge.perp() gives the inward normal for CCW polygons, so we negate it
    // to get the outward normal, then find the one most aligned with the contact normal.
    let ref_normal = ref_rot.inv_mul_vec(normal);
    let mut ref_idx = 0;
    let mut min_dot = f32::MAX;
    let n = ref_verts.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let edge = (ref_verts[j] - ref_verts[i]).perp().normalize();
        let d = edge.dot(ref_normal);
        if d < min_dot {
            min_dot = d;
            ref_idx = i;
        }
    }

    // Find the incident face (edge most anti-aligned with normal).
    // Same logic: edge.perp() is inward, we want the one whose outward normal
    // is most anti-aligned with the contact normal (i.e., most aligned with -normal).
    let inc_normal = inc_rot.inv_mul_vec(-normal);
    let mut inc_idx = 0;
    let mut min_dot = f32::MAX;
    let m = inc_verts.len();
    for i in 0..m {
        let j = (i + 1) % m;
        let edge = (inc_verts[j] - inc_verts[i]).perp().normalize();
        let d = edge.dot(inc_normal);
        if d < min_dot {
            min_dot = d;
            inc_idx = i;
        }
    }

    let ref_next = (ref_idx + 1) % n;
    let inc_next = (inc_idx + 1) % m;

    // Reference edge in world space
    let ref_v1 = ref_pos + ref_rot.mul_vec(ref_verts[ref_idx]);
    let ref_v2 = ref_pos + ref_rot.mul_vec(ref_verts[ref_next]);

    // Incident edge in world space
    let inc_v1 = inc_pos + inc_rot.mul_vec(inc_verts[inc_idx]);
    let inc_v2 = inc_pos + inc_rot.mul_vec(inc_verts[inc_next]);

    // Clip incident edge against reference edge side planes
    let ref_edge = (ref_v2 - ref_v1).normalize();
    let ref_tangent = ref_edge.perp();

    let mut clipped = vec![inc_v1, inc_v2];

    // Clip against left side plane
    let side1 = ref_v1.dot(-ref_edge);
    clip_line_segment(&mut clipped, -ref_edge, side1);

    // Clip against right side plane
    let side2 = ref_v2.dot(ref_edge);
    clip_line_segment(&mut clipped, ref_edge, side2);

    // Keep only points behind reference face (inside reference body).
    // ref_tangent is the inward normal of the reference face, so we want
    // points where point · ref_tangent > ref_offset (inside the body).
    // Equivalently, using -ref_tangent as the outward normal: dist_out <= 0 ↔ dist_in >= 0.
    let outward = -ref_tangent;
    let ref_offset_out = ref_v1.dot(outward);

    let mut contacts = Vec::new();
    for point in &clipped {
        let dist = point.dot(outward) - ref_offset_out;
        if dist <= 0.0 {
            let penetration = -dist;
            let local_a = ref_rot.inv_mul_vec(*point - ref_pos);
            let local_b = inc_rot.inv_mul_vec(*point - inc_pos);
            contacts.push((local_a, local_b, penetration));
        }
    }

    // Limit to 2 contacts for 2D block solver
    if contacts.len() > 2 {
        contacts.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
        contacts.truncate(2);
    }

    contacts
}

fn clip_line_segment(points: &mut Vec<Vec2>, normal: Vec2, offset: f32) {
    if points.len() < 2 {
        return;
    }

    let d0 = points[0].dot(normal) - offset;
    let d1 = points[1].dot(normal) - offset;

    if d0 * d1 < 0.0 {
        // One point is on each side, compute intersection
        let t = d0 / (d0 - d1);
        let intersection = points[0] + (points[1] - points[0]) * t;

        if d0 < 0.0 {
            points[1] = intersection;
        } else {
            points[0] = intersection;
        }
    } else if d0 <= 0.0 && d1 <= 0.0 {
        // Both points are behind or on the clip plane, keep both
    } else {
        // Both points are in front, remove all
        points.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ball_ball_overlap() {
        let m = sat_ball_ball(Vec2::new(0.0, 0.0), 1.0, Vec2::new(1.5, 0.0), 1.0);
        assert!(m.is_some());
        let m = m.unwrap();
        assert!((m.max_penetration() - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_ball_ball_no_overlap() {
        let m = sat_ball_ball(Vec2::new(0.0, 0.0), 1.0, Vec2::new(3.0, 0.0), 1.0);
        assert!(m.is_none());
    }

    #[test]
    fn test_ball_cuboid() {
        let m = sat_ball_cuboid(
            Vec2::new(0.0, 0.0),
            0.5,
            Vec2::new(0.8, 0.0),
            Rot::identity(),
            0.5,
            0.5,
        );
        assert!(m.is_some());
        let m = m.unwrap();
        assert!(m.max_penetration() > 0.0);
    }

    #[test]
    fn test_cuboid_cuboid() {
        let m = sat_cuboid_cuboid(
            Vec2::new(0.0, 0.0),
            Rot::identity(),
            0.5,
            0.5,
            Vec2::new(0.8, 0.0),
            Rot::identity(),
            0.5,
            0.5,
        );
        assert!(m.is_some());
        let m = m.unwrap();
        eprintln!(
            "cuboid_cuboid normal=({}, {}) pen={}",
            m.normal.x,
            m.normal.y,
            m.max_penetration()
        );
        for c in &m.contacts {
            eprintln!(
                "  local_a=({}, {}) local_b=({}, {}) pen={}",
                c.local_a.x, c.local_a.y, c.local_b.x, c.local_b.y, c.penetration
            );
        }
        assert!((m.max_penetration() - 0.2).abs() < 1e-5);
    }

    #[test]
    fn test_cuboid_cuboid_no_overlap() {
        let m = sat_cuboid_cuboid(
            Vec2::new(0.0, 0.0),
            Rot::identity(),
            0.5,
            0.5,
            Vec2::new(5.0, 0.0),
            Rot::identity(),
            0.5,
            0.5,
        );
        assert!(m.is_none());
    }

    #[test]
    fn test_polygon_polygon() {
        let tri_a = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.0, 0.5),
        ];
        let tri_b = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.0, 0.5),
        ];

        let m = sat_polygon_polygon(
            Vec2::new(0.0, 0.0),
            Rot::identity(),
            &tri_a,
            Vec2::new(0.5, 0.0),
            Rot::identity(),
            &tri_b,
        );
        assert!(m.is_some());
    }
}
