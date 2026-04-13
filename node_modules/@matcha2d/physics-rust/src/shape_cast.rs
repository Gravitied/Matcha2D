//! GJK **distance between two poses** in world space (Ball / Cuboid / Polygon).
//!
//! Used by TOI recovery as the **separation metric at a fixed trajectory parameter** `u`:
//! the world evaluates both bodies’ integrated poses at that `u`, then runs this distance query.
//! That is **not** Box2D’s `b2ShapeCast` conservative advancement (no fixed translation vector per
//! cast, no inner CA loop on Minkowski support). A future shape-cast / CA pass could reuse this
//! distance core with per-pair relative motion as the cast direction.
//!
//! Ported from Box2D `b2ShapeDistance` / `b2SolveSimplex2` / `b2SolveSimplex3` (MIT license).

use crate::collider::Shape;
use crate::math::{Rot, Vec2};

const MAX_ITERATIONS: usize = 24;

#[inline]
fn cross_sv(s: f32, v: Vec2) -> Vec2 {
    Vec2::new(-s * v.y, s * v.x)
}

/// Furthest point on `shape` in world space in direction `dir`.
fn support_world(shape: &Shape, origin: Vec2, rot: Rot, dir: Vec2) -> (Vec2, u8) {
    let d = dir.normalize();
    match shape {
        Shape::Ball { radius } => (origin + d * (*radius), 0),
        Shape::Cuboid { hx, hy } => {
            let corners = [
                Vec2::new(-*hx, -*hy),
                Vec2::new(*hx, -*hy),
                Vec2::new(*hx, *hy),
                Vec2::new(-*hx, *hy),
            ];
            let mut best_i = 0u8;
            let mut best_dot = f32::NEG_INFINITY;
            for (i, lc) in corners.iter().enumerate() {
                let w = origin + rot.mul_vec(*lc);
                let dot = w.dot(d);
                if dot > best_dot {
                    best_dot = dot;
                    best_i = i as u8;
                }
            }
            (origin + rot.mul_vec(corners[best_i as usize]), best_i)
        }
        Shape::Polygon { vertices } => {
            if vertices.is_empty() {
                return (origin, 0);
            }
            let mut best_i = 0u8;
            let mut best_dot = f32::NEG_INFINITY;
            for (i, lc) in vertices.iter().enumerate() {
                let w = origin + rot.mul_vec(*lc);
                let dot = w.dot(d);
                if dot > best_dot {
                    best_dot = dot;
                    best_i = (i.min(255)) as u8;
                }
            }
            (origin + rot.mul_vec(vertices[best_i as usize]), best_i)
        }
    }
}

#[derive(Clone)]
struct Vert {
    w: Vec2,
    w_a: Vec2,
    w_b: Vec2,
    ia: u8,
    ib: u8,
    a: f32,
}

fn solve_simplex2(v: &mut [Vert; 3], count: &mut usize) -> Vec2 {
    let w1 = v[0].w;
    let w2 = v[1].w;
    let e12 = w2 - w1;

    let d12_2 = -w1.dot(e12);
    if d12_2 <= 0.0 {
        *count = 1;
        return -w1;
    }

    let d12_1 = w2.dot(e12);
    if d12_1 <= 0.0 {
        v[0] = v[1].clone();
        *count = 1;
        return -v[0].w;
    }

    let denom12 = (d12_1 + d12_2).max(1e-30);
    let inv_d12 = 1.0 / denom12;
    v[0].a = d12_1 * inv_d12;
    v[1].a = d12_2 * inv_d12;
    *count = 2;
    let s = (w1 + w2).cross(e12);
    cross_sv(s, e12)
}

fn solve_simplex3(v: &mut [Vert; 3], count: &mut usize) -> Vec2 {
    let w1 = v[0].w;
    let w2 = v[1].w;
    let w3 = v[2].w;

    let e12 = w2 - w1;
    let w1e12 = w1.dot(e12);
    let w2e12 = w2.dot(e12);
    let d12_1 = w2e12;
    let d12_2 = -w1e12;

    let e13 = w3 - w1;
    let w1e13 = w1.dot(e13);
    let w3e13 = w3.dot(e13);
    let d13_1 = w3e13;
    let d13_2 = -w1e13;

    let e23 = w3 - w2;
    let w2e23 = w2.dot(e23);
    let w3e23 = w3.dot(e23);
    let d23_1 = w3e23;
    let d23_2 = -w2e23;

    let n123 = e12.cross(e13);

    let d123_1 = n123 * w2.cross(w3);
    let d123_2 = n123 * w3.cross(w1);
    let d123_3 = n123 * w1.cross(w2);

    if d12_2 <= 0.0 && d13_2 <= 0.0 {
        v[0].a = 1.0;
        *count = 1;
        return -w1;
    }

    if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
        let denom12b = (d12_1 + d12_2).max(1e-30);
        let inv_d12 = 1.0 / denom12b;
        v[0].a = d12_1 * inv_d12;
        v[1].a = d12_2 * inv_d12;
        *count = 2;
        let s = (w1 + w2).cross(e12);
        return cross_sv(s, e12);
    }

    if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
        let denom13 = (d13_1 + d13_2).max(1e-30);
        let inv_d13 = 1.0 / denom13;
        v[0].a = d13_1 * inv_d13;
        v[2].a = d13_2 * inv_d13;
        v[1] = v[2].clone();
        *count = 2;
        let s = (w1 + w3).cross(e13);
        return cross_sv(s, e13);
    }

    if d12_1 <= 0.0 && d23_2 <= 0.0 {
        v[0] = v[1].clone();
        v[0].a = 1.0;
        *count = 1;
        return -w2;
    }

    if d13_1 <= 0.0 && d23_1 <= 0.0 {
        v[0] = v[2].clone();
        v[0].a = 1.0;
        *count = 1;
        return -w3;
    }

    if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
        let denom23 = (d23_1 + d23_2).max(1e-30);
        let inv_d23 = 1.0 / denom23;
        v[1].a = d23_1 * inv_d23;
        v[2].a = d23_2 * inv_d23;
        v[0] = v[2].clone();
        *count = 2;
        let e = v[0].w - v[1].w;
        let s = (v[1].w + v[0].w).cross(e);
        return cross_sv(s, e);
    }

    let denom123 = (d123_1 + d123_2 + d123_3).max(1e-30);
    let inv_d123 = 1.0 / denom123;
    v[0].a = d123_1 * inv_d123;
    v[1].a = d123_2 * inv_d123;
    v[2].a = d123_3 * inv_d123;
    *count = 3;
    Vec2::zero()
}

fn witness_points(v: &[Vert; 3], count: usize) -> (Vec2, Vec2) {
    match count {
        1 => (v[0].w_a, v[0].w_b),
        2 => (
            v[0].w_a * v[0].a + v[1].w_a * v[1].a,
            v[0].w_b * v[0].a + v[1].w_b * v[1].a,
        ),
        3 => {
            let a = v[0].w_a * v[0].a + v[1].w_a * v[1].a + v[2].w_a * v[2].a;
            (a, a)
        }
        _ => (Vec2::zero(), Vec2::zero()),
    }
}

/// Collider shape frame in world space (same convention as `narrowphase::get_world_transform`).
#[inline]
pub fn collider_world_frame(body_pos: Vec2, body_rot: Rot, col: &crate::collider::Collider) -> (Vec2, Rot) {
    (
        body_pos + body_rot.mul_vec(col.local_pos),
        body_rot.mul_rot(col.local_rot),
    )
}

/// GJK distance between two convex shapes in world space. `None` = overlap (CSO contains origin).
pub fn gjk_distance(
    shape_a: &Shape,
    origin_a: Vec2,
    rot_a: Rot,
    shape_b: &Shape,
    origin_b: Vec2,
    rot_b: Rot,
) -> Option<f32> {
    let (wa0, ia0) = support_world(shape_a, origin_a, rot_a, Vec2::new(1.0, 0.0));
    let (wb0, ib0) = support_world(shape_b, origin_b, rot_b, Vec2::new(-1.0, 0.0));

    let mut v: [Vert; 3] = [
        Vert {
            w: wa0 - wb0,
            w_a: wa0,
            w_b: wb0,
            ia: ia0,
            ib: ib0,
            a: 1.0,
        },
        Vert {
            w: Vec2::zero(),
            w_a: Vec2::zero(),
            w_b: Vec2::zero(),
            ia: 0,
            ib: 0,
            a: 0.0,
        },
        Vert {
            w: Vec2::zero(),
            w_a: Vec2::zero(),
            w_b: Vec2::zero(),
            ia: 0,
            ib: 0,
            a: 0.0,
        },
    ];

    let mut count = 1usize;
    let mut save_a = [0u8; 3];
    let mut save_b = [0u8; 3];

    let mut iteration = 0usize;
    while iteration < MAX_ITERATIONS {
        let save_count = count;
        for i in 0..save_count {
            save_a[i] = v[i].ia;
            save_b[i] = v[i].ib;
        }

        let d = match count {
            1 => -v[0].w,
            2 => solve_simplex2(&mut v, &mut count),
            3 => solve_simplex3(&mut v, &mut count),
            _ => return None,
        };

        if count == 3 {
            return None;
        }

        if d.length_sq() < f32::EPSILON * f32::EPSILON {
            return None;
        }

        let dir = d.normalize();
        let (wa, ia) = support_world(shape_a, origin_a, rot_a, dir);
        let (wb, ib) = support_world(shape_b, origin_b, rot_b, -dir);
        let w = wa - wb;

        iteration += 1;

        let mut duplicate = false;
        for i in 0..save_count {
            if ia == save_a[i] && ib == save_b[i] {
                duplicate = true;
                break;
            }
        }
        if duplicate {
            let (pa, pb) = witness_points(&v, count);
            return Some(pa.distance(pb));
        }

        v[count] = Vert {
            w,
            w_a: wa,
            w_b: wb,
            ia,
            ib,
            a: 0.0,
        };
        count += 1;
    }

    let (pa, pb) = witness_points(&v, count);
    Some(pa.distance(pb))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::Shape;

    #[test]
    fn gjk_separated_unit_boxes() {
        let a = Shape::cuboid(0.5, 0.5);
        let b = Shape::cuboid(0.5, 0.5);
        let d = gjk_distance(&a, Vec2::new(0.0, 0.0), Rot::identity(), &b, Vec2::new(2.0, 0.0), Rot::identity());
        assert!(d.is_some());
        let d = d.unwrap();
        assert!((d - 1.0).abs() < 0.05, "expected ~1.0 gap, got {}", d);
    }

    #[test]
    fn gjk_overlapping_boxes() {
        let a = Shape::cuboid(0.5, 0.5);
        let b = Shape::cuboid(0.5, 0.5);
        let d = gjk_distance(&a, Vec2::new(0.0, 0.0), Rot::identity(), &b, Vec2::new(0.5, 0.0), Rot::identity());
        assert!(d.is_none());
    }
}
