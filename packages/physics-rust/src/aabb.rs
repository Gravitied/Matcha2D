use crate::math::Vec2;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AABB {
    pub min: Vec2,
    pub max: Vec2,
}

impl AABB {
    #[inline]
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    #[inline]
    pub fn new_from_coords(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Self {
        Self {
            min: Vec2::new(min_x, min_y),
            max: Vec2::new(max_x, max_y),
        }
    }

    #[inline]
    pub fn from_center_half_extents(center: Vec2, half_extents: Vec2) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    #[inline]
    pub fn overlaps(self, other: Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    #[inline]
    pub fn contains(self, other: Self) -> bool {
        self.min.x <= other.min.x
            && self.max.x >= other.max.x
            && self.min.y <= other.min.y
            && self.max.y >= other.max.y
    }

    #[inline]
    pub fn merge(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    #[inline]
    pub fn area(self) -> f32 {
        let d = self.max - self.min;
        d.x * d.y
    }

    #[inline]
    pub fn perimeter(self) -> f32 {
        let d = self.max - self.min;
        2.0 * (d.x + d.y)
    }

    #[inline]
    pub fn center(self) -> Vec2 {
        (self.min + self.max) * 0.5
    }

    #[inline]
    pub fn half_extents(self) -> Vec2 {
        (self.max - self.min) * 0.5
    }

    #[inline]
    pub fn expanded(self, margin: Vec2) -> Self {
        Self {
            min: self.min - margin,
            max: self.max + margin,
        }
    }

    #[inline]
    pub fn ray_cast(self, origin: Vec2, direction: Vec2) -> Option<f32> {
        let inv_dir = Vec2::new(1.0 / direction.x, 1.0 / direction.y);
        let mut t1 = (self.min.x - origin.x) * inv_dir.x;
        let mut t2 = (self.max.x - origin.x) * inv_dir.x;
        let mut tmin = t1.min(t2);
        let mut tmax = t1.max(t2);

        t1 = (self.min.y - origin.y) * inv_dir.y;
        t2 = (self.max.y - origin.y) * inv_dir.y;
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));

        if tmax >= tmin.max(0.0) && tmin < f32::MAX {
            Some(tmin.max(0.0))
        } else {
            None
        }
    }
}

/// Compute AABB for a circle at the given position
#[inline]
pub fn aabb_ball(pos: Vec2, radius: f32) -> AABB {
    let r = Vec2::new(radius, radius);
    AABB::new(pos - r, pos + r)
}

/// Compute AABB for a box (cuboid) at the given position and rotation
#[inline]
pub fn aabb_cuboid(pos: Vec2, rot: crate::math::Rot, hx: f32, hy: f32) -> AABB {
    let ex = rot.mul_vec(Vec2::new(hx, 0.0)).abs();
    let ey = rot.mul_vec(Vec2::new(0.0, hy)).abs();
    let half = ex + ey;
    AABB::new(pos - half, pos + half)
}

/// Compute AABB for a convex polygon at the given position and rotation
#[inline]
pub fn aabb_polygon(pos: Vec2, rot: crate::math::Rot, vertices: &[Vec2]) -> AABB {
    if vertices.is_empty() {
        return AABB::new(pos, pos);
    }
    let mut min = Vec2::new(f32::MAX, f32::MAX);
    let mut max = Vec2::new(f32::MIN, f32::MIN);
    for &v in vertices {
        let world_v = pos + rot.mul_vec(v);
        min = min.min(world_v);
        max = max.max(world_v);
    }
    AABB::new(min, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rot;

    #[test]
    fn test_overlap() {
        let a = AABB::new_from_coords(0.0, 0.0, 2.0, 2.0);
        let b = AABB::new_from_coords(1.0, 1.0, 3.0, 3.0);
        let c = AABB::new_from_coords(5.0, 5.0, 6.0, 6.0);
        assert!(a.overlaps(b));
        assert!(!a.overlaps(c));
    }

    #[test]
    fn test_merge() {
        let a = AABB::new_from_coords(0.0, 0.0, 1.0, 1.0);
        let b = AABB::new_from_coords(2.0, 3.0, 4.0, 5.0);
        let merged = a.merge(b);
        assert_eq!(merged.min, Vec2::new(0.0, 0.0));
        assert_eq!(merged.max, Vec2::new(4.0, 5.0));
    }

    #[test]
    fn test_area_perimeter() {
        let a = AABB::new_from_coords(0.0, 0.0, 3.0, 4.0);
        assert_eq!(a.area(), 12.0);
        assert_eq!(a.perimeter(), 14.0);
    }

    #[test]
    fn test_aabb_ball() {
        let aabb = aabb_ball(Vec2::new(1.0, 2.0), 0.5);
        assert_eq!(aabb.min, Vec2::new(0.5, 1.5));
        assert_eq!(aabb.max, Vec2::new(1.5, 2.5));
    }

    #[test]
    fn test_aabb_cuboid() {
        let aabb = aabb_cuboid(Vec2::new(0.0, 0.0), Rot::identity(), 1.0, 1.0);
        assert_eq!(aabb.min, Vec2::new(-1.0, -1.0));
        assert_eq!(aabb.max, Vec2::new(1.0, 1.0));

        // Rotated 45 degrees - AABB should be larger
        let aabb_rot = aabb_cuboid(
            Vec2::new(0.0, 0.0),
            Rot::from_angle(std::f32::consts::FRAC_PI_4),
            1.0,
            1.0,
        );
        let diag = std::f32::consts::SQRT_2;
        assert!((aabb_rot.max.x - diag).abs() < 1e-5);
    }

    #[test]
    fn test_ray_cast() {
        let aabb = AABB::new_from_coords(0.0, 0.0, 2.0, 2.0);
        let t = aabb.ray_cast(Vec2::new(-1.0, 1.0), Vec2::new(1.0, 0.0));
        assert!(t.is_some());
        assert!((t.unwrap() - 1.0).abs() < 1e-5);

        let miss = aabb.ray_cast(Vec2::new(-1.0, 1.0), Vec2::new(0.0, 1.0));
        assert!(miss.is_none());
    }
}
