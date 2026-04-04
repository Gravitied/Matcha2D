use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    #[inline]
    pub fn cross(self, rhs: Self) -> f32 {
        self.x * rhs.y - self.y * rhs.x
    }

    #[inline]
    pub fn length_sq(self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_sq().sqrt()
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > f32::EPSILON {
            self * (1.0 / len)
        } else {
            Self::zero()
        }
    }

    #[inline]
    pub fn try_normalize(self) -> Option<Self> {
        let len_sq = self.length_sq();
        if len_sq > f32::EPSILON * f32::EPSILON {
            Some(self * (1.0 / len_sq.sqrt()))
        } else {
            None
        }
    }

    #[inline]
    pub fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }

    #[inline]
    pub fn negate(self) -> Self {
        Self::new(-self.x, -self.y)
    }

    #[inline]
    pub fn distance(self, rhs: Self) -> f32 {
        (self - rhs).length()
    }

    #[inline]
    pub fn distance_sq(self, rhs: Self) -> f32 {
        (self - rhs).length_sq()
    }

    #[inline]
    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        self + (rhs - self) * t
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs())
    }

    #[inline]
    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y))
    }

    #[inline]
    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y))
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s)
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, s: f32) {
        self.x *= s;
        self.y *= s;
    }
}

impl Div<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn div(self, s: f32) -> Self {
        let inv = 1.0 / s;
        Self::new(self.x * inv, self.y * inv)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Rot {
    pub cos: f32,
    pub sin: f32,
}

impl Rot {
    #[inline]
    pub fn identity() -> Self {
        Self { cos: 1.0, sin: 0.0 }
    }

    #[inline]
    pub fn from_angle(angle: f32) -> Self {
        Self {
            cos: angle.cos(),
            sin: angle.sin(),
        }
    }

    #[inline]
    pub fn mul_vec(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.cos * v.x - self.sin * v.y,
            self.sin * v.x + self.cos * v.y,
        )
    }

    #[inline]
    pub fn inv_mul_vec(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.cos * v.x + self.sin * v.y,
            -self.sin * v.x + self.cos * v.y,
        )
    }

    #[inline]
    pub fn inv(self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    #[inline]
    pub fn mul_rot(self, rhs: Self) -> Self {
        Self {
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
        }
    }

    #[inline]
    pub fn angle(self) -> f32 {
        self.sin.atan2(self.cos)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec2_basic() {
        let a = Vec2::new(3.0, 4.0);
        assert_eq!(a.length_sq(), 25.0);
        assert!((a.length() - 5.0).abs() < 1e-6);

        let b = a.normalize();
        assert!((b.length() - 1.0).abs() < 1e-6);
        assert!((b.x - 0.6).abs() < 1e-6);
        assert!((b.y - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_vec2_ops() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        assert_eq!(a + b, Vec2::new(4.0, 6.0));
        assert_eq!(a - b, Vec2::new(-2.0, -2.0));
        assert_eq!(a * 2.0, Vec2::new(2.0, 4.0));
        assert_eq!(a.dot(b), 11.0);
        assert_eq!(a.cross(b), -2.0);
        assert_eq!(a.perp(), Vec2::new(-2.0, 1.0));
    }

    #[test]
    fn test_rot() {
        let r = Rot::from_angle(std::f32::consts::PI / 2.0);
        let v = Vec2::new(1.0, 0.0);
        let rotated = r.mul_vec(v);
        assert!((rotated.x).abs() < 1e-6);
        assert!((rotated.y - 1.0).abs() < 1e-6);

        let back = r.inv_mul_vec(rotated);
        assert!((back.x - 1.0).abs() < 1e-6);
        assert!((back.y).abs() < 1e-6);
    }
}
