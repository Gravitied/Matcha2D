use crate::math::{Rot, Vec2};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidBodyType {
    Dynamic = 0,
    Static = 1,
    Kinematic = 2,
}

impl Default for RigidBodyType {
    fn default() -> Self {
        Self::Dynamic
    }
}

#[derive(Clone, Debug)]
pub struct RigidBody {
    pub pos: Vec2,
    pub rot: Rot,
    pub linvel: Vec2,
    pub angvel: f32,
    pub force: Vec2,
    pub torque: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: f32,
    pub inv_inertia: f32,
    pub body_type: RigidBodyType,
    pub enabled: bool,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub user_data: u32,
    // Sleep system
    pub sleeping: bool,
    pub sleep_timer: f32,
    // Collision filtering: bitfield group and mask
    pub collision_groups: u32,
    pub collision_mask: u32,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            pos: Vec2::zero(),
            rot: Rot::identity(),
            linvel: Vec2::zero(),
            angvel: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            mass: 1.0,
            inv_mass: 1.0,
            inertia: 1.0,
            inv_inertia: 0.0,
            body_type: RigidBodyType::Dynamic,
            enabled: true,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            user_data: 0,
            sleeping: false,
            sleep_timer: 0.0,
            collision_groups: 0xFFFF_FFFF, // default: member of all groups
            collision_mask: 0xFFFF_FFFF,   // default: collides with everything
        }
    }
}

impl RigidBody {
    pub fn dynamic(pos: Vec2) -> Self {
        Self {
            pos,
            ..Default::default()
        }
    }

    pub fn static_body(pos: Vec2) -> Self {
        Self {
            pos,
            body_type: RigidBodyType::Static,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            ..Default::default()
        }
    }

    pub fn kinematic(pos: Vec2) -> Self {
        Self {
            pos,
            body_type: RigidBodyType::Kinematic,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            ..Default::default()
        }
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == RigidBodyType::Dynamic
    }

    pub fn is_static(&self) -> bool {
        self.body_type == RigidBodyType::Static
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type == RigidBodyType::Kinematic
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
    }

    pub fn set_inertia(&mut self, inertia: f32) {
        self.inertia = inertia;
        self.inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
    }

    pub fn set_mass_properties(&mut self, mass: f32, inertia: f32) {
        self.set_mass(mass);
        self.set_inertia(inertia);
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        if self.is_dynamic() {
            self.linvel += impulse * self.inv_mass;
        }
    }

    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        if self.is_dynamic() {
            self.angvel += impulse * self.inv_inertia;
        }
    }

    pub fn apply_force(&mut self, force: Vec2) {
        if self.is_dynamic() {
            self.force += force;
        }
    }

    pub fn apply_torque(&mut self, torque: f32) {
        if self.is_dynamic() {
            self.torque += torque;
        }
    }

    /// Wake a body from sleep (resets sleep timer).
    pub fn wake(&mut self) {
        self.sleeping = false;
        self.sleep_timer = 0.0;
    }

    /// Put a body to sleep immediately.
    pub fn sleep(&mut self) {
        if self.is_dynamic() {
            self.sleeping = true;
            self.linvel = Vec2::zero();
            self.angvel = 0.0;
        }
    }

    /// Returns true if this body should collide with `other` based on group/mask filtering.
    pub fn can_collide_with(&self, other: &RigidBody) -> bool {
        (self.collision_mask & other.collision_groups) != 0
            && (other.collision_mask & self.collision_groups) != 0
    }
}

pub struct RigidBodyBuilder {
    body: RigidBody,
}

impl RigidBodyBuilder {
    pub fn dynamic(pos: Vec2) -> Self {
        Self {
            body: RigidBody::dynamic(pos),
        }
    }

    pub fn static_body(pos: Vec2) -> Self {
        Self {
            body: RigidBody::static_body(pos),
        }
    }

    pub fn kinematic(pos: Vec2) -> Self {
        Self {
            body: RigidBody::kinematic(pos),
        }
    }

    pub fn linvel(mut self, v: Vec2) -> Self {
        self.body.linvel = v;
        self
    }

    pub fn angvel(mut self, v: f32) -> Self {
        self.body.angvel = v;
        self
    }

    pub fn mass(mut self, m: f32) -> Self {
        self.body.set_mass(m);
        self
    }

    pub fn inertia(mut self, i: f32) -> Self {
        self.body.set_inertia(i);
        self
    }

    pub fn gravity_scale(mut self, s: f32) -> Self {
        self.body.gravity_scale = s;
        self
    }

    pub fn linear_damping(mut self, d: f32) -> Self {
        self.body.linear_damping = d;
        self
    }

    pub fn angular_damping(mut self, d: f32) -> Self {
        self.body.angular_damping = d;
        self
    }

    pub fn user_data(mut self, data: u32) -> Self {
        self.body.user_data = data;
        self
    }

    pub fn collision_groups(mut self, groups: u32) -> Self {
        self.body.collision_groups = groups;
        self
    }

    pub fn collision_mask(mut self, mask: u32) -> Self {
        self.body.collision_mask = mask;
        self
    }

    pub fn sleeping(mut self, sleeping: bool) -> Self {
        self.body.sleeping = sleeping;
        self
    }

    pub fn build(self) -> RigidBody {
        self.body
    }
}

/// Generational handle for rigid bodies
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RigidBodyHandle(pub u32);

impl RigidBodyHandle {
    pub const INVALID: Self = Self(u32::MAX);

    #[inline]
    pub fn index(self) -> usize {
        (self.0 & 0x00FF_FFFF) as usize
    }

    #[inline]
    pub fn generation(self) -> u8 {
        (self.0 >> 24) as u8
    }

    #[inline]
    pub fn is_valid(self) -> bool {
        self.0 != u32::MAX
    }
}

/// Arena-based set of rigid bodies with generational handles
pub struct RigidBodySet {
    bodies: Vec<RigidBody>,
    generations: Vec<u8>,
    free_indices: Vec<usize>,
}

impl RigidBodySet {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            generations: Vec::new(),
            free_indices: Vec::new(),
        }
    }

    pub fn insert(&mut self, body: RigidBody) -> RigidBodyHandle {
        if let Some(idx) = self.free_indices.pop() {
            self.generations[idx] = self.generations[idx].wrapping_add(1);
            self.bodies[idx] = body;
            RigidBodyHandle(idx as u32 | ((self.generations[idx] as u32) << 24))
        } else {
            let idx = self.bodies.len();
            self.bodies.push(body);
            self.generations.push(0);
            RigidBodyHandle(idx as u32)
        }
    }

    pub fn remove(&mut self, handle: RigidBodyHandle) -> Option<RigidBody> {
        let idx = handle.index();
        if idx < self.bodies.len() && self.generations[idx] == handle.generation() {
            self.generations[idx] = self.generations[idx].wrapping_add(1);
            let body = std::mem::replace(&mut self.bodies[idx], RigidBody::default());
            self.free_indices.push(idx);
            Some(body)
        } else {
            None
        }
    }

    #[inline]
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        let idx = handle.index();
        if idx < self.bodies.len() && self.generations[idx] == handle.generation() {
            Some(&self.bodies[idx])
        } else {
            None
        }
    }

    #[inline]
    pub fn get_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        let idx = handle.index();
        if idx < self.bodies.len() && self.generations[idx] == handle.generation() {
            Some(&mut self.bodies[idx])
        } else {
            None
        }
    }

    #[inline]
    pub fn get_unchecked(&self, idx: usize) -> &RigidBody {
        &self.bodies[idx]
    }

    #[inline]
    pub fn get_unchecked_mut(&mut self, idx: usize) -> &mut RigidBody {
        &mut self.bodies[idx]
    }

    pub fn len(&self) -> usize {
        self.bodies.len() - self.free_indices.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter().enumerate().filter_map(move |(i, body)| {
            if !self.free_indices.contains(&i) {
                Some((
                    RigidBodyHandle(i as u32 | ((self.generations[i] as u32) << 24)),
                    body,
                ))
            } else {
                None
            }
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, &mut RigidBody)> {
        let free = &self.free_indices;
        let gens = &self.generations;
        self.bodies
            .iter_mut()
            .enumerate()
            .filter_map(move |(i, body)| {
                if !free.contains(&i) {
                    Some((RigidBodyHandle(i as u32 | ((gens[i] as u32) << 24)), body))
                } else {
                    None
                }
            })
    }

    /// Returns an iterator over all live body indices (not handles)
    pub fn live_indices(&self) -> Vec<usize> {
        (0..self.bodies.len())
            .filter(|i| !self.free_indices.contains(i))
            .collect()
    }
}

impl Default for RigidBodySet {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_handle_generation() {
        let h = RigidBodyHandle(0x0500_0003);
        assert_eq!(h.index(), 3);
        assert_eq!(h.generation(), 5);
    }

    #[test]
    fn test_body_set() {
        let mut set = RigidBodySet::new();
        let h1 = set.insert(RigidBody::dynamic(Vec2::new(1.0, 2.0)));
        let h2 = set.insert(RigidBody::static_body(Vec2::new(3.0, 4.0)));
        assert_eq!(set.len(), 2);

        let body = set.get(h1).unwrap();
        assert_eq!(body.pos, Vec2::new(1.0, 2.0));
        assert!(body.is_dynamic());

        let body = set.get(h2).unwrap();
        assert!(body.is_static());

        set.remove(h1).unwrap();
        assert_eq!(set.len(), 1);
        assert!(set.get(h1).is_none());
    }

    #[test]
    fn test_builder() {
        let body = RigidBodyBuilder::dynamic(Vec2::new(1.0, 2.0))
            .linvel(Vec2::new(3.0, 4.0))
            .mass(2.0)
            .build();
        assert_eq!(body.pos, Vec2::new(1.0, 2.0));
        assert_eq!(body.linvel, Vec2::new(3.0, 4.0));
        assert_eq!(body.mass, 2.0);
        assert_eq!(body.inv_mass, 0.5);
    }
}
