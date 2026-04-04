use crate::collider::ColliderHandle;
use crate::math::Vec2;

#[derive(Clone, Debug)]
pub struct ContactPoint {
    pub local_a: Vec2,
    pub local_b: Vec2,
    pub penetration: f32,
    pub impulse_id: u32,
    /// Whether this contact is new this frame (for restitution gating)
    pub is_new: bool,
}

#[derive(Clone, Debug)]
pub struct ContactManifold {
    pub normal: Vec2,
    pub contacts: Vec<ContactPoint>,
}

impl ContactManifold {
    pub fn new(normal: Vec2) -> Self {
        Self {
            normal,
            contacts: Vec::new(),
        }
    }

    pub fn with_point(mut self, local_a: Vec2, local_b: Vec2, penetration: f32) -> Self {
        self.contacts.push(ContactPoint {
            local_a,
            local_b,
            penetration,
            impulse_id: self.contacts.len() as u32,
            is_new: true,
        });
        self
    }

    pub fn max_penetration(&self) -> f32 {
        self.contacts
            .iter()
            .map(|c| c.penetration)
            .fold(0.0, f32::max)
    }
}

#[derive(Clone, Debug)]
pub struct ContactPair {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub manifold: ContactManifold,
    pub prev_accumulated_normal_impulse: f32,
    pub prev_accumulated_tangent_impulse: f32,
    /// Per-contact accumulated impulses for block solver
    pub prev_accumulated_impulses: Vec<(f32, f32)>,
}

impl ContactPair {
    pub fn new(
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
        manifold: ContactManifold,
    ) -> Self {
        let prev_impulses = manifold.contacts.iter().map(|_| (0.0, 0.0)).collect();
        Self {
            collider_a,
            collider_b,
            manifold,
            prev_accumulated_normal_impulse: 0.0,
            prev_accumulated_tangent_impulse: 0.0,
            prev_accumulated_impulses: prev_impulses,
        }
    }
}

pub enum ContactEvent {
    Started(ColliderHandle, ColliderHandle),
    Stopped(ColliderHandle, ColliderHandle),
}

pub struct ContactTracker {
    pairs: std::collections::HashMap<(ColliderHandle, ColliderHandle), ContactPair>,
    active_pairs: Vec<(ColliderHandle, ColliderHandle)>,
    pub events: Vec<ContactEvent>,
}

impl ContactTracker {
    pub fn new() -> Self {
        Self {
            pairs: std::collections::HashMap::new(),
            active_pairs: Vec::new(),
            events: Vec::new(),
        }
    }

    pub fn update(
        &mut self,
        new_manifolds: Vec<(ColliderHandle, ColliderHandle, ContactManifold)>,
    ) {
        self.active_pairs.clear();
        self.events.clear();

        // Track which old pairs are still active
        let mut still_active = std::collections::HashSet::new();

        for (ca, cb, mut manifold) in new_manifolds {
            let key = (ca, cb);
            still_active.insert(key);
            self.active_pairs.push(key);

            if let Some(pair) = self.pairs.get_mut(&key) {
                let old_contacts = &pair.manifold.contacts;
                pair.prev_accumulated_impulses = manifold
                    .contacts
                    .iter()
                    .enumerate()
                    .map(|(_new_idx, new_c)| {
                        let mut best_match: Option<(usize, f32, u32)> = None;
                        for (old_idx, old_c) in old_contacts.iter().enumerate() {
                            let dx = new_c.local_a.x - old_c.local_a.x;
                            let dy = new_c.local_a.y - old_c.local_a.y;
                            let dist_sq = dx * dx + dy * dy;
                            if best_match.is_none() || dist_sq < best_match.unwrap().1 {
                                best_match = Some((old_idx, dist_sq, old_c.impulse_id));
                            }
                        }

                        if let Some((_old_idx, dist_sq, impulse_id)) = best_match {
                            if dist_sq < 0.01 {
                                let idx = impulse_id as usize;
                                if idx < pair.prev_accumulated_impulses.len() {
                                    return pair.prev_accumulated_impulses[idx];
                                }
                            }
                        }
                        (0.0, 0.0)
                    })
                    .collect();

                // Update is_new flags on the new manifold contacts
                let old_contacts = &pair.manifold.contacts;
                for new_c in &mut manifold.contacts {
                    let mut found = false;
                    for old_c in old_contacts {
                        let dx = new_c.local_a.x - old_c.local_a.x;
                        let dy = new_c.local_a.y - old_c.local_a.y;
                        if dx * dx + dy * dy < 0.01 {
                            new_c.is_new = false;
                            found = true;
                            break;
                        }
                    }
                    if !found {
                        new_c.is_new = true;
                    }
                }

                pair.manifold = manifold;
            } else {
                // New contact
                self.events.push(ContactEvent::Started(ca, cb));
                self.pairs.insert(key, ContactPair::new(ca, cb, manifold));
            }
        }

        // Remove pairs that are no longer active
        let old_keys: Vec<_> = self.pairs.keys().copied().collect();
        for key in old_keys {
            if !still_active.contains(&key) {
                self.events.push(ContactEvent::Stopped(key.0, key.1));
                self.pairs.remove(&key);
            }
        }
    }

    pub fn pairs(&self) -> impl Iterator<Item = &ContactPair> {
        self.pairs.values()
    }

    pub fn pairs_mut(&mut self) -> impl Iterator<Item = &mut ContactPair> {
        self.pairs.values_mut()
    }

    pub fn pair_count(&self) -> usize {
        self.pairs.len()
    }

    pub fn clear(&mut self) {
        self.pairs.clear();
        self.active_pairs.clear();
        self.events.clear();
    }
}

impl Default for ContactTracker {
    fn default() -> Self {
        Self::new()
    }
}
