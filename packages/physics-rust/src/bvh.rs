use crate::aabb::AABB;
use crate::collider::ColliderHandle;
use crate::math::Vec2;

const NULL_NODE: usize = usize::MAX;
const AABB_MARGIN: f32 = 0.1;

fn expand_aabb(aabb: AABB, margin: f32) -> AABB {
    aabb.expanded(Vec2::new(margin, margin))
}

#[derive(Clone)]
struct BvhNode {
    aabb: AABB,
    parent: usize,
    child1: usize,
    child2: usize,
    height: i32,
    /// If this is a leaf node, stores the associated collider handle
    leaf: Option<ColliderHandle>,
}

impl BvhNode {
    fn is_leaf(&self) -> bool {
        self.child1 == NULL_NODE
    }
}

pub struct DynamicBvh {
    nodes: Vec<BvhNode>,
    root: usize,
    free_list: Vec<usize>,
}

impl DynamicBvh {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL_NODE,
            free_list: Vec::new(),
        }
    }

    fn alloc_node(&mut self) -> usize {
        if let Some(idx) = self.free_list.pop() {
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(BvhNode {
                aabb: AABB::default(),
                parent: NULL_NODE,
                child1: NULL_NODE,
                child2: NULL_NODE,
                height: -1,
                leaf: None,
            });
            idx
        }
    }

    fn free_node(&mut self, idx: usize) {
        self.nodes[idx].parent = NULL_NODE;
        self.nodes[idx].height = -1;
        self.nodes[idx].leaf = None;
        self.free_list.push(idx);
    }

    pub fn insert_leaf(&mut self, aabb: AABB, handle: ColliderHandle) -> usize {
        let leaf = self.alloc_node();
        self.nodes[leaf].aabb = expand_aabb(aabb, AABB_MARGIN);
        self.nodes[leaf].height = 0;
        self.nodes[leaf].leaf = Some(handle);

        if self.root == NULL_NODE {
            self.root = leaf;
            return leaf;
        }

        // Find the best sibling using area metric
        let mut sibling = self.root;
        while !self.nodes[sibling].is_leaf() {
            let child1 = self.nodes[sibling].child1;
            let child2 = self.nodes[sibling].child2;

            let area = self.nodes[sibling].aabb.perimeter();
            let merged_aabb = self.nodes[sibling].aabb.merge(self.nodes[leaf].aabb);
            let merged_area = merged_aabb.perimeter();

            // Cost of creating a new parent for this node and the new leaf
            let cost = 2.0 * merged_area;
            // Minimum cost of pushing the leaf further down the tree
            let inheritance_cost = 2.0 * (merged_area - area);

            // Cost of descending into child1
            let cost1 = if self.nodes[child1].is_leaf() {
                self.nodes[leaf]
                    .aabb
                    .merge(self.nodes[child1].aabb)
                    .perimeter()
                    + inheritance_cost
            } else {
                self.nodes[leaf]
                    .aabb
                    .merge(self.nodes[child1].aabb)
                    .perimeter()
                    - self.nodes[child1].aabb.perimeter()
                    + inheritance_cost
            };

            // Cost of descending into child2
            let cost2 = if self.nodes[child2].is_leaf() {
                self.nodes[leaf]
                    .aabb
                    .merge(self.nodes[child2].aabb)
                    .perimeter()
                    + inheritance_cost
            } else {
                self.nodes[leaf]
                    .aabb
                    .merge(self.nodes[child2].aabb)
                    .perimeter()
                    - self.nodes[child2].aabb.perimeter()
                    + inheritance_cost
            };

            // Descend according to minimum cost
            if cost < cost1 && cost < cost2 {
                break;
            }

            sibling = if cost1 < cost2 { child1 } else { child2 };
        }

        let old_parent = self.nodes[sibling].parent;
        let new_parent = self.alloc_node();
        self.nodes[new_parent].parent = old_parent;
        self.nodes[new_parent].aabb = self.nodes[leaf].aabb.merge(self.nodes[sibling].aabb);
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;

        if old_parent != NULL_NODE {
            if self.nodes[old_parent].child1 == sibling {
                self.nodes[old_parent].child1 = new_parent;
            } else {
                self.nodes[old_parent].child2 = new_parent;
            }
        } else {
            self.root = new_parent;
        }

        self.nodes[new_parent].child1 = sibling;
        self.nodes[new_parent].child2 = leaf;
        self.nodes[sibling].parent = new_parent;
        self.nodes[leaf].parent = new_parent;

        // Walk back up the tree fixing heights and AABBs
        self.fix_upwards_to_root(new_parent);

        leaf
    }

    pub fn remove_leaf(&mut self, leaf: usize) {
        if leaf == self.root {
            self.root = NULL_NODE;
            self.free_node(leaf);
            return;
        }

        let parent = self.nodes[leaf].parent;
        let grand_parent = self.nodes[parent].parent;
        let sibling = if self.nodes[parent].child1 == leaf {
            self.nodes[parent].child2
        } else {
            self.nodes[parent].child1
        };

        if grand_parent != NULL_NODE {
            // Connect sibling to grand parent
            if self.nodes[grand_parent].child1 == parent {
                self.nodes[grand_parent].child1 = sibling;
            } else {
                self.nodes[grand_parent].child2 = sibling;
            }
            self.nodes[sibling].parent = grand_parent;
            self.free_node(parent);
            self.fix_upwards_to_root(grand_parent);
        } else {
            self.root = sibling;
            self.nodes[sibling].parent = NULL_NODE;
            self.free_node(parent);
        }

        self.free_node(leaf);
    }

    pub fn update_leaf(&mut self, leaf: usize, aabb: AABB) -> usize {
        let fat_aabb = expand_aabb(aabb, AABB_MARGIN);
        if self.nodes[leaf].aabb.contains(fat_aabb) {
            // Still fits in fat AABB, no update needed
            return leaf;
        }

        // Capture handle BEFORE remove invalidates the node
        let handle = self.nodes[leaf].leaf.unwrap_or(ColliderHandle::INVALID);
        self.remove_leaf(leaf);
        self.insert_leaf(aabb, handle)
    }

    fn fix_upwards_to_root(&mut self, mut node: usize) {
        while node != NULL_NODE {
            node = self.balance(node);

            let child1 = self.nodes[node].child1;
            let child2 = self.nodes[node].child2;

            if child1 != NULL_NODE && child2 != NULL_NODE {
                self.nodes[node].height =
                    1 + self.nodes[child1].height.max(self.nodes[child2].height);
                self.nodes[node].aabb = self.nodes[child1].aabb.merge(self.nodes[child2].aabb);
            } else if child1 != NULL_NODE {
                self.nodes[node].height = self.nodes[child1].height;
                self.nodes[node].aabb = self.nodes[child1].aabb;
            } else if child2 != NULL_NODE {
                self.nodes[node].height = self.nodes[child2].height;
                self.nodes[node].aabb = self.nodes[child2].aabb;
            } else {
                self.nodes[node].height = 0;
            }

            node = self.nodes[node].parent;
        }
    }

    fn balance(&mut self, a: usize) -> usize {
        if self.nodes[a].is_leaf() || self.nodes[a].height < 2 {
            return a;
        }

        let b = self.nodes[a].child1;
        let c = self.nodes[a].child2;
        let balance = self.nodes[c].height - self.nodes[b].height;

        // Rotate c up if c is significantly taller
        if balance > 1 {
            let f = self.nodes[c].child1;
            let g = self.nodes[c].child2;

            // Swap a and c
            self.nodes[c].child1 = a;
            self.nodes[c].parent = self.nodes[a].parent;
            self.nodes[a].parent = c;

            if self.nodes[c].parent != NULL_NODE {
                let parent = self.nodes[c].parent;
                if self.nodes[parent].child1 == a {
                    self.nodes[parent].child1 = c;
                } else {
                    self.nodes[parent].child2 = c;
                }
            } else {
                self.root = c;
            }

            // Rotate
            if self.nodes[f].height > self.nodes[g].height {
                self.nodes[c].child2 = f;
                self.nodes[a].child2 = g;
                self.nodes[g].parent = a;
                self.nodes[a].aabb = self.nodes[b].aabb.merge(self.nodes[g].aabb);
                self.nodes[c].aabb = self.nodes[a].aabb.merge(self.nodes[f].aabb);
                self.nodes[a].height = 1 + self.nodes[b].height.max(self.nodes[g].height);
                self.nodes[c].height = 1 + self.nodes[a].height.max(self.nodes[f].height);
            } else {
                self.nodes[c].child2 = g;
                self.nodes[a].child2 = f;
                self.nodes[f].parent = a;
                self.nodes[a].aabb = self.nodes[b].aabb.merge(self.nodes[f].aabb);
                self.nodes[c].aabb = self.nodes[a].aabb.merge(self.nodes[g].aabb);
                self.nodes[a].height = 1 + self.nodes[b].height.max(self.nodes[f].height);
                self.nodes[c].height = 1 + self.nodes[a].height.max(self.nodes[g].height);
            }

            return c;
        }

        // Rotate b up if b is significantly taller
        if balance < -1 {
            let d = self.nodes[b].child1;
            let e = self.nodes[b].child2;

            // Swap a and b
            self.nodes[b].child1 = a;
            self.nodes[b].parent = self.nodes[a].parent;
            self.nodes[a].parent = b;

            if self.nodes[b].parent != NULL_NODE {
                let parent = self.nodes[b].parent;
                if self.nodes[parent].child1 == a {
                    self.nodes[parent].child1 = b;
                } else {
                    self.nodes[parent].child2 = b;
                }
            } else {
                self.root = b;
            }

            // Rotate
            if self.nodes[d].height > self.nodes[e].height {
                self.nodes[b].child2 = d;
                self.nodes[a].child1 = e;
                self.nodes[e].parent = a;
                self.nodes[a].aabb = self.nodes[c].aabb.merge(self.nodes[e].aabb);
                self.nodes[b].aabb = self.nodes[a].aabb.merge(self.nodes[d].aabb);
                self.nodes[a].height = 1 + self.nodes[c].height.max(self.nodes[e].height);
                self.nodes[b].height = 1 + self.nodes[a].height.max(self.nodes[d].height);
            } else {
                self.nodes[b].child2 = e;
                self.nodes[a].child1 = d;
                self.nodes[d].parent = a;
                self.nodes[a].aabb = self.nodes[c].aabb.merge(self.nodes[d].aabb);
                self.nodes[b].aabb = self.nodes[a].aabb.merge(self.nodes[e].aabb);
                self.nodes[a].height = 1 + self.nodes[c].height.max(self.nodes[d].height);
                self.nodes[b].height = 1 + self.nodes[a].height.max(self.nodes[e].height);
            }

            return b;
        }

        a
    }

    /// Query the tree for all overlapping pairs
    pub fn query_pairs(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let mut pairs = Vec::new();
        if self.root == NULL_NODE {
            return pairs;
        }

        // Use a stack for iterative traversal
        let mut stack = vec![self.root];
        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];
            if node.is_leaf() {
                continue;
            }

            // Test children against each other
            let child1 = node.child1;
            let child2 = node.child2;
            if self.nodes[child1].aabb.overlaps(self.nodes[child2].aabb) {
                self.report_pairs(child1, child2, &mut pairs);
            }

            // Push children for further traversal
            if !self.nodes[child1].is_leaf() {
                stack.push(child1);
            }
            if !self.nodes[child2].is_leaf() {
                stack.push(child2);
            }
        }

        pairs
    }

    fn report_pairs(
        &self,
        node_a: usize,
        node_b: usize,
        pairs: &mut Vec<(ColliderHandle, ColliderHandle)>,
    ) {
        let a = &self.nodes[node_a];
        let b = &self.nodes[node_b];

        if a.is_leaf() && b.is_leaf() {
            if let (Some(ha), Some(hb)) = (a.leaf, b.leaf) {
                if ha.0 < hb.0 {
                    pairs.push((ha, hb));
                } else {
                    pairs.push((hb, ha));
                }
            }
            return;
        }

        if b.is_leaf() || (!a.is_leaf() && a.aabb.area() >= b.aabb.area()) {
            if self.nodes[a.child1].aabb.overlaps(b.aabb) {
                self.report_pairs(a.child1, node_b, pairs);
            }
            if self.nodes[a.child2].aabb.overlaps(b.aabb) {
                self.report_pairs(a.child2, node_b, pairs);
            }
        } else {
            if a.aabb.overlaps(self.nodes[b.child1].aabb) {
                self.report_pairs(node_a, b.child1, pairs);
            }
            if a.aabb.overlaps(self.nodes[b.child2].aabb) {
                self.report_pairs(node_a, b.child2, pairs);
            }
        }
    }

    pub fn query_point(&self, point: crate::math::Vec2) -> Vec<ColliderHandle> {
        let mut results = Vec::new();
        if self.root == NULL_NODE {
            return results;
        }

        let point_aabb = AABB::new(point, point);
        let mut stack = vec![self.root];
        while let Some(idx) = stack.pop() {
            let node = &self.nodes[idx];
            if !node.aabb.overlaps(point_aabb) {
                continue;
            }
            if node.is_leaf() {
                if let Some(handle) = node.leaf {
                    results.push(handle);
                }
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
        }
        results
    }

    pub fn query_aabb(&self, aabb: AABB) -> Vec<ColliderHandle> {
        let mut results = Vec::new();
        if self.root == NULL_NODE {
            return results;
        }

        let mut stack = vec![self.root];
        while let Some(idx) = stack.pop() {
            let node = &self.nodes[idx];
            if !node.aabb.overlaps(aabb) {
                continue;
            }
            if node.is_leaf() {
                if let Some(handle) = node.leaf {
                    results.push(handle);
                }
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
        }
        results
    }

    pub fn compute_height(&self) -> usize {
        if self.root == NULL_NODE {
            return 0;
        }
        self.nodes[self.root].height as usize
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len() - self.free_list.len()
    }

    pub fn clear(&mut self) {
        self.nodes.clear();
        self.free_list.clear();
        self.root = NULL_NODE;
    }
}

impl Default for DynamicBvh {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::ColliderHandle;
    use crate::math::Vec2;

    #[test]
    fn test_insert_and_query() {
        let mut bvh = DynamicBvh::new();
        let a = AABB::new_from_coords(0.0, 0.0, 1.0, 1.0);
        let b = AABB::new_from_coords(0.5, 0.5, 1.5, 1.5);
        let c = AABB::new_from_coords(5.0, 5.0, 6.0, 6.0);

        bvh.insert_leaf(a, ColliderHandle(0));
        bvh.insert_leaf(b, ColliderHandle(1));
        bvh.insert_leaf(c, ColliderHandle(2));

        let pairs = bvh.query_pairs();
        assert_eq!(pairs.len(), 1);
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn test_no_overlap() {
        let mut bvh = DynamicBvh::new();
        let a = AABB::new_from_coords(0.0, 0.0, 1.0, 1.0);
        let b = AABB::new_from_coords(5.0, 5.0, 6.0, 6.0);

        bvh.insert_leaf(a, ColliderHandle(0));
        bvh.insert_leaf(b, ColliderHandle(1));

        let pairs = bvh.query_pairs();
        assert!(pairs.is_empty());
    }

    #[test]
    fn test_remove() {
        let mut bvh = DynamicBvh::new();
        let a = AABB::new_from_coords(0.0, 0.0, 1.0, 1.0);
        let b = AABB::new_from_coords(0.5, 0.5, 1.5, 1.5);

        let leaf_a = bvh.insert_leaf(a, ColliderHandle(0));
        bvh.insert_leaf(b, ColliderHandle(1));

        let pairs = bvh.query_pairs();
        assert_eq!(pairs.len(), 1);

        bvh.remove_leaf(leaf_a);
        let pairs = bvh.query_pairs();
        assert!(pairs.is_empty());
    }

    #[test]
    fn test_query_aabb() {
        let mut bvh = DynamicBvh::new();
        bvh.insert_leaf(AABB::new_from_coords(0.0, 0.0, 1.0, 1.0), ColliderHandle(0));
        bvh.insert_leaf(AABB::new_from_coords(2.0, 2.0, 3.0, 3.0), ColliderHandle(1));
        bvh.insert_leaf(AABB::new_from_coords(5.0, 5.0, 6.0, 6.0), ColliderHandle(2));

        let results = bvh.query_aabb(AABB::new_from_coords(-1.0, -1.0, 2.5, 2.5));
        assert_eq!(results.len(), 2);
        assert!(results.contains(&ColliderHandle(0)));
        assert!(results.contains(&ColliderHandle(1)));
    }
}
