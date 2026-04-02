import type { BodyHandle, CollisionPair, MatchaBuffers } from '@matcha2d/types'
import { BodyFlags } from '@matcha2d/types'
import { computeBodyAABB } from './aabb.js'

/** Internal node for the dynamic AABB tree. */
interface TreeNode {
  id: number
  parent: number
  child1: number
  child2: number
  isLeaf: boolean
  minX: number
  minY: number
  maxX: number
  maxY: number
  body: number
}

const NULL_NODE = -1

/**
 * Incremental dynamic AABB tree for broadphase collision detection.
 *
 * Unlike the previous rebuild-every-frame BVH, this tree persists across frames
 * and supports O(log n) insert/remove with automatic tree rotations for balance.
 *
 * Algorithm adapted from Sopiro/Physics with adaptations for SoA buffers.
 */
export class DynamicTree {
  private nodes: TreeNode[] = []
  private nodeID = 0
  private root = NULL_NODE
  /** Fat AABB margin for static bodies to reduce reinsertions. */
  aabbMargin = 0.05
  /** Maps body index → tree node index for O(1) lookup. */
  private bodyToNode = new Map<number, number>()

  reset(): void {
    this.nodes = []
    this.nodeID = 0
    this.root = NULL_NODE
    this.bodyToNode.clear()
  }

  /**
   * Insert a body into the tree.
   * Creates a fat AABB for static bodies, finds the best sibling via cost-based
   * descent, then walks up refitting ancestors with tree rotations.
   */
  insert(bodyIdx: number, buffers: MatchaBuffers): number {
    const [minX, minY, maxX, maxY] = computeBodyAABB(bodyIdx, buffers)
    const isStatic = (buffers.flags[bodyIdx] & BodyFlags.STATIC) !== 0
    const margin = isStatic ? 0.0 : this.aabbMargin

    const nodeIdx = this.allocateNode()
    const node = this.nodes[nodeIdx]
    node.minX = minX - margin
    node.minY = minY - margin
    node.maxX = maxX + margin
    node.maxY = maxY + margin
    node.isLeaf = true
    node.body = bodyIdx
    node.parent = NULL_NODE
    node.child1 = NULL_NODE
    node.child2 = NULL_NODE

    this.bodyToNode.set(bodyIdx, nodeIdx)

    if (this.root === NULL_NODE) {
      this.root = nodeIdx
      return nodeIdx
    }

    // Find the best sibling for the new leaf via cost-based descent.
    const { sibling: bestSibling, cost: _bestCost } = this.findBestSibling(nodeIdx)

    // Create a new parent to hold bestSibling and the new node.
    const parentIdx = this.allocateNode()
    const parent = this.nodes[parentIdx]
    const sibNode = this.nodes[bestSibling]

    parent.minX = Math.min(node.minX, sibNode.minX)
    parent.minY = Math.min(node.minY, sibNode.minY)
    parent.maxX = Math.max(node.maxX, sibNode.maxX)
    parent.maxY = Math.max(node.maxY, sibNode.maxY)
    parent.isLeaf = false
    parent.body = NULL_NODE
    parent.child1 = bestSibling
    parent.child2 = nodeIdx

    const oldParentIdx = sibNode.parent
    node.parent = parentIdx
    sibNode.parent = parentIdx

    if (oldParentIdx !== NULL_NODE) {
      const oldParent = this.nodes[oldParentIdx]
      if (oldParent.child1 === bestSibling) {
        oldParent.child1 = parentIdx
      } else {
        oldParent.child2 = parentIdx
      }
      parent.parent = oldParentIdx
    } else {
      parent.parent = NULL_NODE
      this.root = parentIdx
    }

    // Walk back up refitting ancestors.
    this.refitAncestors(nodeIdx)

    return nodeIdx
  }

  /**
   * Remove a body from the tree.
   * Replaces the node with its sibling, updates parent links, and refits ancestors.
   */
  remove(bodyIdx: number): void {
    const nodeIdx = this.bodyToNode.get(bodyIdx)
    if (nodeIdx === undefined) return

    const node = this.nodes[nodeIdx]
    this.bodyToNode.delete(bodyIdx)

    const parentIdx = node.parent
    if (parentIdx !== NULL_NODE) {
      const parent = this.nodes[parentIdx]
      const siblingIdx = parent.child1 === nodeIdx ? parent.child2 : parent.child1
      const sibling = this.nodes[siblingIdx]
      const grandParentIdx = parent.parent

      if (grandParentIdx !== NULL_NODE) {
        const grandParent = this.nodes[grandParentIdx]
        sibling.parent = grandParentIdx
        if (grandParent.child1 === parentIdx) {
          grandParent.child1 = siblingIdx
        } else {
          grandParent.child2 = siblingIdx
        }
      } else {
        this.root = siblingIdx
        sibling.parent = NULL_NODE
      }

      this.refitAncestors(siblingIdx)
      this.freeNode(parentIdx)
    } else {
      if (this.root === nodeIdx) {
        this.root = NULL_NODE
      }
    }

    this.freeNode(nodeIdx)
  }

  /**
   * Update a body's AABB in the tree.
   * If the AABB has moved outside the fat margin, remove and reinsert.
   */
  update(bodyIdx: number, buffers: MatchaBuffers): void {
    const nodeIdx = this.bodyToNode.get(bodyIdx)
    if (nodeIdx === undefined) return

    const node = this.nodes[nodeIdx]
    const [minX, minY, maxX, maxY] = computeBodyAABB(bodyIdx, buffers)
    const isStatic = (buffers.flags[bodyIdx] & BodyFlags.STATIC) !== 0
    const margin = isStatic ? 0.0 : this.aabbMargin

    // Check if the body is still within its fat AABB.
    if (
      minX >= node.minX && minY >= node.minY &&
      maxX <= node.maxX && maxY <= node.maxY
    ) {
      return // Still contained, no update needed.
    }

    // AABB has moved outside fat margin — remove and reinsert.
    this.remove(bodyIdx)
    this.insert(bodyIdx, buffers)
  }

  /** Update all bodies in the tree. */
  updateAll(buffers: MatchaBuffers): void {
    for (const [bodyIdx] of this.bodyToNode) {
      this.update(bodyIdx, buffers)
    }
  }

  /**
   * Query all overlapping body pairs.
   * Uses dual-subtree traversal: self-pairs within subtrees + cross-pairs between siblings.
   * Filters out static-static and sleeping-sleeping pairs.
   */
  queryPairs(buffers: MatchaBuffers): CollisionPair[] {
    if (this.root === NULL_NODE) return []

    const pairs: CollisionPair[] = []
    const root = this.nodes[this.root]

    if (!root.isLeaf) {
      this.queryCross(root.child1, root.child2, pairs, new Set(), buffers)
    }

    return pairs
  }

  /** Query all bodies whose AABBs overlap a region. */
  queryRegion(minX: number, minY: number, maxX: number, maxY: number): number[] {
    if (this.root === NULL_NODE) return []

    const results: number[] = []
    const stack = [this.root]

    while (stack.length > 0) {
      const idx = stack.pop()!
      const node = this.nodes[idx]

      if (node.maxX < minX || maxX < node.minX || node.maxY < minY || maxY < node.minY) {
        continue
      }

      if (node.isLeaf) {
        results.push(node.body)
      } else {
        stack.push(node.child1)
        stack.push(node.child2)
      }
    }

    return results
  }

  /** Find the best sibling for a new leaf via cost-based descent (BFS with priority queue). */
  private findBestSibling(newNodeIdx: number): { sibling: number; cost: number } {
    const newNode = this.nodes[newNodeIdx]
    const newArea = (newNode.maxX - newNode.minX) * (newNode.maxY - newNode.minY)

    let bestSibling = this.root
    let bestCost = this.unionArea(this.root, newNodeIdx)

    // Priority queue: { nodeIdx, inheritedCost }
    const queue: Array<{ nodeIdx: number; inheritedCost: number }> = [
      { nodeIdx: this.root, inheritedCost: 0 }
    ]

    while (queue.length > 0) {
      // Find lowest cost entry (simple priority queue via sort).
      let bestIdx = 0
      for (let i = 1; i < queue.length; i++) {
        if (queue[i].inheritedCost < queue[bestIdx].inheritedCost) {
          bestIdx = i
        }
      }
      const { nodeIdx, inheritedCost } = queue.splice(bestIdx, 1)[0]
      const node = this.nodes[nodeIdx]

      const directCost = this.unionArea(nodeIdx, newNodeIdx)
      const costForCurrent = directCost + inheritedCost

      if (costForCurrent < bestCost) {
        bestCost = costForCurrent
        bestSibling = nodeIdx
      }

      const inheritedCostForChildren = inheritedCost + directCost - (node.maxX - node.minX) * (node.maxY - node.minY)
      const lowerBoundCost = newArea + inheritedCostForChildren

      if (lowerBoundCost < bestCost && !node.isLeaf) {
        queue.push({ nodeIdx: node.child1, inheritedCost: inheritedCostForChildren })
        queue.push({ nodeIdx: node.child2, inheritedCost: inheritedCostForChildren })
      }
    }

    return { sibling: bestSibling, cost: bestCost }
  }

  /** Compute the area of the union of two nodes' AABBs. */
  private unionArea(aIdx: number, bIdx: number): number {
    const a = this.nodes[aIdx]
    const b = this.nodes[bIdx]
    const unionMinX = Math.min(a.minX, b.minX)
    const unionMinY = Math.min(a.minY, b.minY)
    const unionMaxX = Math.max(a.maxX, b.maxX)
    const unionMaxY = Math.max(a.maxY, b.maxY)
    return (unionMaxX - unionMinX) * (unionMaxY - unionMinY)
  }

  /** Refit AABBs up the tree from a given node, applying rotations for balance. */
  private refitAncestors(fromIdx: number): void {
    let ancestorIdx = this.nodes[fromIdx].parent

    while (ancestorIdx !== NULL_NODE) {
      const ancestor = this.nodes[ancestorIdx]
      const child1 = this.nodes[ancestor.child1]
      const child2 = this.nodes[ancestor.child2]

      ancestor.minX = Math.min(child1.minX, child2.minX)
      ancestor.minY = Math.min(child1.minY, child2.minY)
      ancestor.maxX = Math.max(child1.maxX, child2.maxX)
      ancestor.maxY = Math.max(child1.maxY, child2.maxY)

      this.rotate(ancestorIdx)

      ancestorIdx = this.nodes[ancestorIdx].parent
    }
  }

  /**
   * Tree rotation heuristic for balance.
   * Tries 4 swap types and picks the one with lowest area cost.
   * Adapted from Sopiro/Physics AABBTree.rotate().
   */
  private rotate(nodeIdx: number): void {
    const node = this.nodes[nodeIdx]
    const parentIdx = node.parent
    if (parentIdx === NULL_NODE) return

    const parent = this.nodes[parentIdx]
    const siblingIdx = parent.child1 === nodeIdx ? parent.child2 : parent.child1
    const sibling = this.nodes[siblingIdx]

    if (sibling.isLeaf || node.isLeaf) return

    const nodeArea = (node.maxX - node.minX) * (node.maxY - node.minY)
    const siblingArea = (sibling.maxX - sibling.minX) * (sibling.maxY - sibling.minY)

    const costDiffs = [
      this.unionArea(siblingIdx, node.child1) - nodeArea,
      this.unionArea(siblingIdx, node.child2) - nodeArea,
      this.unionArea(nodeIdx, sibling.child1) - siblingArea,
      this.unionArea(nodeIdx, sibling.child2) - siblingArea,
    ]

    let bestDiffIdx = 0
    for (let i = 1; i < 4; i++) {
      if (costDiffs[i] < costDiffs[bestDiffIdx]) {
        bestDiffIdx = i
      }
    }

    if (costDiffs[bestDiffIdx] >= 0) return

    switch (bestDiffIdx) {
      case 0: {
        // Swap sibling with node.child1
        const child1Idx = node.child1
        if (parent.child1 === siblingIdx) {
          parent.child1 = child1Idx
        } else {
          parent.child2 = child1Idx
        }
        this.nodes[child1Idx].parent = parentIdx

        node.child1 = siblingIdx
        sibling.parent = nodeIdx
        this.refitNode(nodeIdx)
        break
      }
      case 1: {
        // Swap sibling with node.child2
        const child2Idx = node.child2
        if (parent.child1 === siblingIdx) {
          parent.child1 = child2Idx
        } else {
          parent.child2 = child2Idx
        }
        this.nodes[child2Idx].parent = parentIdx

        node.child2 = siblingIdx
        sibling.parent = nodeIdx
        this.refitNode(nodeIdx)
        break
      }
      case 2: {
        // Swap node with sibling.child1
        const sibChild1Idx = sibling.child1
        if (parent.child1 === nodeIdx) {
          parent.child1 = sibChild1Idx
        } else {
          parent.child2 = sibChild1Idx
        }
        this.nodes[sibChild1Idx].parent = parentIdx

        sibling.child1 = nodeIdx
        node.parent = siblingIdx
        this.refitNode(siblingIdx)
        break
      }
      case 3: {
        // Swap node with sibling.child2
        const sibChild2Idx = sibling.child2
        if (parent.child1 === nodeIdx) {
          parent.child1 = sibChild2Idx
        } else {
          parent.child2 = sibChild2Idx
        }
        this.nodes[sibChild2Idx].parent = parentIdx

        sibling.child2 = nodeIdx
        node.parent = siblingIdx
        this.refitNode(siblingIdx)
        break
      }
    }
  }

  /** Refit a single node's AABB from its children. */
  private refitNode(nodeIdx: number): void {
    const node = this.nodes[nodeIdx]
    if (node.isLeaf) return

    const child1 = this.nodes[node.child1]
    const child2 = this.nodes[node.child2]
    node.minX = Math.min(child1.minX, child2.minX)
    node.minY = Math.min(child1.minY, child2.minY)
    node.maxX = Math.max(child1.maxX, child2.maxX)
    node.maxY = Math.max(child1.maxY, child2.maxY)
  }

  /** Dual-subtree traversal to find all overlapping leaf pairs. */
  private queryCross(
    aIdx: number,
    bIdx: number,
    pairs: CollisionPair[],
    checked: Set<number>,
    buffers: MatchaBuffers,
  ): void {
    const key = this.pairKey(aIdx, bIdx)
    if (checked.has(key)) return
    checked.add(key)

    const a = this.nodes[aIdx]
    const b = this.nodes[bIdx]

    // Early exit: AABBs don't overlap.
    if (a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY) {
      return
    }

    if (a.isLeaf && b.isLeaf) {
      // Both leaves — filter static-static and sleeping-sleeping pairs.
      const i = a.body
      const j = b.body
      const fi = buffers.flags[i]
      const fj = buffers.flags[j]
      const iStatic = (fi & BodyFlags.STATIC) !== 0
      const jStatic = (fj & BodyFlags.STATIC) !== 0
      const iSleeping = (fi & BodyFlags.SLEEPING) !== 0
      const jSleeping = (fj & BodyFlags.SLEEPING) !== 0

      if (iStatic && jStatic) return
      if (iSleeping && jSleeping) return

      pairs.push({ a: i as BodyHandle, b: j as BodyHandle })
      return
    }

    if (!a.isLeaf && !b.isLeaf) {
      // Both internal — recurse into self-pairs first, then cross-pairs.
      this.queryCross(a.child1, a.child2, pairs, checked, buffers)
      this.queryCross(b.child1, b.child2, pairs, checked, buffers)

      this.queryCross(a.child1, b.child1, pairs, checked, buffers)
      this.queryCross(a.child1, b.child2, pairs, checked, buffers)
      this.queryCross(a.child2, b.child1, pairs, checked, buffers)
      this.queryCross(a.child2, b.child2, pairs, checked, buffers)
    } else if (a.isLeaf && !b.isLeaf) {
      this.queryCross(b.child1, b.child2, pairs, checked, buffers)
      this.queryCross(aIdx, b.child1, pairs, checked, buffers)
      this.queryCross(aIdx, b.child2, pairs, checked, buffers)
    } else if (!a.isLeaf && b.isLeaf) {
      this.queryCross(a.child1, a.child2, pairs, checked, buffers)
      this.queryCross(a.child1, bIdx, pairs, checked, buffers)
      this.queryCross(a.child2, bIdx, pairs, checked, buffers)
    }
  }

  /** Generate a unique key for a pair of node indices. */
  private pairKey(a: number, b: number): number {
    return a < b ? (a * 73856093) ^ (b * 19349663) : (b * 73856093) ^ (a * 19349663)
  }

  /** Allocate a new node from the pool. */
  private allocateNode(): number {
    const idx = this.nodes.length
    this.nodes.push({
      id: this.nodeID++,
      parent: NULL_NODE,
      child1: NULL_NODE,
      child2: NULL_NODE,
      isLeaf: false,
      minX: 0,
      minY: 0,
      maxX: 0,
      maxY: 0,
      body: NULL_NODE,
    })
    return idx
  }

  /** Free a node (mark for reuse). */
  private freeNode(idx: number): void {
    // Mark as freed by setting body to NULL_NODE and isLeaf to false.
    // In a production system, you'd use a free list for reuse.
    this.nodes[idx].body = NULL_NODE
    this.nodes[idx].isLeaf = false
  }

  /** Get the number of bodies in the tree. */
  get bodyCount(): number {
    return this.bodyToNode.size
  }

  /** Get the total cost (sum of all node AABB areas) — useful for debugging. */
  get cost(): number {
    let total = 0
    for (const node of this.nodes) {
      total += (node.maxX - node.minX) * (node.maxY - node.minY)
    }
    return total
  }
}
