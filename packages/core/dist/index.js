import { ShapeType, BodyFlags, MAX_VERTICES_PER_SHAPE } from '@matcha2d/types';

// src/math/vec2.ts
function vec2Set(outX, outY, idx, x, y) {
  outX[idx] = x;
  outY[idx] = y;
}
function vec2Add(outX, outY, outIdx, aX, aY, aIdx, bX, bY, bIdx) {
  outX[outIdx] = aX[aIdx] + bX[bIdx];
  outY[outIdx] = aY[aIdx] + bY[bIdx];
}
function vec2Sub(outX, outY, outIdx, aX, aY, aIdx, bX, bY, bIdx) {
  outX[outIdx] = aX[aIdx] - bX[bIdx];
  outY[outIdx] = aY[aIdx] - bY[bIdx];
}
function vec2Scale(outX, outY, outIdx, aX, aY, aIdx, s) {
  outX[outIdx] = aX[aIdx] * s;
  outY[outIdx] = aY[aIdx] * s;
}
function vec2Dot(aX, aY, aIdx, bX, bY, bIdx) {
  return aX[aIdx] * bX[bIdx] + aY[aIdx] * bY[bIdx];
}
function vec2Cross(aX, aY, aIdx, bX, bY, bIdx) {
  return aX[aIdx] * bY[bIdx] - aY[aIdx] * bX[bIdx];
}
function vec2LengthSq(x, y, idx) {
  return x[idx] * x[idx] + y[idx] * y[idx];
}
function vec2Length(x, y, idx) {
  return Math.sqrt(x[idx] * x[idx] + y[idx] * y[idx]);
}
function vec2Normalize(outX, outY, outIdx, aX, aY, aIdx) {
  const len = Math.sqrt(aX[aIdx] * aX[aIdx] + aY[aIdx] * aY[aIdx]);
  if (len > 0) {
    const invLen = 1 / len;
    outX[outIdx] = aX[aIdx] * invLen;
    outY[outIdx] = aY[aIdx] * invLen;
  } else {
    outX[outIdx] = 0;
    outY[outIdx] = 0;
  }
}
function vec2DistanceSq(aX, aY, aIdx, bX, bY, bIdx) {
  const dx = aX[aIdx] - bX[bIdx];
  const dy = aY[aIdx] - bY[bIdx];
  return dx * dx + dy * dy;
}
function dot(ax, ay, bx, by) {
  return ax * bx + ay * by;
}
function cross(ax, ay, bx, by) {
  return ax * by - ay * bx;
}
function lengthSq(x, y) {
  return x * x + y * y;
}
function length(x, y) {
  return Math.sqrt(x * x + y * y);
}

// src/math/mat2.ts
function mat2FromAngle(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c, -s, s, c];
}
function mat2MulVec(m00, m01, m10, m11, x, y) {
  return [m00 * x + m01 * y, m10 * x + m11 * y];
}
function mat2TransposeMulVec(m00, m01, m10, m11, x, y) {
  return [m00 * x + m10 * y, m01 * x + m11 * y];
}
function aabbOverlap(a, b) {
  return a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;
}
function aabbMerge(a, b) {
  return {
    minX: Math.min(a.minX, b.minX),
    minY: Math.min(a.minY, b.minY),
    maxX: Math.max(a.maxX, b.maxX),
    maxY: Math.max(a.maxY, b.maxY)
  };
}
function aabbContains(outer, inner) {
  return outer.minX <= inner.minX && outer.maxX >= inner.maxX && outer.minY <= inner.minY && outer.maxY >= inner.maxY;
}
function aabbArea(a) {
  return (a.maxX - a.minX) * (a.maxY - a.minY);
}
function aabbPerimeter(a) {
  return 2 * (a.maxX - a.minX + (a.maxY - a.minY));
}
function computeBodyAABB(idx, buffers) {
  const px = buffers.positionX[idx];
  const py = buffers.positionY[idx];
  const type = buffers.shapeType[idx];
  if (type === ShapeType.Circle) {
    const r = buffers.shapeRadius[idx];
    return [px - r, py - r, px + r, py + r];
  }
  if (type === ShapeType.Box) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    const [c2, negS2, s2] = mat2FromAngle(buffers.angle[idx]);
    let minX2 = Infinity, minY2 = Infinity, maxX2 = -Infinity, maxY2 = -Infinity;
    const lx = [-hx, hx, hx, -hx];
    const ly = [-hy, -hy, hy, hy];
    for (let i = 0; i < 4; i++) {
      const wx = c2 * lx[i] + negS2 * ly[i] + px;
      const wy = s2 * lx[i] + c2 * ly[i] + py;
      if (wx < minX2) minX2 = wx;
      if (wx > maxX2) maxX2 = wx;
      if (wy < minY2) minY2 = wy;
      if (wy > maxY2) maxY2 = wy;
    }
    return [minX2, minY2, maxX2, maxY2];
  }
  const count = buffers.shapeVertexCount[idx];
  const base = idx * MAX_VERTICES_PER_SHAPE;
  const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (let i = 0; i < count; i++) {
    const lx = buffers.shapeVerticesX[base + i];
    const ly = buffers.shapeVerticesY[base + i];
    const wx = c * lx + negS * ly + px;
    const wy = s * lx + c * ly + py;
    if (wx < minX) minX = wx;
    if (wx > maxX) maxX = wx;
    if (wy < minY) minY = wy;
    if (wy > maxY) maxY = wy;
  }
  if (count === 0) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    return [px - hx, py - hy, px + hx, py + hy];
  }
  return [minX, minY, maxX, maxY];
}
var NULL_NODE = -1;
var DynamicTree = class {
  constructor() {
    this.nodes = [];
    this.nodeID = 0;
    this.root = NULL_NODE;
    /** Fat AABB margin for static bodies to reduce reinsertions. */
    this.aabbMargin = 0.05;
    /** Maps body index → tree node index for O(1) lookup. */
    this.bodyToNode = /* @__PURE__ */ new Map();
  }
  reset() {
    this.nodes = [];
    this.nodeID = 0;
    this.root = NULL_NODE;
    this.bodyToNode.clear();
  }
  /**
   * Insert a body into the tree.
   * Creates a fat AABB for static bodies, finds the best sibling via cost-based
   * descent, then walks up refitting ancestors with tree rotations.
   */
  insert(bodyIdx, buffers) {
    const [minX, minY, maxX, maxY] = computeBodyAABB(bodyIdx, buffers);
    const isStatic = (buffers.flags[bodyIdx] & BodyFlags.STATIC) !== 0;
    const margin = isStatic ? 0 : this.aabbMargin;
    const nodeIdx = this.allocateNode();
    const node = this.nodes[nodeIdx];
    node.minX = minX - margin;
    node.minY = minY - margin;
    node.maxX = maxX + margin;
    node.maxY = maxY + margin;
    node.isLeaf = true;
    node.body = bodyIdx;
    node.parent = NULL_NODE;
    node.child1 = NULL_NODE;
    node.child2 = NULL_NODE;
    this.bodyToNode.set(bodyIdx, nodeIdx);
    if (this.root === NULL_NODE) {
      this.root = nodeIdx;
      return nodeIdx;
    }
    const { sibling: bestSibling, cost: _bestCost } = this.findBestSibling(nodeIdx);
    const parentIdx = this.allocateNode();
    const parent = this.nodes[parentIdx];
    const sibNode = this.nodes[bestSibling];
    parent.minX = Math.min(node.minX, sibNode.minX);
    parent.minY = Math.min(node.minY, sibNode.minY);
    parent.maxX = Math.max(node.maxX, sibNode.maxX);
    parent.maxY = Math.max(node.maxY, sibNode.maxY);
    parent.isLeaf = false;
    parent.body = NULL_NODE;
    parent.child1 = bestSibling;
    parent.child2 = nodeIdx;
    const oldParentIdx = sibNode.parent;
    node.parent = parentIdx;
    sibNode.parent = parentIdx;
    if (oldParentIdx !== NULL_NODE) {
      const oldParent = this.nodes[oldParentIdx];
      if (oldParent.child1 === bestSibling) {
        oldParent.child1 = parentIdx;
      } else {
        oldParent.child2 = parentIdx;
      }
      parent.parent = oldParentIdx;
    } else {
      parent.parent = NULL_NODE;
      this.root = parentIdx;
    }
    this.refitAncestors(nodeIdx);
    return nodeIdx;
  }
  /**
   * Remove a body from the tree.
   * Replaces the node with its sibling, updates parent links, and refits ancestors.
   */
  remove(bodyIdx) {
    const nodeIdx = this.bodyToNode.get(bodyIdx);
    if (nodeIdx === void 0) return;
    const node = this.nodes[nodeIdx];
    this.bodyToNode.delete(bodyIdx);
    const parentIdx = node.parent;
    if (parentIdx !== NULL_NODE) {
      const parent = this.nodes[parentIdx];
      const siblingIdx = parent.child1 === nodeIdx ? parent.child2 : parent.child1;
      const sibling = this.nodes[siblingIdx];
      const grandParentIdx = parent.parent;
      if (grandParentIdx !== NULL_NODE) {
        const grandParent = this.nodes[grandParentIdx];
        sibling.parent = grandParentIdx;
        if (grandParent.child1 === parentIdx) {
          grandParent.child1 = siblingIdx;
        } else {
          grandParent.child2 = siblingIdx;
        }
      } else {
        this.root = siblingIdx;
        sibling.parent = NULL_NODE;
      }
      this.refitAncestors(siblingIdx);
      this.freeNode(parentIdx);
    } else {
      if (this.root === nodeIdx) {
        this.root = NULL_NODE;
      }
    }
    this.freeNode(nodeIdx);
  }
  /**
   * Update a body's AABB in the tree.
   * If the AABB has moved outside the fat margin, remove and reinsert.
   */
  update(bodyIdx, buffers) {
    const nodeIdx = this.bodyToNode.get(bodyIdx);
    if (nodeIdx === void 0) return;
    const node = this.nodes[nodeIdx];
    const [minX, minY, maxX, maxY] = computeBodyAABB(bodyIdx, buffers);
    const isStatic = (buffers.flags[bodyIdx] & BodyFlags.STATIC) !== 0;
    isStatic ? 0 : this.aabbMargin;
    if (minX >= node.minX && minY >= node.minY && maxX <= node.maxX && maxY <= node.maxY) {
      return;
    }
    this.remove(bodyIdx);
    this.insert(bodyIdx, buffers);
  }
  /** Update all bodies in the tree. */
  updateAll(buffers) {
    for (const [bodyIdx] of this.bodyToNode) {
      this.update(bodyIdx, buffers);
    }
  }
  /**
   * Query all overlapping body pairs.
   * Uses dual-subtree traversal: self-pairs within subtrees + cross-pairs between siblings.
   * Filters out static-static and sleeping-sleeping pairs.
   */
  queryPairs(buffers) {
    if (this.root === NULL_NODE) return [];
    const pairs = [];
    const root = this.nodes[this.root];
    if (!root.isLeaf) {
      this.queryCross(root.child1, root.child2, pairs, /* @__PURE__ */ new Set(), buffers);
    }
    return pairs;
  }
  /** Query all bodies whose AABBs overlap a region. */
  queryRegion(minX, minY, maxX, maxY) {
    if (this.root === NULL_NODE) return [];
    const results = [];
    const stack = [this.root];
    while (stack.length > 0) {
      const idx = stack.pop();
      const node = this.nodes[idx];
      if (node.maxX < minX || maxX < node.minX || node.maxY < minY || maxY < node.minY) {
        continue;
      }
      if (node.isLeaf) {
        results.push(node.body);
      } else {
        stack.push(node.child1);
        stack.push(node.child2);
      }
    }
    return results;
  }
  /** Find the best sibling for a new leaf via cost-based descent (BFS with priority queue). */
  findBestSibling(newNodeIdx) {
    const newNode = this.nodes[newNodeIdx];
    const newArea = (newNode.maxX - newNode.minX) * (newNode.maxY - newNode.minY);
    let bestSibling = this.root;
    let bestCost = this.unionArea(this.root, newNodeIdx);
    const queue = [
      { nodeIdx: this.root, inheritedCost: 0 }
    ];
    while (queue.length > 0) {
      let bestIdx = 0;
      for (let i = 1; i < queue.length; i++) {
        if (queue[i].inheritedCost < queue[bestIdx].inheritedCost) {
          bestIdx = i;
        }
      }
      const { nodeIdx, inheritedCost } = queue.splice(bestIdx, 1)[0];
      const node = this.nodes[nodeIdx];
      const directCost = this.unionArea(nodeIdx, newNodeIdx);
      const costForCurrent = directCost + inheritedCost;
      if (costForCurrent < bestCost) {
        bestCost = costForCurrent;
        bestSibling = nodeIdx;
      }
      const inheritedCostForChildren = inheritedCost + directCost - (node.maxX - node.minX) * (node.maxY - node.minY);
      const lowerBoundCost = newArea + inheritedCostForChildren;
      if (lowerBoundCost < bestCost && !node.isLeaf) {
        queue.push({ nodeIdx: node.child1, inheritedCost: inheritedCostForChildren });
        queue.push({ nodeIdx: node.child2, inheritedCost: inheritedCostForChildren });
      }
    }
    return { sibling: bestSibling, cost: bestCost };
  }
  /** Compute the area of the union of two nodes' AABBs. */
  unionArea(aIdx, bIdx) {
    const a = this.nodes[aIdx];
    const b = this.nodes[bIdx];
    const unionMinX = Math.min(a.minX, b.minX);
    const unionMinY = Math.min(a.minY, b.minY);
    const unionMaxX = Math.max(a.maxX, b.maxX);
    const unionMaxY = Math.max(a.maxY, b.maxY);
    return (unionMaxX - unionMinX) * (unionMaxY - unionMinY);
  }
  /** Refit AABBs up the tree from a given node, applying rotations for balance. */
  refitAncestors(fromIdx) {
    let ancestorIdx = this.nodes[fromIdx].parent;
    while (ancestorIdx !== NULL_NODE) {
      const ancestor = this.nodes[ancestorIdx];
      const child1 = this.nodes[ancestor.child1];
      const child2 = this.nodes[ancestor.child2];
      ancestor.minX = Math.min(child1.minX, child2.minX);
      ancestor.minY = Math.min(child1.minY, child2.minY);
      ancestor.maxX = Math.max(child1.maxX, child2.maxX);
      ancestor.maxY = Math.max(child1.maxY, child2.maxY);
      this.rotate(ancestorIdx);
      ancestorIdx = this.nodes[ancestorIdx].parent;
    }
  }
  /**
   * Tree rotation heuristic for balance.
   * Tries 4 swap types and picks the one with lowest area cost.
   * Adapted from Sopiro/Physics AABBTree.rotate().
   */
  rotate(nodeIdx) {
    const node = this.nodes[nodeIdx];
    const parentIdx = node.parent;
    if (parentIdx === NULL_NODE) return;
    const parent = this.nodes[parentIdx];
    const siblingIdx = parent.child1 === nodeIdx ? parent.child2 : parent.child1;
    const sibling = this.nodes[siblingIdx];
    if (sibling.isLeaf || node.isLeaf) return;
    const nodeArea = (node.maxX - node.minX) * (node.maxY - node.minY);
    const siblingArea = (sibling.maxX - sibling.minX) * (sibling.maxY - sibling.minY);
    const costDiffs = [
      this.unionArea(siblingIdx, node.child1) - nodeArea,
      this.unionArea(siblingIdx, node.child2) - nodeArea,
      this.unionArea(nodeIdx, sibling.child1) - siblingArea,
      this.unionArea(nodeIdx, sibling.child2) - siblingArea
    ];
    let bestDiffIdx = 0;
    for (let i = 1; i < 4; i++) {
      if (costDiffs[i] < costDiffs[bestDiffIdx]) {
        bestDiffIdx = i;
      }
    }
    if (costDiffs[bestDiffIdx] >= 0) return;
    switch (bestDiffIdx) {
      case 0: {
        const child1Idx = node.child1;
        if (parent.child1 === siblingIdx) {
          parent.child1 = child1Idx;
        } else {
          parent.child2 = child1Idx;
        }
        this.nodes[child1Idx].parent = parentIdx;
        node.child1 = siblingIdx;
        sibling.parent = nodeIdx;
        this.refitNode(nodeIdx);
        break;
      }
      case 1: {
        const child2Idx = node.child2;
        if (parent.child1 === siblingIdx) {
          parent.child1 = child2Idx;
        } else {
          parent.child2 = child2Idx;
        }
        this.nodes[child2Idx].parent = parentIdx;
        node.child2 = siblingIdx;
        sibling.parent = nodeIdx;
        this.refitNode(nodeIdx);
        break;
      }
      case 2: {
        const sibChild1Idx = sibling.child1;
        if (parent.child1 === nodeIdx) {
          parent.child1 = sibChild1Idx;
        } else {
          parent.child2 = sibChild1Idx;
        }
        this.nodes[sibChild1Idx].parent = parentIdx;
        sibling.child1 = nodeIdx;
        node.parent = siblingIdx;
        this.refitNode(siblingIdx);
        break;
      }
      case 3: {
        const sibChild2Idx = sibling.child2;
        if (parent.child1 === nodeIdx) {
          parent.child1 = sibChild2Idx;
        } else {
          parent.child2 = sibChild2Idx;
        }
        this.nodes[sibChild2Idx].parent = parentIdx;
        sibling.child2 = nodeIdx;
        node.parent = siblingIdx;
        this.refitNode(siblingIdx);
        break;
      }
    }
  }
  /** Refit a single node's AABB from its children. */
  refitNode(nodeIdx) {
    const node = this.nodes[nodeIdx];
    if (node.isLeaf) return;
    const child1 = this.nodes[node.child1];
    const child2 = this.nodes[node.child2];
    node.minX = Math.min(child1.minX, child2.minX);
    node.minY = Math.min(child1.minY, child2.minY);
    node.maxX = Math.max(child1.maxX, child2.maxX);
    node.maxY = Math.max(child1.maxY, child2.maxY);
  }
  /** Dual-subtree traversal to find all overlapping leaf pairs. */
  queryCross(aIdx, bIdx, pairs, checked, buffers) {
    const key = this.pairKey(aIdx, bIdx);
    if (checked.has(key)) return;
    checked.add(key);
    const a = this.nodes[aIdx];
    const b = this.nodes[bIdx];
    if (a.maxX < b.minX || b.maxX < a.minX || a.maxY < b.minY || b.maxY < a.minY) {
      return;
    }
    if (a.isLeaf && b.isLeaf) {
      const i = a.body;
      const j = b.body;
      const fi = buffers.flags[i];
      const fj = buffers.flags[j];
      if (!(fi & BodyFlags.ACTIVE) || !(fj & BodyFlags.ACTIVE)) return;
      const iStatic = (fi & BodyFlags.STATIC) !== 0;
      const jStatic = (fj & BodyFlags.STATIC) !== 0;
      const iSleeping = (fi & BodyFlags.SLEEPING) !== 0;
      const jSleeping = (fj & BodyFlags.SLEEPING) !== 0;
      if (iStatic && jStatic) return;
      if (iSleeping && jSleeping) return;
      pairs.push({ a: i, b: j });
      return;
    }
    if (!a.isLeaf && !b.isLeaf) {
      this.queryCross(a.child1, a.child2, pairs, checked, buffers);
      this.queryCross(b.child1, b.child2, pairs, checked, buffers);
      this.queryCross(a.child1, b.child1, pairs, checked, buffers);
      this.queryCross(a.child1, b.child2, pairs, checked, buffers);
      this.queryCross(a.child2, b.child1, pairs, checked, buffers);
      this.queryCross(a.child2, b.child2, pairs, checked, buffers);
    } else if (a.isLeaf && !b.isLeaf) {
      this.queryCross(b.child1, b.child2, pairs, checked, buffers);
      this.queryCross(aIdx, b.child1, pairs, checked, buffers);
      this.queryCross(aIdx, b.child2, pairs, checked, buffers);
    } else if (!a.isLeaf && b.isLeaf) {
      this.queryCross(a.child1, a.child2, pairs, checked, buffers);
      this.queryCross(a.child1, bIdx, pairs, checked, buffers);
      this.queryCross(a.child2, bIdx, pairs, checked, buffers);
    }
  }
  /** Generate a unique key for a pair of node indices using Szudzik's bijective pairing. */
  pairKey(a, b) {
    const lo = a < b ? a : b;
    const hi = a < b ? b : a;
    return hi * hi + hi + lo;
  }
  /** Allocate a new node from the pool. */
  allocateNode() {
    const idx = this.nodes.length;
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
      body: NULL_NODE
    });
    return idx;
  }
  /** Free a node (mark for reuse). */
  freeNode(idx) {
    this.nodes[idx].body = NULL_NODE;
    this.nodes[idx].isLeaf = false;
  }
  /** Get the number of bodies in the tree. */
  get bodyCount() {
    return this.bodyToNode.size;
  }
  /** Get the total cost (sum of all node AABB areas) — useful for debugging. */
  get cost() {
    let total = 0;
    for (const node of this.nodes) {
      total += (node.maxX - node.minX) * (node.maxY - node.minY);
    }
    return total;
  }
};

// src/collision/broadphase.ts
function broadphase(buffers, count, treeOrMethod, method) {
  let tree = null;
  let actualMethod = "dynamicTree";
  if (typeof treeOrMethod === "string") {
    actualMethod = treeOrMethod;
  } else {
    tree = treeOrMethod != null ? treeOrMethod : null;
    actualMethod = method != null ? method : "dynamicTree";
  }
  if (actualMethod === "dynamicTree" && tree) return tree.queryPairs(buffers);
  if (actualMethod === "bvh") return broadphaseBVH(buffers, count);
  return broadphaseSAP(buffers, count);
}
function broadphaseSAP(buffers, count) {
  const { flags } = buffers;
  const aabbCache = new Float64Array(count * 4);
  for (let i = 0; i < count; i++) {
    if (!(flags[i] & BodyFlags.ACTIVE)) continue;
    const [minX, minY, maxX, maxY] = computeBodyAABB(i, buffers);
    aabbCache[i * 4 + 0] = minX;
    aabbCache[i * 4 + 1] = minY;
    aabbCache[i * 4 + 2] = maxX;
    aabbCache[i * 4 + 3] = maxY;
  }
  const epCount = count * 2;
  const epValue = new Float64Array(epCount);
  const epBody = new Int32Array(epCount);
  const epIsMin = new Uint8Array(epCount);
  let n = 0;
  for (let i = 0; i < count; i++) {
    if (!(flags[i] & BodyFlags.ACTIVE)) continue;
    epValue[n] = aabbCache[i * 4 + 0];
    epBody[n] = i;
    epIsMin[n] = 1;
    n++;
    epValue[n] = aabbCache[i * 4 + 2];
    epBody[n] = i;
    epIsMin[n] = 0;
    n++;
  }
  const order = new Int32Array(n);
  for (let i = 0; i < n; i++) order[i] = i;
  order.subarray(0, n).sort(
    (a, b) => epValue[a] - epValue[b] || epIsMin[b] - epIsMin[a]
  );
  const pairs = [];
  const active = new Int32Array(count);
  let activeCount = 0;
  for (let k = 0; k < n; k++) {
    const ep = order[k];
    const i = epBody[ep];
    if (epIsMin[ep]) {
      const minYi = aabbCache[i * 4 + 1];
      const maxYi = aabbCache[i * 4 + 3];
      const fi = flags[i];
      for (let m = 0; m < activeCount; m++) {
        const j = active[m];
        if (fi & BodyFlags.STATIC && flags[j] & BodyFlags.STATIC) continue;
        const minYj = aabbCache[j * 4 + 1];
        const maxYj = aabbCache[j * 4 + 3];
        if (maxYi >= minYj && minYi <= maxYj) {
          pairs.push({ a: i, b: j });
        }
      }
      active[activeCount++] = i;
    } else {
      for (let m = 0; m < activeCount; m++) {
        if (active[m] === i) {
          active[m] = active[activeCount - 1];
          activeCount--;
          break;
        }
      }
    }
  }
  return pairs;
}
function broadphaseBVH(buffers, count) {
  return broadphaseSAP(buffers, count);
}
var registry = /* @__PURE__ */ new Map();
function registerShapeHandler(type, handler) {
  registry.set(type, handler);
}
function getShapeHandler(type) {
  return registry.get(type);
}
var circleHandler = {
  getAABB(idx, buffers) {
    const r = buffers.shapeRadius[idx];
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    return [px - r, py - r, px + r, py + r];
  },
  getWorldVertices(_idx, _buffers, _outX, _outY) {
    return 0;
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    const r = buffers.shapeRadius[idx];
    const len = length(dx, dy);
    if (len < 1e-10) return [px + r, py];
    return [px + dx / len * r, py + dy / len * r];
  }
};
var boxHandler = {
  getAABB(idx, buffers) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const lx = [-hx, hx, hx, -hx];
    const ly = [-hy, -hy, hy, hy];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (let i = 0; i < 4; i++) {
      const wx = c * lx[i] + negS * ly[i] + px;
      const wy = s * lx[i] + c * ly[i] + py;
      if (wx < minX) minX = wx;
      if (wx > maxX) maxX = wx;
      if (wy < minY) minY = wy;
      if (wy > maxY) maxY = wy;
    }
    return [minX, minY, maxX, maxY];
  },
  getWorldVertices(idx, buffers, outX, outY) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    const lx = [-hx, hx, hx, -hx];
    const ly = [-hy, -hy, hy, hy];
    for (let i = 0; i < 4; i++) {
      outX[i] = c * lx[i] + negS * ly[i] + px;
      outY[i] = s * lx[i] + c * ly[i] + py;
    }
    return 4;
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    const lx = [-hx, hx, hx, -hx];
    const ly = [-hy, -hy, hy, hy];
    let maxD = -Infinity;
    let bestX = px, bestY = py;
    for (let i = 0; i < 4; i++) {
      const wx = c * lx[i] + negS * ly[i] + px;
      const wy = s * lx[i] + c * ly[i] + py;
      const d = dot(wx, wy, dx, dy);
      if (d > maxD) {
        maxD = d;
        bestX = wx;
        bestY = wy;
      }
    }
    return [bestX, bestY];
  }
};
var polygonHandler = {
  getAABB(idx, buffers) {
    const count = buffers.shapeVertexCount[idx];
    const base = idx * MAX_VERTICES_PER_SHAPE;
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i];
      const ly = buffers.shapeVerticesY[base + i];
      const wx = c * lx + negS * ly + px;
      const wy = s * lx + c * ly + py;
      if (wx < minX) minX = wx;
      if (wx > maxX) maxX = wx;
      if (wy < minY) minY = wy;
      if (wy > maxY) maxY = wy;
    }
    if (count === 0) {
      const hx = buffers.halfExtentX[idx];
      const hy = buffers.halfExtentY[idx];
      return [px - hx, py - hy, px + hx, py + hy];
    }
    return [minX, minY, maxX, maxY];
  },
  getWorldVertices(idx, buffers, outX, outY) {
    const count = buffers.shapeVertexCount[idx];
    const base = idx * MAX_VERTICES_PER_SHAPE;
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i];
      const ly = buffers.shapeVerticesY[base + i];
      outX[i] = c * lx + negS * ly + px;
      outY[i] = s * lx + c * ly + py;
    }
    return count;
  },
  getSupportPoint(idx, buffers, dx, dy) {
    const count = buffers.shapeVertexCount[idx];
    const base = idx * MAX_VERTICES_PER_SHAPE;
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const px = buffers.positionX[idx];
    const py = buffers.positionY[idx];
    let maxD = -Infinity;
    let bestX = px, bestY = py;
    for (let i = 0; i < count; i++) {
      const lx = buffers.shapeVerticesX[base + i];
      const ly = buffers.shapeVerticesY[base + i];
      const wx = c * lx + negS * ly + px;
      const wy = s * lx + c * ly + py;
      const d = dot(wx, wy, dx, dy);
      if (d > maxD) {
        maxD = d;
        bestX = wx;
        bestY = wy;
      }
    }
    return [bestX, bestY];
  }
};
registerShapeHandler(ShapeType.Circle, circleHandler);
registerShapeHandler(ShapeType.Box, boxHandler);
registerShapeHandler(ShapeType.Polygon, polygonHandler);

// src/collision/simplex.ts
var Simplex = class {
  constructor() {
    this.verticesX = [];
    this.verticesY = [];
  }
  get count() {
    return this.verticesX.length;
  }
  clear() {
    this.verticesX = [];
    this.verticesY = [];
  }
  addVertex(x, y) {
    if (this.count >= 3) throw new Error("Simplex can have at most 3 vertices");
    this.verticesX.push(x);
    this.verticesY.push(y);
  }
  containsVertex(x, y, epsilon = 1e-10) {
    for (let i = 0; i < this.count; i++) {
      const dx = this.verticesX[i] - x;
      const dy = this.verticesY[i] - y;
      if (dx * dx + dy * dy < epsilon) return true;
    }
    return false;
  }
  shrink(indices) {
    const newX = [];
    const newY = [];
    for (let i = 0; i < indices.length; i++) {
      newX.push(this.verticesX[indices[i]]);
      newY.push(this.verticesY[indices[i]]);
    }
    this.verticesX = newX;
    this.verticesY = newY;
  }
  /**
   * Returns the closest point on this simplex to the origin (qx=0, qy=0).
   * Also returns which vertices contributed to the closest point.
   */
  getClosest(qx, qy) {
    switch (this.count) {
      case 1:
        return { resultX: this.verticesX[0], resultY: this.verticesY[0], contributors: [0] };
      case 2: {
        const ax = this.verticesX[0];
        const ay = this.verticesY[0];
        const bx = this.verticesX[1];
        const by = this.verticesY[1];
        const w = getUV(ax, ay, bx, by, qx, qy);
        if (w.v <= 0) {
          return { resultX: ax, resultY: ay, contributors: [0] };
        } else if (w.v >= 1) {
          return { resultX: bx, resultY: by, contributors: [1] };
        } else {
          const rx = ax + w.v * (bx - ax);
          const ry = ay + w.v * (by - ay);
          return { resultX: rx, resultY: ry, contributors: [0, 1] };
        }
      }
      case 3: {
        const ax = this.verticesX[0];
        const ay = this.verticesY[0];
        const bx = this.verticesX[1];
        const by = this.verticesY[1];
        const cx = this.verticesX[2];
        const cy = this.verticesY[2];
        const wab = getUV(ax, ay, bx, by, qx, qy);
        const wbc = getUV(bx, by, cx, cy, qx, qy);
        const wca = getUV(cx, cy, ax, ay, qx, qy);
        if (wca.u <= 0 && wab.v <= 0) {
          return { resultX: ax, resultY: ay, contributors: [0] };
        } else if (wab.u <= 0 && wbc.v <= 0) {
          return { resultX: bx, resultY: by, contributors: [1] };
        } else if (wbc.u <= 0 && wca.v <= 0) {
          return { resultX: cx, resultY: cy, contributors: [2] };
        }
        const area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
        const u = (bx - qx) * (cy - qy) - (cx - qx) * (by - qy);
        const v = (cx - qx) * (ay - qy) - (ax - qx) * (cy - qy);
        const w = (ax - qx) * (by - qy) - (bx - qx) * (ay - qy);
        if (wab.u > 0 && wab.v > 0 && w * area <= 0) {
          const rx = ax + wab.u * (bx - ax);
          const ry = ay + wab.v * (by - ay);
          return {
            resultX: rx,
            resultY: ry,
            contributors: area !== 0 ? [0, 1] : [0, 1, 2]
          };
        } else if (wbc.u > 0 && wbc.v > 0 && u * area <= 0) {
          const rx = bx + wbc.u * (cx - bx);
          const ry = by + wbc.v * (cy - by);
          return {
            resultX: rx,
            resultY: ry,
            contributors: area !== 0 ? [1, 2] : [0, 1, 2]
          };
        } else if (wca.u > 0 && wca.v > 0 && v * area <= 0) {
          const rx = cx + wca.u * (ax - cx);
          const ry = cy + wca.v * (ay - cy);
          return {
            resultX: rx,
            resultY: ry,
            contributors: area !== 0 ? [2, 0] : [0, 1, 2]
          };
        } else {
          return { resultX: qx, resultY: qy, contributors: [] };
        }
      }
      default:
        throw new Error("Simplex contains more than 3 vertices");
    }
  }
};
function getUV(ax, ay, bx, by, qx, qy) {
  const abx = bx - ax;
  const aby = by - ay;
  const aqx = qx - ax;
  const aqy = qy - ay;
  const abLenSq = abx * abx + aby * aby;
  if (abLenSq < 1e-20) {
    return { u: 1, v: 0 };
  }
  const v = (aqx * abx + aqy * aby) / abLenSq;
  const u = 1 - v;
  return { u, v };
}

// src/collision/polytope.ts
var Polytope = class {
  constructor(simplex) {
    if (simplex.count !== 3) throw new Error("Input simplex must be a triangle");
    this.verticesX = [simplex.verticesX[0], simplex.verticesX[1], simplex.verticesX[2]];
    this.verticesY = [simplex.verticesY[0], simplex.verticesY[1], simplex.verticesY[2]];
  }
  get count() {
    return this.verticesX.length;
  }
  /**
   * Returns the edge closest to the origin.
   * For each edge, computes the outward normal and the signed distance
   * from the origin to the edge's supporting line.
   */
  getClosestEdge() {
    let minIndex = 0;
    let minDistance = Infinity;
    let minNormalX = 0;
    let minNormalY = 0;
    for (let i = 0; i < this.count; i++) {
      const j = (i + 1) % this.count;
      const vx = this.verticesX[i];
      const vy = this.verticesY[i];
      const wx = this.verticesX[j];
      const wy = this.verticesY[j];
      const ex = wx - vx;
      const ey = wy - vy;
      let nx = ey;
      let ny = -ex;
      const len = Math.sqrt(nx * nx + ny * ny);
      if (len < 1e-20) continue;
      nx /= len;
      ny /= len;
      let distance = nx * vx + ny * vy;
      if (distance < 0) {
        distance = -distance;
        nx = -nx;
        ny = -ny;
      }
      if (distance < minDistance) {
        minDistance = distance;
        minNormalX = nx;
        minNormalY = ny;
        minIndex = i;
      }
    }
    return {
      index: minIndex,
      distance: minDistance,
      normalX: minNormalX,
      normalY: minNormalY
    };
  }
};

// src/collision/gjk.ts
var GJK_MAX_ITERATIONS = 30;
var GJK_TOLERANCE = 1e-12;
var EPA_MAX_ITERATIONS = 50;
var EPA_TOLERANCE = 1e-8;
var TANGENT_MIN_LENGTH = 0.01;
var FALLBACK_PENETRATION_MIN = 1e-3;
var CONTACT_MERGE_THRESHOLD = 1.415 * TANGENT_MIN_LENGTH;
var EPA_DEPTH_CAP = 0.5;
function getShapeVertices(idx, buffers, outX, outY) {
  const type = buffers.shapeType[idx];
  if (type === ShapeType.Box) {
    const hx = buffers.halfExtentX[idx];
    const hy = buffers.halfExtentY[idx];
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const bx = buffers.positionX[idx];
    const by = buffers.positionY[idx];
    const lx = [-hx, hx, hx, -hx];
    const ly = [-hy, -hy, hy, hy];
    for (let i = 0; i < 4; i++) {
      outX[i] = c * lx[i] + negS * ly[i] + bx;
      outY[i] = s * lx[i] + c * ly[i] + by;
    }
    return 4;
  }
  if (type === ShapeType.Circle) {
    return 0;
  }
  if (type === ShapeType.Polygon) {
    const count = buffers.shapeVertexCount[idx];
    const [c, negS, s] = mat2FromAngle(buffers.angle[idx]);
    const bx = buffers.positionX[idx];
    const by = buffers.positionY[idx];
    const base = idx * MAX_VERTICES_PER_SHAPE;
    for (let i = 0; i < count; i++) {
      const lvx = buffers.shapeVerticesX[base + i];
      const lvy = buffers.shapeVerticesY[base + i];
      outX[i] = c * lvx + negS * lvy + bx;
      outY[i] = s * lvx + c * lvy + by;
    }
    return count;
  }
  const handler = getShapeHandler(type);
  if (handler) return handler.getWorldVertices(idx, buffers, outX, outY);
  return 0;
}
function supportCircle(cx, cy, radius, dx, dy) {
  const len = length(dx, dy);
  if (len < 1e-10) return { x: cx + radius, y: cy, index: -1 };
  return { x: cx + dx / len * radius, y: cy + dy / len * radius, index: -1 };
}
function supportPolygon(vertsX, vertsY, count, dx, dy) {
  let maxDot = -Infinity;
  let bestX = 0;
  let bestY = 0;
  let bestIdx = 0;
  for (let i = 0; i < count; i++) {
    const d = dot(vertsX[i], vertsY[i], dx, dy);
    if (d > maxDot) {
      maxDot = d;
      bestX = vertsX[i];
      bestY = vertsY[i];
      bestIdx = i;
    }
  }
  return { x: bestX, y: bestY, index: bestIdx };
}
function minkowskiSupport(vertsAX, vertsAY, countA, typeA, vertsBX, vertsBY, countB, typeB, aIdx, bIdx, buffers, dx, dy) {
  let saX, saY;
  if (typeA === ShapeType.Circle) {
    const sr = supportCircle(
      buffers.positionX[aIdx],
      buffers.positionY[aIdx],
      buffers.shapeRadius[aIdx],
      dx,
      dy
    );
    saX = sr.x;
    saY = sr.y;
  } else {
    const handlerA = typeA !== ShapeType.Box && typeA !== ShapeType.Polygon ? getShapeHandler(typeA) : void 0;
    if (handlerA) {
      [saX, saY] = handlerA.getSupportPoint(aIdx, buffers, dx, dy);
    } else {
      const sr = supportPolygon(vertsAX, vertsAY, countA, dx, dy);
      saX = sr.x;
      saY = sr.y;
    }
  }
  let sbX, sbY;
  if (typeB === ShapeType.Circle) {
    const sr = supportCircle(
      buffers.positionX[bIdx],
      buffers.positionY[bIdx],
      buffers.shapeRadius[bIdx],
      -dx,
      -dy
    );
    sbX = sr.x;
    sbY = sr.y;
  } else {
    const handlerB = typeB !== ShapeType.Box && typeB !== ShapeType.Polygon ? getShapeHandler(typeB) : void 0;
    if (handlerB) {
      [sbX, sbY] = handlerB.getSupportPoint(bIdx, buffers, -dx, -dy);
    } else {
      const sr = supportPolygon(vertsBX, vertsBY, countB, -dx, -dy);
      sbX = sr.x;
      sbY = sr.y;
    }
  }
  return [saX - sbX, saY - sbY];
}
function gjk(vertsAX, vertsAY, countA, typeA, vertsBX, vertsBY, countB, typeB, aIdx, bIdx, buffers) {
  const simplex = new Simplex();
  const dx = buffers.positionX[aIdx] - buffers.positionX[bIdx];
  const dy = buffers.positionY[aIdx] - buffers.positionY[bIdx];
  let dirX, dirY;
  if (dx * dx + dy * dy < 1e-10) {
    dirX = 1;
    dirY = 0;
  } else {
    dirX = dx;
    dirY = dy;
  }
  const [sx, sy] = minkowskiSupport(
    vertsAX,
    vertsAY,
    countA,
    typeA,
    vertsBX,
    vertsBY,
    countB,
    typeB,
    aIdx,
    bIdx,
    buffers,
    dirX,
    dirY
  );
  simplex.addVertex(sx, sy);
  dirX = -sx;
  dirY = -sy;
  for (let k = 0; k < GJK_MAX_ITERATIONS; k++) {
    const closest = simplex.getClosest(0, 0);
    const distSq = closest.resultX * closest.resultX + closest.resultY * closest.resultY;
    if (distSq < GJK_TOLERANCE) {
      return { collided: true, simplex };
    }
    if (simplex.count !== 1) {
      simplex.shrink(closest.contributors);
    }
    dirX = -closest.resultX;
    dirY = -closest.resultY;
    const dirLen = length(dirX, dirY);
    if (dirLen < 1e-10) {
      return { collided: true, simplex };
    }
    const [nx, ny] = minkowskiSupport(
      vertsAX,
      vertsAY,
      countA,
      typeA,
      vertsBX,
      vertsBY,
      countB,
      typeB,
      aIdx,
      bIdx,
      buffers,
      dirX,
      dirY
    );
    const supportProj = (nx * dirX + ny * dirY) / dirLen;
    const deltaProj = ((nx - closest.resultX) * dirX + (ny - closest.resultY) * dirY) / dirLen;
    if (dirLen > deltaProj) {
      return { collided: false, simplex };
    }
    if (supportProj < -1e-6) {
      return { collided: false, simplex };
    }
    if (supportProj < -1e-6) {
      return { collided: false, simplex };
    }
    if (simplex.containsVertex(nx, ny)) {
      return { collided: false, simplex };
    }
    simplex.addVertex(nx, ny);
  }
  return { collided: true, simplex };
}
function epa(simplex, vertsAX, vertsAY, countA, typeA, vertsBX, vertsBY, countB, typeB, aIdx, bIdx, buffers) {
  let poly;
  try {
    poly = new Polytope(simplex);
  } catch (e) {
    return null;
  }
  let closestEdge = poly.getClosestEdge();
  for (let i = 0; i < EPA_MAX_ITERATIONS; i++) {
    const [sx, sy] = minkowskiSupport(
      vertsAX,
      vertsAY,
      countA,
      typeA,
      vertsBX,
      vertsBY,
      countB,
      typeB,
      aIdx,
      bIdx,
      buffers,
      closestEdge.normalX,
      closestEdge.normalY
    );
    const newDistance = dot(sx, sy, closestEdge.normalX, closestEdge.normalY);
    if (Math.abs(closestEdge.distance - newDistance) > EPA_TOLERANCE) {
      let alreadyPresent = false;
      for (let v = 0; v < poly.count; v++) {
        const dvx = sx - poly.verticesX[v];
        const dvy = sy - poly.verticesY[v];
        if (dvx * dvx + dvy * dvy < 1e-10) {
          alreadyPresent = true;
          break;
        }
      }
      if (alreadyPresent) break;
      const insertIdx = closestEdge.index + 1;
      poly.verticesX.splice(insertIdx, 0, sx);
      poly.verticesY.splice(insertIdx, 0, sy);
      closestEdge = poly.getClosestEdge();
    } else {
      break;
    }
  }
  return {
    penetrationDepth: Math.max(closestEdge.distance, 0),
    contactNormalX: closestEdge.normalX,
    contactNormalY: closestEdge.normalY
  };
}
function clipEdge(edge, px, py, dirx, diry, remove = false) {
  const d1 = dot(edge.p1x - px, edge.p1y - py, dirx, diry);
  const d2 = dot(edge.p2x - px, edge.p2y - py, dirx, diry);
  if (d1 >= 0 && d2 >= 0) return;
  const per = Math.abs(d1) + Math.abs(d2);
  if (per < 1e-10) return;
  if (d1 < 0) {
    if (remove) {
      edge.p1x = edge.p2x;
      edge.p1y = edge.p2y;
      edge.id1 = edge.id2;
    } else {
      const t = -d1 / per;
      edge.p1x = edge.p1x + (edge.p2x - edge.p1x) * t;
      edge.p1y = edge.p1y + (edge.p2y - edge.p1y) * t;
    }
  } else if (d2 < 0) {
    if (remove) {
      edge.p2x = edge.p1x;
      edge.p2y = edge.p1y;
      edge.id2 = edge.id1;
    } else {
      const t = -d2 / per;
      edge.p2x = edge.p2x + (edge.p1x - edge.p2x) * t;
      edge.p2y = edge.p2y + (edge.p1y - edge.p2y) * t;
    }
  }
}
function findFarthestEdge(bodyIdx, buffers, vertsX, vertsY, count, type, dirx, diry) {
  if (type === ShapeType.Circle) {
    const bx2 = buffers.positionX[bodyIdx];
    const by2 = buffers.positionY[bodyIdx];
    const radius = buffers.shapeRadius[bodyIdx];
    const len = length(dirx, diry);
    const nx = len < 1e-10 ? 1 : dirx / len;
    const ny = len < 1e-10 ? 0 : diry / len;
    const cx = bx2 + nx * radius;
    const cy = by2 + ny * radius;
    const tangentX = -diry * TANGENT_MIN_LENGTH;
    const tangentY = dirx * TANGENT_MIN_LENGTH;
    return { p1x: cx, p1y: cy, p2x: cx + tangentX, p2y: cy + tangentY, id1: -1, id2: -1 };
  }
  buffers.positionX[bodyIdx];
  buffers.positionY[bodyIdx];
  const sr = supportPolygon(vertsX, vertsY, count, dirx, diry);
  const currX = sr.x;
  const currY = sr.y;
  const idx = sr.index;
  const prevIdx = (idx - 1 + count) % count;
  const nextIdx = (idx + 1) % count;
  const e1x = currX - vertsX[prevIdx];
  const e1y = currY - vertsY[prevIdx];
  const e1Len = length(e1x, e1y);
  const e1dot = e1Len < 1e-10 ? 0 : (e1x * dirx + e1y * diry) / e1Len;
  const e2x = currX - vertsX[nextIdx];
  const e2y = currY - vertsY[nextIdx];
  const e2Len = length(e2x, e2y);
  const e2dot = e2Len < 1e-10 ? 0 : (e2x * dirx + e2y * diry) / e2Len;
  const usePrev = Math.abs(e1dot) <= Math.abs(e2dot);
  if (usePrev) {
    return { p1x: vertsX[prevIdx], p1y: vertsY[prevIdx], p2x: currX, p2y: currY, id1: prevIdx, id2: idx };
  } else {
    return { p1x: currX, p1y: currY, p2x: vertsX[nextIdx], p2y: vertsY[nextIdx], id1: idx, id2: nextIdx };
  }
}
function findContactPoints(nx, ny, aIdx, buffers, vertsAX, vertsAY, countA, typeA, bIdx, vertsBX, vertsBY, countB, typeB) {
  if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
    const cax = buffers.positionX[aIdx];
    const cay = buffers.positionY[aIdx];
    buffers.positionX[bIdx];
    buffers.positionY[bIdx];
    const ra = buffers.shapeRadius[aIdx];
    return [{ x: cax + nx * ra, y: cay + ny * ra, id: -1 }];
  }
  const edgeA = findFarthestEdge(aIdx, buffers, vertsAX, vertsAY, countA, typeA, nx, ny);
  const edgeB = findFarthestEdge(bIdx, buffers, vertsBX, vertsBY, countB, typeB, -nx, -ny);
  let ref = edgeA;
  let inc = edgeB;
  let flip = false;
  const aEdgeLen = length(edgeA.p2x - edgeA.p1x, edgeA.p2y - edgeA.p1y);
  const bEdgeLen = length(edgeB.p2x - edgeB.p1x, edgeB.p2y - edgeB.p1y);
  if (aEdgeLen < 1e-10 && bEdgeLen < 1e-10) {
    return [{ x: edgeA.p1x, y: edgeA.p1y, id: edgeA.id1 }];
  }
  const aPerp = aEdgeLen < 1e-10 ? 1 : Math.abs(dot(edgeA.p2x - edgeA.p1x, edgeA.p2y - edgeA.p1y, nx, ny)) / aEdgeLen;
  const bPerp = bEdgeLen < 1e-10 ? 1 : Math.abs(dot(edgeB.p2x - edgeB.p1x, edgeB.p2y - edgeB.p1y, nx, ny)) / bEdgeLen;
  if (aPerp >= bPerp) {
    ref = edgeB;
    inc = edgeA;
    flip = true;
  }
  const refDirX = ref.p2x - ref.p1x;
  const refDirY = ref.p2y - ref.p1y;
  const refDirLen = length(refDirX, refDirY);
  if (refDirLen < 1e-10) {
    return [{ x: ref.p1x, y: ref.p1y, id: ref.id1 }];
  }
  const refDirNX = refDirX / refDirLen;
  const refDirNY = refDirY / refDirLen;
  clipEdge(inc, ref.p1x, ref.p1y, -refDirNX, -refDirNY, false);
  clipEdge(inc, ref.p2x, ref.p2y, refDirNX, refDirNY, false);
  const clipNormalX = flip ? nx : -nx;
  const clipNormalY = flip ? ny : -ny;
  clipEdge(inc, ref.p1x, ref.p1y, clipNormalX, clipNormalY, true);
  const incLen = length(inc.p2x - inc.p1x, inc.p2y - inc.p1y);
  if (incLen <= CONTACT_MERGE_THRESHOLD) {
    return [{ x: inc.p1x, y: inc.p1y, id: inc.id1 }];
  }
  if (incLen < 1e-10) {
    return [{ x: inc.p1x, y: inc.p1y, id: inc.id1 }];
  }
  return [
    { x: inc.p1x, y: inc.p1y, id: inc.id1 },
    { x: inc.p2x, y: inc.p2y, id: inc.id2 }
  ];
}
function circleCircleContact(aIdx, bIdx, buffers) {
  const dx = buffers.positionX[bIdx] - buffers.positionX[aIdx];
  const dy = buffers.positionY[bIdx] - buffers.positionY[aIdx];
  const distSq = dx * dx + dy * dy;
  const radiusSum = buffers.shapeRadius[aIdx] + buffers.shapeRadius[bIdx];
  if (distSq >= radiusSum * radiusSum) return null;
  let contactNormalX;
  let contactNormalY;
  let penetration;
  let contactX;
  let contactY;
  if (distSq < 1e-10) {
    contactNormalX = 1;
    contactNormalY = 0;
    penetration = radiusSum;
    contactX = buffers.positionX[aIdx];
    contactY = buffers.positionY[aIdx];
  } else {
    const dist = Math.sqrt(distSq);
    contactNormalX = dx / dist;
    contactNormalY = dy / dist;
    penetration = radiusSum - dist;
    contactX = buffers.positionX[aIdx] + contactNormalX * buffers.shapeRadius[aIdx];
    contactY = buffers.positionY[aIdx] + contactNormalY * buffers.shapeRadius[aIdx];
  }
  const [cA, negSA, sA] = mat2FromAngle(buffers.angle[aIdx]);
  const [cB, negSB, sB] = mat2FromAngle(buffers.angle[bIdx]);
  const [lAx, lAy] = mat2TransposeMulVec(
    cA,
    negSA,
    sA,
    cA,
    contactX - buffers.positionX[aIdx],
    contactY - buffers.positionY[aIdx]
  );
  const [lBx, lBy] = mat2TransposeMulVec(
    cB,
    negSB,
    sB,
    cB,
    contactX - buffers.positionX[bIdx],
    contactY - buffers.positionY[bIdx]
  );
  return {
    bodyA: aIdx,
    bodyB: bIdx,
    normal: { x: contactNormalX, y: contactNormalY },
    contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration, idA: -1, idB: -1 }]
  };
}
function buildContactManifold(aIdx, bIdx, buffers, vertsAX, vertsAY, countA, typeA, vertsBX, vertsBY, countB, typeB, depth, nx, ny) {
  const contactPoints = findContactPoints(
    nx,
    ny,
    aIdx,
    buffers,
    vertsAX,
    vertsAY,
    countA,
    typeA,
    bIdx,
    vertsBX,
    vertsBY,
    countB,
    typeB
  );
  if (contactPoints.length === 0) {
    const bodyAX = buffers.positionX[aIdx];
    const bodyAY = buffers.positionY[aIdx];
    const bodyBX = buffers.positionX[bIdx];
    const bodyBY = buffers.positionY[bIdx];
    const contactX = bodyAX + nx * depth * 0.5;
    const contactY = bodyAY + ny * depth * 0.5;
    const [cA2, negSA2, sA2] = mat2FromAngle(buffers.angle[aIdx]);
    const [cB2, negSB2, sB2] = mat2FromAngle(buffers.angle[bIdx]);
    const [lAx, lAy] = mat2TransposeMulVec(
      cA2,
      negSA2,
      sA2,
      cA2,
      contactX - bodyAX,
      contactY - bodyAY
    );
    const [lBx, lBy] = mat2TransposeMulVec(
      cB2,
      negSB2,
      sB2,
      cB2,
      contactX - bodyBX,
      contactY - bodyBY
    );
    return {
      bodyA: aIdx,
      bodyB: bIdx,
      normal: { x: nx, y: ny },
      contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration: depth, idA: -1, idB: -1 }]
    };
  }
  const [cA, negSA, sA] = mat2FromAngle(buffers.angle[aIdx]);
  const [cB, negSB, sB] = mat2FromAngle(buffers.angle[bIdx]);
  const contacts = [];
  for (let i = 0; i < contactPoints.length; i++) {
    const cp = contactPoints[i];
    const [lAx, lAy] = mat2TransposeMulVec(
      cA,
      negSA,
      sA,
      cA,
      cp.x - buffers.positionX[aIdx],
      cp.y - buffers.positionY[aIdx]
    );
    const [lBx, lBy] = mat2TransposeMulVec(
      cB,
      negSB,
      sB,
      cB,
      cp.x - buffers.positionX[bIdx],
      cp.y - buffers.positionY[bIdx]
    );
    contacts.push({
      localA: { x: lAx, y: lAy },
      localB: { x: lBx, y: lBy },
      penetration: depth,
      idA: cp.id >= 0 ? cp.id : -1,
      idB: cp.id >= 0 ? cp.id : -1
    });
  }
  return {
    bodyA: aIdx,
    bodyB: bIdx,
    normal: { x: nx, y: ny },
    contacts
  };
}
function gjkNarrowphase(buffers, pairs) {
  const manifolds = [];
  const vertsAX = new Array(MAX_VERTICES_PER_SHAPE);
  const vertsAY = new Array(MAX_VERTICES_PER_SHAPE);
  const vertsBX = new Array(MAX_VERTICES_PER_SHAPE);
  const vertsBY = new Array(MAX_VERTICES_PER_SHAPE);
  for (let i = 0; i < pairs.length; i++) {
    const a = pairs[i].a;
    const b = pairs[i].b;
    if ((buffers.flags[a] & BodyFlags.ACTIVE) === 0 || (buffers.flags[b] & BodyFlags.ACTIVE) === 0) continue;
    if ((buffers.flags[a] & BodyFlags.SLEEPING) !== 0 && (buffers.flags[b] & BodyFlags.SLEEPING) !== 0) continue;
    const typeA = buffers.shapeType[a];
    const typeB = buffers.shapeType[b];
    if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
      const m = circleCircleContact(a, b, buffers);
      if (m) manifolds.push(m);
      continue;
    }
    const countA = getShapeVertices(a, buffers, vertsAX, vertsAY);
    const countB = getShapeVertices(b, buffers, vertsBX, vertsBY);
    const gjkResult = gjk(
      vertsAX,
      vertsAY,
      countA,
      typeA,
      vertsBX,
      vertsBY,
      countB,
      typeB,
      a,
      b,
      buffers
    );
    if (!gjkResult.collided) continue;
    if (typeA === ShapeType.Box && typeB === ShapeType.Box) {
      const angleA = buffers.angle[a];
      const angleB = buffers.angle[b];
      const dx2 = buffers.positionX[b] - buffers.positionX[a];
      const dy2 = buffers.positionY[b] - buffers.positionY[a];
      const cosA = Math.cos(angleA), sinA = Math.sin(angleA);
      const cosB = Math.cos(angleB), sinB = Math.sin(angleB);
      const axes = [[cosA, sinA], [-sinA, cosA], [cosB, sinB], [-sinB, cosB]];
      let overlapping = true;
      for (const [ax, ay] of axes) {
        const rA = Math.abs(buffers.halfExtentX[a] * (ax * cosA + ay * sinA)) + Math.abs(buffers.halfExtentY[a] * (ax * -sinA + ay * cosA));
        const rB = Math.abs(buffers.halfExtentX[b] * (ax * cosB + ay * sinB)) + Math.abs(buffers.halfExtentY[b] * (ax * -sinB + ay * cosB));
        if (rA + rB <= Math.abs(dx2 * ax + dy2 * ay)) {
          overlapping = false;
          break;
        }
      }
      if (!overlapping) continue;
    }
    let simplex = gjkResult.simplex;
    if (simplex.count === 1) {
      const vx = simplex.verticesX[0];
      const vy = simplex.verticesY[0];
      const dirToOriginX = -vx;
      const dirToOriginY = -vy;
      const dirLen = length(dirToOriginX, dirToOriginY);
      if (dirLen < 1e-10) {
        const perpX = 0;
        const perpY = 1;
        const [sx, sy] = minkowskiSupport(
          vertsAX,
          vertsAY,
          countA,
          typeA,
          vertsBX,
          vertsBY,
          countB,
          typeB,
          a,
          b,
          buffers,
          perpX,
          perpY
        );
        if (Math.abs(sx - vx) > 1e-10 || Math.abs(sy - vy) > 1e-10) {
          simplex.addVertex(sx, sy);
        }
      } else {
        const ndx = dirToOriginX / dirLen;
        const ndy = dirToOriginY / dirLen;
        const [sx, sy] = minkowskiSupport(
          vertsAX,
          vertsAY,
          countA,
          typeA,
          vertsBX,
          vertsBY,
          countB,
          typeB,
          a,
          b,
          buffers,
          ndx,
          ndy
        );
        if (Math.abs(sx - vx) > 1e-10 || Math.abs(sy - vy) > 1e-10) {
          simplex.addVertex(sx, sy);
        }
      }
    }
    if (simplex.count === 2) {
      const ex = simplex.verticesX[1] - simplex.verticesX[0];
      const ey = simplex.verticesY[1] - simplex.verticesY[0];
      const eLen = length(ex, ey);
      const perpX = eLen < 1e-10 ? 0 : -ey / eLen;
      const perpY = eLen < 1e-10 ? 1 : ex / eLen;
      const midX = (simplex.verticesX[0] + simplex.verticesX[1]) * 0.5;
      const midY = (simplex.verticesY[0] + simplex.verticesY[1]) * 0.5;
      const toOriginX = -midX;
      const toOriginY = -midY;
      const dotProd = toOriginX * perpX + toOriginY * perpY;
      const searchX = dotProd >= 0 ? perpX : -perpX;
      const searchY = dotProd >= 0 ? perpY : -perpY;
      const [sx, sy] = minkowskiSupport(
        vertsAX,
        vertsAY,
        countA,
        typeA,
        vertsBX,
        vertsBY,
        countB,
        typeB,
        a,
        b,
        buffers,
        searchX,
        searchY
      );
      if (!simplex.containsVertex(sx, sy)) {
        simplex.addVertex(sx, sy);
      }
    }
    if (simplex.count < 3) continue;
    let abx = simplex.verticesX[1] - simplex.verticesX[0];
    let aby = simplex.verticesY[1] - simplex.verticesY[0];
    let acx = simplex.verticesX[2] - simplex.verticesX[0];
    let acy = simplex.verticesY[2] - simplex.verticesY[0];
    let crossMag = abx * acy - aby * acx;
    if (Math.abs(crossMag) < 1e-10) {
      const perpX = -aby;
      const perpY = abx;
      const perpLen = length(perpX, perpY);
      const searchX = perpLen < 1e-10 ? 1 : perpX / perpLen;
      const searchY = perpLen < 1e-10 ? 0 : perpY / perpLen;
      const [sx, sy] = minkowskiSupport(
        vertsAX,
        vertsAY,
        countA,
        typeA,
        vertsBX,
        vertsBY,
        countB,
        typeB,
        a,
        b,
        buffers,
        searchX,
        searchY
      );
      if (!simplex.containsVertex(sx, sy)) {
        simplex.verticesX[2] = sx;
        simplex.verticesY[2] = sy;
        acx = sx - simplex.verticesX[0];
        acy = sy - simplex.verticesY[0];
        crossMag = abx * acy - aby * acx;
      }
      if (Math.abs(crossMag) < 1e-10) continue;
    }
    const epaResult = epa(
      simplex,
      vertsAX,
      vertsAY,
      countA,
      typeA,
      vertsBX,
      vertsBY,
      countB,
      typeB,
      a,
      b,
      buffers
    );
    if (epaResult && epaResult.penetrationDepth > 1e-6) {
      let nx = -epaResult.contactNormalX;
      let ny = -epaResult.contactNormalY;
      const toBX = buffers.positionX[b] - buffers.positionX[a];
      const toBY = buffers.positionY[b] - buffers.positionY[a];
      if (nx * toBX + ny * toBY < 0) {
        nx = -nx;
        ny = -ny;
      }
      let depth = epaResult.penetrationDepth;
      if (typeA === ShapeType.Box && typeB === ShapeType.Box) {
        const angleA = buffers.angle[a];
        const angleB = buffers.angle[b];
        const dx2 = buffers.positionX[b] - buffers.positionX[a];
        const dy2 = buffers.positionY[b] - buffers.positionY[a];
        const axes = [];
        const cosA = Math.cos(angleA);
        const sinA = Math.sin(angleA);
        axes.push([cosA, sinA]);
        axes.push([-sinA, cosA]);
        const cosB = Math.cos(angleB);
        const sinB = Math.sin(angleB);
        axes.push([cosB, sinB]);
        axes.push([-sinB, cosB]);
        let minOverlap = Infinity;
        let minAxisX = 0;
        let minAxisY = 0;
        let satValid = true;
        for (const [ax, ay] of axes) {
          const rA = Math.abs(buffers.halfExtentX[a] * (ax * cosA + ay * sinA)) + Math.abs(buffers.halfExtentY[a] * (ax * -sinA + ay * cosA));
          const rB = Math.abs(buffers.halfExtentX[b] * (ax * cosB + ay * sinB)) + Math.abs(buffers.halfExtentY[b] * (ax * -sinB + ay * cosB));
          const d = Math.abs(dx2 * ax + dy2 * ay);
          const overlap = rA + rB - d;
          if (overlap <= 0) {
            satValid = false;
            break;
          }
          if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxisX = ax;
            minAxisY = ay;
          }
        }
        if (satValid) {
          depth = minOverlap;
          const sign = dx2 * minAxisX + dy2 * minAxisY >= 0 ? 1 : -1;
          nx = minAxisX * sign;
          ny = minAxisY * sign;
        }
      }
      if (depth > EPA_DEPTH_CAP) {
        depth = EPA_DEPTH_CAP;
      }
      const manifold = buildContactManifold(
        a,
        b,
        buffers,
        vertsAX,
        vertsAY,
        countA,
        typeA,
        vertsBX,
        vertsBY,
        countB,
        typeB,
        depth,
        nx,
        ny
      );
      if (manifold) {
        manifolds.push(manifold);
        continue;
      }
    }
    const dx = buffers.positionX[b] - buffers.positionX[a];
    const dy = buffers.positionY[b] - buffers.positionY[a];
    const len = length(dx, dy);
    const cnx = len < 1e-10 ? 1 : dx / len;
    const cny = len < 1e-10 ? 0 : dy / len;
    let penetration = 0.01;
    if (typeA === ShapeType.Box && typeB === ShapeType.Box) {
      const penX = buffers.halfExtentX[a] + buffers.halfExtentX[b] - Math.abs(dx);
      const penY = buffers.halfExtentY[a] + buffers.halfExtentY[b] - Math.abs(dy);
      if (penX > 0 && penY > 0) {
        penetration = Math.min(penX, penY);
      } else {
        penetration = Math.max(penX, penY, FALLBACK_PENETRATION_MIN);
      }
    } else if (typeA === ShapeType.Circle && typeB === ShapeType.Circle) {
      penetration = buffers.shapeRadius[a] + buffers.shapeRadius[b] - len;
    } else {
      penetration = Math.max(0.01, len * 0.1);
    }
    const contactX = buffers.positionX[a] + cnx * penetration * 0.5;
    const contactY = buffers.positionY[a] + cny * penetration * 0.5;
    const [cA, negSA, sA] = mat2FromAngle(buffers.angle[a]);
    const [cB, negSB, sB] = mat2FromAngle(buffers.angle[b]);
    const [lAx, lAy] = mat2TransposeMulVec(
      cA,
      negSA,
      sA,
      cA,
      contactX - buffers.positionX[a],
      contactY - buffers.positionY[a]
    );
    const [lBx, lBy] = mat2TransposeMulVec(
      cB,
      negSB,
      sB,
      cB,
      contactX - buffers.positionX[b],
      contactY - buffers.positionY[b]
    );
    manifolds.push({
      bodyA: a,
      bodyB: b,
      normal: { x: cnx, y: cny },
      contacts: [{ localA: { x: lAx, y: lAy }, localB: { x: lBx, y: lBy }, penetration, idA: -1, idB: -1 }]
    });
  }
  return manifolds;
}

// src/collision/narrowphase.ts
function narrowphase(buffers, pairs) {
  return gjkNarrowphase(buffers, pairs);
}

// src/collision/pipeline.ts
function collide(buffers, count, treeOrMethod, broadphaseMethod, _narrowphaseMethod) {
  const pairs = broadphase(buffers, count, treeOrMethod, broadphaseMethod);
  if (pairs.length === 0) return [];
  return gjkNarrowphase(buffers, pairs);
}
function narrowphaseDispatch(buffers, pairs, _method = "gjk") {
  if (pairs.length === 0) return [];
  return gjkNarrowphase(buffers, pairs);
}

// src/collision/contact-tracker.ts
var ContactTracker = class {
  constructor() {
    this.callbacks = null;
    this.activePairs = /* @__PURE__ */ new Map();
    this.prevKeys = /* @__PURE__ */ new Set();
    this.currKeys = /* @__PURE__ */ new Set();
  }
  setCallbacks(callbacks) {
    this.callbacks = callbacks;
  }
  /**
   * Update contact state and fire callbacks.
   * Call this after solveVelocity, before storing manifolds for next frame.
   */
  update(manifolds, _config) {
    var _a, _b, _c, _d, _e, _f;
    if (!this.callbacks) return;
    this.prevKeys = new Set(this.currKeys);
    this.currKeys.clear();
    for (let m = 0; m < manifolds.length; m++) {
      const manifold = manifolds[m];
      const key = makePairKey(manifold.bodyA, manifold.bodyB);
      this.currKeys.add(key);
      const totalImpulse = manifold.accumulatedImpulses ? manifold.accumulatedImpulses.reduce((sum, v) => sum + v, 0) : 0;
      const contactPoints = [];
      for (let c = 0; c < manifold.contacts.length; c++) {
        const cp = manifold.contacts[c];
        contactPoints.push({ x: cp.localA.x, y: cp.localA.y });
      }
      const info = {
        bodyA: manifold.bodyA,
        bodyB: manifold.bodyB,
        contactNormal: { x: manifold.normal.x, y: manifold.normal.y },
        contactPoints,
        totalImpulse
      };
      if (!this.prevKeys.has(key)) {
        (_b = (_a = this.callbacks).onBeginContact) == null ? void 0 : _b.call(_a, info);
      } else {
        (_d = (_c = this.callbacks).onStayContact) == null ? void 0 : _d.call(_c, info);
      }
      this.activePairs.set(key, info);
    }
    for (const key of this.prevKeys) {
      if (!this.currKeys.has(key)) {
        const info = this.activePairs.get(key);
        if (info) {
          (_f = (_e = this.callbacks).onEndContact) == null ? void 0 : _f.call(_e, info.bodyA, info.bodyB);
          this.activePairs.delete(key);
        }
      }
    }
  }
  clear() {
    this.activePairs.clear();
    this.prevKeys.clear();
    this.currKeys.clear();
  }
};
function makePairKey(a, b) {
  if (a < b) return a * 73856093 ^ b * 19349663;
  return b * 73856093 ^ a * 19349663;
}
function computeContactSolver(buffers, contact, normalX, normalY, bodyAIdx, bodyBIdx, dt, config) {
  const invMassA = buffers.invMass[bodyAIdx];
  const invMassB = buffers.invMass[bodyBIdx];
  const invInertiaA = buffers.invInertia[bodyAIdx];
  const invInertiaB = buffers.invInertia[bodyBIdx];
  const angleA = buffers.angle[bodyAIdx];
  buffers.angle[bodyBIdx];
  const cosA = Math.cos(angleA);
  const sinA = Math.sin(angleA);
  const cpX = cosA * contact.localA.x - sinA * contact.localA.y + buffers.positionX[bodyAIdx];
  const cpY = sinA * contact.localA.x + cosA * contact.localA.y + buffers.positionY[bodyAIdx];
  const rAX = cpX - buffers.positionX[bodyAIdx];
  const rAY = cpY - buffers.positionY[bodyAIdx];
  const rBX = cpX - buffers.positionX[bodyBIdx];
  const rBY = cpY - buffers.positionY[bodyBIdx];
  const tangentX = -normalY;
  const tangentY = normalX;
  const rnA = rAX * normalY - rAY * normalX;
  const rnB = rBX * normalY - rBY * normalX;
  const normalMass = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB;
  const rtA = rAX * tangentY - rAY * tangentX;
  const rtB = rBX * tangentY - rBY * tangentX;
  const tangentMass = invMassA + invMassB + invInertiaA * rtA * rtA + invInertiaB * rtB * rtB;
  const velAX = buffers.velocityX[bodyAIdx];
  const velAY = buffers.velocityY[bodyAIdx];
  const velBX = buffers.velocityX[bodyBIdx];
  const velBY = buffers.velocityY[bodyBIdx];
  const angVelA = buffers.angularVel[bodyAIdx];
  const angVelB = buffers.angularVel[bodyBIdx];
  const relVelNormalX = velBX + -angVelB * rBY - (velAX + -angVelA * rAY);
  const relVelNormalY = velBY + angVelB * rBX - (velAY + angVelA * rAX);
  const relVelNormal = relVelNormalX * normalX + relVelNormalY * normalY;
  const restitution = config.defaultRestitution;
  let bias = 0;
  if (config.positionCorrection) {
    const penetration = Math.max(contact.penetration - config.penetrationSlop, 0);
    bias = -(config.positionCorrectionBeta / dt) * penetration;
  }
  bias += restitution * Math.min(relVelNormal + config.restitutionSlop, 0);
  return {
    bodyA: bodyAIdx,
    bodyB: bodyBIdx,
    normalX,
    normalY,
    tangentX,
    tangentY,
    rAX,
    rAY,
    rBX,
    rBY,
    normalMass: normalMass > 1e-10 ? 1 / normalMass : 0,
    tangentMass: tangentMass > 1e-10 ? 1 / tangentMass : 0,
    friction: config.defaultFriction,
    restitution,
    bias,
    normalImpulse: 0,
    tangentImpulse: 0,
    pointX: cpX,
    pointY: cpY
  };
}
function solveContact(solver, buffers, normalContact, config) {
  var _a;
  if (solver.normalMass === 0) return;
  const invMassA = buffers.invMass[solver.bodyA];
  const invMassB = buffers.invMass[solver.bodyB];
  const invInertiaA = buffers.invInertia[solver.bodyA];
  const invInertiaB = buffers.invInertia[solver.bodyB];
  const velAX = buffers.velocityX[solver.bodyA];
  const velAY = buffers.velocityY[solver.bodyA];
  const velBX = buffers.velocityX[solver.bodyB];
  const velBY = buffers.velocityY[solver.bodyB];
  const angVelA = buffers.angularVel[solver.bodyA];
  const angVelB = buffers.angularVel[solver.bodyB];
  const relVelNX = velBX + -angVelB * solver.rBY - (velAX + -angVelA * solver.rAY);
  const relVelNY = velBY + angVelB * solver.rBX - (velAY + angVelA * solver.rAX);
  const relVelN = relVelNX * solver.normalX + relVelNY * solver.normalY;
  const lambda = solver.normalMass * -(relVelN + solver.bias);
  const oldImpulseN = solver.normalImpulse;
  if (config == null ? void 0 : config.impulseAccumulation) {
    solver.normalImpulse = Math.max(0, solver.normalImpulse + lambda);
  } else {
    solver.normalImpulse = Math.max(0, lambda);
  }
  const dLambdaN = (config == null ? void 0 : config.impulseAccumulation) ? solver.normalImpulse - oldImpulseN : solver.normalImpulse;
  const impX = dLambdaN * solver.normalX;
  const impY = dLambdaN * solver.normalY;
  buffers.velocityX[solver.bodyA] -= impX * invMassA;
  buffers.velocityY[solver.bodyA] -= impY * invMassA;
  buffers.angularVel[solver.bodyA] -= (solver.rAX * impY - solver.rAY * impX) * invInertiaA;
  buffers.velocityX[solver.bodyB] += impX * invMassB;
  buffers.velocityY[solver.bodyB] += impY * invMassB;
  buffers.angularVel[solver.bodyB] += (solver.rBX * impY - solver.rBY * impX) * invInertiaB;
  const newVelAX = buffers.velocityX[solver.bodyA];
  const newVelAY = buffers.velocityY[solver.bodyA];
  const newVelBX = buffers.velocityX[solver.bodyB];
  const newVelBY = buffers.velocityY[solver.bodyB];
  const newAngVelA = buffers.angularVel[solver.bodyA];
  const newAngVelB = buffers.angularVel[solver.bodyB];
  const relVelTX = newVelBX + -newAngVelB * solver.rBY - (newVelAX + -newAngVelA * solver.rAY);
  const relVelTY = newVelBY + newAngVelB * solver.rBX - (newVelAY + newAngVelA * solver.rAX);
  const relVelT = relVelTX * solver.tangentX + relVelTY * solver.tangentY;
  const maxFriction = solver.friction * ((_a = void 0 ) != null ? _a : solver.normalImpulse);
  const lambdaT = solver.tangentMass * -relVelT;
  const oldImpulseT = solver.tangentImpulse;
  if (config == null ? void 0 : config.impulseAccumulation) {
    solver.tangentImpulse = Math.max(-maxFriction, Math.min(maxFriction, solver.tangentImpulse + lambdaT));
  } else {
    solver.tangentImpulse = Math.max(-maxFriction, Math.min(maxFriction, lambdaT));
  }
  const dLambdaT = (config == null ? void 0 : config.impulseAccumulation) ? solver.tangentImpulse - oldImpulseT : solver.tangentImpulse;
  const impTX = dLambdaT * solver.tangentX;
  const impTY = dLambdaT * solver.tangentY;
  buffers.velocityX[solver.bodyA] -= impTX * invMassA;
  buffers.velocityY[solver.bodyA] -= impTY * invMassA;
  buffers.angularVel[solver.bodyA] -= (solver.rAX * impTY - solver.rAY * impTX) * invInertiaA;
  buffers.velocityX[solver.bodyB] += impTX * invMassB;
  buffers.velocityY[solver.bodyB] += impTY * invMassB;
  buffers.angularVel[solver.bodyB] += (solver.rBX * impTY - solver.rBY * impTX) * invInertiaB;
}
function solveBlock(block, buffers) {
  const { solver1: s1, solver2: s2, k, b } = block;
  const velAX = buffers.velocityX[s1.bodyA];
  const velAY = buffers.velocityY[s1.bodyA];
  const velBX = buffers.velocityX[s1.bodyB];
  const velBY = buffers.velocityY[s1.bodyB];
  const angVelA = buffers.angularVel[s1.bodyA];
  const angVelB = buffers.angularVel[s1.bodyB];
  const rv1NX = velBX + -angVelB * s1.rBY - (velAX + -angVelA * s1.rAY);
  const rv1NY = velBY + angVelB * s1.rBX - (velAY + angVelA * s1.rAX);
  const relVelN1 = rv1NX * s1.normalX + rv1NY * s1.normalY;
  const rv2NX = velBX + -angVelB * s2.rBY - (velAX + -angVelA * s2.rAY);
  const rv2NY = velBY + angVelB * s2.rBX - (velAY + angVelA * s2.rAX);
  const relVelN2 = rv2NX * s2.normalX + rv2NY * s2.normalY;
  const b0 = relVelN1 + b[0];
  const b1 = relVelN2 + b[1];
  const aX = s1.normalImpulse;
  const aY = s2.normalImpulse;
  const bPrimeX = b0 - (k[0] * aX + k[1] * aY);
  const bPrimeY = b1 - (k[2] * aX + k[3] * aY);
  const det = k[0] * k[3] - k[1] * k[2];
  if (Math.abs(det) > 1e-12) {
    const x1 = (-bPrimeX * k[3] + bPrimeY * k[1]) / det;
    const x2 = (bPrimeX * k[2] - bPrimeY * k[0]) / det;
    if (x1 >= 0 && x2 >= 0) {
      applyBlockImpulses(s1, s2, x1, x2, buffers);
      return;
    }
  }
  if (k[0] > 1e-12) {
    const x1 = -bPrimeX / k[0];
    if (x1 >= 0) {
      const lambda2 = k[2] * x1 + bPrimeY;
      if (lambda2 <= 0) {
        applyBlockImpulses(s1, s2, x1, 0, buffers);
        return;
      }
    }
  }
  if (k[3] > 1e-12) {
    const x2 = -bPrimeY / k[3];
    if (x2 >= 0) {
      const lambda1 = k[1] * x2 + bPrimeX;
      if (lambda1 <= 0) {
        applyBlockImpulses(s1, s2, 0, x2, buffers);
        return;
      }
    }
  }
  applyBlockImpulses(s1, s2, 0, 0, buffers);
}
function applyBlockImpulses(s1, s2, lambda1, lambda2, buffers) {
  const bodyA = s1.bodyA;
  const bodyB = s1.bodyB;
  const invMassA = buffers.invMass[bodyA];
  const invMassB = buffers.invMass[bodyB];
  const invInertiaA = buffers.invInertia[bodyA];
  const invInertiaB = buffers.invInertia[bodyB];
  const impX = lambda1 * s1.normalX + lambda2 * s2.normalX;
  const impY = lambda1 * s1.normalY + lambda2 * s2.normalY;
  buffers.velocityX[bodyA] -= impX * invMassA;
  buffers.velocityY[bodyA] -= impY * invMassA;
  buffers.angularVel[bodyA] -= (lambda1 * (s1.rAX * s1.normalY - s1.rAY * s1.normalX) + lambda2 * (s2.rAX * s2.normalY - s2.rAY * s2.normalX)) * invInertiaA;
  buffers.velocityX[bodyB] += impX * invMassB;
  buffers.velocityY[bodyB] += impY * invMassB;
  buffers.angularVel[bodyB] += (lambda1 * (s1.rBX * s1.normalY - s1.rBY * s1.normalX) + lambda2 * (s2.rBX * s2.normalY - s2.rBY * s2.normalX)) * invInertiaB;
  s1.normalImpulse = lambda1;
  s2.normalImpulse = lambda2;
}
function buildBlockSolver(manifold, buffers, dt, config) {
  if (manifold.contacts.length !== 2) return null;
  const bodyAIdx = manifold.bodyA;
  const bodyBIdx = manifold.bodyB;
  const nx = manifold.normal.x;
  const ny = manifold.normal.y;
  const s1 = computeContactSolver(buffers, manifold.contacts[0], nx, ny, bodyAIdx, bodyBIdx, dt, config);
  const s2 = computeContactSolver(buffers, manifold.contacts[1], nx, ny, bodyAIdx, bodyBIdx, dt, config);
  const rn1A = s1.rAX * ny - s1.rAY * nx;
  const rn1B = s1.rBX * ny - s1.rBY * nx;
  const rn2A = s2.rAX * ny - s2.rAY * nx;
  const rn2B = s2.rBX * ny - s2.rBY * nx;
  const invMassA = buffers.invMass[bodyAIdx];
  const invMassB = buffers.invMass[bodyBIdx];
  const invInertiaA = buffers.invInertia[bodyAIdx];
  const invInertiaB = buffers.invInertia[bodyBIdx];
  const k00 = invMassA + invMassB + invInertiaA * rn1A * rn1A + invInertiaB * rn1B * rn1B;
  const k01 = invMassA + invMassB + invInertiaA * rn1A * rn2A + invInertiaB * rn1B * rn2B;
  const k10 = invMassA + invMassB + invInertiaA * rn2A * rn1A + invInertiaB * rn2B * rn1B;
  const k11 = invMassA + invMassB + invInertiaA * rn2A * rn2A + invInertiaB * rn2B * rn2B;
  const b0 = -(config.baumgarteFactor / dt) * Math.max(manifold.contacts[0].penetration - config.penetrationSlop, 0);
  const b1 = -(config.baumgarteFactor / dt) * Math.max(manifold.contacts[1].penetration - config.penetrationSlop, 0);
  return {
    solver1: s1,
    solver2: s2,
    k: [k00, k01, k10, k11],
    b: [b0, b1],
    impulses: [0, 0]
  };
}
function solvePositionConstraint(solver, buffers, config, penetration) {
  if (solver.normalMass === 0) return;
  const invMassA = buffers.invMass[solver.bodyA];
  const invMassB = buffers.invMass[solver.bodyB];
  const invInertiaA = buffers.invInertia[solver.bodyA];
  const invInertiaB = buffers.invInertia[solver.bodyB];
  const slop = config.penetrationSlop;
  const bias = Math.max(penetration - slop, 0);
  if (bias <= 0) return;
  const rAX = solver.rAX;
  const rAY = solver.rAY;
  const rBX = solver.rBX;
  const rBY = solver.rBY;
  const nx = solver.normalX;
  const ny = solver.normalY;
  const rnA = rAX * ny - rAY * nx;
  const rnB = rBX * ny - rBY * nx;
  const effectiveMass = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB;
  if (effectiveMass < 1e-12) return;
  const lambda = config.baumgarteFactor * bias / effectiveMass;
  const impX = lambda * nx;
  const impY = lambda * ny;
  buffers.positionX[solver.bodyA] -= impX * invMassA;
  buffers.positionY[solver.bodyA] -= impY * invMassA;
  buffers.angle[solver.bodyA] -= (rAX * impY - rAY * impX) * invInertiaA;
  buffers.positionX[solver.bodyB] += impX * invMassB;
  buffers.positionY[solver.bodyB] += impY * invMassB;
  buffers.angle[solver.bodyB] += (rBX * impY - rBY * impX) * invInertiaB;
}
function solveVelocity(buffers, manifolds, config) {
  const dt = config.fixedTimestep;
  const iterations = config.velocityIterations;
  const contactSolvers = [];
  const blockSolvers = [];
  for (let m = 0; m < manifolds.length; m++) {
    const manifold = manifolds[m];
    const bodyAIdx = manifold.bodyA;
    const bodyBIdx = manifold.bodyB;
    if ((buffers.flags[bodyAIdx] & BodyFlags.SLEEPING) !== 0 && (buffers.flags[bodyBIdx] & BodyFlags.SLEEPING) !== 0) continue;
    const solvers = [];
    for (let c = 0; c < manifold.contacts.length; c++) {
      const solver = computeContactSolver(
        buffers,
        manifold.contacts[c],
        manifold.normal.x,
        manifold.normal.y,
        bodyAIdx,
        bodyBIdx,
        dt,
        config
      );
      solvers.push(solver);
    }
    contactSolvers.push(solvers);
    if (config.blockSolver && manifold.contacts.length === 2) {
      blockSolvers.push(buildBlockSolver(manifold, buffers, dt, config));
    } else {
      blockSolvers.push(null);
    }
  }
  for (let iter = 0; iter < iterations; iter++) {
    for (let m = 0; m < contactSolvers.length; m++) {
      const solvers = contactSolvers[m];
      const block = blockSolvers[m];
      if (block) {
        solveBlock(block, buffers);
      } else {
        for (let c = 0; c < solvers.length; c++) {
          solveContact(solvers[c], buffers, void 0, config);
        }
      }
    }
  }
}
function solvePosition(buffers, manifolds, config) {
  const iterations = config.positionIterations;
  for (let iter = 0; iter < iterations; iter++) {
    for (let m = 0; m < manifolds.length; m++) {
      const manifold = manifolds[m];
      const bodyAIdx = manifold.bodyA;
      const bodyBIdx = manifold.bodyB;
      if ((buffers.flags[bodyAIdx] & BodyFlags.STATIC) !== 0 && (buffers.flags[bodyBIdx] & BodyFlags.STATIC) !== 0) continue;
      for (let c = 0; c < manifold.contacts.length; c++) {
        const solver = computeContactSolver(
          buffers,
          manifold.contacts[c],
          manifold.normal.x,
          manifold.normal.y,
          bodyAIdx,
          bodyBIdx,
          config.fixedTimestep,
          config
        );
        solvePositionConstraint(solver, buffers, config, manifold.contacts[c].penetration);
      }
    }
  }
}
function integrate(buffers, count, dt, gravity) {
  for (let i = 0; i < count; i++) {
    if ((buffers.flags[i] & BodyFlags.ACTIVE) === 0) continue;
    if ((buffers.flags[i] & BodyFlags.STATIC) !== 0) continue;
    if ((buffers.flags[i] & BodyFlags.SLEEPING) !== 0) continue;
    const invMass = buffers.invMass[i];
    if (invMass === 0) continue;
    const ax = gravity.x;
    const ay = gravity.y;
    buffers.velocityX[i] += ax * dt;
    buffers.velocityY[i] += ay * dt;
    buffers.positionX[i] += buffers.velocityX[i] * dt;
    buffers.positionY[i] += buffers.velocityY[i] * dt;
    buffers.angle[i] += buffers.angularVel[i] * dt;
  }
}
async function loadEmscriptenModule() {
  const mod = await import(
    /* webpackIgnore: true */
    './box2d-7IVUWJOQ.js'
  );
  return mod.default();
}
function wrapCwrap(Module, name, returnType, argTypes) {
  return Module.cwrap(name, returnType, argTypes);
}
async function loadWasmModule() {
  const Module = await loadEmscriptenModule();
  const b2Init = wrapCwrap(Module, "b2_init", "number", ["number", "number", "number"]);
  const b2Destroy = wrapCwrap(Module, "b2_destroy", null, ["number"]);
  const b2SyncBodies = wrapCwrap(Module, "b2_sync_bodies", null, [
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number"
  ]);
  const b2SyncShapes = wrapCwrap(Module, "b2_sync_shapes", null, [
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number"
  ]);
  const b2Step = wrapCwrap(Module, "b2_step", null, ["number", "number", "number"]);
  const b2ReadBodies = wrapCwrap(Module, "b2_read_bodies", null, [
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number"
  ]);
  const b2GetContactCount = wrapCwrap(Module, "b2_get_contact_count", "number", ["number"]);
  const b2GetContacts = wrapCwrap(Module, "b2_get_contacts", null, [
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number",
    "number"
  ]);
  const persistentAllocs = [];
  function allocAndCopy(source, count) {
    const byteSize = count * source.BYTES_PER_ELEMENT;
    const ptr = Module._malloc(byteSize);
    if (ptr === 0) {
      throw new Error("WASM: malloc returned NULL \u2014 out of memory");
    }
    let heapView;
    if (source instanceof Float32Array) {
      heapView = new Float32Array(Module.HEAPU8.buffer, ptr, count);
    } else if (source instanceof Int32Array) {
      heapView = new Int32Array(Module.HEAPU8.buffer, ptr, count);
    } else {
      heapView = new Uint8Array(Module.HEAPU8.buffer, ptr, count);
    }
    heapView.set(source.subarray(0, count));
    return ptr;
  }
  function readFromHeap(dest, heapPtr, count) {
    if (dest instanceof Float32Array) {
      const heapView = new Float32Array(Module.HEAPU8.buffer, heapPtr, count);
      dest.set(heapView.subarray(0, count));
    } else if (dest instanceof Int32Array) {
      const heapView = new Int32Array(Module.HEAPU8.buffer, heapPtr, count);
      dest.set(heapView.subarray(0, count));
    }
  }
  function freeMany(ptrs) {
    for (const ptr of ptrs) {
      Module._free(ptr);
    }
  }
  const instance = {
    init(gravityX, gravityY, maxBodies) {
      return b2Init(gravityX, gravityY, maxBodies);
    },
    destroy(worldHandle) {
      b2Destroy(worldHandle);
    },
    syncBodies(worldHandle, buffers, count) {
      const ptrPosX = allocAndCopy(buffers.positionX, count);
      const ptrPosY = allocAndCopy(buffers.positionY, count);
      const ptrVelX = allocAndCopy(buffers.velocityX, count);
      const ptrVelY = allocAndCopy(buffers.velocityY, count);
      const ptrAngle = allocAndCopy(buffers.angle, count);
      const ptrAngVel = allocAndCopy(buffers.angularVel, count);
      const ptrMass = allocAndCopy(buffers.mass, count);
      const ptrInvMass = allocAndCopy(buffers.invMass, count);
      const ptrInertia = allocAndCopy(buffers.inertia, count);
      const ptrInvInertia = allocAndCopy(buffers.invInertia, count);
      const ptrFlags = allocAndCopy(buffers.flags, count);
      b2SyncBodies(
        worldHandle,
        ptrPosX,
        ptrPosY,
        ptrVelX,
        ptrVelY,
        ptrAngle,
        ptrAngVel,
        ptrMass,
        ptrInvMass,
        ptrInertia,
        ptrInvInertia,
        ptrFlags,
        count
      );
      freeMany([ptrPosX, ptrPosY, ptrVelX, ptrVelY, ptrAngle, ptrAngVel, ptrMass, ptrInvMass, ptrInertia, ptrInvInertia, ptrFlags]);
    },
    syncShapes(worldHandle, buffers, count) {
      const ptrPosX = allocAndCopy(buffers.positionX, count);
      const ptrPosY = allocAndCopy(buffers.positionY, count);
      const ptrAngle = allocAndCopy(buffers.angle, count);
      const ptrHalfExtentX = allocAndCopy(buffers.halfExtentX, count);
      const ptrHalfExtentY = allocAndCopy(buffers.halfExtentY, count);
      const ptrShapeType = allocAndCopy(buffers.shapeType, count);
      const ptrShapeRadius = allocAndCopy(buffers.shapeRadius, count);
      const ptrShapeVertexCount = allocAndCopy(buffers.shapeVertexCount, count);
      const vertexCount = count * MAX_VERTICES_PER_SHAPE;
      const ptrVerticesX = allocAndCopy(buffers.shapeVerticesX, vertexCount);
      const ptrVerticesY = allocAndCopy(buffers.shapeVerticesY, vertexCount);
      b2SyncShapes(
        worldHandle,
        ptrPosX,
        ptrPosY,
        ptrAngle,
        ptrHalfExtentX,
        ptrHalfExtentY,
        ptrShapeType,
        ptrShapeRadius,
        ptrShapeVertexCount,
        ptrVerticesX,
        ptrVerticesY,
        count
      );
      freeMany([ptrPosX, ptrPosY, ptrAngle, ptrHalfExtentX, ptrHalfExtentY, ptrShapeType, ptrShapeRadius, ptrShapeVertexCount, ptrVerticesX, ptrVerticesY]);
    },
    step(worldHandle, dt, subSteps) {
      b2Step(worldHandle, dt, subSteps);
    },
    readBodies(worldHandle, buffers, count) {
      const ptrPosX = allocAndCopy(new Float32Array(count), count);
      const ptrPosY = allocAndCopy(new Float32Array(count), count);
      const ptrVelX = allocAndCopy(new Float32Array(count), count);
      const ptrVelY = allocAndCopy(new Float32Array(count), count);
      const ptrAngle = allocAndCopy(new Float32Array(count), count);
      const ptrAngVel = allocAndCopy(new Float32Array(count), count);
      b2ReadBodies(
        worldHandle,
        ptrPosX,
        ptrPosY,
        ptrVelX,
        ptrVelY,
        ptrAngle,
        ptrAngVel,
        count
      );
      readFromHeap(buffers.positionX, ptrPosX, count);
      readFromHeap(buffers.positionY, ptrPosY, count);
      readFromHeap(buffers.velocityX, ptrVelX, count);
      readFromHeap(buffers.velocityY, ptrVelY, count);
      readFromHeap(buffers.angle, ptrAngle, count);
      readFromHeap(buffers.angularVel, ptrAngVel, count);
      freeMany([ptrPosX, ptrPosY, ptrVelX, ptrVelY, ptrAngle, ptrAngVel]);
    },
    getContacts(worldHandle) {
      const contactCount = b2GetContactCount(worldHandle);
      if (contactCount === 0) return [];
      const ptrBodyA = allocAndCopy(new Int32Array(contactCount), contactCount);
      const ptrBodyB = allocAndCopy(new Int32Array(contactCount), contactCount);
      const ptrNx = allocAndCopy(new Float32Array(contactCount), contactCount);
      const ptrNy = allocAndCopy(new Float32Array(contactCount), contactCount);
      const ptrPx = allocAndCopy(new Float32Array(contactCount), contactCount);
      const ptrPy = allocAndCopy(new Float32Array(contactCount), contactCount);
      const ptrPen = allocAndCopy(new Float32Array(contactCount), contactCount);
      b2GetContacts(
        worldHandle,
        ptrBodyA,
        ptrBodyB,
        ptrNx,
        ptrNy,
        ptrPx,
        ptrPy,
        ptrPen,
        contactCount
      );
      const heapBodyA = new Int32Array(Module.HEAPU8.buffer, ptrBodyA, contactCount);
      const heapBodyB = new Int32Array(Module.HEAPU8.buffer, ptrBodyB, contactCount);
      const heapNx = new Float32Array(Module.HEAPU8.buffer, ptrNx, contactCount);
      const heapNy = new Float32Array(Module.HEAPU8.buffer, ptrNy, contactCount);
      const heapPx = new Float32Array(Module.HEAPU8.buffer, ptrPx, contactCount);
      const heapPy = new Float32Array(Module.HEAPU8.buffer, ptrPy, contactCount);
      const heapPen = new Float32Array(Module.HEAPU8.buffer, ptrPen, contactCount);
      const manifolds = [];
      for (let i = 0; i < contactCount; i++) {
        manifolds.push({
          bodyA: heapBodyA[i],
          bodyB: heapBodyB[i],
          normal: { x: heapNx[i], y: heapNy[i] },
          contacts: [{
            localA: { x: 0, y: 0 },
            localB: { x: heapPx[i], y: heapPy[i] },
            penetration: heapPen[i]
          }]
        });
      }
      freeMany([ptrBodyA, ptrBodyB, ptrNx, ptrNy, ptrPx, ptrPy, ptrPen]);
      return manifolds;
    },
    dispose() {
      for (const ptr of persistentAllocs) {
        Module._free(ptr);
      }
      persistentAllocs.length = 0;
    }
  };
  return instance;
}

// src/wasm/WasmPhysicsBackend.ts
var WasmPhysicsBackend = class {
  constructor() {
    this.wasm = null;
    this.worldHandle = 0;
    this.initialized = false;
    this.dt = 1 / 60;
    this.subSteps = 4;
  }
  /**
   * Initialize the WASM backend.
   *
   * Loads the WASM module, creates a Box2D world with the given gravity,
   * and allocates internal data structures for up to maxBodies bodies.
   * 
   * @param config - World configuration including fixedTimestep and other settings
   * @param subSteps - Number of sub-steps per simulation step (default: 4)
   */
  async init(config, subSteps) {
    var _a;
    if (this.initialized) return;
    this.wasm = await loadWasmModule();
    const maxBodies = (_a = config.maxBodies) != null ? _a : 8192;
    this.worldHandle = this.wasm.init(
      config.gravity.x,
      config.gravity.y,
      maxBodies
    );
    if (this.worldHandle < 0) {
      throw new Error(
        `WASM backend: b2_init failed (handle=${this.worldHandle}). Check that the WASM module was built correctly.`
      );
    }
    this.dt = config.fixedTimestep;
    this.subSteps = subSteps != null ? subSteps : 4;
    this.initialized = true;
  }
  /**
   * Run broadphase collision detection.
   *
   * In WASM mode, broadphase is handled internally by Box2D during
   * b2_step(). This method returns an empty array — use collide()
   * instead for the full pipeline.
   */
  broadphase(_buffers, _count, _method) {
    return [];
  }
  /**
   * Run narrowphase collision detection on candidate pairs.
   *
   * In WASM mode, narrowphase is handled internally by Box2D during
   * b2_step(). This method returns an empty array — use collide()
   * instead for the full pipeline.
   */
  narrowphase(_buffers, _pairs, _method) {
    return [];
  }
  /**
   * Run the full collision pipeline.
   *
   * This is the primary entry point for the WASM backend. It:
   * 1. Syncs body data (position, velocity, mass, etc.) to WASM
   * 2. Syncs shape data (boxes, circles, polygons) to WASM
   * 3. Advances the simulation by one time step
   * 4. Reads updated body state back from WASM
   * 5. Extracts contact manifolds from the simulation
   *
   * The method and narrowMethod parameters are ignored in WASM mode
   * since Box2D uses its own internal algorithms.
   */
  collide(buffers, count, _broadMethod, _narrowMethod) {
    if (!this.initialized || !this.wasm) {
      throw new Error(
        "WASM backend not initialized. Call init() before collide()."
      );
    }
    this.wasm.syncBodies(this.worldHandle, buffers, count);
    this.wasm.syncShapes(this.worldHandle, buffers, count);
    this.wasm.step(this.worldHandle, this.dt, this.subSteps);
    this.wasm.readBodies(this.worldHandle, buffers, count);
    return this.wasm.getContacts(this.worldHandle);
  }
  /**
   * Solve velocity constraints.
   *
   * In WASM mode, velocity solving is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  solveVelocity(_buffers, _manifolds, _config) {
  }
  /**
   * Integrate positions from velocities.
   *
   * In WASM mode, integration is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  integrate(_buffers, _count, _dt, _gravity) {
  }
  /**
   * Solve position constraints (penetration correction).
   *
   * In WASM mode, position solving is handled internally by Box2D
   * during b2_step(). This is a no-op.
   */
  solvePosition(_buffers, _manifolds, _config) {
  }
  /**
   * Clean up WASM resources.
   *
   * Destroys the Box2D world and frees all WASM-allocated memory.
   */
  dispose() {
    if (this.wasm) {
      if (this.worldHandle >= 0) {
        this.wasm.destroy(this.worldHandle);
      }
      this.wasm.dispose();
      this.wasm = null;
    }
    this.initialized = false;
    this.worldHandle = 0;
  }
};

export { ContactTracker, DynamicTree, Polytope, Simplex, WasmPhysicsBackend, aabbArea, aabbContains, aabbMerge, aabbOverlap, aabbPerimeter, broadphase, broadphaseBVH, collide, computeBodyAABB, cross, dot, getShapeHandler, gjkNarrowphase, integrate, length, lengthSq, loadWasmModule, mat2FromAngle, mat2MulVec, mat2TransposeMulVec, narrowphase, narrowphaseDispatch, registerShapeHandler, solvePosition, solveVelocity, vec2Add, vec2Cross, vec2DistanceSq, vec2Dot, vec2Length, vec2LengthSq, vec2Normalize, vec2Scale, vec2Set, vec2Sub };
//# sourceMappingURL=index.js.map
//# sourceMappingURL=index.js.map