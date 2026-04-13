// src/serialization.ts
function serializeBuffers(_buffers, _count) {
  return new ArrayBuffer(0);
}
function deserializeBuffers(_data) {
  return null;
}

// src/profiler.ts
var Profiler = class {
  constructor() {
    this._marks = /* @__PURE__ */ new Map();
  }
  begin(label) {
    this._marks.set(label, performance.now());
  }
  end(label) {
    const start = this._marks.get(label);
    if (start === void 0) return 0;
    const elapsed = performance.now() - start;
    this._marks.delete(label);
    return elapsed;
  }
};

export { Profiler, deserializeBuffers, serializeBuffers };
//# sourceMappingURL=index.js.map
//# sourceMappingURL=index.js.map