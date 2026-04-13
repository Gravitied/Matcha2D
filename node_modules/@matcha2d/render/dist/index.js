// src/canvas2d.ts
var BodyFlags = {
  ACTIVE: 1,
  STATIC: 2,
  SLEEPING: 4,
  SENSOR: 8
};
var Canvas2DRenderer = class {
  constructor(_ctx, options) {
    this._ctx = _ctx;
    this._scale = 50;
    // pixels per meter
    this._offsetX = 0;
    this._offsetY = 0;
    if (options == null ? void 0 : options.scale) this._scale = options.scale;
    if (options == null ? void 0 : options.offsetX) this._offsetX = options.offsetX;
    if (options == null ? void 0 : options.offsetY) this._offsetY = options.offsetY;
  }
  /** Set the camera scale (pixels per meter). */
  setScale(scale) {
    this._scale = scale;
  }
  /** Set the camera offset in pixels. */
  setOffset(x, y) {
    this._offsetX = x;
    this._offsetY = y;
  }
  begin() {
    this._ctx.clearRect(0, 0, this._ctx.canvas.width, this._ctx.canvas.height);
    this._ctx.save();
    this._ctx.translate(this._offsetX, this._ctx.canvas.height + this._offsetY);
    this._ctx.scale(this._scale, -this._scale);
  }
  drawBodies(buffers, count, _alpha) {
    const {
      positionX,
      positionY,
      angle,
      shapeType,
      shapeRadius,
      halfExtentX,
      halfExtentY,
      shapeVertexCount,
      shapeVerticesX,
      shapeVerticesY,
      flags
    } = buffers;
    for (let i = 0; i < count; i++) {
      if (!(flags[i] & BodyFlags.ACTIVE)) continue;
      const px = positionX[i];
      const py = positionY[i];
      const rot = angle[i];
      const shape = shapeType[i];
      const isStatic = (flags[i] & BodyFlags.STATIC) !== 0;
      const isSleeping = (flags[i] & BodyFlags.SLEEPING) !== 0;
      const isSensor = (flags[i] & BodyFlags.SENSOR) !== 0;
      this._ctx.save();
      this._ctx.translate(px, py);
      this._ctx.rotate(rot);
      if (isSensor) {
        this._ctx.globalAlpha = 0.4;
      }
      switch (shape) {
        case 0:
          this._drawBox(halfExtentX[i], halfExtentY[i], isStatic, isSleeping);
          break;
        case 1:
          this._drawCircle(shapeRadius[i], isStatic, isSleeping);
          break;
        case 2:
          this._drawPolygon(i, shapeVertexCount[i], shapeVerticesX, shapeVerticesY, isStatic, isSleeping);
          break;
      }
      this._ctx.restore();
    }
  }
  end() {
    this._ctx.restore();
  }
  /**
   * Draw narrowphase-accurate collider outlines in **world meters** (Y-up).
   * Call between `begin()` and `end()` so the same transform as `drawBodies` applies.
   */
  drawNarrowphaseColliderOutlinesMeters(outlines, options) {
    var _a, _b, _c;
    const ctx = this._ctx;
    const lw = ((_a = options == null ? void 0 : options.lineWidth) != null ? _a : 2) / this._scale;
    const strokeSolid = (_b = options == null ? void 0 : options.strokeSolid) != null ? _b : "rgba(0, 255, 140, 0.9)";
    const strokeSensor = (_c = options == null ? void 0 : options.strokeSensor) != null ? _c : "rgba(255, 210, 60, 0.85)";
    for (const o of outlines) {
      if (o.vertices.length === 0) continue;
      ctx.strokeStyle = o.sensor ? strokeSensor : strokeSolid;
      ctx.lineWidth = lw;
      ctx.beginPath();
      for (let v = 0; v < o.vertices.length; v++) {
        const p = o.vertices[v];
        if (v === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.closePath();
      ctx.stroke();
    }
  }
  destroy() {
  }
  _drawBox(hx, hy, isStatic, isSleeping) {
    const ctx = this._ctx;
    ctx.strokeStyle = isStatic ? "#444" : isSleeping ? "#888" : "#2196F3";
    ctx.lineWidth = 2 / this._scale;
    ctx.fillStyle = isStatic ? "#666" : isSleeping ? "#aaa" : "rgba(33, 150, 243, 0.3)";
    ctx.beginPath();
    ctx.rect(-hx, -hy, hx * 2, hy * 2);
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = isStatic ? "#888" : "#FF5722";
    ctx.lineWidth = 1.5 / this._scale;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(hx * 0.8, 0);
    ctx.stroke();
  }
  _drawCircle(radius, isStatic, isSleeping) {
    const ctx = this._ctx;
    ctx.strokeStyle = isStatic ? "#444" : isSleeping ? "#888" : "#4CAF50";
    ctx.lineWidth = 2 / this._scale;
    ctx.fillStyle = isStatic ? "#666" : isSleeping ? "#aaa" : "rgba(76, 175, 80, 0.3)";
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = isStatic ? "#888" : "#FF5722";
    ctx.lineWidth = 1.5 / this._scale;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(radius * 0.8, 0);
    ctx.stroke();
  }
  _drawPolygon(bodyIdx, vertexCount, verticesX, verticesY, isStatic, isSleeping) {
    if (vertexCount === 0) return;
    const ctx = this._ctx;
    const stride = 16;
    const base = bodyIdx * stride;
    ctx.strokeStyle = isStatic ? "#444" : isSleeping ? "#888" : "#9C27B0";
    ctx.lineWidth = 2 / this._scale;
    ctx.fillStyle = isStatic ? "#666" : isSleeping ? "#aaa" : "rgba(156, 39, 176, 0.3)";
    ctx.beginPath();
    for (let v = 0; v < vertexCount; v++) {
      const vx = verticesX[base + v];
      const vy = verticesY[base + v];
      if (v === 0) {
        ctx.moveTo(vx, vy);
      } else {
        ctx.lineTo(vx, vy);
      }
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    const firstVx = verticesX[base];
    const firstVy = verticesY[base];
    const len = Math.sqrt(firstVx * firstVx + firstVy * firstVy);
    if (len > 1e-3) {
      ctx.strokeStyle = isStatic ? "#888" : "#FF5722";
      ctx.lineWidth = 1.5 / this._scale;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(firstVx * 0.8, firstVy * 0.8);
      ctx.stroke();
    }
  }
};

// src/narrowphaseOverlay.ts
function drawNarrowphaseColliderOutlinesCanvas(ctx, outlines, opts) {
  var _a, _b, _c;
  const ppm = opts.pixelsPerMeter;
  const H = opts.canvasHeight;
  const physicsYUp = opts.physicsYUp !== false;
  const strokeSolid = (_a = opts.strokeSolid) != null ? _a : "rgba(0, 255, 140, 0.95)";
  const strokeSensor = (_b = opts.strokeSensor) != null ? _b : "rgba(255, 210, 60, 0.9)";
  const lw = (_c = opts.lineWidth) != null ? _c : 1.75;
  const toCx = (x) => x * ppm;
  const toCy = (y) => physicsYUp ? H - y * ppm : y * ppm;
  for (const o of outlines) {
    if (o.vertices.length === 0) continue;
    ctx.strokeStyle = o.sensor ? strokeSensor : strokeSolid;
    ctx.lineWidth = lw;
    ctx.beginPath();
    for (let v = 0; v < o.vertices.length; v++) {
      const wx = toCx(o.vertices[v].x);
      const wy = toCy(o.vertices[v].y);
      if (v === 0) ctx.moveTo(wx, wy);
      else ctx.lineTo(wx, wy);
    }
    ctx.closePath();
    ctx.stroke();
  }
}

export { Canvas2DRenderer, drawNarrowphaseColliderOutlinesCanvas };
//# sourceMappingURL=index.js.map
//# sourceMappingURL=index.js.map