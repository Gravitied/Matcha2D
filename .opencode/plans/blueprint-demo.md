# Plan: Rapier Blueprint Demo Window

## Goal
Add a separate demo page running the Rapier physics engine (`@dimforge/rapier2d-compat`) so you can visually compare physics behavior between Matcha2D and the reference Rapier implementation side-by-side.

## Approach
Use `@dimforge/rapier2d-compat` (WASM embedded in base64, no bundler needed) via CDN. This avoids building Rapier from source and works directly with the existing static file server.

## Files to Create

### 1. `demo/blueprint.html` — Rapier (Blueprint) Collision Demo
A mirror of `demo/collision.html` but using the Rapier engine. Same controls, same scenarios, same canvas size.

**Key differences from collision.html:**
- Imports `@dimforge/rapier2d-compat` from CDN (Skypack/unpkg)
- Uses `RAPIER.World`, `RAPIER.RigidBodyDesc`, `RAPIER.ColliderDesc` API
- Y-axis flip: Rapier uses Y-up physics coords, canvas uses Y-down — flip on render
- Wall collisions: Rapier uses static rigid bodies for walls (not JS boundary checks)
- Shapes: `ball()` for circles, `cuboid()` for boxes, `convexHull()` for polygons
- Stats show "Rapier/Rust-WASM" label

**Rapier API mapping:**
```
Matcha2D                         Rapier
─────────────────────────────    ─────────────────────────────
World.create({ gravity })        new RAPIER.World({ x: 0, y: 0 })
world.createBody({ position })   world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y))
world.addCollider({ shape })     world.createCollider(RAPIER.ColliderDesc.ball/cuboid/convexHull(...), body)
body.translation()               body.translation() / body.setRotation()
world.step(dt)                   world.timestep = dt; world.step()
world.destroyBody(b)             world.removeRigidBody(b)
```

### 2. `demo/compare.html` — Side-by-Side Comparison Page
A single page with two canvases side-by-side: left = Matcha2D, right = Rapier. Shared controls apply parameters to both engines simultaneously.

**Layout:**
- Title: "Matcha2D vs Rapier — Side-by-Side Comparison"
- Shared controls bar (same as collision.html)
- Two canvases side-by-side (each ~530px wide to fit two on screen)
- Two stats bars (one per canvas)
- Both engines run the same scenario: same shape, count, friction, mass, speed
- Single seed for random positions/velocities so both start identically

### 3. `demo/demo-hub.html` — Navigation Page (Optional)
Simple page with links to all three demos:
- Collision Demo (Matcha2D)
- Blueprint Demo (Rapier)
- Side-by-Side Comparison

## Implementation Steps

1. **Install `@dimforge/rapier2d-compat`** — add to `package.json` as a dev dependency (for offline use), or just use CDN
   - CDN approach: `https://cdn.skypack.dev/@dimforge/rapier2d-compat` — zero setup
   - Local approach: `npm install @dimforge/rapier2d-compat` then serve from `node_modules`

2. **Create `demo/blueprint.html`** — Rapier demo with same controls and rendering style as collision.html

3. **Create `demo/compare.html`** — Side-by-side view with shared controls and seeded randomness

4. **Update `demo/demo-hub.html`** — Simple navigation landing page

5. **Update `server.js` default route** — Point `/` to `demo-hub.html` instead of `collision.html`, or add route for `/demo/`

## Coordinate System Note
Rapier uses Y-up (standard physics), canvas uses Y-down. When rendering Rapier bodies:
```
renderY = canvasHeight - rapierY
renderAngle = -rapierAngle
```

## Verification
- Run `npm run dev`
- Open `http://localhost:3000/demo/blueprint.html` — Rapier demo works
- Open `http://localhost:3000/demo/compare.html` — Both demos side-by-side
- Compare collision behavior: stacking, bouncing, friction differences visible
