# Phase 02: Collision Pipeline & WASM Integration Fix - Research

**Researched:** 2026-04-02
**Domain:** WASM/C physics engine, contact manifold extraction, TypeScript-WASM bridge, vitest testing
**Confidence:** HIGH

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|-----------------|
| COL-01 | WASM backend produces correct contact normals and contact frames matching the TypeScript backend | Root cause identified: `b2_get_contacts` uses `localB` (already in local space) as a world-space position, and passes `maxContacts` as the 9th arg but C function signature only takes 8 args — data is never written. Full contact point reconstruction path identified. |
| COL-02 | All shape types (box, circle, polygon) render correctly in the demo using both backends | Rendering code verified correct in both demos; the WASM demo's `render()` function never exposes manifolds for hit-highlighting. Shape drawing logic itself is correct — issue is manifold flow, not renderer. |
| COL-03 | Box2D WASM bridge C code renamed/rewritten as matcha_physics (matcha2D-specific, no generic Box2D naming) | `matcha_physics.c/.h` files already exist and own all physics logic. Remaining `b2_` prefixed API stubs in the header and C file need renaming to `m2_` throughout, and CMakeLists.txt target `box2d` needs renaming to `matcha2d`. |
| COL-04 | Collision algorithms consistent with standard 2D physics engine conventions (normal direction, contact point, penetration depth) | C code convention: normal points from A toward B (identical to TS backend). Contact points are stored as `localA`/`localB` in body-local space. The `b2_get_contacts` extractor incorrectly outputs `localB` as a position — fix is to reconstruct world-space point from `localA`. |
| COL-05 | Comprehensive test suite covering WASM backend collision results, contact manifold correctness, and shape rendering correctness | Existing test infrastructure: vitest, `__tests__/` pattern. WASM module cannot be loaded in Node.js vitest environment (browser WASM). Tests for COL-05 should cover: TS backend manifold correctness by shape pair, contact normal direction convention, contact point reconstruction, penetration depth sign, and C-to-TS data alignment. A separate integration/smoke test using a headless browser or pure C test is appropriate for true WASM path. |
</phase_requirements>

---

## Summary

The WASM backend (`matcha_physics.c`) implements a full GJK+EPA narrowphase and stores contact manifolds correctly in a `WorldState.manifolds[]` scratch buffer. The bug is in **extraction**, not generation. `b2_get_contacts` in the C code passes a `maxContacts` parameter as the 9th argument to `m2GetContacts` in `WasmModule.ts`, but the TypeScript-side `CwrapGetContacts` signature only declares 8 arguments — the C function therefore receives garbage for `maxContacts` and may write zero entries. Additionally, the extractor copies `localBx/localBy` as if they are world-space contact positions, which they are not — they are body-local coordinates.

The `collision-wasm.html` demo's `render()` function fetches no manifolds at all — it uses an empty `hit` array and never calls any contact retrieval method, so even a correct backend produces no visual contact output in that demo.

The `collision.html` demo hides contact/normal visualization behind a `!useWorldAPI` guard — so WASM mode never renders contact frames even if manifolds were returned correctly.

**Primary recommendation:** Fix the `b2_get_contacts` C signature and argument mismatch in `WasmModule.ts`, fix the contact point world-space reconstruction, wire manifold data into the demo rendering path for WASM mode, rename all `b2_` C API symbols to `m2_`, and rename the CMake target from `box2d` to `matcha2d`.

---

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| vitest | ^3.0 (already installed) | Test runner | Project standard; already used in all packages |
| Emscripten | installed on system | C-to-WASM compiler | Project decision from Phase 01 |
| TypeScript | ^5.7 (already installed) | Type-checked bridge layer | Project standard |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `@matcha2d/types` | workspace | Shared buffer layout, ContactManifold interface | All cross-package contracts |

**Installation:** No new packages needed — all dependencies are already present.

---

## Architecture Patterns

### Data Flow: WASM Collision to TypeScript Manifold

```
JS SoA buffers
  -> WasmModule.ts: allocAndCopy() each buffer into WASM heap
  -> C: m2_full_step() runs broadphase -> narrowphase -> integrate -> solve
       Narrowphase writes Manifold[] into WorldState.manifolds[nManifolds]
  -> C: b2_get_contact_count() returns ws->nManifolds
  -> C: b2_get_contacts() copies manifolds into caller-supplied arrays
  -> WasmModule.ts: reads arrays off WASM heap, constructs ContactManifold[]
  -> WasmPhysicsBackend.ts: collide() returns ContactManifold[]
  -> World._physicsStep(): passes manifolds to ContactTracker
```

### C Manifold Layout (matcha_physics.c)

```c
typedef struct {
    float localAx, localAy;   // contact point in body A's local space
    float localBx, localBy;   // contact point in body B's local space
    float penetration;
    int   idA, idB;
} ContactPointData;

typedef struct {
    int             bodyA, bodyB;
    float           normalX, normalY;  // unit normal pointing A -> B
    ContactPointData contacts[2];
    int             contactCount;
} Manifold;
```

### TypeScript ContactManifold Layout (@matcha2d/types)

```typescript
interface ContactManifold {
  bodyA: BodyHandle
  bodyB: BodyHandle
  normal: { x: number; y: number }  // unit normal pointing A -> B
  contacts: Array<{
    localA: { x: number; y: number }   // contact point in A's local space
    localB: { x: number; y: number }   // contact point in B's local space
    penetration: number
    idA?: number
    idB?: number
  }>
}
```

These layouts match. The mismatch is only in how `b2_get_contacts` extracts the data for the TypeScript side.

### Anti-Patterns to Avoid

- **Treating `localBx/localBy` as a world-space position:** These are body-local. To get world-space, apply `mat2_from_angle(b->angle[bi])` and add `b->posX/posY[bi]`.
- **Single contact per manifold in the output:** The C code can produce 2 contacts per pair (edge-edge). The current `b2_get_contacts` flattens to one per manifold. The TypeScript `ContactManifold.contacts[]` supports multiple — extract all.
- **Hiding contact visualization behind `!useWorldAPI`:** The WASM backend does return manifolds; the rendering path should be backend-agnostic.

---

## Root Cause Analysis

### Bug 1: Argument count mismatch in `getContacts`

In `WasmModule.ts`, `m2GetContacts` is called with 9 arguments:

```typescript
m2GetContacts(worldHandle,
  pBodyA, pBodyB, pNx, pNy, pPx, pPy, pPen, contactCount)  // 9 args
```

The C signature for `b2_get_contacts` is:

```c
WASM_EXPORT void b2_get_contacts(
    int worldHandle,
    int *bodyA, int *bodyB,
    float *nx, float *ny,
    float *px, float *py,
    float *penetration,
    int maxContacts)   // 9 C params — this is correct
```

The `CwrapGetContacts` TypeScript type declaration in `WasmModule.ts`:

```typescript
type CwrapGetContacts = (
  handle: number,
  bodyA: number, bodyB: number,
  nx: number, ny: number,
  px: number, py: number,
  penetration: number,
  maxContacts: number,  // 9 params — matches
) => void
```

The `cwrap` registration declares 9 `'number'` args — **this is correct in WasmModule.ts**. The call passes `contactCount` as `maxContacts`, which is correct. This specific mismatch concern from the initial hypothesis is **not the bug**.

### Bug 2: Contact point extraction uses `localB` as world-space position

In `b2_get_contacts` (C, line ~1807):

```c
px[i] = m->contacts[0].localBx;   // WRONG: this is local space, not world
py[i] = m->contacts[0].localBy;
```

And in `WasmModule.ts`, these are placed into `localB`:

```typescript
contacts: [{
  localA: { x: 0, y: 0 },           // WRONG: always zero
  localB: { x: heapPx[i], y: heapPy[i] },  // this contains localB from C
  penetration: heapPen[i],
}],
```

The `ContactManifold` interface expects `localA` to be the contact point in body A's local space, and `localB` in body B's local space. The current code sets `localA` to `{0,0}` (always) and puts the C's `localBx/localBy` into `localB`. The solver in `World._physicsStep()` uses `localA` to reconstruct the world-space contact position, so this zero-vector causes incorrect torque calculations.

### Bug 3: Only one contact point extracted per manifold

The C stores up to 2 contacts per manifold. `b2_get_contacts` only copies `contacts[0]`. The extractor needs to handle multi-contact manifolds.

### Bug 4: `collision.html` hides WASM contact rendering

At line ~559 in `collision.html`:

```javascript
// Show contacts and normals (TypeScript mode only)
if (!useWorldAPI) {
  // ... normals/contacts visualization
}
```

And in `render()`, when in WASM mode, `manifolds` is never populated:

```javascript
if (useWorldAPI) {
  // WASM mode - just render, collision is handled internally
  const t0 = performance.now()
  elapsed = performance.now() - t0  // manifolds stays []
}
```

So even if contacts were returned correctly, they would never render.

### Bug 5: `collision-wasm.html` never retrieves manifolds

The `render()` function in `collision-wasm.html` creates an empty `hit` array and returns `{ manifolds: 0 }` without querying any contact data from the world.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Contact point world-space reconstruction | Custom matrix math | Existing `mat2FromAngle` / `mat2_from_angle` already in both TS and C | Already correct and tested |
| Test helpers for buffer setup | Ad-hoc per-test | Pattern from `gjk.test.ts`: `placeBox`, `placeCircle`, `placePolygon` helpers | Consistent, reusable pattern already established |
| WASM loading in vitest | Node.js WASM loader | Don't — test collision logic at the TS backend level; WASM requires browser environment | WASM is browser-only (`ENVIRONMENT='web'` in CMakeLists) |

---

## Common Pitfalls

### Pitfall 1: WASM environment constraint
**What goes wrong:** Vitest runs in Node.js. The WASM build uses `-s ENVIRONMENT='web'` which means the `.wasm` file cannot be loaded by Node.js without polyfills.
**Why it happens:** Emscripten generates browser-specific glue code.
**How to avoid:** Test WASM behavior through the TS backend equivalents (same algorithms). WASM integration testing requires a headless browser (Playwright/Puppeteer) or changing `ENVIRONMENT='web,node'` in CMake — but that is a separate concern.
**Warning signs:** `Cannot read properties of undefined (reading 'fetch')` or similar browser API errors in test output.

### Pitfall 2: Multi-contact manifold truncation
**What goes wrong:** Extracting only `contacts[0]` from the C manifold loses the second contact point for edge-edge collisions.
**Why it happens:** The current `b2_get_contacts` only writes one px/py/penetration per pair index.
**How to avoid:** Either expand the C API to output both contacts, or change the output format to flatten all contacts with their body pair duplicated.

### Pitfall 3: Normal direction after extraction
**What goes wrong:** The C narrowphase ensures the normal points from A to B. If `b2_get_contacts` is changed to reconstruct world-space contact points, the sign convention must be preserved.
**Why it happens:** The EPA normal flip (`nx = -epa.normalX`) at line ~1370 already enforces A-to-B. Do not re-flip when extracting.

### Pitfall 4: `mat2_from_angle` output parameter naming
**What goes wrong:** The C function `mat2_from_angle(angle, &c, &negS, &s)` sets `negS = -sin(angle)`. The rotation formula is `wx = c * lx + negS * ly`.
**Why it happens:** The name `negS` implies it already carries the negative sign. Using `+negS` (not `-s`) in the matrix multiply is correct — this matches the TS implementation.

### Pitfall 5: `collision.html` broadphase dropdown is irrelevant in WASM mode
**What goes wrong:** The demo has SAP/BVH broadphase and SAT/GJK narrowphase selectors. In WASM mode, these are ignored — the C code always uses dynamic AABB tree + GJK+EPA.
**How to avoid:** The demo should disable or hide these selectors when WASM backend is selected.

---

## Code Examples

### Correct contact extraction pattern (C side)

The `b2_get_contacts` function needs to reconstruct world-space contact points from `localA`. Example of correct reconstruction:

```c
// For each manifold, for each contact point:
float cA, negSA, sA;
mat2_from_angle(body_angle_A, &cA, &negSA, &sA);
float wx = cA * localAx + negSA * localAy + posX_A;
float wy = sA * localAx + cA    * localAy + posY_A;
px[outIdx] = wx;
py[outIdx] = wy;
```

Note: The angle for body A is not available in `b2_get_contacts` since only the manifold is stored, not the body buffers. Two options:
1. Store world-space contact points in `Manifold` at generation time in `build_manifold`.
2. Pass the body buffers into `b2_get_contacts` (increases API surface).
3. Pass `localA` and `localB` separately and reconstruct in TypeScript.

**Recommended approach (option 3):** Extend the output to include `localAx/localAy/localBx/localBy` per contact, and reconstruct world-space in TypeScript using `mat2FromAngle`.

### Correct TypeScript extraction in `WasmModule.ts`

```typescript
// Source: WasmModule.ts getContacts() — corrected version
for (let i = 0; i < contactCount; i++) {
  manifolds.push({
    bodyA: heapBodyA[i] as BodyHandle,
    bodyB: heapBodyB[i] as BodyHandle,
    normal: { x: heapNx[i], y: heapNy[i] },
    contacts: [{
      localA: { x: heapLocalAX[i], y: heapLocalAY[i] },
      localB: { x: heapLocalBX[i], y: heapLocalBY[i] },
      penetration: heapPen[i],
    }],
  })
}
```

### Renaming b2_ to m2_ pattern

All exported functions prefixed `b2_` in `matcha_physics.h` and `matcha_physics.c` must be renamed to `m2_`:
- `b2_init` → `m2_init`
- `b2_destroy` → `m2_destroy`
- `b2_sync_bodies` → `m2_sync_bodies` (stub, kept for API compat)
- `b2_sync_shapes` → `m2_sync_shapes` (stub)
- `b2_step` → `m2_step` (stub)
- `b2_read_bodies` → `m2_read_bodies` (stub)
- `b2_get_contact_count` → `m2_get_contact_count`
- `b2_get_contacts` → `m2_get_contacts`
- CMakeLists.txt: `add_executable(box2d ...)` → `add_executable(matcha2d ...)`

The `cwrap` calls in `WasmModule.ts` must be updated to match the new names:
```typescript
const m2Init = wrapCwrap<CwrapInit>(Module, 'm2_init', 'number', ...)
const m2Destroy = wrapCwrap<CwrapDestroy>(Module, 'm2_destroy', null, ...)
const m2GetContactCount = wrapCwrap<...>(Module, 'm2_get_contact_count', ...)
const m2GetContacts = wrapCwrap<...>(Module, 'm2_get_contacts', ...)
```

The compiled output filename is controlled by the CMake target name. Renaming `box2d` to `matcha2d` means the output becomes `matcha2d.js` and `matcha2d.wasm`. The import in `WasmModule.ts` must change from `'./box2d.js'` to `'./matcha2d.js'`, and `locateFile` path calculation must update accordingly. The `tsup.config.ts` copy rules and `.gitignore` patterns must also be updated.

### Demo: showing contacts in WASM mode

The `render()` function in `collision.html` needs to expose manifolds in WASM mode. Since `World` does not currently expose the last frame's manifolds as a public property, options are:
1. Add a `lastManifolds` getter to `World` that returns the manifolds from the most recent `_physicsStep`.
2. Call `backend.collide()` again in the render function (expensive, double step).
3. Store manifolds on `World` as a public `manifolds: ContactManifold[]` field updated each `_physicsStep`.

**Recommended:** Add `get manifolds(): ContactManifold[]` getter to `World` (readonly, returns last frame's results).

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Separate `b2_sync_bodies + b2_step + b2_read_bodies` | Single `m2_full_step` with direct SoA pointers | Phase 01 | All state in one call, no intermediate copies |
| Box2D C++ as physics core | matcha_physics.c (owned GJK+EPA+solver) | Phase 01 (matcha_physics.c added) | No Box2D runtime dependency |
| `wasm_bridge.c` (deleted) | `matcha_physics.c` | Phase 01 | Fully custom implementation |

**Deprecated/outdated:**
- `wasm_bridge.c / wasm_bridge.h`: Deleted. `matcha_physics.c/.h` is the sole WASM source.
- `b2_` prefix on exported functions: Legacy from original Box2D naming. Phase 02 renames to `m2_`.

---

## Open Questions

1. **Multi-contact per manifold extraction**
   - What we know: C stores up to 2 contacts per manifold in `contacts[MAX_CONTACTS_PER_MANIFOLD]`
   - What's unclear: Should the output arrays be per-manifold (current) or per-contact-point (flattened)?
   - Recommendation: Per-manifold output; extend arrays to `2 * contactCount` max and output a `contactCountPerManifold` array alongside.

2. **WASM test coverage without browser**
   - What we know: `ENVIRONMENT='web'` prevents Node.js loading.
   - What's unclear: Whether to change to `ENVIRONMENT='web,node'` to enable vitest coverage of the C physics.
   - Recommendation: Change to `web,node` for testability — vitest can then load the WASM module directly. Low risk since the web environment is a superset. This is a CMakeLists.txt change only.

3. **`collision.html` vs `collision-wasm.html` consolidation**
   - What we know: Two separate demo files exist; `collision-wasm.html` uses `World.createWithTS/createWithWasm` while `collision.html` mixes direct buffer manipulation with World API.
   - What's unclear: Should they be merged into one file?
   - Recommendation: Make `collision-wasm.html` the canonical demo (uses proper World API for both backends). `collision.html` can be kept as the TypeScript-only diagnostic demo.

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | vitest ^3.0 |
| Config file | `vitest.config.ts` or via `package.json` scripts |
| Quick run command | `npm test -w packages/core` |
| Full suite command | `npm test` |

### Phase Requirements -> Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| COL-01 | WASM backend `getContacts()` returns non-empty ContactManifold[] when bodies overlap | unit (TS-equivalent) | `npm test -w packages/core` | ❌ Wave 0 |
| COL-01 | Contact normal is unit vector pointing from A toward B | unit | `npm test -w packages/core` | ❌ Wave 0 |
| COL-01 | Contact localA is non-zero for box-box overlap | unit | `npm test -w packages/core` | ❌ Wave 0 |
| COL-01 | Penetration depth > 0 for overlapping shapes | unit | `npm test -w packages/core` | ❌ Wave 0 |
| COL-02 | Rendering code draws polygon vertices correctly (vertex transform) | manual | open demo in browser | N/A |
| COL-03 | No `b2_` symbol references in source after rename | lint/grep | `grep -r "b2_" packages/core/wasm/` | N/A |
| COL-04 | Normal direction consistent between TS and C backends for same body pair | unit | `npm test -w packages/core` | ❌ Wave 0 |
| COL-04 | Penetration depth > 0 and < shape size | unit | `npm test -w packages/core` | ❌ Wave 0 |
| COL-05 | Comprehensive test file for contact manifold correctness | unit | `npm test -w packages/core` | ❌ Wave 0 |

### Sampling Rate
- **Per task commit:** `npm test -w packages/core`
- **Per wave merge:** `npm test`
- **Phase gate:** Full suite green before `/gsd:verify-work`

### Wave 0 Gaps
- [ ] `packages/core/__tests__/wasm-contacts.test.ts` — covers COL-01, COL-04, COL-05 (tests contact manifold extraction correctness using TS backend as ground truth, since WASM may require browser)
- [ ] If `ENVIRONMENT='web,node'` is added to CMake: `packages/core/__tests__/wasm-backend.test.ts` — end-to-end WASM pipeline test

---

## Sources

### Primary (HIGH confidence)
- Direct source code analysis: `packages/core/wasm/matcha_physics.c` — full read of narrowphase, contact extraction, and world API
- Direct source code analysis: `packages/core/src/wasm/WasmModule.ts` — full read of cwrap bindings and contact extraction
- Direct source code analysis: `packages/core/src/wasm/WasmPhysicsBackend.ts` — full read of PhysicsBackend implementation
- Direct source code analysis: `packages/core/wasm/matcha_physics.h` — exported function signatures
- Direct source code analysis: `packages/core/wasm/CMakeLists.txt` — build configuration
- Direct source code analysis: `demo/collision.html` lines 514-590 — rendering path and WASM mode guards
- Direct source code analysis: `demo/collision-wasm.html` lines 371-396 — missing manifold retrieval

### Secondary (MEDIUM confidence)
- Existing test files in `packages/core/__tests__/` — confirm vitest patterns, helper functions, and test structure conventions
- `packages/world/src/world.ts` — confirms `_physicsStep` control flow and that manifolds are not exposed publicly

### Tertiary (LOW confidence)
- None

---

## Metadata

**Confidence breakdown:**
- Root cause identification: HIGH — confirmed by direct code trace through C and TypeScript source
- Standard stack: HIGH — all libraries already installed, no new dependencies
- Architecture patterns: HIGH — existing codebase defines the patterns
- Test patterns: HIGH — existing `__tests__/` files provide clear template
- Pitfalls: HIGH — identified from direct code inspection

**Research date:** 2026-04-02
**Valid until:** 2026-05-02 (stable codebase, no fast-moving external dependencies)
