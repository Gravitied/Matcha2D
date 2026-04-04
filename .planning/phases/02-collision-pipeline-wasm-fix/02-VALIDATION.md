---
phase: 02
slug: collision-pipeline-wasm-fix
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-04-02
---

# Phase 02 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | vitest 3.x |
| **Config file** | `packages/core/package.json` (vitest) |
| **Quick run command** | `npm test -w packages/core -- --run` |
| **Full suite command** | `npm test` |
| **Estimated runtime** | ~10 seconds |

---

## Sampling Rate

- **After every task commit:** Run `npm test -w packages/core -- --run`
- **After every plan wave:** Run `npm test`
- **Before `/gsd:verify-work`:** Full suite must be green
- **Max feedback latency:** 15 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|-----------|-------------------|-------------|--------|
| 02-01-01 | 01 | 1 | COL-03 | unit | `npm test -w packages/core -- --run` | ❌ W0 | ⬜ pending |
| 02-01-02 | 01 | 1 | COL-04 | unit | `npm test -w packages/core -- --run` | ❌ W0 | ⬜ pending |
| 02-02-01 | 02 | 1 | COL-01 | unit | `npm test -w packages/core -- --run` | ❌ W0 | ⬜ pending |
| 02-02-02 | 02 | 1 | COL-01 | integration | `npm test -w packages/core -- --run` | ❌ W0 | ⬜ pending |
| 02-03-01 | 03 | 2 | COL-02 | manual | demo in browser | N/A | ⬜ pending |
| 02-04-01 | 04 | 2 | COL-05 | unit | `npm test -w packages/core -- --run` | ❌ W0 | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `packages/core/__tests__/wasm-collision.test.ts` — stubs for COL-01 (contact manifold correctness)
- [ ] `packages/core/__tests__/matcha-physics-rename.test.ts` — stubs for COL-03 (rename verification)
- [ ] `packages/core/__tests__/contact-conventions.test.ts` — stubs for COL-04 (normal/depth conventions)

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| WASM backend renders shapes correctly in browser | COL-02 | Requires browser Canvas2D rendering | Open demo/collision.html, switch to WASM backend, verify boxes/circles/polygons render |
| Contact normals visualized in demo | COL-01 | Requires browser visual inspection | Open demo, enable WASM backend, trigger collisions, verify normal lines drawn |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 15s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
