# Matcha2D Wiki Log

Append-only activity log. Never edit past entries.

Parse recent entries: `grep "^## \[" wiki/log.md | tail -10`

---

## [2026-04-05] init | Matcha2D Wiki
- Created wiki directory structure (raw/, pages/entities/, pages/concepts/, pages/sources/, pages/analyses/)
- Created SCHEMA.md, index.md, log.md
- Wiki is empty — ready for first ingest

## [2026-04-06] ingest | CLAUDE.md — full codebase documentation
- Summary: Extracted all architecture, API, algorithm, and design knowledge from CLAUDE.md into structured wiki pages
- Pages created:
  - pages/entities/physics-backend-interface.md
  - pages/entities/world-class.md
  - pages/entities/world-config.md
  - pages/entities/collision-pipeline.md
  - pages/entities/broadphase.md
  - pages/entities/narrowphase-gjk.md
  - pages/entities/sequential-impulse-solver.md
  - pages/entities/contact-tracker.md
  - pages/entities/wasm-physics-backend.md
  - pages/entities/math-primitives.md
  - pages/entities/render-package.md
  - pages/entities/tools-package.md
  - pages/concepts/architecture-overview.md
  - pages/concepts/buffer-layout.md
  - pages/concepts/data-oriented-design.md
  - pages/concepts/fixed-timestep.md
  - pages/concepts/wasm-dual-backend.md
  - pages/concepts/solver-concepts.md
- Pages updated: index.md
