# Matcha2D Wiki Schema

This file defines the conventions, directory layout, and workflows for the Matcha2D wiki. Read this at the start of every session before touching any wiki file.

---

## Directory Layout

```
wiki/
  raw/              # Immutable source documents — never modify
    assets/         # Images, PDFs, diagrams referenced by raw sources
  pages/            # LLM-maintained wiki pages
    entities/       # Specific named things: algorithms, classes, systems, APIs
    concepts/       # Abstract ideas, patterns, design decisions
    sources/        # One summary page per raw source ingested
    analyses/       # Saved query results, comparisons, explorations
  SCHEMA.md         # This file — wiki conventions
  index.md          # Content catalog — updated on every ingest
  log.md            # Append-only activity log
```

**Raw sources are sacred.** The LLM reads them but never modifies them.
**Wiki pages are owned by the LLM.** The LLM creates, updates, and cross-references them.

---

## Page Format

Every wiki page starts with YAML frontmatter:

```yaml
---
title: "Page Title"
type: entity | concept | source | analysis
tags: [tag1, tag2]
related: [page-slug-1.md, page-slug-2.md]
sources: [raw/source-file.md]
created: YYYY-MM-DD
updated: YYYY-MM-DD
---
```

Then standard markdown body. Use `[[page-slug]]` syntax for internal links (Obsidian-compatible).

---

## Page Types

### Entity Pages (`pages/entities/`)
Cover specific named things in the codebase or domain:
- Algorithms (GJK, EPA, SAP, Dynamic AABB Tree, Sequential Impulse)
- Systems (Broadphase, Narrowphase, Solver, Sleep, Contact Tracker)
- Classes/Modules (World, BodyManager, PhysicsBackend, WasmPhysicsBackend)
- Data structures (MatchaBuffers, ContactManifold, CollisionPair)

Structure: what it is → how it works → inputs/outputs → edge cases → cross-references.

### Concept Pages (`pages/concepts/`)
Cover abstract ideas, patterns, and design decisions:
- Data-Oriented Design, SoA buffers, flat typed arrays
- Fixed timestep pattern, render interpolation
- Warm starting, Baumgarte correction, impulse accumulation
- WASM integration strategy, TypeScript/WASM dual-backend design
- Collision filtering, sensor bodies, sleep optimization

Structure: the idea → why it matters for Matcha2D → trade-offs → related entities.

### Source Pages (`pages/sources/`)
One page per ingested raw document. Named `<slug>.md` matching the raw file.

Structure: one-paragraph summary → key takeaways (bullets) → what wiki pages were updated → direct quotes or data worth preserving.

### Analysis Pages (`pages/analyses/`)
Saved answers to queries. These are first-class wiki citizens.

Structure: the question asked → the answer → citations to pages used → date.

---

## Workflows

### Ingest a New Source

1. Read the raw source file.
2. Discuss key takeaways with the user (what matters most for Matcha2D).
3. Write a source summary page in `pages/sources/`.
4. Identify which entity and concept pages need updating. Update each one.
5. Create new entity or concept pages if new things are introduced.
6. Update `index.md` — add the source summary and any new pages.
7. Append to `log.md`:
   ```
   ## [YYYY-MM-DD] ingest | <source title>
   - Summary: <one line>
   - Pages updated: <list>
   - Pages created: <list>
   ```

### Answer a Query

1. Read `index.md` to find relevant pages.
2. Read the relevant pages.
3. Synthesize an answer with citations (`[[page-slug]]`).
4. Ask the user: "Should I save this as an analysis page?"
5. If yes, write to `pages/analyses/<slug>.md` and update `index.md` + `log.md`.

Log entry:
```
## [YYYY-MM-DD] query | <question summary>
- Pages consulted: <list>
- Saved as: pages/analyses/<slug>.md (or "not saved")
```

### Lint the Wiki

Check for:
- Contradictions between pages (flag with `> **Contradiction:** ...` blockquote)
- Stale claims superseded by newer sources
- Orphan pages (no inbound links from other pages)
- Concepts mentioned but lacking their own page
- Missing cross-references between related pages
- Data gaps that a web search could fill

Log entry:
```
## [YYYY-MM-DD] lint
- Issues found: <count>
- Fixed: <list>
- Open gaps: <list>
```

---

## Cross-Reference Conventions

- Use `[[slug]]` for links to other wiki pages (Obsidian-compatible)
- Use `[title](../relative/path.md)` for standard markdown links
- Always update `related:` frontmatter when linking two pages
- When a source contradicts a wiki page, add a note: `> **Note (YYYY-MM-DD):** <source> says X, which contradicts the claim above that Y.`

---

## Matcha2D-Specific Tagging

Common tags to use consistently:

| Tag | Meaning |
|-----|---------|
| `algorithm` | A specific computational algorithm |
| `collision` | Broadphase or narrowphase collision detection |
| `solver` | Constraint/velocity/position solver |
| `wasm` | WASM / Rust / Box2D integration |
| `performance` | Performance-critical code or patterns |
| `api` | Public API surface |
| `data-structures` | Buffer layout, SoA, typed arrays |
| `sleep` | Sleep/wake system |
| `math` | Vec2, Mat2, geometric primitives |
| `architecture` | Design decisions, package structure |
| `dev-a` | Owned by Dev A (physics core) |
| `dev-b` | Owned by Dev B (engine shell) |

---

## index.md Format

The index is organized by page type. Each entry is one line:

```markdown
- [Page Title](pages/type/slug.md) — one-line description
```

Sections: `## Entities`, `## Concepts`, `## Sources`, `## Analyses`.

Keep entries under 150 chars. The LLM reads the full index on every query.

---

## log.md Format

Append-only. Never edit past entries. Each entry:

```markdown
## [YYYY-MM-DD] <operation> | <title>
<body>
```

Operations: `ingest`, `query`, `lint`, `init`, `schema-update`.

The log is parseable: `grep "^## \[" wiki/log.md | tail -10` shows the last 10 entries.
