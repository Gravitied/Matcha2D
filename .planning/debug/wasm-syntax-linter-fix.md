---
status: awaiting_human_verify
trigger: "Fix the syntax and linter errors in the wasm"
created: 2025-04-02T00:00:00.000Z
updated: 2025-04-02T00:00:00.000Z
---

## Current Focus

hypothesis: There are syntax or type errors in the WASM-related TypeScript files that need to be fixed
test: Run type checks and identify specific errors
expecting: List of actual TypeScript/ESLint errors
next_action: Identify and fix LSP/type errors in tsup.config.ts and WasmModule.ts

## Symptoms

expected: All WASM TypeScript files should compile without errors
actual: LSP reports errors:
  - tsup.config.ts: Cannot find module 'node:fs'/ 'node:path', missing 'process' types
  - wasm_bridge.c: Missing Box2D header (but this file is not actually compiled)
  - WasmModule.ts: Complex type assertions (as unknown as)
reproduction: Build the project or run linter; IDE shows LSP errors
started: Recent WASM refactoring changed WasmModule.ts structure

## Eliminated

- hypothesis: Build fails completely
  evidence: `npm run build` succeeded for core package; `tsc -b` clean
  timestamp: 2025-04-02T00:00:00.000Z

## Evidence

- timestamp: 2025-04-02T00:00:00.000Z
  checked: `npm run build -w packages/core`
  found: Build succeeded, no TypeScript errors
  implication: No syntax errors preventing compilation; errors are LSP-specific

- timestamp: 2025-04-02T00:00:00.000Z
  checked: LSP diagnostics from editor
  found: 
    - tsup.config.ts: "Cannot find module 'node:fs'" - missing @types/node
    - tsup.config.ts: "Cannot find name 'process'" - missing @types/node
    - wasm_bridge.c: "file not found: box2d/box2d.h" - unused legacy file
    - WasmModule.ts:275-276: type assertions use `as unknown as` pattern (linter may flag)
  implication: These are the linter errors to fix

- timestamp: 2025-04-02T00:00:00.000Z
  checked: CMakeLists.txt
  found: Only compiles matcha_physics.c, not wasm_bridge.c
  implication: wasm_bridge.c is dead code; can be removed or fixed with proper include path

## Resolution

root_cause: 
  1. Missing `@types/node` causing LSP errors in tsup.config.ts
  2. Legacy wasm_bridge.c with broken includes (not built but flagged by LSP)
  3. Overly complex type assertions in WasmModule.ts (code quality)
fix:
  1. Added `@types/node` to devDependencies (root package.json)
  2. Removed obsolete wasm_bridge.c and wasm_bridge.h (no longer used)
  3. Simplified `as unknown as ContactManifold['bodyA']` to `as BodyHandle` (imported BodyHandle)
  4. Removed unnecessary generic: `Array<string>(31)` -> `Array(31)`
verification:
  - ✓ npm run lint: clean
  - ✓ npm run build -w packages/core: success
  - LSP errors expected to be resolved (editor may need reload)
files_changed:
  - packages/core/package.json (devDep added via root)
  - packages/core/src/wasm/WasmModule.ts (cleanup)
  - removed: packages/core/wasm/wasm_bridge.c
  - removed: packages/core/wasm/wasm_bridge.h
