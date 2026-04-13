import { defineConfig } from 'vitest/config'

/** Render package: tests were removed per suite consolidation; keep CI green. */
export default defineConfig({
  test: {
    passWithNoTests: true,
  },
})
