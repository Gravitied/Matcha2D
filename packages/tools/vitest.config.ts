import { defineConfig } from 'vitest/config'

/** Tools package: tests were removed per suite consolidation; keep CI green. */
export default defineConfig({
  test: {
    passWithNoTests: true,
  },
})
