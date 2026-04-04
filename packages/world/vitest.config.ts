import { defineConfig } from 'vitest/config'
import { resolve } from 'path'
import { readFileSync } from 'fs'

export default defineConfig({
  test: {
    alias: {
      '@matcha2d/physics-rust': resolve(__dirname, '../physics-rust/pkg/matcha2d_physics.js'),
    },
    setupFiles: ['./__tests__/setup-wasm.ts'],
  },
})
