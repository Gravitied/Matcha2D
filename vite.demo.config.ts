import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = __dirname
const packages = path.join(repoRoot, 'packages')
const demoRoot = path.join(repoRoot, 'demo')

export default defineConfig({
  root: demoRoot,
  server: {
    port: 3000,
    fs: {
      allow: [repoRoot, packages],
    },
  },
  resolve: {
    alias: {
      '@matcha2d/types': path.join(packages, 'types/src/index.ts'),
      '@matcha2d/world': path.join(packages, 'world/src/index.ts'),
      '@matcha2d/physics-rust': path.join(packages, 'physics-rust/pkg/matcha2d_physics.js'),
    },
  },
  optimizeDeps: {
    exclude: ['@matcha2d/physics-rust'],
  },
  build: {
    target: 'es2022',
    outDir: path.join(repoRoot, 'demo-dist'),
    emptyOutDir: true,
    rollupOptions: {
      // wasm-bindgen glue: import shims are only linked from .wasm; keep them if pkg sideEffects regresses
      treeshake: {
        moduleSideEffects(id) {
          if (id.includes('matcha2d_physics.js')) return true
          return null
        },
      },
      input: {
        index: path.join(demoRoot, 'index.html'),
        'demo-hub': path.join(demoRoot, 'demo-hub.html'),
        collision: path.join(demoRoot, 'collision.html'),
        compare: path.join(demoRoot, 'compare.html'),
        blueprint: path.join(demoRoot, 'blueprint.html'),
      },
    },
  },
})
