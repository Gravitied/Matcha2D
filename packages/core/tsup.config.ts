import { defineConfig } from 'tsup'
import { copyFileSync, existsSync, mkdirSync } from 'node:fs'
import { join } from 'node:path'

export default defineConfig({
  entry: ['src/index.ts'],
  format: ['esm'],
  dts: { compilerOptions: { composite: false } },
  sourcemap: true,
  clean: true,
  target: 'es2017',
  treeshake: true,
  async onSuccess() {
    // Copy WASM build artifacts from wasm/build/ to dist/wasm/
    const wasmBuildDir = join(process.cwd(), 'wasm', 'build')
    const distWasmDir = join(process.cwd(), 'dist', 'wasm')
    const wasmFiles = ['box2d.wasm', 'box2d.js']

    const allExist = wasmFiles.every(f =>
      existsSync(join(wasmBuildDir, f)),
    )

    if (allExist) {
      if (!existsSync(distWasmDir)) {
        mkdirSync(distWasmDir, { recursive: true })
      }
      for (const file of wasmFiles) {
        const src = join(wasmBuildDir, file)
        const dest = join(distWasmDir, file)
        copyFileSync(src, dest)
      }
    } else {
      console.warn(
        '[core] WASM build artifacts not found in wasm/build/. ' +
        'Run `npm run build:wasm` to build the WASM module. ' +
        'TypeScript build completed without WASM files.',
      )
    }
  },
})
