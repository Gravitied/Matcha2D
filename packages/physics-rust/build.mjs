#!/usr/bin/env node
// Build script for physics-rust WASM package
// Checks if WASM is already built, builds if needed

import { existsSync } from 'fs'
import { execSync } from 'child_process'
import { join, dirname } from 'path'
import { fileURLToPath } from 'url'

const __dirname = dirname(fileURLToPath(import.meta.url))
const pkgDir = join(__dirname, 'pkg')
const wasmFile = join(pkgDir, 'matcha2d_physics_bg.wasm')

if (existsSync(wasmFile)) {
  console.log('WASM already built, skipping. Use "npm run rebuild" to force rebuild.')
  process.exit(0)
}

// Try to find wasm-pack
let wasmPack = 'wasm-pack'
const cargoBinDir = join(process.env.HOME || process.env.USERPROFILE || '', '.cargo', 'bin')
const cargoBin = join(cargoBinDir, 'wasm-pack')
const pathSep = process.platform === 'win32' ? ';' : ':'
const envWithCargo = { ...process.env, PATH: `${cargoBinDir}${pathSep}${process.env.PATH || ''}` }
try {
  // Try with cargo bin in PATH first
  execSync('wasm-pack --version', { stdio: 'ignore', env: envWithCargo })
  wasmPack = 'wasm-pack'
} catch {
  // Try explicit path
  if (existsSync(cargoBin + '.exe') || existsSync(cargoBin)) {
    wasmPack = cargoBin
  } else {
    console.error('wasm-pack not found. Install it with: cargo install wasm-pack')
    process.exit(1)
  }
}

console.log('Building WASM...')
execSync(`${wasmPack} build --target web --out-dir pkg`, {
  cwd: __dirname,
  stdio: 'inherit',
  env: envWithCargo
})
console.log('WASM build complete.')
