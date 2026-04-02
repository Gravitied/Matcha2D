#!/usr/bin/env bash
# Build Matcha2D WASM module using Emscripten
# Usage: ./build.sh [Debug|Release]

set -euo pipefail

BUILD_TYPE="${1:-Release}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
DIST_DIR="${SCRIPT_DIR}/../dist/wasm"

# Check for emcc
if ! command -v emcc &> /dev/null; then
  echo "Error: Emscripten (emcc) not found in PATH."
  echo ""
  echo "Install Emscripten SDK:"
  echo "  git clone https://github.com/emscripten-core/emsdk.git"
  echo "  cd emsdk && ./emsdk install latest && ./emsdk activate latest"
  echo "  source /path/to/emsdk/emsdk_env.sh"
  exit 1
fi

echo "Building Matcha2D WASM module (${BUILD_TYPE})..."

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure with Emscripten CMake
echo "Running emcmake cmake..."
emcmake cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" ..

# Build
echo "Running cmake --build..."
cmake --build . --config "${BUILD_TYPE}"

# Copy output to dist
echo "Copying output to ${DIST_DIR}..."
mkdir -p "${DIST_DIR}"
if [ -f "${BUILD_DIR}/matcha2d_wasm.wasm" ]; then
  cp "${BUILD_DIR}/matcha2d_wasm.wasm" "${DIST_DIR}/"
  echo "  -> ${DIST_DIR}/matcha2d_wasm.wasm"
fi
if [ -f "${BUILD_DIR}/matcha2d_wasm.js" ]; then
  cp "${BUILD_DIR}/matcha2d_wasm.js" "${DIST_DIR}/"
  echo "  -> ${DIST_DIR}/matcha2d_wasm.js"
fi

echo "Build complete!"
echo "Output files:"
ls -la "${DIST_DIR}/"
