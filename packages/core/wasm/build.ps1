# Build Matcha2D WASM module using Emscripten (PowerShell)
# Usage: .\build.ps1 [-BuildType Release|Debug]

param(
  [ValidateSet("Release", "Debug")]
  [string]$BuildType = "Release"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$BuildDir = Join-Path $ScriptDir "build"
$DistDir = Join-Path $ScriptDir "..\dist\wasm" | Resolve-Path

# Check for emcc
try {
  $null = Get-Command emcc -ErrorAction Stop
} catch {
  Write-Host "Error: Emscripten (emcc) not found in PATH." -ForegroundColor Red
  Write-Host ""
  Write-Host "Install Emscripten SDK:"
  Write-Host "  git clone https://github.com/emscripten-core/emsdk.git"
  Write-Host "  cd emsdk && .\emsdk install latest && .\emsdk activate latest"
  Write-Host "  .\emsdk_env.ps1"
  exit 1
}

Write-Host "Building Matcha2D WASM module ($BuildType)..." -ForegroundColor Cyan

# Create build directory
if (-not (Test-Path $BuildDir)) {
  New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

Set-Location $BuildDir

# Configure with Emscripten CMake
Write-Host "Running emcmake cmake..." -ForegroundColor Yellow
emcmake cmake -DCMAKE_BUILD_TYPE=$BuildType ..

# Build
Write-Host "Running cmake --build..." -ForegroundColor Yellow
cmake --build . --config $BuildType

# Copy output to dist
Write-Host "Copying output to $DistDir..." -ForegroundColor Yellow
if (-not (Test-Path $DistDir)) {
  New-Item -ItemType Directory -Path $DistDir | Out-Null
}

if (Test-Path (Join-Path $BuildDir "box2d.wasm")) {
  Copy-Item (Join-Path $BuildDir "box2d.wasm") $DistDir -Force
  Write-Host "  -> $(Join-Path $DistDir box2d.wasm)"
}
if (Test-Path (Join-Path $BuildDir "box2d.js")) {
  Copy-Item (Join-Path $BuildDir "box2d.js") $DistDir -Force
  Write-Host "  -> $(Join-Path $DistDir box2d.js)"
}

Write-Host "Build complete!" -ForegroundColor Green
Write-Host "Output files:"
Get-ChildItem $DistDir | Format-Table Name, Length, LastWriteTime
