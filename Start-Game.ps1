#Requires -Version 5.1
# Traffic Control 2D - One-click launcher
# Usage: .\Start-Game.ps1

# --- Paths -------------------------------------------------------------------
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

# --- Tool paths (adjust if your installation differs) ------------------------
$NodeDir  = "C:\Users\lukia\AppData\Local\nodejs-portable\node-v20.19.1-win-x64"
$CargoDir = "C:\Users\lukia\.cargo\bin"
$MinGWDir = "C:\Users\lukia\AppData\Local\mingw64\mingw64\bin"

# --- Environment -------------------------------------------------------------
$env:PATH        = "$MinGWDir;$CargoDir;$NodeDir;" + $env:PATH
$env:RUSTUP_HOME = "C:\Users\lukia\.rustup"
$env:CARGO_HOME  = "C:\Users\lukia\.cargo"
# Rust / env_logger: richer backend logs in this console (override before running script if needed).
if (-not $env:RUST_LOG) {
    $env:RUST_LOG = "debug"
}

# --- Virtual drive T: (avoids spaces-in-path issues for cargo/gcc) -----------
subst T: $ProjectRoot 2>$null | Out-Null
Set-Location "T:\"

# --- Header ------------------------------------------------------------------
Clear-Host
Write-Host ""
Write-Host "  ==========================================" -ForegroundColor Cyan
Write-Host "   Traffic Control 2D  -  Launcher         " -ForegroundColor Cyan
Write-Host "  ==========================================" -ForegroundColor Cyan
Write-Host ""

# --- Tool version check ------------------------------------------------------
try {
    $nodeVer  = & node --version 2>&1
    $cargoVer = & cargo --version 2>&1
    Write-Host "  Node  : $nodeVer"  -ForegroundColor Green
    Write-Host "  Cargo : $cargoVer" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Node or Cargo not found. Check tool paths in this script." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host ""

# --- Kill leftover processes -------------------------------------------------
Write-Host "  [1/3] Cleaning up old processes..." -ForegroundColor Yellow
Get-Process -Name "traffic-control" -ErrorAction SilentlyContinue |
    Stop-Process -Force -ErrorAction SilentlyContinue

$oldPort = Get-NetTCPConnection -LocalPort 1420 -State Listen -ErrorAction SilentlyContinue
if ($oldPort) {
    Stop-Process -Id $oldPort.OwningProcess -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 800
}

# --- Start Vite dev server ---------------------------------------------------
Write-Host "  [2/3] Starting Vite dev server on port 1420..." -ForegroundColor Yellow
$viteProc = Start-Process `
    -FilePath     "node" `
    -ArgumentList "start-dev.mjs" `
    -PassThru `
    -NoNewWindow

# Wait up to 10 s for Vite to bind the port
$ready   = $false
$retries = 0
do {
    Start-Sleep -Milliseconds 500
    $retries++
    $conn = Get-NetTCPConnection -LocalPort 1420 -State Listen -ErrorAction SilentlyContinue
    if ($conn) { $ready = $true }
} while (-not $ready -and $retries -lt 20)

if (-not $ready) {
    Write-Host "  ERROR: Vite did not start within 10 s." -ForegroundColor Red
    Stop-Process -Id $viteProc.Id -Force -ErrorAction SilentlyContinue
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "  Vite ready at http://localhost:1420/" -ForegroundColor Green
Write-Host ""

# --- Launch Tauri (Rust compile + WebView window) ----------------------------
Write-Host "  [3/3] Launching Tauri (first compile may take 1-2 min)..." -ForegroundColor Yellow
Write-Host "        Window opens automatically when compilation finishes." -ForegroundColor DarkGray
Write-Host ""

node_modules\.bin\tauri dev

# --- Cleanup when window is closed -------------------------------------------
Write-Host ""
Write-Host "  Game closed. Stopping Vite..." -ForegroundColor Yellow
Stop-Process -Id $viteProc.Id -Force -ErrorAction SilentlyContinue
Write-Host "  Done." -ForegroundColor Cyan
Write-Host ""
