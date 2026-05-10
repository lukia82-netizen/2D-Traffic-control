#Requires -Version 5.1
<#
  Start-Tests.ps1
  Runs all project unit tests (frontend + Rust backend).

  Usage:
    .\Start-Tests.ps1
    .\Start-Tests.ps1 -SkipFrontend
    .\Start-Tests.ps1 -SkipBackend
#>

param(
    [switch]$SkipFrontend,
    [switch]$SkipBackend
)

$ErrorActionPreference = "Stop"

# --- Paths -------------------------------------------------------------------
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$OriginalLocation = Get-Location

# --- Tool paths (current Windows user — same logic as Start-Game.ps1) --------
$UserHome = $env:USERPROFILE
$NodeDir  = Join-Path $UserHome "AppData\Local\nodejs-portable\node-v20.19.1-win-x64"
$CargoDir = Join-Path $UserHome ".cargo\bin"
$MinGWDir = Join-Path $UserHome "AppData\Local\mingw64\mingw64\bin"

# --- Environment --------------------------------------------------------------
$prepend = @()
foreach ($d in @($MinGWDir, $CargoDir, $NodeDir)) {
    if (Test-Path -LiteralPath $d) { $prepend += $d }
}
if ($prepend.Count -gt 0) {
    $env:PATH = ($prepend -join ";") + ";" + $env:PATH
}

$env:RUSTUP_HOME = Join-Path $UserHome ".rustup"
$env:CARGO_HOME  = Join-Path $UserHome ".cargo"
. (Join-Path $ProjectRoot "scripts\Windows-RustEnv.ps1") -ProjectRoot $ProjectRoot

function Invoke-Step {
    param(
        [Parameter(Mandatory = $true)][string]$Label,
        [Parameter(Mandatory = $true)][scriptblock]$Action
    )

    Write-Host ""
    Write-Host "=== $Label ===" -ForegroundColor Cyan
    & $Action
    if ($LASTEXITCODE -ne 0) {
        throw "$Label failed with exit code $LASTEXITCODE"
    }
    Write-Host "$Label OK" -ForegroundColor Green
}

try {
    Set-Location $ProjectRoot

    Write-Host ""
    Write-Host "==========================================" -ForegroundColor Cyan
    Write-Host " Traffic Control 2D - Test Runner        " -ForegroundColor Cyan
    Write-Host "==========================================" -ForegroundColor Cyan

    if (-not $SkipFrontend) {
        Invoke-Step -Label "Frontend tests (vitest)" -Action {
            npm run test
        }
    } else {
        Write-Host ""
        Write-Host "Skipping frontend tests." -ForegroundColor Yellow
    }

    if (-not $SkipBackend) {
        Invoke-Step -Label "Backend tests (cargo test)" -Action {
            cargo test --manifest-path ".\src-tauri\Cargo.toml"
        }
    } else {
        Write-Host ""
        Write-Host "Skipping backend tests." -ForegroundColor Yellow
    }

    Write-Host ""
    Write-Host "All selected tests passed." -ForegroundColor Green
    exit 0
}
catch {
    Write-Host ""
    Write-Host "Test run failed: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
finally {
    Set-Location $OriginalLocation
}
