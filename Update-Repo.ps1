#Requires -Version 5.1
<#
  Update-Repo.ps1
  Fetches latest changes from remote and updates local branch with rebase.

  Usage:
    .\Update-Repo.ps1
    .\Update-Repo.ps1 -Branch main
    .\Update-Repo.ps1 -AutoStash
#>

param(
    [string]$Branch = "main",
    [switch]$AutoStash
)

$ErrorActionPreference = "Stop"

function Invoke-Git {
    param([Parameter(Mandatory = $true)][string]$Args)
    & git $Args
    if ($LASTEXITCODE -ne 0) {
        throw "git $Args failed with exit code $LASTEXITCODE"
    }
}

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $projectRoot

Write-Host ""
Write-Host "=== Repo update ===" -ForegroundColor Cyan
Write-Host "Path   : $projectRoot" -ForegroundColor DarkGray
Write-Host "Branch : $Branch" -ForegroundColor DarkGray
Write-Host ""

try {
    & git --version | Out-Null
} catch {
    Write-Host "ERROR: git is not available in PATH." -ForegroundColor Red
    exit 1
}

# Ensure this is a git working tree.
& git rev-parse --is-inside-work-tree 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: current folder is not a git repository." -ForegroundColor Red
    exit 1
}

$stashCreated = $false
try {
    $status = (& git status --porcelain)
    if ($status) {
        if (-not $AutoStash) {
            Write-Host "Local changes detected. Commit/stash first, or run with -AutoStash." -ForegroundColor Yellow
            exit 2
        }
        Write-Host "Local changes detected -> creating temporary stash..." -ForegroundColor Yellow
        & git stash push -m "auto-stash before Update-Repo.ps1" | Out-Null
        if ($LASTEXITCODE -ne 0) {
            throw "Could not create stash"
        }
        $stashCreated = $true
    }

    Write-Host "[1/3] Fetching from origin..." -ForegroundColor Yellow
    Invoke-Git "fetch origin"

    Write-Host "[2/3] Rebasing current branch on origin/$Branch..." -ForegroundColor Yellow
    Invoke-Git "pull --rebase origin $Branch"

    Write-Host "[3/3] Done. Current status:" -ForegroundColor Yellow
    & git status -sb

    Write-Host ""
    Write-Host "Repo is updated." -ForegroundColor Green
}
catch {
    Write-Host ""
    Write-Host "Update failed: $($_.Exception.Message)" -ForegroundColor Red
    if ($stashCreated) {
        Write-Host "Temporary stash was kept. Restore manually with: git stash pop" -ForegroundColor Yellow
    }
    exit 1
}

if ($stashCreated) {
    Write-Host ""
    Write-Host "Restoring temporary stash..." -ForegroundColor Yellow
    & git stash pop
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Stash restore needs manual resolution (conflicts are possible)." -ForegroundColor Yellow
        exit 3
    }
    Write-Host "Stash restored." -ForegroundColor Green
}
