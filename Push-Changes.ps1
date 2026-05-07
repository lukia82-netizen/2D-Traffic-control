#Requires -Version 5.1
<#
  Push-Changes.ps1
  Commits local changes (if any) and pushes to origin.

  Usage:
    .\Push-Changes.ps1 -Message "Fix traffic light logic"
    .\Push-Changes.ps1 -Branch main -Message "Update scripts"
    .\Push-Changes.ps1 -WhatIf
#>

param(
    [string]$Branch = "main",
    [string]$Message = "",
    [switch]$WhatIf
)

$ErrorActionPreference = "Stop"

function Run-Git {
    param([Parameter(Mandatory = $true)][string[]]$Args)
    if ($WhatIf) {
        Write-Host ("[WhatIf] git " + ($Args -join " ")) -ForegroundColor DarkGray
        return
    }
    & git @Args
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Args -join ' ') failed with exit code $LASTEXITCODE"
    }
}

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $projectRoot

Write-Host ""
Write-Host "=== Push Changes ===" -ForegroundColor Cyan
Write-Host "Path   : $projectRoot" -ForegroundColor DarkGray
Write-Host "Branch : $Branch" -ForegroundColor DarkGray
Write-Host ""

& git rev-parse --is-inside-work-tree 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: current folder is not a git repository." -ForegroundColor Red
    exit 1
}

# Ensure we are on target branch.
$currentBranch = (& git rev-parse --abbrev-ref HEAD).Trim()
if ($currentBranch -ne $Branch) {
    Write-Host "Switching branch: $currentBranch -> $Branch" -ForegroundColor Yellow
    Run-Git @("checkout", $Branch)
}

$status = (& git status --porcelain)
if ($status) {
    if ([string]::IsNullOrWhiteSpace($Message)) {
        $Message = "Update local changes"
    }
    Write-Host "Local changes detected -> add + commit" -ForegroundColor Yellow
    Run-Git @("add", "-A")
    Run-Git @("commit", "-m", $Message)
} else {
    Write-Host "No local changes to commit." -ForegroundColor Green
}

Write-Host "Pushing to origin/$Branch..." -ForegroundColor Yellow
Run-Git @("push", "origin", $Branch)

Write-Host ""
Write-Host "Done." -ForegroundColor Green
