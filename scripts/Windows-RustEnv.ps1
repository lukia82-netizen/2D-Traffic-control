# Dot-source from Start-*.ps1 after $env:RUSTUP_HOME is set, for example:
#   . (Join-Path $ProjectRoot "scripts\Windows-RustEnv.ps1") -ProjectRoot $ProjectRoot
#
# - Picks a MinGW-based Rust toolchain (no MSVC link.exe).
# - Prepends portable MinGW so GNU Rust can find dlltool.exe.
# - If the repo path contains spaces, sets CARGO_TARGET_DIR under %USERPROFILE%
#   (Tauri windres/gcc otherwise truncates the path at the space).
#
# Install toolchain if missing: rustup toolchain install 1.88-x86_64-pc-windows-gnu

param(
    [string]$ProjectRoot = ""
)

if ($env:OS -ne "Windows_NT") { return }

if ($ProjectRoot -and ($ProjectRoot -match '\s')) {
    $targetBase = Join-Path $env:USERPROFILE ".cargo-target-traffic-control"
    New-Item -ItemType Directory -Force -Path $targetBase | Out-Null
    $env:CARGO_TARGET_DIR = $targetBase
}

$mingwBin = Join-Path $env:USERPROFILE "AppData\Local\mingw64\mingw64\bin"
if (Test-Path -LiteralPath (Join-Path $mingwBin "dlltool.exe")) {
    if ($env:PATH -notlike "*$mingwBin*") {
        $env:PATH = "$mingwBin;$env:PATH"
    }
}

foreach ($tc in @("1.88-x86_64-pc-windows-gnu", "stable-x86_64-pc-windows-gnu")) {
    $rustcPath = Join-Path $env:RUSTUP_HOME "toolchains\$tc\bin\rustc.exe"
    if (Test-Path -LiteralPath $rustcPath) {
        $env:RUSTUP_TOOLCHAIN = $tc
        break
    }
}
