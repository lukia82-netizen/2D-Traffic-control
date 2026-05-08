#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TAURI_DIR="${REPO_ROOT}/src-tauri"

echo "[env-setup] Updating apt index..."
sudo apt-get update -y

echo "[env-setup] Installing GTK3 / GDK / Tauri native build dependencies..."
sudo apt-get install -y --no-install-recommends \
  build-essential \
  curl \
  file \
  git \
  pkg-config \
  libgtk-3-dev \
  libglib2.0-dev \
  libpango1.0-dev \
  libcairo2-dev \
  libatk1.0-dev \
  libgdk-pixbuf-2.0-dev \
  libsoup-3.0-dev \
  libwebkit2gtk-4.1-dev \
  libayatana-appindicator3-dev \
  librsvg2-dev \
  patchelf

echo "[env-setup] Ensuring Rust toolchain 1.88 with rustfmt + clippy..."
rustup toolchain install 1.88 --profile minimal
rustup component add rustfmt clippy --toolchain 1.88

echo "[env-setup] Prefetching Rust dependencies (including osm2streets/streets_reader git deps)..."
cd "${TAURI_DIR}"
cargo +1.88 fetch

echo "[env-setup] Done. Backend environment is ready."
