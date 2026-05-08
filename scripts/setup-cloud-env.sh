#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

echo "[setup] Installing system packages required for Tauri GTK builds..."
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  curl \
  file \
  libayatana-appindicator3-dev \
  libgdk-pixbuf-2.0-dev \
  libglib2.0-dev \
  libgtk-3-dev \
  librsvg2-dev \
  libsoup-3.0-dev \
  libssl-dev \
  libwebkit2gtk-4.1-dev \
  patchelf \
  pkg-config \
  wget

echo "[setup] Installing Node dependencies..."
npm ci

if command -v rustup >/dev/null 2>&1; then
  echo "[setup] Updating Rust stable toolchain..."
  rustup toolchain install stable --profile minimal
  rustup default stable
else
  echo "[setup] rustup was not found; install Rust from https://rustup.rs."
  exit 1
fi

echo "[setup] Environment setup complete."
