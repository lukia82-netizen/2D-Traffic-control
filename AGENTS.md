# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

Traffic Control 2D is a Tauri v2 desktop app (Rust backend + TypeScript/Vite frontend) that simulates traffic flow on real-world OpenStreetMap road networks.

### Running the app

1. Start Vite dev server: `node start-dev.mjs` (port 1420). Use this instead of `npm run dev` — the `start-dev.mjs` script avoids config-file watching issues.
2. Start Tauri: `npx tauri dev --no-watch` (compiles the Rust backend and opens the WebView window).
3. The Tauri window requires a display server (`DISPLAY=:1` is available in the Cloud VM).

### Lint & test commands

| Check | Command | Notes |
|---|---|---|
| Rust lint | `cd src-tauri && cargo clippy` | Has warnings (not errors); pass `-- -D warnings` to treat warnings as errors |
| Rust tests | `cd src-tauri && cargo test` | 4 unit tests in `idm` and `day_cycle` modules |
| TypeScript typecheck | `npx tsc --noEmit` | Strict mode; no dedicated ESLint config |

### Gotchas

- **Icon file**: Tauri's `generate_context!()` macro requires `src-tauri/icons/icon.png` in RGBA format. The repo only ships `icon.ico`. Generate the PNG with: `convert src-tauri/icons/icon.ico -type TrueColorAlpha PNG32:src-tauri/icons/icon.png`
- **Rust toolchain**: The project's transitive dependency `time-macros` requires Rust edition 2024, so `rustc >= 1.85` is needed. Run `rustup default stable` if the default is pinned to an older version.
- **OpenSSL**: The `reqwest` crate needs `libssl-dev` and `pkg-config` on Ubuntu.
- **Tauri system deps** (Ubuntu): `libwebkit2gtk-4.1-dev`, `libappindicator3-dev`, `librsvg2-dev`, `libgtk-3-dev`, `libjavascriptcoregtk-4.1-dev`, `libsoup-3.0-dev`, `patchelf`, `libssl-dev`, `pkg-config`.
- **DRI3 warnings**: `libEGL warning: DRI3 error` messages in the Cloud VM are cosmetic and do not affect functionality.
- **No frontend tests**: There are no `*.test.ts` or `*.spec.ts` files in the codebase.
