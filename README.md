# 2D-Traffic-control-

<<<<<<< cursor/osm2streets-citymap-e3e7
## Cloud agent environment bootstrap (Tauri + osm2streets)

Run this once in fresh Linux cloud agents before building `src-tauri`:

```bash
bash scripts/cloud-agent-env-setup.sh
```

The script installs GTK3/GDK and `pkg-config` requirements for Tauri, ensures Rust 1.88
toolchain components, and prefetches Cargo dependencies (including git deps like
`osm2streets` and `streets_reader`).
=======
## Cloud build environment setup

For Linux cloud agents, run the setup script to install frontend dependencies and
the GTK3/pkg-config stack required by Tauri:

```bash
bash scripts/setup-cloud-env.sh
```

Then validate builds with:

```bash
npm run build
cargo check --manifest-path src-tauri/Cargo.toml
```
>>>>>>> main
