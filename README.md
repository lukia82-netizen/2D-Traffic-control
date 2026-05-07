# 2D-Traffic-control-

## Cloud agent environment bootstrap (Tauri + osm2streets)

Run this once in fresh Linux cloud agents before building `src-tauri`:

```bash
bash scripts/cloud-agent-env-setup.sh
```

The script installs GTK3/GDK and `pkg-config` requirements for Tauri, ensures Rust 1.88
toolchain components, and prefetches Cargo dependencies (including git deps like
`osm2streets` and `streets_reader`).