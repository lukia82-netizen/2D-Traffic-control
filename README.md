# 2D-Traffic-control-

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