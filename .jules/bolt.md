## 2025-07-15 - [Environment] Lockfile Conflict
**Learning:** The project uses `package-lock.json` but `pnpm` is available and used for builds. Running `pnpm install` generates a `pnpm-lock.yaml` which must not be committed to avoid inconsistent lockfiles and PR pollution.
**Action:** Always delete `pnpm-lock.yaml` before submitting a PR if the repository already contains a `package-lock.json`.

## 2025-07-15 - [Performance] O(1) Road Group Lookups
**Learning:** Vehicles in this simulation provide a `currentLaneId`. This can be mapped to a road group (e.g., motorway, residential) during map initialization using node OSM IDs to link lanes to edges. This allows O(1) road-group filtering during rendering, bypassing O(Edges) spatial searches.
**Action:** Use `currentLaneId` and pre-populated caches for per-frame entity property lookups instead of spatial calculations when possible.
