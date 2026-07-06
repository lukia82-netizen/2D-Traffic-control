
## 2025-05-15 - VehicleRenderer optimization
**Learning:** In high-frequency rendering loops (60 FPS), MapLibre API calls like `map.getBounds()` and `map.getCenter()` (via `pxPerMetre`) can add significant CPU overhead. Additionally, creating temporary coordinate arrays `[lng, lat]` for `map.project()` and temporary objects for smoothed state results in high GC pressure.
**Action:** Hoist MapLibre API calls outside per-entity loops. Pass entity objects directly to `map.project()` if they contain `lng` and `lat` properties to avoid allocations. In-line simple helper functions to avoid object creation. Use O(1) lookups (e.g. mapping `laneId` to `roadGroup`) to bypass O(E) spatial searches.
