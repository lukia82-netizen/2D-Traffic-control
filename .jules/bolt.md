## 2025-05-15 - Rendering Optimization in VehicleRenderer
**Learning:** High-frequency rendering loops (60 FPS) are significantly impacted by micro-allocations (like `[lng, lat]` arrays) and redundant API calls (like `map.getBounds()`). TypeScript's structural typing allows casting entity state to `any` or `LngLatLike` to pass it directly to MapLibre's `project` method, eliminating array allocations.
**Action:** Always hoist bounds-checking and coordinate-scale calculations outside of entity loops. Avoid creating temporary objects or arrays within per-frame render methods.

**Learning:** Spatial searches (O(E)) for filtering are expensive when applied to thousands of entities. If the backend provides stable IDs (like `currentLaneId`), pre-computing a lookup table during map load converts this to O(1).
**Action:** Identify stable entity-to-infrastructure mappings and cache them at the class level during initialization or map-data updates.
