## 2025-02-22 - Viewport AABB Culling and WeakMap Caching in BuildingRenderer
**Learning:** In maps with hundreds of building footprints, projecting every polygon vertex via MapLibre's `projectPoint` on every camera movement creates significant CPU overhead even when buildings are far off-screen.
**Action:** Pre-calculate AABB bounding boxes (`minLat`, `maxLat`, `minLng`, `maxLng`) cached via `WeakMap` for polygon geometries, and filter off-screen buildings against `map.getBounds()` (with a 20% safety margin) before vertex projection.
