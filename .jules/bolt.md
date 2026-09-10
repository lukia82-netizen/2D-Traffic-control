## 2025-05-18 - Viewport Spatial Culling for Building Polygon Re-renders
**Learning:** `BuildingRenderer.rebuildOnCameraChange` was re-projecting and re-drawing every building polygon in `mapData` on every camera movement (pan/zoom). Precomputing building AABBs once in `build()` allows skipping off-screen buildings during camera rebuilds.
**Action:** Precalculate AABBs for static polygon geometry during map loading to enable instant viewport bounding box culling in camera change listeners.
