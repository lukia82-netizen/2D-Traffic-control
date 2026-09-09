## 2025-05-18 - Building Polygon Viewport Bounds Culling
**Learning:** Re-building building footprint polygons on every camera move (`rebuildOnCameraChange`) without spatial culling forces MapLibre to project every vertex of every building polygon across the entire map dataset, wasting CPU cycles and PixiJS vector drawing calls for off-screen entities.
**Action:** Pre-compute AABB bounding boxes (`[minLat, maxLat, minLng, maxLng]`) for polygon features during data loading and perform O(1) viewport bounds checks before coordinate projection.
