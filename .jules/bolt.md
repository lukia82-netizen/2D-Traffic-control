## 2026-10-25 - Precomputed Bounding Box Culling in BuildingRenderer

**Learning:** Polygon renderers like `BuildingRenderer` without viewport culling perform thousands of coordinate projections (`map.project`) and Pixi geometry creations per camera move for off-screen entities. Precomputing geographic bounding boxes (`minLat`, `maxLat`, `minLng`, `maxLng`) on data load enables fast O(1) viewport bounding box checks with a 10% padding margin before attempting projection.

**Action:** Always precompute bounding boxes for multi-vertex vector geometries (polygons, polylines) on map data ingestion and check against map viewport bounds before projecting vertices.
