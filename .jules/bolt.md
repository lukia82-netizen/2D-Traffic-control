## 2025-05-18 - BuildingRenderer Viewport Bounding Box Culling
**Learning:** `BuildingRenderer` was projecting and drawing 100% of building polygons on every camera move regardless of whether they were visible on screen. Adding cached AABB bounds culling with a 5% margin skips off-screen MapLibre `projectPoint` calls and PixiJS polygon draws.
**Action:** Always check high-frequency map overlay renderers for off-screen AABB viewport culling to avoid redundant projection calculations and GPU/draw call overhead.
