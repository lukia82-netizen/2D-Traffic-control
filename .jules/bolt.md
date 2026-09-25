## 2025-05-18 - AABB Caching and Viewport Bounds Culling in BuildingRenderer
**Learning:** `BuildingRenderer` was projecting every vertex of every building footprint in `mapData.buildings` on every camera move via MapLibre's `projectPoint`. Pre-computing building AABBs and checking against padded viewport bounds skips vertex projections and PixiJS polygon draws for all off-screen buildings.
**Action:** Always pre-compute spatial bounding boxes for complex geometric structures (like building footprints) and check against expanded map viewport bounds before projecting vertices.
