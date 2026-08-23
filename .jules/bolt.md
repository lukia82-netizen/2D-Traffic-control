## 2025-05-18 - Viewport Culling for Building Footprints

**Learning:** Building footprint rendering in `BuildingRenderer.ts` rebuilds geometry on every camera pan/zoom. Projecting vertices for off-screen buildings and allocating PixiJS Graphics paths for them creates unnecessary CPU overhead during viewport navigation.

**Action:** Apply axis-aligned bounding box (AABB) culling with a 10% viewport margin before coordinate projection and PixiJS graphics allocation in building rendering loops.
