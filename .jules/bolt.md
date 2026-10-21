## 2025-05-18 - Building Polygon Viewport Bounds Culling & AABB Caching

**Learning:** `BuildingRenderer` was projecting all building polygon vertices on every camera movement event (`rebuildOnCameraChange`), regardless of whether the buildings were within the visible screen viewport. Caching building polygon geographic AABBs in a `WeakMap` and filtering against padded viewport bounds avoids vertex projection (`projectPoint`) and PixiJS shape path creation for off-screen buildings.

**Action:** When drawing spatial data structures (buildings, roads, traffic lights) in MapLibre + PixiJS, compute geographic AABBs and apply viewport bounding box culling (with ~10-20% margin padding) before coordinate projections and graphics rendering.
