## 2025-05-18 - Viewport Bounds Culling for Building Footprints
**Learning:** `BuildingRenderer` was projecting every vertex of every building polygon in the map dataset via `projectPoint` on every camera movement event, even when 80-95% of buildings were outside the visible viewport.
**Action:** Use a `WeakMap` to cache AABB min/max geographic bounds for building polygons, and filter buildings against viewport bounds (with a 20% margin) prior to vertex projection and PixiJS path drawing.
