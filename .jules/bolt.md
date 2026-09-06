## 2025-05-18 - Viewport Bounding Box Culling in PixiJS Renderers
**Learning:** High-frequency rendering routines like `BuildingRenderer` and `RoadRenderer` project all mapData features using `map.project()` on camera movement regardless of zoom level. Pre-filtering features against padded viewport bounds (`[west, south, east, north]`) skips expensive projection calls and GPU command generation for off-screen features.
**Action:** Always apply viewport bounds checks before invoking MapLibre coordinate projections or PixiJS geometry generation in camera event listeners.
