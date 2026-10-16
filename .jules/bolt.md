# Bolt's Journal - Performance Learnings

## 2025-05-18 - Hoisting Map Bounds Extraction and Direct Coordinate Projection in Vehicle Loop
**Learning:** In MapLibre GL + PixiJS rendering loops, calling `bounds.getWest()`/`getEast()`/`getSouth()`/`getNorth()` inside a per-vehicle loop causes thousands of redundant function evaluations per frame. Additionally, calling `map.project([lng, lat])` allocates a new 2-element array per vehicle every frame, causing unnecessary GC pressure under high vehicle counts (1,000+).
**Action:** Always hoist map bounding box numerical boundaries outside per-entity rendering loops and pass coordinate objects (`s` / `{ lng, lat }`) directly to `map.project(s)` without wrapping them in array literals `[lng, lat]`.
