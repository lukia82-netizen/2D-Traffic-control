# Bolt's Journal

## 2025-08-07 - Viewport Culling & MapBounds Hoisting in Rendering Loops
**Learning:** In MapLibre + PixiJS applications, projecting raw coordinate points (fromLng/fromLat) via map.project is highly CPU-bound. Off-screen elements like road segments, buildings, stop lines, and traffic lights should be culled prior to projection using simple O(1) mathematical bounding box overlap checks. Furthermore, invoking map.getBounds() getter methods (getWest, getEast, etc.) inside per-vehicle loops introduces substantial, redundant call stack overhead.
**Action:** Always compute padded bounding boxes once outside of any high-frequency loops. Compare coordinate ranges in raw degrees first before calling any projection helper methods or PixiJS geometry API paths.
