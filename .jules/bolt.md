## 2026-08-09 - Viewport culling on auxiliary renderers
**Learning:** For large maps and high zoom levels, drawing static road overlays, traffic light structures, zebra markings, stop lines, and buildings causes high rendering overhead when projected off-screen. Adding O(1) bounding box viewport culling with 10% safety margin avoids coordinate projections and PixiJS shape allocations for off-screen elements.
**Action:** Always implement coordinate-based viewport culling with proper safety margins for large-scale static map objects before performing screen space projects.
