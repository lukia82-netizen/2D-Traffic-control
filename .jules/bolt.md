## 2025-01-24 - O(1) Road-Group Filtering and Loop Hoisting
**Learning:** High-frequency rendering loops (60 FPS) in large-scale simulations are heavily impacted by spatial searches and object allocations. In this codebase, filtering vehicles by road group was $O(E)$ (Edges), which became a bottleneck as the map size grew. Additionally, calling MapLibre API methods like `getBounds()` and allocating `[lng, lat]` arrays for projection added significant CPU overhead and GC pressure.
**Action:**
1. Implement O(1) lookups for entity properties (like road groups) by pre-computing mappings (e.g., `laneId` to `roadGroup`) during static map data loads.
2. Hoist constant values (like `pxPerMetre`) and viewport bounds properties (`west`, `east`, etc.) out of per-entity loops.
3. Minimize temporary object allocations in hot paths by passing existing state objects to projection methods via type casting (e.g., `s as LngLatLike`) instead of creating new coordinate arrays.
