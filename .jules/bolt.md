# Bolt's Performance Journal

## 2025-02-23 - Hoisting Map Bounds and Coordinate Arrays in Hot Rendering Loops
**Learning:** In 60 FPS rendering loops, invoking `map.getBounds()` getters (`getWest()`, `getEast()`, `getSouth()`, `getNorth()`) and constructing temporary `[lng, lat]` arrays inside per-entity loops causes thousands of unnecessary method calls and GC allocations per frame.
**Action:** Always hoist scalar map bounds getters outside per-entity loops and pass position objects directly to `map.project()` when compatible.
