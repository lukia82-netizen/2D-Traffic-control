# Bolt's Performance Optimization Journal

## 2025-02-14 - Vehicle Rendering Optimizations
**Learning:** High-frequency rendering loops (e.g. 60 FPS `VehicleRenderer.ts` processing thousands of vehicles) suffer heavily from redundant object allocations, function calls, and O(E) spatial searches.
1. Redundant object allocations can be completely eliminated by passing compatible entity structures (like `VehicleState` which has `lng` and `lat` properties) directly to MapLibre's `project()` method, bypassing the need to allocate intermediate `[lng, lat]` coordinate arrays on every frame per vehicle.
2. Hoisting expensive map bounds API getters (`bounds.getWest()`, etc.) outside the entity render loops drastically reduces per-frame call overhead.
3. Using a static `laneGroupCache` mapping lane IDs to road groups populated once during `setEdgeIndex` replaces O(E) spatial segment distance computations with an O(1) cache lookup for hiding/filtering road groups.

**Action:** When working in high-frequency/rendering code (like Canvas, PixiJS, or WebGL layers), always:
- Pre-cache and reuse Map/Set structures.
- Hoist method calls and getters outside loops.
- Pass existing object shapes directly to third-party APIs if compatible.
- Build static lookup mappings on data load to turn runtime searches into O(1) lookups.
