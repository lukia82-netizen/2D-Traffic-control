# Bolt's Performance Optimization Journal

## 2026-07-28 - Vehicle Renderer Optimizations
**Learning:** High-frequency rendering loops (e.g. 60 FPS update loops) are highly sensitive to redundant function calls and garbage collection (GC) pressure. Specifically:
1. Calling MapLibre bounds getter methods (`bounds.getWest()`, etc.) inside per-vehicle loops adds massive overhead; hoisting these values outside loops reduces function call overhead.
2. Allocating short-lived coordinate arrays like `[s.lng, s.lat]` on every vehicle on every frame creates garbage collection pressure in the JS engine; passing compatible `{ lat, lng }` state objects directly to MapLibre's `project()` completely eliminates this allocation.
3. Performing O(E) spatial searches (point-to-segment distance calculations) on every vehicle to filter hidden road groups is extremely expensive; building a static `laneGroupCache` during map initialization enables O(1) road-group lookups per vehicle based on `currentLaneId`.

**Action:** Always hoist map bounds getters outside rendering loops, leverage native object properties of state objects when projecting coordinates to avoid array allocations, and pre-compute O(1) caches mapping static elements (like lanes/nodes) to their associated attributes during initialization.
