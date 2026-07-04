## 2025-07-04 - [O(1) Road Group Lookup]
**Learning:** Performing spatial searches (O(E)) every frame to determine vehicle visibility is a major CPU bottleneck when vehicle counts are high. The backend provides `currentLaneId`, which can be mapped to road groups at load time.
**Action:** Implement `laneGroupCache` to allow O(1) lookups during the render loop.

## 2025-07-04 - [Loop Hoisting]
**Learning:** Frequent calls to `map.getBounds()` and its getters (`getWest()`, etc.) inside per-vehicle loops add significant overhead due to object property access and function call depth.
**Action:** Hoist viewport boundary values outside of `for` loops in rendering methods.
