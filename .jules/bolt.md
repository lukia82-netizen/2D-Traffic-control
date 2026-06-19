## 2025-05-15 - VehicleRenderer Optimization

**Learning:**
1. The `VehicleRenderer` was performing O(V*E) spatial searches (where V is vehicle count and E is edge count) for road-group filtering in every frame when vehicles moved. With thousands of vehicles and hundreds of edges, this is a significant bottleneck.
2. High GC pressure was caused by allocating thousands of temporary `SmoothState` objects and `[lng, lat]` arrays for `map.project` calls in every frame (60 FPS).
3. Redundant `map.getBounds()` and its getter calls inside rendering loops added unnecessary overhead.

**Action:**
1. Use `v.currentLaneId` (provided by backend) to perform O(1) road-group lookups via a pre-built `laneGroupCache`.
2. Hoist expensive API calls and constant values (like map bounds) outside of per-entity loops.
3. Inline vehicle state access and pass vehicle objects directly to `map.project` to avoid temporary object/array allocations, reducing GC pressure.
