# Bolt's Journal - Critical Learnings Only

## 2025-02-25 - [O(1) Road-Group Lookup with Lane Group Cache]
**Learning:** Checking vehicle visibility by querying the road group using a spatial line distance calculation is an O(E) operation (where E is the number of edges) and is incredibly expensive when done on every frame for every vehicle. Since each vehicle state includes a `currentLaneId` mapped to map data's `LaneData`, we can precompute a mapping of `laneId` to road group (using fromNodeOsmId and toNodeOsmId to match with edges) during initial map load. This reduces the per-vehicle visibility check to an O(1) cache lookup in almost all cases.
**Action:** Implement `laneGroupCache` in `VehicleRenderer.ts` populated during `setEdgeIndex`, and query it inside `isVehicleHidden` before falling back to the line-distance spatial query.
