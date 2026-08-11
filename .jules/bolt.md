# Bolt's Journal

## 2026-05-11 - O(1) Road-Group Lookups via Lane IDs
**Learning:** Checking vehicle visibility by filtering road groups via spatial searching is an O(E) operation per vehicle per frame, which quickly scales to O(V * E) where V is the number of vehicles and E is the number of edges. By mapping lane IDs to road groups during map initialization, we can bypass spatial search with an O(1) cache lookup when lane-level data is available.
**Action:** Pre-compute lane-to-road-group mappings in `VehicleRenderer.setEdgeIndex` and use `VehicleState.currentLaneId` in `isVehicleHidden` for O(1) lookups. Fall back to spatial search only when lane-level data is missing.
