# Bolt's Journal - Performance Learnings

## 2025-03-05 - O(1) Road-Group Filtering via Lane-to-Edge Node Pair Caching
**Learning:** High-frequency rendering loops (running at 60 FPS in `VehicleRenderer.ts` for every active vehicle) should completely avoid O(E) spatial searches. Using the map's structural association (linking `LaneData` to `EdgeData` via node OSM ID pairs) allows pre-computing a static lane-to-road-group map (`laneGroupCache`). This allows looking up a vehicle's road group in O(1) time using `v.currentLaneId`, bypassing segment distance calculations entirely unless the lane ID is unknown.
**Action:** Always map static network properties (like lanes to edges or roads) on load/initialization into fast O(1) maps, and fallback to spatial searches only on dynamic or missing data paths.
