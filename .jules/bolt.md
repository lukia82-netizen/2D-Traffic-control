# Bolt's Performance Journal ⚡

## 2025-05-15 - Initializing Bolt's Journal
**Learning:** The codebase uses a spatial search (O(E) where E is number of edges) in `VehicleRenderer.ts` to determine which road group a vehicle belongs to for filtering. However, `VehicleState` provides a `currentLaneId` which can be used for O(1) lookup if mapped during initialization.
**Action:** Always check for unique IDs in state objects that can be used to bypass spatial searches or expensive calculations.

## 2025-05-15 - Optimizing Road-Group Filtering
**Learning:** Mapping `laneId` to road `group` during `setEdgeIndex` allows for O(1) lookups in the rendering loop. This significantly reduces CPU overhead, especially when thousands of vehicles are being rendered and filtered every frame.
**Action:** Pre-compute associations between different data layers (e.g., lanes to edges/road types) during map loading to keep the hot path (frame updates) as lean as possible.
