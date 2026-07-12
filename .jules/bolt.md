## 2025-05-14 - [O(1) Road Group Filtering]
**Learning:** The vehicle rendering loop was bottlenecked by O(E) spatial searches in `isVehicleHidden`. Even with spatial indexing, performing this check for every vehicle every frame is expensive. Since most vehicles are associated with a lane, we can use a pre-computed lane-to-road-group mapping.
**Action:** Always look for ways to transition from spatial queries to direct ID-based lookups in hot loops by caching associations during data load or initialization.
