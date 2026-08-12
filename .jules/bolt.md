## 2025-02-12 - Optimization of high-frequency VehicleRenderer and O(1) road-group lookup

**Learning:**
Spatial segment searches (O(E) distance checks) in isVehicleHidden are highly inefficient when called for thousands of vehicles at 60 FPS. Leveraging the pre-parsed `currentLaneId` in the `VehicleState` payload allows for an O(1) road-group lookup by setting up a mapping from lane ID to road group at initialization. Furthermore, hoisting Map bounds getter methods (`bounds.getWest()`, `bounds.getEast()`, `bounds.getSouth()`, `bounds.getNorth()`) outside loops prevents thousands of redundant, expensive function calls per frame. Lastly, avoiding temporary coordinate array allocations `[s.lng, s.lat]` in rendering loops and replacing per-frame Set allocations with a class-level Set `renderedIds` greatly reduces Garbage Collection pressure and CPU usage.

**Action:**
1. Build robust O(1) caches mapping static entity IDs to properties (like lane ID to road group) when maps or environments are loaded/updated.
2. Hoist layout metrics, calculations, map bounds, and scale getters outside of high-frequency render or update loops.
3. Pass coordinate-compatible objects directly to projection libraries (such as MapLibre project) rather than allocating new arrays.
4. Reuse class-level containers or Sets to avoid memory churn and Garbage Collection overhead in game loop systems.
