# Bolt's Performance Journal

## 2025-03-05 - [Vehicle Road Group Lookup & Hoisting]
**Learning:** Checking vehicle visibility via a spatial line segment distance search `resolveVehicleGroup` (O(E) complexity) is extremely slow and causes severe frame rate drops when rendering thousands of vehicles. Since the simulation sends `currentLaneId` in v3 packages, we can pre-compute a mapping from lane ID to its road group (O(1) complexity) using lane node OSM IDs and edge road types.
Additionally, calling MapLibre's `map.getBounds()` and coordinates parsing methods inside high-frequency per-vehicle rendering loops wastes precious CPU cycles. Hoisting these boundaries and avoiding coordinate array allocations significantly optimizes rendering.
**Action:** Implement `laneGroupCache` in `VehicleRenderer.ts` populated in `setEdgeIndex`, fall back to spatial lookup only for junction connector lanes, hoist bounds query outside rendering loops, and pass `LngLatLike` objects directly to `map.project()`.
