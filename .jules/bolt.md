# Bolt's Journal

## 2025-02-15 - Initial Journal Setup
**Learning:** Initial setup of Bolt's Journal to capture critical lessons, performance bottlenecks, and decisions.
**Action:** Keep the journal updated with any insights or surprising results during optimization.

## 2025-02-15 - O(1) Road-group Lookup & Redundant Bounds Call Caching
**Learning:** In high-frequency rendering loops (e.g. 60 FPS in `VehicleRenderer.ts` rendering thousands of vehicles), calling map bounds getters (`bounds.getWest()`, etc.) inside the loop incurs significant CPU overhead (thousands of redundant calls per frame). Additionally, performing O(E) spatial segment distance checks for road-group filtering heavily slows down performance.
**Action:** Always hoist map bounds getters outside rendering loops. Pre-populate a static O(1) road-group cache mapped by `currentLaneId` (derived from edge node pairs) during `setEdgeIndex`, falling back to O(E) spatial lookups only when the lane cache lookup misses.
