## 2026-07-09 - [O(1) Road Group Lookup]
**Learning:** In 'VehicleRenderer.ts', replace O(E) spatial searches for road-group filtering with O(1) lookups using a pre-computed cache (e.g., mapping lane IDs to road groups) to significantly reduce per-frame CPU usage.
**Action:** Always check if there's a unique identifier (like laneId) available for O(1) cache lookups in hot paths before falling back to expensive spatial searches.
