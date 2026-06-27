## 2025-05-15 - Road-group spatial search bottleneck
**Learning:** In 'VehicleRenderer.ts', vehicles were performing O(E) spatial searches to determine their road group for filtering, even though the simulation backend provides 'currentLaneId'.
**Action:** Replace spatial searches with O(1) lookups using a pre-computed 'laneToGroup' map. Hoist expensive map bounds checks and avoid coordinate array allocations in high-frequency rendering loops.
