# Bolt's Journal - Critical Learnings Only

## 2025-02-18 - [VehicleRenderer spatial query bottleneck]
**Learning:** Performing spatial searches per-vehicle on every frame is highly inefficient when vehicle's current lane ID is known. Using `laneGroupCache` for O(1) road group lookup instead of O(E) segment distance calculations dramatically reduces CPU time.
**Action:** Always pre-compute static map structure relationships like lane ID to edge or road group, and look them up using fast O(1) mappings during runtime.
