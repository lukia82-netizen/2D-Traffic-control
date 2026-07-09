## 2025-05-15 - Optimize vehicle road-group filtering with O(1) lane lookup
**Learning:** In simulations with high vehicle counts, per-frame spatial searches for visibility filtering are a significant bottleneck. Leveraging backend-provided identifiers (like `laneId`) to perform O(1) lookups via a pre-computed cache can dramatically reduce CPU overhead in the rendering loop.
**Action:** Always check if backend entities provide unique IDs that can be correlated with map metadata during initialization to avoid expensive spatial computations in hot paths.
