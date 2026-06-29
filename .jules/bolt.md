## 2026-06-29 - [O(1) Road-Group Filtering]
**Learning:** The spatial O(E) search for road-group filtering was a major CPU bottleneck in high-vehicle-count scenarios. The backend already provides `currentLaneId`, which can be used for O(1) lookups if mapped during map data ingestion.
**Action:** Always prefer graph-based or ID-based lookups over spatial searches in the rendering hot path. Pre-calculate associations between lanes/nodes and high-level metadata (like road groups) once per map load.
