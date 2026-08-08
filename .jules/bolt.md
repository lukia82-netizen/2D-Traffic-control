## 2025-03-04 - [O(1) Road Group Lookups]
**Learning:** Performing spatial ptSegDist2 queries on every frame per vehicle to filter road groups is extremely CPU-intensive, scaling O(V * E) with active vehicles and edges. Associating road groups to lane IDs during initialization and caching them allows O(1) road group filtering for on-screen vehicles.
**Action:** Always prefer caching topological and static identifiers (e.g., mapping lane IDs to road groups during initial map loading) to bypass high-frequency spatial searches in rendering loops.
