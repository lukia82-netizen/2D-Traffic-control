# Bolt's Performance Journal

## 2025-02-14 - Vehicle Rendering Loop Optimization
**Learning:** High-frequency rendering loops (running at ~60 FPS with thousands of vehicles) suffer severe CPU bottlenecks and GC pressure when repeatedly querying MapLibre boundary getters (`bounds.getWest()`, etc.) and constructing temporary coordinate arrays (`[lng, lat]`) for map projections. Furthermore, performing O(E) spatial lookups to match lane IDs to edge road groups on every frame per vehicle is highly inefficient when map lane/edge connections can be resolved once during initialization.
**Action:** Always hoist external API queries and constant calculations out of entity rendering loops. Leverage existing interface structures (like `SmoothState` containing `lng` and `lat`) to pass entities directly to projection APIs without temporary allocations. Precompute static relational mappings (such as lane-to-edge relationships via node OSM IDs) into an O(1) cache.
