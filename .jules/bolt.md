## 2025-05-18 - Hoist Map Bounds & Reuse Allocations in Vehicle Rendering
**Learning:** In high-frequency 60 FPS rendering loops, repeated calls to MapLibre bounds getters (`bounds.getWest()`, etc.), `Set` instantiations, and temporary array creations (`[s.lng, s.lat]`) generate unnecessary CPU overhead and GC pressure for thousands of vehicles.
**Action:** Hoist MapLibre bounds getters outside entity loops, reuse class-level data structures (e.g., `renderedIds: Set<number>`), and pass coordinate objects directly to `map.project(s)`.
