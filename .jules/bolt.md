## 2025-05-18 - Hoist Map Bounds & Direct Projection in Vehicle Rendering Loop
**Learning:** Calling MapLibre's `bounds.getWest()`, `bounds.getEast()`, `bounds.getSouth()`, and `bounds.getNorth()` inside high-frequency per-vehicle loops causes hundreds/thousands of redundant function invocations per frame at 60 FPS. Also, passing `[s.lng, s.lat]` array literals creates thousands of temporary array allocations per frame, increasing GC pressure.
**Action:** Always hoist map bounds getters outside per-entity iteration loops and pass `s` (`{ lng, lat }`) directly into `map.project(s)`.
