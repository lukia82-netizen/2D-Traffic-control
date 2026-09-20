# Bolt's Journal

## 2025-05-18 - Map bounds getter hoisting & direct LngLatLike object projection
**Learning:** Calling `bounds.getWest()`, `bounds.getEast()`, `bounds.getSouth()`, and `bounds.getNorth()` repeatedly inside per-entity loops (e.g. 1000+ vehicles rendered per frame) creates thousands of redundant method invocations per frame. Similarly, passing temporary arrays `[s.lng, s.lat]` into `map.project()` causes frequent garbage collection pressure.
**Action:** Always hoist map bounds getter results outside of entity loops, and pass objects with `lat` and `lng` directly into `map.project()` as they natively satisfy `maplibregl.LngLatLike`.
