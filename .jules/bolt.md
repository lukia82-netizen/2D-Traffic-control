## 2026-03-31 - Hoisting Map Bounds and Direct Projection in VehicleRenderer

**Learning:** In high-frequency per-frame rendering loops over hundreds/thousands of entities, calling `map.getBounds()` and its accessors (`getWest()`, `getEast()`, etc.) inside the loop introduces significant redundant function call overhead. Additionally, passing `[s.lng, s.lat]` allocates temporary 2-element array instances every frame for every vehicle, creating GC pressure. Since `s` already contains `lng` and `lat` properties, `map.project(s)` can be called directly.

**Action:** Always hoist viewport bounds (`west`, `east`, `south`, `north`) outside per-entity rendering loops and pass existing `{ lng, lat }` objects directly to `map.project()` without wrapping them in temporary arrays.
