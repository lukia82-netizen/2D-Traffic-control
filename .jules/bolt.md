## 2025-05-20 - Viewport AABB Culling with WeakMap Caching in BuildingRenderer

**Learning:** In MapLibre + PixiJS canvas overlays, re-projecting and drawing off-screen building footprints on every camera movement creates massive CPU overhead. Caching pre-computed polygon AABBs with a `WeakMap` and using a 20% padded viewport check (with antimeridian safety handling) avoids unnecessary coordinate projection and graphics rendering.

**Action:** Always apply AABB viewport bounds culling with appropriate padding and antimeridian checks when rendering spatial entities (buildings, roads, traffic lights) on MapLibre overlays.
