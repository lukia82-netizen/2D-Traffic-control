# Bolt's Performance Journal

## 2025-02-23 - Viewport AABB Culling in BuildingRenderer
**Learning:** In MapLibre/PixiJS map applications, projecting all map features (e.g., thousands of building polygon vertices) on every camera event (`render` / pan / zoom) is a major CPU bottleneck, even when zoomed in and only a fraction of features are visible. Pre-computing geographic AABBs in WGS84 and caching them with a `WeakMap` allows fast O(1) viewport bounds checking before calling `projectPoint` or building PixiJS graphics geometry. Adding a 20% margin prevents visual popping during rotation/scaling.
**Action:** Always apply geographic bounding box culling for complex map overlays before performing screen projection or drawing operations.
