## 2025-05-20 - Viewport culling in TrafficLightRenderer
**Learning:** `TrafficLightRenderer.rebuild()` runs on every camera movement event (`map.on('render')`) and was previously processing and projecting every approach across the map.
**Action:** Always apply viewport bounding box culling with coordinate padding in renderers before running projection routines (`projectPoint`).
