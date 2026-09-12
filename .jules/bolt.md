## 2025-05-14 - Hoisting Map Bounds Getters in High-Frequency Vehicle Rendering Loops
**Learning:** Calling `bounds.getWest()`, `bounds.getEast()`, `bounds.getSouth()`, and `bounds.getNorth()` inside per-vehicle iteration loops executed every frame (60 FPS) introduces thousands of unnecessary method invocations per frame. Hoisting these getters to extract scalar bounding values once per frame before vehicle iteration saves CPU cycles.
**Action:** Always hoist bounds getters (`bounds.getWest()`, etc.) outside per-entity rendering loops in high-frequency renderers.
