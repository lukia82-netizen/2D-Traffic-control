## 2025-02-23 - Viewport Bounds Culling for Traffic Light Approaches
**Learning:** `TrafficLightRenderer.rebuild()` iterates over all intersection approaches, projecting geographic coordinates and instantiating individual PixiJS `Graphics` objects regardless of whether the approach is within the view. On large map areas or zoomed-in views, projecting off-screen approaches adds unnecessary CPU overhead.
**Action:** Extract map viewport bounds with a 10% padding margin before iterating through approaches and skip those that lie completely outside the padded bounds.
