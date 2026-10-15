## 2025-05-18 - Viewport Culling & Coordinate Reuse in RoadRenderer

**Learning:** In MapLibre/Pixi overlay renderers (like `RoadRenderer.ts`), calling `projectPoint` for every road edge on camera move incurs heavy CPU overhead when zoomed in or panned, as 80-90% of edges are off-screen. Additionally, calling `projectPoint` multiple times for the same geographic coordinates (e.g., endpoint nodes in Step 1 and junction circles in Step 2) redundantly doubles projection calculations per frame.

**Action:** Extract map geographic viewport bounds (`map.getBounds()`) with a 10% padding margin before looping through edges, perform line-segment bounding checks (`bothWest`, `bothEast`, `bothSouth`, `bothNorth`), and reuse already projected screen coordinates (`r.from`, `r.to`) in subsequent rendering passes.
