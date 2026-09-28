## 2026-09-28 - Viewport Bounds Culling for Building Footprints
**Learning:** `BuildingRenderer.ts` re-projects all vertices of every building polygon on every camera motion frame during map pan/zoom. Precomputing polygon AABBs once on load and culling off-screen buildings against padded viewport bounds avoids thousands of redundant MapLibre `project()` projections and PixiJS graphics drawing operations.
**Action:** Always precompute geographic AABBs for static polygon geometry and cull against current viewport bounds in high-frequency camera move listeners.
