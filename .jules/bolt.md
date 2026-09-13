## 2025-02-23 - Viewport Spatial Culling for Polygon Renderers

**Learning:** Footprint/polygon renderers (like `BuildingRenderer`) re-project all vertices on camera movements (`render` events). Calculating AABB min/max coordinates once per polygon and checking against padded viewport bounds skips vertex coordinate projections (`map.project`) and graphics drawing calls for off-screen elements. A generous 50% padding prevents popping artifacts during fast panning.

**Action:** Always precalculate polygon AABBs when dataset loads and perform spatial culling with ample padding in camera re-render loops.
