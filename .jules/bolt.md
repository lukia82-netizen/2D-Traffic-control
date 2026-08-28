## 2026-05-17 - Persistent Graphics for Congestion Overlay
**Learning:** Re-creating temporary `PIXI.Graphics` instances and baking them into new `PIXI.RenderTexture`s on every update event (~500ms) creates unnecessary WebGL texture allocations, off-screen render passes, GPU memory churn, and GC pressure.
**Action:** For overlays with dynamic drawing commands (like congestion hotspots), render directly to a persistent `PIXI.Graphics` node in the container layer and call `gfx.clear()` before redrawing.
