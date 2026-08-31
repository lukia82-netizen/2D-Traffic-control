## 2025-08-31 - Persistent PIXI.Graphics for Congestion Overlays

**Learning:** Re-creating `PIXI.Graphics`, `PIXI.RenderTexture`, and `PIXI.Sprite` on every periodic update (~500ms) causes off-screen render passes and GPU texture allocation churn. Using a single persistent `PIXI.Graphics` attached directly to the layer container and calling `.clear()` on updates eliminates allocations and texture overhead completely.

**Action:** Prefer persistent `PIXI.Graphics` objects on PixiJS layers with `.clear()` over dynamic `RenderTexture` creation when vector shapes are updated periodically.
