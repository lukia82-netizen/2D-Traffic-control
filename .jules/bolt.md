## 2026-07-10 - [Vehicle Rendering Optimizations]
**Learning:** Hoisting map bounds and using O(1) lookups for lane-to-group mappings significantly reduces CPU overhead in high-frequency rendering loops. Avoiding temporary coordinate array allocations `[lng, lat]` when calling `map.project()` further reduces GC pressure.
**Action:** Always check for redundant API calls (like `map.getBounds()`) and object allocations inside per-entity rendering loops.
