# Bolt's Journal - Critical Performance Lessons

## 2025-03-05 - Vehicle Road-Group O(1) Filtering and High-Frequency Render Loop Optimization
**Learning:** High-frequency rendering loops (running at 60 FPS in MapLibre/PixiJS) are extremely sensitive to per-frame memory allocations and redundant function calls. Specifically, doing an O(E) spatial lookup for every vehicle to determine hidden road groups wastes massive amounts of CPU cycles. Additionally, allocating temporary coordinate arrays like `[lng, lat]` for `map.project()` on every single vehicle frame creates significant garbage collection pressure.
**Action:** Pre-compute static associations (such as lane ID to road group) during map loading or edge setup to enable O(1) lookups during frame rendering. Hoist bounds getters and center calculations outside of high-frequency per-vehicle loops, and reuse data structures like Sets at the class level rather than allocating them repeatedly.
