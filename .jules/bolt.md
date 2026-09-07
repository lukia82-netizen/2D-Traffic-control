## 2025-05-20 - Hoisting map bounds getters and reusing Set allocations in VehicleRenderer
**Learning:** Calling `bounds.getWest()`, `bounds.getEast()`, `bounds.getSouth()`, and `bounds.getNorth()` inside high-frequency per-vehicle loops evaluates 4 method calls per vehicle every frame (60 FPS). In addition, allocating `new Set()` inside per-frame render routines triggers unnecessary garbage collection overhead.
**Action:** Always hoist map bounds getters outside per-entity iteration loops and reuse class-level data structures (e.g., clearing a persistent Set) for per-frame tracking.
