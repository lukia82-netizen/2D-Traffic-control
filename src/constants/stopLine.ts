/**
 * Lateral stop line on TL/ped approaches — distance from the intersection node
 * along the edge (meters). Kept in sync with Rust `STOP_LINE_OFFSET_M` in
 * `src-tauri/src/simulation/sim_loop.rs`.
 */
export const STOP_LINE_OFFSET_M = 16;

/** Signal heads are drawn this many meters further back from the junction than the stop line. */
export const SIGNAL_HEAD_BACK_M = 5;
