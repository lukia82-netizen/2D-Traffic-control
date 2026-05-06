use crate::map::road_network::{LaneDirection, MapData, RoadEdge};
use crate::vehicles::vehicle::Vehicle;
use petgraph::graph::EdgeIndex;

// ── Turn-direction helpers ────────────────────────────────────────────────────

/// Compute compass bearing (degrees, 0 = north, clockwise) from point 1 to point 2.
fn bearing_deg(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f32 {
    let dlat = (lat2 - lat1) * 111_320.0;
    let dlng = (lng2 - lng1) * 71_700.0;
    (dlng as f32).atan2(dlat as f32).to_degrees()
}

/// Signed angle difference `to – from` mapped into (−180, +180].
fn angle_diff(from_deg: f32, to_deg: f32) -> f32 {
    let mut diff = to_deg - from_deg;
    while diff >  180.0 { diff -= 360.0; }
    while diff < -180.0 { diff += 360.0; }
    diff
}

/// Return the turn direction a vehicle will make when transitioning from
/// `current_edge` to `next_edge`, based on their compass bearings.
///
/// Thresholds (degrees):
/// - angle_diff < −25 → Left
/// - angle_diff > +25 → Right
/// - otherwise        → Straight
pub fn turn_direction_between(
    map: &MapData,
    current_edge: EdgeIndex,
    next_edge: EdgeIndex,
) -> LaneDirection {
    let (cur_from, cur_to) = match map.graph.edge_endpoints(current_edge) {
        Some(e) => e,
        None    => return LaneDirection::Straight,
    };
    let (next_from, next_to) = match map.graph.edge_endpoints(next_edge) {
        Some(e) => e,
        None    => return LaneDirection::Straight,
    };

    let cur_bearing = bearing_deg(
        map.graph[cur_from].lat,
        map.graph[cur_from].lng,
        map.graph[cur_to].lat,
        map.graph[cur_to].lng,
    );
    let next_bearing = bearing_deg(
        map.graph[next_from].lat,
        map.graph[next_from].lng,
        map.graph[next_to].lat,
        map.graph[next_to].lng,
    );

    let diff = angle_diff(cur_bearing, next_bearing);
    if diff < -25.0 {
        LaneDirection::Left
    } else if diff > 25.0 {
        LaneDirection::Right
    } else {
        LaneDirection::Straight
    }
}

/// Return the best (0-indexed) lane for a vehicle that intends to turn `dir`
/// at the end of `edge`.
///
/// Strategy:
/// 1. Find an exact match in `edge.lane_directions`.
/// 2. Accept a Straight lane as a fallback for any direction.
/// 3. If nothing matches, return the middle lane.
pub fn pick_target_lane_for_direction(edge: &RoadEdge, dir: &LaneDirection) -> u8 {
    let dirs = &edge.lane_directions;
    if dirs.is_empty() {
        return 0;
    }

    // Exact match
    for (i, d) in dirs.iter().enumerate() {
        if std::mem::discriminant(d) == std::mem::discriminant(dir) {
            return i as u8;
        }
    }

    // Straight is valid for turning vehicles that don't have a dedicated lane
    for (i, d) in dirs.iter().enumerate() {
        if matches!(d, LaneDirection::Straight) {
            return i as u8;
        }
    }

    // Fallback: middle lane
    ((dirs.len() - 1) / 2) as u8
}

/// Compute which lane a vehicle should target on its current edge, based on
/// which direction it will need to turn at the end.
///
/// Looks one step ahead in the route (current_edge → next_edge) to determine
/// the intended turn direction, then selects the appropriate lane.
pub fn compute_vehicle_target_lane(vehicle: &Vehicle, map: &MapData) -> u8 {
    let route_pos = vehicle.route_pos;
    if route_pos >= vehicle.route.len() {
        return vehicle.current_lane;
    }

    let current_edge_idx = vehicle.route[route_pos];
    let current_edge = match map.graph.edge_weight(current_edge_idx) {
        Some(e) => e,
        None    => return vehicle.current_lane,
    };

    // If there's no next edge, aim for a straight lane
    let needed_dir = if route_pos + 1 < vehicle.route.len() {
        let next_edge_idx = vehicle.route[route_pos + 1];
        turn_direction_between(map, current_edge_idx, next_edge_idx)
    } else {
        LaneDirection::Straight
    };

    // Pick the lane, but skip tram lanes when possible
    pick_target_lane_for_direction_no_tram(current_edge, &needed_dir)
}

/// Like `pick_target_lane_for_direction` but additionally avoids the tram lane
/// on edges that share a tram track (tram lane = leftmost lane by convention).
fn pick_target_lane_for_direction_no_tram(edge: &RoadEdge, dir: &LaneDirection) -> u8 {
    let tram_lane = if edge.has_tram_track && edge.lanes > 1 {
        Some(tram_lane_index(edge))
    } else {
        None
    };

    let dirs = &edge.lane_directions;
    if dirs.is_empty() {
        return 0;
    }

    // Exact match – skip tram lane
    for (i, d) in dirs.iter().enumerate() {
        if std::mem::discriminant(d) == std::mem::discriminant(dir) {
            if Some(i as u8) != tram_lane {
                return i as u8;
            }
        }
    }

    // Straight fallback – skip tram lane
    for (i, d) in dirs.iter().enumerate() {
        if matches!(d, LaneDirection::Straight) {
            if Some(i as u8) != tram_lane {
                return i as u8;
            }
        }
    }

    // Any non-tram lane
    for i in 0..dirs.len() {
        if Some(i as u8) != tram_lane {
            return i as u8;
        }
    }

    // Last resort: middle lane (even if it's the tram lane)
    ((dirs.len() - 1) / 2) as u8
}

/// The tram lane index on a shared road edge.
/// By convention the leftmost lane (index 0) is treated as the tram lane when
/// the edge is going in the same direction as traffic (e.g. dedicated tram lane
/// on the left side of the carriageway in Polish cities).
///
/// For now we always return `0`; this can be refined with OSM `lanes:tram` data.
pub fn tram_lane_index(edge: &RoadEdge) -> u8 {
    let _ = edge;
    0
}

// ── Main lane-change decision ─────────────────────────────────────────────────

/// Decide whether a vehicle should change lanes on the current edge.
///
/// Returns `Some(new_lane)` if a lane change is warranted and safe, otherwise `None`.
pub fn decide_lane_change(
    vehicle: &Vehicle,
    edge: &RoadEdge,
    vehicles_on_edge: &[&Vehicle],
) -> Option<u8> {
    if vehicle.lane_change_cooldown > 0.0 {
        return None;
    }

    let lanes = edge.lanes as usize;
    if lanes <= 1 {
        return None;
    }

    let progress    = vehicle.edge_progress;
    let current_lane = vehicle.current_lane as usize;
    let target_lane  = vehicle.target_lane as usize;

    // Determine whether we are at a decision point (25 / 50 / 75 %).
    let at_decision_point = is_at_decision_point(progress, &edge.decision_points, edge.length_m);
    let past_last_decision = progress >= 0.75;

    if !at_decision_point && !past_last_decision {
        return None;
    }

    // Tram lane is off-limits for regular vehicles.
    let forbidden = if edge.has_tram_track && lanes > 1 {
        Some(tram_lane_index(edge) as usize)
    } else {
        None
    };

    // At 75 % and beyond: must commit to target lane.
    if past_last_decision && current_lane != target_lane {
        let new_lane = target_lane as u8;
        if forbidden.map_or(true, |f| f != target_lane)
            && is_safe_gap(vehicle, new_lane, vehicles_on_edge)
        {
            return Some(new_lane);
        }
        // Force at 90 %+ as a last resort (even if gap is not ideal).
        if progress >= 0.90 && forbidden.map_or(true, |f| f != target_lane) {
            return Some(new_lane);
        }
        return None;
    }

    // At earlier decision points: move opportunistically toward target lane.
    if at_decision_point && current_lane != target_lane {
        let step_lane = if target_lane > current_lane {
            (current_lane + 1) as u8
        } else {
            (current_lane.saturating_sub(1)) as u8
        };

        if forbidden.map_or(true, |f| f != step_lane as usize)
            && is_safe_gap(vehicle, step_lane, vehicles_on_edge)
        {
            return Some(step_lane);
        }
    }

    // Load-balance: prefer less-occupied adjacent lane.
    if at_decision_point {
        let counts = lane_occupancy(lanes, vehicles_on_edge);
        let least  = least_occupied_lane(&counts, current_lane, lanes);
        if least != current_lane
            && forbidden.map_or(true, |f| f != least)
            && is_safe_gap(vehicle, least as u8, vehicles_on_edge)
        {
            return Some(least as u8);
        }
    }

    None
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn is_at_decision_point(progress: f32, decision_points: &[f32; 3], length_m: f32) -> bool {
    if length_m <= 0.0 {
        return false;
    }
    for &dp_m in decision_points.iter() {
        let dp_frac   = dp_m / length_m;
        let half_win  = 0.02; // ±2 % of edge length
        if (progress - dp_frac).abs() < half_win {
            return true;
        }
    }
    false
}

/// Check whether the gap in `new_lane` is safe for a lane change.
/// Safe gap criterion: gap > speed × T_lc (T_lc = 2.0 s)
fn is_safe_gap(vehicle: &Vehicle, new_lane: u8, others: &[&Vehicle]) -> bool {
    const T_LC: f32 = 2.0;
    let required_gap_m = vehicle.speed * T_LC;

    for &other in others {
        if other.id == vehicle.id {
            continue;
        }
        if other.current_lane != new_lane {
            continue;
        }

        let lat_diff = (vehicle.lat - other.lat).abs();
        let lng_diff = (vehicle.lng - other.lng).abs();
        let dist_m = ((lat_diff * 111_320.0).powi(2)
            + (lng_diff * 71_700.0).powi(2))
            .sqrt() as f32;

        if dist_m < required_gap_m {
            return false;
        }
    }

    true
}

fn lane_occupancy(lanes: usize, vehicles: &[&Vehicle]) -> Vec<usize> {
    let mut counts = vec![0usize; lanes];
    for v in vehicles {
        let lane = (v.current_lane as usize).min(lanes - 1);
        counts[lane] += 1;
    }
    counts
}

fn least_occupied_lane(counts: &[usize], current: usize, lanes: usize) -> usize {
    let mut best       = current;
    let mut best_count = counts[current];

    for candidate in [current.saturating_sub(1), (current + 1).min(lanes - 1)] {
        if candidate != current && counts[candidate] < best_count {
            best       = candidate;
            best_count = counts[candidate];
        }
    }

    best
}
