use crate::vehicles::vehicle::Vehicle;
use crate::map::road_network::RoadEdge;

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

    let progress = vehicle.edge_progress;
    let current_lane = vehicle.current_lane as usize;
    let target_lane = vehicle.target_lane as usize;

    // Determine whether we are at a decision point
    let at_decision_point = is_at_decision_point(progress, &edge.decision_points, edge.length_m);
    let past_last_decision = progress >= 0.75;

    if !at_decision_point && !past_last_decision {
        return None;
    }

    // At 75% and beyond: must commit to target lane
    if past_last_decision && current_lane != target_lane {
        let new_lane = target_lane as u8;
        if is_safe_gap(vehicle, new_lane, vehicles_on_edge) {
            return Some(new_lane);
        }
        // Force it even if unsafe at 90%+ (last resort)
        if progress >= 0.90 {
            return Some(new_lane);
        }
        return None;
    }

    // At earlier decision points: opportunistic move toward target lane
    if at_decision_point && current_lane != target_lane {
        // Try to move one step towards the target lane
        let step_lane = if target_lane > current_lane {
            (current_lane + 1) as u8
        } else {
            (current_lane.saturating_sub(1)) as u8
        };

        if is_safe_gap(vehicle, step_lane, vehicles_on_edge) {
            return Some(step_lane);
        }
    }

    // Prefer less-occupied lane (load-balancing)
    if at_decision_point {
        let counts = lane_occupancy(lanes, vehicles_on_edge);
        let least = least_occupied_lane(&counts, current_lane, lanes);
        if least != current_lane && is_safe_gap(vehicle, least as u8, vehicles_on_edge) {
            return Some(least as u8);
        }
    }

    None
}

fn is_at_decision_point(progress: f32, decision_points: &[f32; 3], length_m: f32) -> bool {
    if length_m <= 0.0 {
        return false;
    }
    for &dp_m in decision_points.iter() {
        let dp_frac = dp_m / length_m;
        let half_window = 0.02; // ±2% of edge length
        if (progress - dp_frac).abs() < half_window {
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

        // Approximate longitudinal distance via edge progress difference
        let lat_diff = (vehicle.lat - other.lat).abs();
        let lng_diff = (vehicle.lng - other.lng).abs();
        // Quick Euclidean approximation in degrees → meters at 50°N
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
    let mut best = current;
    let mut best_count = counts[current];

    // Only consider adjacent lanes
    for candidate in [current.saturating_sub(1), (current + 1).min(lanes - 1)] {
        if candidate != current && counts[candidate] < best_count {
            best = candidate;
            best_count = counts[candidate];
        }
    }

    best
}
