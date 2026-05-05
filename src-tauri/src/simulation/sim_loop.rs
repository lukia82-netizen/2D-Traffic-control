use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};
use parking_lot::RwLock;
use std::sync::Arc;
use rayon::prelude::*;
use tauri::ipc::Channel;
use base64::Engine;

use crate::map::road_network::MapData;
use crate::state::SimCommand;
use crate::time::game_clock::GameClock;
use crate::time::day_cycle::DayCycle;
use crate::traffic::intersection::IntersectionManager;
use crate::vehicles::vehicle::Vehicle;
use crate::simulation::idm::idm_acceleration;
use crate::simulation::spatial_grid::SpatialGrid;
use crate::simulation::spawn::SpawnSystem;
use crate::simulation::lane_change::decide_lane_change;
use crate::simulation::congestion::{compute_congestion, CongestionData};
use crate::traffic::traffic_light::LightStateUpdate;

const TARGET_TICK_S: f32 = 1.0 / 60.0; // 60 Hz
// Cell size ≈ 50 m at 50°N latitude
const GRID_CELL_DEG: f64 = 0.00045;
const CONGESTION_INTERVAL_S: f32 = 0.5;

pub fn run_simulation(
    graph_lock: Arc<RwLock<Option<MapData>>>,
    command_rx: Receiver<SimCommand>,
    vehicle_channel: Channel<String>,
    congestion_tx: Sender<Vec<CongestionData>>,
    light_state_tx: Sender<Vec<LightStateUpdate>>,
) {
    // --- Initialise subsystems ---
    let mut clock = GameClock::new();
    let mut vehicles: Vec<Vehicle> = Vec::with_capacity(512);
    let mut spatial_grid = SpatialGrid::new(GRID_CELL_DEG);
    let mut congestion_timer = 0.0f32;

    let (mut intersections, mut spawn_system) = {
        let guard = graph_lock.read();
        let map = guard.as_ref().expect("map must be loaded before starting simulation");
        let intersections = IntersectionManager::from_graph(&map.graph);
        let spawn = SpawnSystem::new(map.spawn_points.clone());
        (intersections, spawn)
    };

    let mut last_tick = Instant::now();

    log::info!("Simulation loop started");

    loop {
        // --- Tick timing ---
        let now = Instant::now();
        let real_dt_s = now.duration_since(last_tick).as_secs_f32();
        last_tick = now;

        // Clamp dt to avoid spiral-of-death after pauses/debugger
        let real_dt_s = real_dt_s.min(0.1);

        // --- Process incoming commands ---
        loop {
            match command_rx.try_recv() {
                Ok(cmd) => handle_command(cmd, &mut clock, &mut intersections),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    log::info!("Simulation command channel closed, stopping");
                    return;
                }
            }
        }

        if clock.paused {
            std::thread::sleep(Duration::from_millis(16));
            last_tick = Instant::now();
            continue;
        }

        let game_dt_s = clock.tick(real_dt_s);
        let game_hour = clock.game_hour();
        let spawn_multiplier = DayCycle::spawn_multiplier(game_hour);

        // --- Update traffic lights ---
        let light_updates = intersections.update(real_dt_s);
        if !light_updates.is_empty() {
            let _ = light_state_tx.send(light_updates);
        }

        // --- Spawn new vehicles ---
        {
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                let new_vehicles = spawn_system.tick(real_dt_s, spawn_multiplier, map, vehicles.len());
                vehicles.extend(new_vehicles);
            }
        }

        // --- Rebuild spatial grid ---
        spatial_grid.clear();
        for v in &vehicles {
            spatial_grid.insert(v.id, v.lat, v.lng);
        }

        // --- Parallel IDM acceleration computation ---
        // We compute the acceleration for each vehicle in parallel, then apply it.
        // We read vehicle positions from a snapshot to avoid data races.
        let accel_inputs: Vec<(f32, f32, f32, f32)> = {
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                vehicles
                    .par_iter()
                    .map(|v| {
                        let leader = find_leader(v, &vehicles, &spatial_grid);
                        let (gap, delta_v) = match leader {
                            Some(lid) => {
                                if let Some(leader_v) = vehicles.iter().find(|o| o.id == lid) {
                                    let dist = geo_dist_approx(v.lat, v.lng, leader_v.lat, leader_v.lng);
                                    let dv = v.speed - leader_v.speed;
                                    (dist, dv)
                                } else {
                                    (1000.0f32, 0.0f32)
                                }
                            }
                            None => (1000.0f32, 0.0f32),
                        };

                        // Desired speed is min(driver factor × vehicle max, road max speed)
                        let desired_speed = compute_desired_speed(v, map);

                        // Check traffic light: if red ahead, treat it as a stopped leader at the edge end
                        let (gap, delta_v) = apply_traffic_light_effect(v, gap, delta_v, &intersections, map);

                        let params = v.driver_profile.params();
                        let vtype = v.vehicle_type.params();
                        let a = idm_acceleration(v.speed, desired_speed, gap, delta_v, &params, &vtype);
                        (a, desired_speed, gap, delta_v)
                    })
                    .collect()
            } else {
                vec![(0.0, 0.0, 0.0, 0.0); vehicles.len()]
            }
        };

        // --- Apply physics & update positions ---
        {
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                for (i, vehicle) in vehicles.iter_mut().enumerate() {
                    let (accel, _desired, _gap, _dv) = accel_inputs[i];
                    apply_vehicle_physics(vehicle, accel, real_dt_s, game_dt_s, map, &intersections);
                }
            }
        }

        // --- Lane change decisions ---
        {
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                let vehicle_snapshot: Vec<&Vehicle> = vehicles.iter().collect();
                let lane_changes: Vec<(u32, u8)> = vehicles
                    .iter()
                    .filter_map(|v| {
                        if v.route_pos >= v.route.len() { return None; }
                        let edge_idx = v.route[v.route_pos];
                        let edge = map.graph.edge_weight(edge_idx)?;
                        let same_edge: Vec<&Vehicle> = vehicle_snapshot
                            .iter()
                            .filter(|o| o.route_pos < o.route.len() && o.route[o.route_pos] == edge_idx)
                            .copied()
                            .collect();
                        decide_lane_change(v, edge, &same_edge).map(|lane| (v.id, lane))
                    })
                    .collect();

                for (vid, new_lane) in lane_changes {
                    if let Some(v) = vehicles.iter_mut().find(|v| v.id == vid) {
                        v.current_lane = new_lane;
                        v.lane_change_cooldown = 3.0;
                    }
                }
            }
        }

        // --- Remove despawned vehicles ---
        vehicles.retain(|v| !v.despawned);

        // --- Serialize and send vehicle frame (skip if no vehicles yet) ---
        if !vehicles.is_empty() {
            let frame = serialize_vehicles(&vehicles);
            let encoded = base64::engine::general_purpose::STANDARD.encode(&frame);
            let _ = vehicle_channel.send(encoded);
        }

        // --- Congestion update every 500 ms ---
        congestion_timer += real_dt_s;
        if congestion_timer >= CONGESTION_INTERVAL_S {
            congestion_timer = 0.0;
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                let cdata = compute_congestion(&map.graph, &vehicles);
                let _ = congestion_tx.send(cdata);
            }
        }

        // --- Sleep to maintain target tick rate ---
        let elapsed = last_tick.elapsed().as_secs_f32();
        let remaining = TARGET_TICK_S - elapsed;
        if remaining > 0.001 {
            std::thread::sleep(Duration::from_secs_f32(remaining));
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn handle_command(cmd: SimCommand, clock: &mut GameClock, intersections: &mut IntersectionManager) {
    match cmd {
        SimCommand::Pause => clock.pause(),
        SimCommand::Resume => clock.resume(),
        SimCommand::SetTimeScale(s) => clock.set_time_scale(s),
        SimCommand::SetLightMode { intersection_id, mode } => {
            intersections.set_mode(intersection_id, mode);
        }
        SimCommand::SetLightPhase { intersection_id, phase } => {
            intersections.set_phase(intersection_id, phase);
        }
        SimCommand::Stop => {}
    }
}

/// Find the id of the vehicle immediately ahead of `ego` on the same edge/lane.
fn find_leader(ego: &Vehicle, vehicles: &[Vehicle], grid: &SpatialGrid) -> Option<u32> {
    if ego.route_pos >= ego.route.len() {
        return None;
    }
    let ego_edge = ego.route[ego.route_pos];

    let nearby = grid.query_nearby(ego.lat, ego.lng, 2);

    let mut best_id: Option<u32> = None;
    let mut best_dist = f32::MAX;

    for &id in &nearby {
        if id == ego.id {
            continue;
        }
        if let Some(other) = vehicles.iter().find(|v| v.id == id) {
            // Same edge and lane
            if other.route_pos < other.route.len()
                && other.route[other.route_pos] == ego_edge
                && other.current_lane == ego.current_lane
                && other.edge_progress > ego.edge_progress
            {
                let dist = geo_dist_approx(ego.lat, ego.lng, other.lat, other.lng);
                if dist < best_dist {
                    best_dist = dist;
                    best_id = Some(id);
                }
            }
        }
    }

    best_id
}

/// Fast geographic distance approximation in metres (accurate enough for short distances).
#[inline]
fn geo_dist_approx(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f32 {
    let dlat = (lat2 - lat1) * 111_320.0;
    let dlng = (lng2 - lng1) * 71_700.0; // at ~50°N
    ((dlat * dlat + dlng * dlng) as f32).sqrt()
}

fn compute_desired_speed(vehicle: &Vehicle, map: &MapData) -> f32 {
    if vehicle.route_pos >= vehicle.route.len() {
        return 0.0;
    }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let road_max = map
        .graph
        .edge_weight(edge_idx)
        .map(|e| e.max_speed)
        .unwrap_or(14.0);

    let vtype_max = vehicle.vehicle_type.params().max_speed;
    let driver_factor = vehicle.driver_profile.params().desired_speed_factor;

    (road_max * driver_factor).min(vtype_max)
}

/// If the vehicle is near the end of its current edge AND the target node has a red light,
/// inject a virtual stopped leader at the edge end.
fn apply_traffic_light_effect(
    vehicle: &Vehicle,
    gap: f32,
    delta_v: f32,
    intersections: &IntersectionManager,
    map: &MapData,
) -> (f32, f32) {
    if vehicle.route_pos >= vehicle.route.len() {
        return (gap, delta_v);
    }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = match map.graph.edge_weight(edge_idx) {
        Some(e) => e,
        None => return (gap, delta_v),
    };

    // Only apply brake effect when within the last 30 m of the edge
    let dist_to_end_m = edge.length_m * (1.0 - vehicle.edge_progress);
    if dist_to_end_m > 30.0 {
        return (gap, delta_v);
    }

    // Find the target node of this edge
    let edge_endpoints = map.graph.edge_endpoints(edge_idx);
    let target_node_idx = match edge_endpoints {
        Some((_, tgt)) => tgt,
        None => return (gap, delta_v),
    };
    let target_osm_id = map.graph[target_node_idx].osm_id;

    if !intersections.can_proceed(target_osm_id) {
        // Treat the stop line as a virtual stopped vehicle
        let virtual_gap = dist_to_end_m.max(0.1);
        let virtual_delta_v = vehicle.speed; // relative speed to a stopped obstacle
        let new_gap = virtual_gap.min(gap);
        let new_dv = if new_gap < gap { virtual_delta_v } else { delta_v };
        (new_gap, new_dv)
    } else {
        (gap, delta_v)
    }
}

fn apply_vehicle_physics(
    vehicle: &mut Vehicle,
    accel: f32,
    real_dt_s: f32,
    _game_dt_s: f32,
    map: &MapData,
    _intersections: &IntersectionManager,
) {
    // Update speed
    vehicle.accel = accel;
    vehicle.speed = (vehicle.speed + accel * real_dt_s).max(0.0);

    // Update satisfaction
    if vehicle.is_stopped() {
        vehicle.wait_time_real_s += real_dt_s;
        let threshold = vehicle.driver_profile.params().wait_threshold_real_s;
        if vehicle.wait_time_real_s > threshold {
            let decay = vehicle.driver_profile.params().frustration_decay_rate;
            vehicle.satisfaction = (vehicle.satisfaction - decay * real_dt_s).max(0.0);
        }
    } else {
        vehicle.wait_time_real_s = (vehicle.wait_time_real_s - real_dt_s).max(0.0);
        let recovery = vehicle.driver_profile.params().recovery_rate;
        vehicle.satisfaction = (vehicle.satisfaction + recovery * real_dt_s).min(100.0);
    }

    // Update lane change cooldown
    if vehicle.lane_change_cooldown > 0.0 {
        vehicle.lane_change_cooldown = (vehicle.lane_change_cooldown - real_dt_s).max(0.0);
    }

    // Advance along edge
    if vehicle.route_pos >= vehicle.route.len() {
        vehicle.despawned = true;
        return;
    }

    let edge_idx = vehicle.route[vehicle.route_pos];
    let (edge_len, src_idx, tgt_idx) = {
        let edge = match map.graph.edge_weight(edge_idx) {
            Some(e) => e,
            None => {
                vehicle.despawned = true;
                return;
            }
        };
        let endpoints = match map.graph.edge_endpoints(edge_idx) {
            Some(e) => e,
            None => {
                vehicle.despawned = true;
                return;
            }
        };
        (edge.length_m, endpoints.0, endpoints.1)
    };

    if edge_len > 0.0 {
        vehicle.edge_progress += vehicle.speed * real_dt_s / edge_len;
    }

    // Edge transition
    if vehicle.edge_progress >= 1.0 {
        vehicle.route_pos += 1;
        vehicle.edge_progress = 0.0;

        if vehicle.route_pos >= vehicle.route.len() {
            vehicle.despawned = true;
            return;
        }
    }

    // Interpolate position
    let src = &map.graph[src_idx];
    let tgt = &map.graph[tgt_idx];
    let t = vehicle.edge_progress as f64;
    vehicle.lat = src.lat + (tgt.lat - src.lat) * t;
    vehicle.lng = src.lng + (tgt.lng - src.lng) * t;

    // Heading angle (east-referenced, converted to map-north for rendering)
    let dlat = tgt.lat - src.lat;
    let dlng = tgt.lng - src.lng;
    // atan2(dlng * cos(lat), dlat) gives bearing from north; use simple atan2 for display
    vehicle.angle = (dlng as f32).atan2(dlat as f32);
}

/// Serialise all vehicles into a packed binary buffer.
/// Each vehicle = 28 bytes (see binary packet format in README).
fn serialize_vehicles(vehicles: &[Vehicle]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(vehicles.len() * 28);

    for v in vehicles {
        let id_bytes = v.id.to_le_bytes();
        let lat_bytes = (v.lat as f32).to_le_bytes();
        let lng_bytes = (v.lng as f32).to_le_bytes();
        let angle_bytes = v.angle.to_le_bytes();
        let speed_bytes = v.speed.to_le_bytes();
        let vtype_byte = v.vehicle_type as u8;
        let profile_byte = v.driver_profile as u8;
        let padding: [u8; 2] = [0, 0];
        let sat_bytes = v.satisfaction.to_le_bytes();

        buf.extend_from_slice(&id_bytes);      // [0..3]
        buf.extend_from_slice(&lat_bytes);     // [4..7]
        buf.extend_from_slice(&lng_bytes);     // [8..11]
        buf.extend_from_slice(&angle_bytes);   // [12..15]
        buf.extend_from_slice(&speed_bytes);   // [16..19]
        buf.push(vtype_byte);                  // [20]
        buf.push(profile_byte);                // [21]
        buf.extend_from_slice(&padding);       // [22..23]
        buf.extend_from_slice(&sat_bytes);     // [24..27]
    }

    buf
}
