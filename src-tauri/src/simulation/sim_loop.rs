use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};
use std::collections::HashMap;
use parking_lot::RwLock;
use std::sync::Arc;
use rayon::prelude::*;
use tauri::ipc::Channel;
use tauri::{AppHandle, Emitter};
use base64::Engine;
use serde::Serialize;
use petgraph::graph::{EdgeIndex, NodeIndex};

use crate::map::road_network::{MapData, IntersectionType};
use crate::state::SimCommand;
use crate::time::game_clock::GameClock;
use crate::time::day_cycle::DayCycle;
use crate::traffic::intersection::IntersectionManager;
use crate::vehicles::vehicle::Vehicle;
use crate::simulation::idm::{idm_acceleration, idm_debug_snapshot};
use crate::simulation::spawn::SpawnSystem;
use crate::simulation::lane_change::{decide_lane_change, compute_vehicle_target_lane};
use crate::simulation::congestion::compute_congestion;
use crate::simulation::od_model::OdModel;
use crate::simulation::speed_config::SpeedConfig;
use crate::simulation::tram_sim::TramSim;

const TARGET_TICK_S: f32 = 1.0 / 60.0;
const CONGESTION_INTERVAL_S: f32 = 0.5;

/// Distance along an incoming edge from the **intersection node** to the stop line / virtual leader.
/// Larger value = stop farther **before** the junction (vehicles do not enter the crossing on red).
const STOP_LINE_OFFSET_M: f32 = 16.0;

/// Smallest admissible IDM free gap to avoid divide-by-zero and explosive braking.
const MIN_IDM_GAP_M: f32 = 0.1;
/// Additional spawn breathing room after bumper gap has been computed.
const SPAWN_BUFFER_M: f32 = 2.0;
const DEFAULT_DEBUG_VEHICLE_ID: u32 = 2;
const DEFAULT_DEBUG_LOG_INTERVAL_S: f32 = 0.5;

/// Only consider cross-traffic IDM within this distance to the node (meters).
const CROSS_TRAFFIC_MAX_DIST_M: f32 = 70.0;
const TURN_CONNECTOR_ENTRY_M: f32 = 10.0;
const TURN_CONNECTOR_EXIT_M: f32 = 10.0;
const TURN_CONNECTOR_MIN_ANGLE_RAD: f32 = 0.35;
const TURN_CONNECTOR_TARGET_SPEED_MPS: f32 = 15.0 / 3.6;

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct GameOverPayload {
    reason: String,
    value: f32,
    timestamp_game: f32,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct IdmDebugPayload {
    vehicle_id: u32,
    speed: f32,
    gap: f32,
    delta_v: f32,
    dist_to_stop_line: f32,
    red_blocking: bool,
    route_points: Vec<[f64; 2]>,
}

pub fn run_simulation(
    graph_lock: Arc<RwLock<Option<MapData>>>,
    command_rx: Receiver<SimCommand>,
    vehicle_channel: Channel<String>,
    app_handle: AppHandle,
) {
    let debug_vehicle_id = std::env::var("IDM_DEBUG_ID")
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(DEFAULT_DEBUG_VEHICLE_ID);
    let debug_log_interval_s = std::env::var("IDM_DEBUG_INTERVAL_S")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
        .filter(|v| *v > 0.0)
        .unwrap_or(DEFAULT_DEBUG_LOG_INTERVAL_S);

    let mut clock = GameClock::new();
    let mut vehicles: Vec<Vehicle> = Vec::with_capacity(512);
    let mut congestion_timer = 0.0f32;
    let mut high_frustration_timer = 0.0f32;
    let mut idm_debug_timer = 0.0f32;
    let mut selected_debug_vehicle: Option<u32> = None;

    // ── Build subsystems from map ────────────────────────────────────────────
    let (mut intersections, mut spawn_system, mut od_model, mut tram_sim) = {
        let guard = graph_lock.read();
        let map   = guard.as_ref().expect("map must be loaded before starting simulation");

        (
            IntersectionManager::from_graph(&map.graph, map.sandbox_simple_cross_tl),
            SpawnSystem::new(
                map.spawn_points.clone(),
                map.boundary_nodes.clone(),
                SpeedConfig::default(),
                map.is_sandbox,
            ),
            OdModel::new(map.od_buildings.clone(), &mut rand::rngs::OsRng),
            // Tram simulation: use IDs starting after the car-id range to avoid collisions
            TramSim::new(&map.tram_data, 100_000),
        )
    };
    let mut map_signature = current_map_signature(&graph_lock);

    // Send all initial light states so the frontend shows the correct
    // staggered phases immediately (instead of defaulting everything to Red).
    let initial_states = intersections.all_state_updates();
    if !initial_states.is_empty() {
        let _ = app_handle.emit("light_state_change", &initial_states);
    }

    // Per-(edge, lane) sorted vehicle index buckets — rebuilt every tick.
    // Key: (EdgeIndex, lane_number), Value: Vec of indices into `vehicles` sorted by edge_progress ascending.
    let mut edge_lane_vehicles: HashMap<(EdgeIndex, u8), Vec<usize>> = HashMap::new();

    // Fixed-step physics accumulator.
    // Physics always steps by exactly PHYSICS_DT regardless of wall-clock variance.
    // This eliminates IDM instability caused by OS scheduling jitter.
    const PHYSICS_DT: f32 = 1.0 / 60.0; // 16.667 ms — never changes
    let mut physics_accumulator = 0.0f32;

    let mut last_tick = Instant::now();
    log::info!("Simulation loop started (fixed-step dt = {:.4} s)", PHYSICS_DT);

    loop {
        // ── Wall-clock time measurement ──────────────────────────────────────
        let now          = Instant::now();
        // Cap at 250 ms to prevent "spiral of death" on very slow machines
        let real_elapsed = now.duration_since(last_tick).as_secs_f32().min(0.25);
        last_tick        = now;

        // ── Commands ─────────────────────────────────────────────────────────
        loop {
            match command_rx.try_recv() {
                Ok(cmd) => handle_command(
                    cmd,
                    &mut clock,
                    &mut intersections,
                    &mut spawn_system,
                    &mut selected_debug_vehicle,
                ),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    log::info!("Simulation command channel closed, stopping");
                    return;
                }
            }
        }

        // Reload simulation-dependent caches when the map has changed.
        // This avoids stale NodeIndex/EdgeIndex references after `load_map`.
        let latest_signature = current_map_signature(&graph_lock);
        if latest_signature != map_signature {
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                intersections = IntersectionManager::from_graph(&map.graph, map.sandbox_simple_cross_tl);
                let mut refreshed_spawn = SpawnSystem::new(
                    map.spawn_points.clone(),
                    map.boundary_nodes.clone(),
                    SpeedConfig::default(),
                    map.is_sandbox,
                );
                // Preserve user-updated runtime tuning across map reloads.
                refreshed_spawn.speed_config = spawn_system.speed_config.clone();
                refreshed_spawn.max_vehicles = spawn_system.max_vehicles;
                spawn_system = refreshed_spawn;
                od_model = OdModel::new(map.od_buildings.clone(), &mut rand::rngs::OsRng);
                tram_sim = TramSim::new(&map.tram_data, 100_000);
                vehicles.clear();
                edge_lane_vehicles.clear();
                map_signature = latest_signature;

                let light_states = intersections.all_state_updates();
                if !light_states.is_empty() {
                    let _ = app_handle.emit("light_state_change", &light_states);
                }
                log::info!("Map changed during simulation; rebuilt spawn/OD/tram caches");
            }
        }

        if clock.paused {
            std::thread::sleep(Duration::from_millis(16));
            last_tick = Instant::now();
            continue;
        }

        physics_accumulator += real_elapsed;

        // ── Fixed physics steps ───────────────────────────────────────────────
        // Each iteration advances the simulation by exactly PHYSICS_DT seconds.
        while physics_accumulator >= PHYSICS_DT {
            physics_accumulator -= PHYSICS_DT;

            let game_dt_s       = clock.tick(PHYSICS_DT);
            let game_hour       = clock.game_hour();
            let spawn_multiplier = DayCycle::spawn_multiplier(game_hour);
            idm_debug_timer += PHYSICS_DT;

            // Traffic lights advance by fixed dt
            let light_updates = intersections.update(PHYSICS_DT);
            if !light_updates.is_empty() {
                let _ = app_handle.emit("light_state_change", &light_updates);
            }

            // Tram simulation
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    if !tram_sim.is_empty() {
                        tram_sim.tick(PHYSICS_DT, game_dt_s, &map.tram_data);
                    }
                }
            }

            // Spawn vehicles — with clearance check at spawn point.
            // If the first edge already has a vehicle within SPAWN_CLEARANCE_M
            // of the spawn node, the spawn is skipped (accumulator retries later).
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    let new_vehicles = spawn_system.tick_with_hour(
                        PHYSICS_DT,
                        spawn_multiplier,
                        game_hour,
                        map,
                        &od_model,
                        vehicles.len(),
                    );
                    for nv in new_vehicles {
                        if let Some(&first_edge) = nv.route.first() {
                            let edge_len = map.graph.edge_weight(first_edge)
                                .map(|e| e.length_m)
                                .unwrap_or(100.0);
                            let blocked = vehicles.iter().any(|v| {
                                let new_len = nv.vehicle_type.params().length_m;
                                let existing_len = v.vehicle_type.params().length_m;
                                let center_dist_m = v.edge_progress * edge_len;
                                let bumper_gap_m =
                                    center_dist_m - 0.5 * (new_len + existing_len);
                                let min_spawn_gap_m = new_len.max(existing_len) + SPAWN_BUFFER_M;
                                v.route_pos < v.route.len()
                                    && v.route[v.route_pos] == first_edge
                                    && v.current_lane == nv.current_lane
                                    && bumper_gap_m < min_spawn_gap_m
                            });
                            if blocked { continue; } // no room, skip this tick
                        }
                        vehicles.push(nv);
                    }
                }
            }

            // Build per-(edge, lane) sorted buckets for O(1) leader lookup.
            // Must be rebuilt each physics step so IDM sees up-to-date positions.
            edge_lane_vehicles.clear();
            for (i, v) in vehicles.iter().enumerate() {
                if v.route_pos < v.route.len() {
                    let key = (v.route[v.route_pos], v.current_lane);
                    edge_lane_vehicles.entry(key).or_default().push(i);
                }
            }
            for bucket in edge_lane_vehicles.values_mut() {
                // Sort ascending by edge_progress; tiebreak by ID (lower = older = ahead).
                bucket.sort_unstable_by(|&a, &b| {
                    vehicles[a].edge_progress
                        .partial_cmp(&vehicles[b].edge_progress)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then_with(|| vehicles[a].id.cmp(&vehicles[b].id))
                });
            }

            let vehicles_by_target_node: HashMap<NodeIndex, Vec<usize>> = {
                let guard = graph_lock.read();
                match guard.as_ref() {
                    Some(map) => build_vehicles_by_target_node(&vehicles, map),
                    None => HashMap::new(),
                }
            };

            // Parallel IDM acceleration computation (read-only, safe to parallelise)
            let tram_snapshot: Vec<(f64, f64, f32)> = tram_sim.trams.iter()
                .map(|t| (t.lat, t.lng, t.speed))
                .collect();

            let accel_inputs: Vec<f32> = {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    vehicles
                        .par_iter()
                        .enumerate()
                        .map(|(i, v)| {
                            let (gap, delta_v) =
                                find_leader_arc(i, v, &vehicles, &edge_lane_vehicles, map);
                            let (gap, delta_v) =
                                apply_tram_leader_effect(v, gap, delta_v, &tram_snapshot);
                            let (gap, delta_v) = apply_cross_traffic_leader_effect(
                                i,
                                v,
                                &vehicles,
                                &vehicles_by_target_node,
                                map,
                                &intersections,
                                gap,
                                delta_v,
                            );
                            let desired = compute_desired_speed(v, map);
                            let (gap, delta_v) =
                                apply_intersection_effect(v, gap, delta_v, &intersections, map);
                            let params = v.driver_profile.params();
                            let vtype  = v.vehicle_type.params();
                            idm_acceleration(v.speed, desired, gap, delta_v, &params, &vtype)
                        })
                        .collect()
                } else {
                    vec![0.0; vehicles.len()]
                }
            };

            if idm_debug_timer >= debug_log_interval_s {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    if let Some((ego_idx, ego)) = vehicles
                        .iter()
                        .enumerate()
                        .find(|(_, v)| v.id == debug_vehicle_id && !v.despawned)
                    {
                        let (base_gap, base_dv) =
                            find_leader_arc(ego_idx, ego, &vehicles, &edge_lane_vehicles, map);
                        let (gap, delta_v) =
                            apply_tram_leader_effect(ego, base_gap, base_dv, &tram_snapshot);
                        let desired = compute_desired_speed(ego, map);
                        let (gap, delta_v) =
                            apply_intersection_effect(ego, gap, delta_v, &intersections, map);
                        let params = ego.driver_profile.params();
                        let vtype = ego.vehicle_type.params();
                        let (s_clamped, s_star, accel_raw, accel_clamped) =
                            idm_debug_snapshot(ego.speed, desired, gap, delta_v, &params, &vtype);
                        let s_ratio = s_star / s_clamped;
                        let leader_speed = (ego.speed - delta_v).max(0.0);
                        log::info!(
                            "IDM_DEBUG id={} v={:.2} v0={:.2} v_leader={:.2} dv={:.2} s={:.2} s*={:.2} s_ratio={:.2} accel_raw={:.2} accel={:.2}",
                            ego.id,
                            ego.speed,
                            desired,
                            leader_speed,
                            delta_v,
                            s_clamped,
                            s_star,
                            s_ratio,
                            accel_raw,
                            accel_clamped,
                        );
                    }
                }
                idm_debug_timer = 0.0;
            }

            // Apply physics — always uses PHYSICS_DT (fixed step)
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    for (i, vehicle) in vehicles.iter_mut().enumerate() {
                        apply_vehicle_physics(
                            vehicle,
                            accel_inputs[i],
                            PHYSICS_DT,
                            map,
                            &intersections,
                            &spawn_system.speed_config,
                        );
                    }
                }
            }

            // Lane changes
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    let snapshot: Vec<&Vehicle> = vehicles.iter().collect();
                    let changes: Vec<(u32, u8)> = vehicles
                        .iter()
                        .filter_map(|v| {
                            if v.route_pos >= v.route.len() { return None; }
                            let edge_idx = v.route[v.route_pos];
                            let edge = map.graph.edge_weight(edge_idx)?;
                            let same_edge: Vec<&Vehicle> = snapshot
                                .iter()
                                .filter(|o| {
                                    o.route_pos < o.route.len()
                                        && o.route[o.route_pos] == edge_idx
                                })
                                .copied()
                                .collect();
                            decide_lane_change(v, edge, &same_edge).map(|lane| (v.id, lane))
                        })
                        .collect();

                    for (vid, new_lane) in changes {
                        if let Some(v) = vehicles.iter_mut().find(|v| v.id == vid) {
                            v.current_lane = new_lane;
                            v.lane_change_cooldown = 3.0;
                        }
                    }
                }
            }

            // Despawn vehicles that have finished their route
            vehicles.retain(|v| !v.despawned);

            // Game-over frustration checks (use PHYSICS_DT for consistent timing)
            if !vehicles.is_empty() {
                let avg_frustration: f32 =
                    vehicles.iter().map(|v| v.frustration).sum::<f32>() / vehicles.len() as f32;
                let rage_count = vehicles.iter().filter(|v| v.frustration >= 100.0).count();
                let mass_rage_fraction = rage_count as f32 / vehicles.len() as f32;

                let cfg = &spawn_system.speed_config.rage;
                if avg_frustration > cfg.global_loss_threshold {
                    high_frustration_timer += PHYSICS_DT;
                    if high_frustration_timer >= cfg.global_loss_duration_s {
                        log::warn!(
                            "GAME OVER: avg frustration {:.1} held for {:.0}s",
                            avg_frustration, high_frustration_timer
                        );
                        let _ = app_handle.emit("game_over", GameOverPayload {
                            reason: "avg_frustration".to_string(),
                            value: avg_frustration,
                            timestamp_game: clock.game_time_s as f32,
                        });
                        high_frustration_timer = 0.0;
                    }
                } else if mass_rage_fraction >= cfg.mass_rage_fraction {
                    log::warn!(
                        "GAME OVER: mass rage – {:.0}% vehicles at 100 frustration",
                        mass_rage_fraction * 100.0
                    );
                    let _ = app_handle.emit("game_over", GameOverPayload {
                        reason: "mass_rage".to_string(),
                        value: mass_rage_fraction * 100.0,
                        timestamp_game: clock.game_time_s as f32,
                    });
                } else {
                    high_frustration_timer = 0.0;
                }
            }

            // IDM debug snapshot (~5 Hz): one representative vehicle.
            idm_debug_timer += PHYSICS_DT;
            if idm_debug_timer >= 0.2 {
                idm_debug_timer = 0.0;
                let debug_target = if let Some(sel_id) = selected_debug_vehicle {
                    vehicles
                        .iter()
                        .enumerate()
                        .find(|(_, v)| v.id == sel_id && v.route_pos < v.route.len())
                } else {
                    vehicles
                        .iter()
                        .enumerate()
                        .find(|(_, v)| v.route_pos < v.route.len() && v.vehicle_type as u8 != 4)
                };
                if let Some((i, ego)) = debug_target
                {
                    let guard = graph_lock.read();
                    if let Some(map) = guard.as_ref() {
                        let (gap, delta_v) = find_leader_arc(i, ego, &vehicles, &edge_lane_vehicles, map);
                        let (dist_to_stop_line, red_blocking) =
                            stop_line_debug(ego, map, &intersections).unwrap_or((1000.0, false));
                        let payload = IdmDebugPayload {
                            vehicle_id: ego.id,
                            speed: ego.speed,
                            gap,
                            delta_v,
                            dist_to_stop_line,
                            red_blocking,
                            route_points: build_route_points(ego, map),
                        };
                        let _ = app_handle.emit("idm_debug", payload);
                    }
                }
            }
        } // end while physics_accumulator >= PHYSICS_DT

        // ── Render: serialise once per outer loop iteration ──────────────────
        // Decoupled from physics steps: the render fires at wall-clock rate
        // (~60 Hz target) even if physics ran 0 or 2 steps this iteration.
        if !vehicles.is_empty() || !tram_sim.is_empty() {
            let frame   = serialize_vehicles(&vehicles, &tram_sim);
            let encoded = base64::engine::general_purpose::STANDARD.encode(&frame);
            let _ = vehicle_channel.send(encoded);
        }

        // ── Congestion update (real-time interval, not per physics step) ──────
        congestion_timer += real_elapsed;
        if congestion_timer >= CONGESTION_INTERVAL_S {
            congestion_timer = 0.0;
            let guard = graph_lock.read();
            if let Some(map) = guard.as_ref() {
                let cdata = compute_congestion(&map.graph, &vehicles);
                let _ = app_handle.emit("congestion_update", &cdata);
            }
        }

        // ── Sleep to target ~60 FPS outer loop ────────────────────────────────
        let elapsed   = last_tick.elapsed().as_secs_f32();
        let remaining = TARGET_TICK_S - elapsed;
        if remaining > 0.001 {
            std::thread::sleep(Duration::from_secs_f32(remaining));
        }
    }
}

fn stop_line_debug(
    vehicle: &Vehicle,
    map: &MapData,
    intersections: &IntersectionManager,
) -> Option<(f32, bool)> {
    if vehicle.route_pos >= vehicle.route.len() {
        return None;
    }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = map.graph.edge_weight(edge_idx)?;
    let (_, tgt) = map.graph.edge_endpoints(edge_idx)?;
    let tgt_osm_id = map.graph[tgt].osm_id;
    let itype = &map.graph[tgt].intersection_type;

    if !matches!(itype, IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing) {
        return None;
    }

    let dist_to_end = edge.length_m * (1.0 - vehicle.edge_progress);
    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(vehicle, dist_to_end);
    let red_blocking =
        !intersections.can_vehicle_proceed(tgt_osm_id, vehicle.has_stopped_at_stop_sign, vehicle, map);
    Some((dist_to_stop_line, red_blocking))
}

fn current_map_signature(graph_lock: &Arc<RwLock<Option<MapData>>>) -> Option<(usize, usize, usize, usize)> {
    let guard = graph_lock.read();
    guard.as_ref().map(|map| {
        (
            map.graph.node_count(),
            map.graph.edge_count(),
            map.od_buildings.len(),
            map.tram_data.graph.node_count(),
        )
    })
}

// ── Command handler ────────────────────────────────────────────────────────────

fn handle_command(
    cmd: SimCommand,
    clock: &mut GameClock,
    intersections: &mut IntersectionManager,
    spawn_system: &mut SpawnSystem,
    selected_debug_vehicle: &mut Option<u32>,
) {
    match cmd {
        SimCommand::Pause                  => clock.pause(),
        SimCommand::Resume                 => clock.resume(),
        SimCommand::SetTimeScale(s)        => clock.set_time_scale(s),
        SimCommand::SetSpeedConfig(cfg)    => spawn_system.set_speed_config(cfg),
        SimCommand::SetMaxVehicles(n)      => spawn_system.max_vehicles = n,
        SimCommand::SetLightMode { intersection_id, mode } => {
            intersections.set_mode(intersection_id, mode);
        }
        SimCommand::SetLightPhase { intersection_id, phase } => {
            intersections.set_phase(intersection_id, phase);
        }
        SimCommand::SetLightDurations { intersection_id, green_s, red_s } => {
            intersections.set_durations(intersection_id, green_s, red_s);
        }
        SimCommand::SetDebugVehicle(id) => {
            *selected_debug_vehicle = id;
        }
        SimCommand::Stop => {}
    }
}

fn build_route_points(vehicle: &Vehicle, map: &MapData) -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    points.push([vehicle.lng, vehicle.lat]);
    if vehicle.on_turn_connector {
        let samples = 12usize;
        for i in 0..=samples {
            let t = i as f32 / samples as f32;
            let (lat, lng) = bezier_point_lat_lng(
                vehicle.turn_p1_lat,
                vehicle.turn_p1_lng,
                vehicle.turn_ctrl_lat,
                vehicle.turn_ctrl_lng,
                vehicle.turn_p2_lat,
                vehicle.turn_p2_lng,
                t,
            );
            points.push([lng, lat]);
        }
    }
    if vehicle.route_pos >= vehicle.route.len() {
        return points;
    }
    for &edge_idx in vehicle.route.iter().skip(vehicle.route_pos) {
        if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
            let n = &map.graph[tgt];
            points.push([n.lng, n.lat]);
        }
    }
    points
}

// ── Physics helpers ────────────────────────────────────────────────────────────

/// Find the leader vehicle using per-edge sorted buckets and arc-length gap.
///
/// Returns `(gap_m, delta_v_ms)` — bumper-to-bumper gap in metres and speed
/// difference (ego − leader) in m/s.  Uses 1000 m / 0.0 when no leader exists.
///
/// Gap is measured along the road (arc length), not Euclidean distance, which
/// prevents phantom collisions on curved segments and at edge boundaries.
fn find_leader_arc(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    edge_lane_vehicles: &HashMap<(EdgeIndex, u8), Vec<usize>>,
    map: &MapData,
) -> (f32, f32) {
    if ego.route_pos >= ego.route.len() {
        return (1000.0, 0.0);
    }

    let ego_edge = ego.route[ego.route_pos];
    let ego_lane = ego.current_lane;
    let ego_edge_len = map.graph.edge_weight(ego_edge).map(|e| e.length_m).unwrap_or(50.0);

    // ── Same edge, same lane ──────────────────────────────────────────────────
    if let Some(bucket) = edge_lane_vehicles.get(&(ego_edge, ego_lane)) {
        // Bucket is sorted by edge_progress ascending; find ego's position.
        if let Some(pos) = bucket.iter().position(|&idx| idx == ego_idx) {
            if let Some(&leader_idx) = bucket.get(pos + 1) {
                let leader = &vehicles[leader_idx];
                let center_dist_m = (leader.edge_progress - ego.edge_progress) * ego_edge_len;
                let gap = bumper_gap(center_dist_m, ego, leader);
                return (gap.max(MIN_IDM_GAP_M), ego.speed - leader.speed);
            }
        }
    }

    // ── Look ahead to next edge when ego is past 60 % of current edge ────────
    if ego.edge_progress >= 0.60 && ego.route_pos + 1 < ego.route.len() {
        let next_edge = ego.route[ego.route_pos + 1];
        let next_edge_len = map.graph.edge_weight(next_edge).map(|e| e.length_m).unwrap_or(50.0);
        let dist_to_end = (1.0 - ego.edge_progress) * ego_edge_len;

        // Check same lane on next edge, then adjacent lanes (merging / slight
        // offset between directional lanes on a two-way road).
        let lanes_to_check: [u8; 3] = [
            ego_lane,
            ego_lane.saturating_sub(1),
            ego_lane.saturating_add(1),
        ];
        let mut best_gap = f32::MAX;
        let mut best_dv  = 0.0f32;

        for &lane in &lanes_to_check {
            if let Some(bucket) = edge_lane_vehicles.get(&(next_edge, lane)) {
                // Lowest progress in bucket = vehicle closest to the edge start = closest to us
                if let Some(&leader_idx) = bucket.first() {
                    let leader = &vehicles[leader_idx];
                    let leader_from_start = leader.edge_progress * next_edge_len;
                    let center_dist_m = dist_to_end + leader_from_start;
                    let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
                    if gap < best_gap {
                        best_gap = gap;
                        best_dv  = ego.speed - leader.speed;
                    }
                }
            }
        }

        if best_gap < 1000.0 {
            return (best_gap, best_dv);
        }
    }

    (1000.0, 0.0) // free road ahead
}

#[inline]
fn bumper_gap(center_dist_m: f32, ego: &Vehicle, leader: &Vehicle) -> f32 {
    let ego_len = ego.vehicle_type.params().length_m;
    let leader_len = leader.vehicle_type.params().length_m;
    center_dist_m - 0.5 * (ego_len + leader_len)
}

#[inline]
fn geo_dist_approx(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f32 {
    let dlat = (lat2 - lat1) * 111_320.0;
    let dlng = (lng2 - lng1) * 71_700.0;
    ((dlat * dlat + dlng * dlng) as f32).sqrt()
}

/// Desired speed = road max_speed × personal_compliance, capped by vehicle type max.
fn compute_desired_speed(vehicle: &Vehicle, map: &MapData) -> f32 {
    if vehicle.route_pos >= vehicle.route.len() {
        return 0.0;
    }
    let road_max = map
        .graph
        .edge_weight(vehicle.route[vehicle.route_pos])
        .map(|e| e.max_speed)
        .unwrap_or(14.0);

    let v0_road  = road_max * vehicle.personal_compliance;
    let vtype_max = vehicle.vehicle_type.params().max_speed;
    let mut desired = v0_road.min(vtype_max);
    if vehicle.on_turn_connector {
        desired = desired.min(TURN_CONNECTOR_TARGET_SPEED_MPS);
    }
    desired
}

/// Apply traffic-light, stop-sign, and yield-sign braking effect.
///
/// - **Traffic light (red/yellow):** treat the stop-line as a stationary obstacle.
/// - **Stop sign:** slow to zero if not yet stopped; once stopped, allow proceed.
/// - **Yield sign:** cap approach speed to a low value within 20 m.
fn apply_intersection_effect(
    vehicle: &Vehicle,
    gap: f32,
    delta_v: f32,
    intersections: &IntersectionManager,
    map: &MapData,
) -> (f32, f32) {
    if vehicle.route_pos >= vehicle.route.len() { return (gap, delta_v); }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = match map.graph.edge_weight(edge_idx) {
        Some(e) => e,
        None    => return (gap, delta_v),
    };
    let dist_to_end = edge.length_m * (1.0 - vehicle.edge_progress);
    // IDM must see free space from the FRONT BUMPER to the stop line.
    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(vehicle, dist_to_end);

    let (tgt_node_idx, tgt_osm_id) = match map.graph.edge_endpoints(edge_idx) {
        Some((_, tgt)) => (tgt, map.graph[tgt].osm_id),
        None           => return (gap, delta_v),
    };
    let intersection_type = &map.graph[tgt_node_idx].intersection_type;

    // ── Traffic light OR pedestrian crossing ───────────────────────────────
    if matches!(
        intersection_type,
        IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing
    ) {
        // Dynamic braking look-ahead: enough distance for comfortable stop.
        // This makes red lights visible to IDM early enough on longer approaches.
        let braking_lookahead_m = (vehicle.speed * vehicle.speed) / (2.0 * 3.5) + 15.0;
        if dist_to_stop_line <= braking_lookahead_m.max(25.0)
            && !intersections.can_vehicle_proceed(
                tgt_osm_id,
                vehicle.has_stopped_at_stop_sign,
                vehicle,
                map,
            )
        {
            // Virtual leader: standing at stop line.
            let vgap = dist_to_stop_line.max(MIN_IDM_GAP_M);
            let vdv  = vehicle.speed; // leader speed = 0
            let new_gap = vgap.min(gap);
            let new_dv  = if new_gap < gap { vdv } else { delta_v };
            return (new_gap, new_dv);
        }
    }

    // ── Stop sign ──────────────────────────────────────────────────────────
    // Must decelerate to full stop within 8 m of the stop line.
    // `has_stopped_at_stop_sign` is set by `apply_vehicle_physics` once the
    // vehicle reaches speed < 0.3 m/s.  After stopping, the vehicle may proceed.
    if matches!(intersection_type, IntersectionType::Stop) {
        if dist_to_end <= 15.0 && !vehicle.has_stopped_at_stop_sign {
            let vgap = dist_to_stop_line.max(MIN_IDM_GAP_M);
            let new_gap = vgap.min(gap);
            let new_dv  = vehicle.speed;
            return (new_gap, new_dv);
        }
    }

    // ── Yield / give-way sign ──────────────────────────────────────────────
    // Slow to ≤ 5 km/h (1.39 m/s) within 20 m of the junction.
    if matches!(intersection_type, IntersectionType::Yield) {
        const YIELD_SPEED: f32 = 1.39; // 5 km/h
        if dist_to_end <= 20.0 && vehicle.speed > YIELD_SPEED {
            // Treat the junction entry as a slow virtual leader.
            let virtual_gap = dist_to_stop_line.max(0.5);
            let virtual_dv  = vehicle.speed - YIELD_SPEED;
            let new_gap = virtual_gap.min(gap);
            let new_dv  = if new_gap < gap { virtual_dv } else { delta_v };
            return (new_gap, new_dv);
        }
    }

    (gap, delta_v)
}

#[inline]
fn distance_to_stop_line_from_front_bumper(vehicle: &Vehicle, dist_to_end: f32) -> f32 {
    let dist_center_to_stop_line = (dist_to_end - STOP_LINE_OFFSET_M).max(0.0);
    let half_length = vehicle.vehicle_type.params().length_m * 0.5;
    (dist_center_to_stop_line - half_length).max(MIN_IDM_GAP_M)
}

/// Check nearby trams as potential IDM obstacles.
/// Uses a simple proximity scan since there are typically very few trams (< 20).
/// `trams` is a Vec of (lat, lng, speed) snapshots.
fn apply_tram_leader_effect(
    vehicle: &Vehicle,
    gap: f32,
    delta_v: f32,
    trams: &[(f64, f64, f32)],
) -> (f32, f32) {
    const TRAM_LENGTH_M: f32 = 20.0;
    let mut best_gap = gap;
    let mut best_dv  = delta_v;

    for &(tlat, tlng, tspeed) in trams {
        let dist = geo_dist_approx(vehicle.lat, vehicle.lng, tlat, tlng) - TRAM_LENGTH_M;
        let dist = dist.max(0.1);

        // Only treat a tram as our leader if it is in front of us and close.
        if dist < best_gap && dist < 150.0 {
            let dv = vehicle.speed - tspeed;
            if dv > 0.0 || dist < 20.0 {
                best_gap = dist;
                best_dv  = dv;
            }
        }
    }

    (best_gap, best_dv)
}

/// Index vehicles by the graph node at the **end** of their current edge
/// (the intersection / lane-merge node they are driving toward).
fn build_vehicles_by_target_node(vehicles: &[Vehicle], map: &MapData) -> HashMap<NodeIndex, Vec<usize>> {
    let mut m: HashMap<NodeIndex, Vec<usize>> = HashMap::new();
    for (i, v) in vehicles.iter().enumerate() {
        if v.route_pos >= v.route.len() {
            continue;
        }
        let edge = v.route[v.route_pos];
        if let Some((_, tgt)) = map.graph.edge_endpoints(edge) {
            m.entry(tgt).or_default().push(i);
        }
    }
    m
}

#[inline]
fn cross_traffic_intersection_type(itype: &IntersectionType) -> bool {
    // All node types except roundabouts: traffic-light junctions need this for
    // turn conflicts when two streams may have simultaneous green (or permissive turns).
    !matches!(itype, IntersectionType::Roundabout)
}

/// `true` when two edges meet at `tgt` from meaningfully different directions
/// (crossing or opposing), not parallel duplicates of the same approach corridor.
#[inline]
fn edges_are_conflicting_approaches(
    map: &MapData,
    ego_edge: EdgeIndex,
    other_edge: EdgeIndex,
    tgt: NodeIndex,
) -> bool {
    let (es, et) = match map.graph.edge_endpoints(ego_edge) {
        Some(x) => x,
        None => return false,
    };
    let (os, ot) = match map.graph.edge_endpoints(other_edge) {
        Some(x) => x,
        None => return false,
    };
    if et != tgt || ot != tgt {
        return false;
    }

    let n_tgt = &map.graph[tgt];
    let n_es = &map.graph[es];
    let n_os = &map.graph[os];
    // Unit vectors along each edge toward `tgt` (incoming directions).
    let vx = (n_tgt.lng - n_es.lng) as f32;
    let vy = (n_tgt.lat - n_es.lat) as f32;
    let wx = (n_tgt.lng - n_os.lng) as f32;
    let wy = (n_tgt.lat - n_os.lat) as f32;
    let lv = (vx * vx + vy * vy).sqrt().max(1e-9);
    let lw = (wx * wx + wy * wy).sqrt().max(1e-9);
    let dot = (vx / lv) * (wx / lw) + (vy / lv) * (wy / lw);
    // Same dual-carriageway / parallel approach lanes → dot ≈ 1 → skip.
    dot < 0.94
}

/// Lateral / cross-street conflict (plain, yield, stop, **traffic lights**, pedestrian signals).
///
/// Vehicles on a **different incoming edge** that share the same target node and are
/// closer to that node along their approach temporarily act as an IDM leader: we use our
/// remaining arc length to the node as gap and their speed in the IDM relative-velocity term.
///
/// For signalised nodes, a vehicle counts only if [`IntersectionManager::can_vehicle_proceed`]
/// is true for that approach — so queued traffic on red does not spuriously block cross traffic.
fn apply_cross_traffic_leader_effect(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    by_target_node: &HashMap<NodeIndex, Vec<usize>>,
    map: &MapData,
    intersections: &IntersectionManager,
    gap: f32,
    delta_v: f32,
) -> (f32, f32) {
    if ego.route_pos >= ego.route.len() {
        return (gap, delta_v);
    }
    let ego_edge = ego.route[ego.route_pos];
    let ego_edge_len = match map.graph.edge_weight(ego_edge) {
        Some(e) => e.length_m,
        None => return (gap, delta_v),
    };
    let tgt_node = match map.graph.edge_endpoints(ego_edge) {
        Some((_, tgt)) => tgt,
        None => return (gap, delta_v),
    };
    if !cross_traffic_intersection_type(&map.graph[tgt_node].intersection_type) {
        return (gap, delta_v);
    }

    let tgt_osm_id = map.graph[tgt_node].osm_id;
    if !intersections.can_vehicle_proceed(tgt_osm_id, ego.has_stopped_at_stop_sign, ego, map) {
        return (gap, delta_v);
    }

    let d_ego = (1.0 - ego.edge_progress) * ego_edge_len;
    if d_ego > CROSS_TRAFFIC_MAX_DIST_M {
        return (gap, delta_v);
    }

    let Some(candidates) = by_target_node.get(&tgt_node) else {
        return (gap, delta_v);
    };

    // Others must be at least this much nearer to the node (m) to count as priority cross traffic.
    const PRIORITY_HEADWAY_M: f32 = 0.5;

    let mut best_d_other = f32::MAX;
    let mut best_v_other = 0.0f32;
    let mut found = false;

    for &other_idx in candidates {
        if other_idx == ego_idx {
            continue;
        }
        let other = &vehicles[other_idx];
        if other.route_pos >= other.route.len() {
            continue;
        }
        let other_edge = other.route[other.route_pos];
        if other_edge == ego_edge {
            continue;
        }
        let other_len = match map.graph.edge_weight(other_edge) {
            Some(e) => e.length_m,
            None => continue,
        };
        let other_tgt = match map.graph.edge_endpoints(other_edge) {
            Some((_, t)) => t,
            None => continue,
        };
        if other_tgt != tgt_node {
            continue;
        }
        if !edges_are_conflicting_approaches(map, ego_edge, other_edge, tgt_node) {
            continue;
        }
        if !intersections.can_vehicle_proceed(
            tgt_osm_id,
            other.has_stopped_at_stop_sign,
            other,
            map,
        ) {
            continue;
        }

        let d_other = (1.0 - other.edge_progress) * other_len;
        if d_other + PRIORITY_HEADWAY_M >= d_ego {
            continue;
        }
        if !found || d_other < best_d_other {
            found = true;
            best_d_other = d_other;
            best_v_other = other.speed;
        }
    }

    if !found {
        return (gap, delta_v);
    }

    let cross_gap = d_ego.max(0.1);
    let cross_dv = ego.speed - best_v_other;

    if cross_gap < gap {
        (cross_gap, cross_dv)
    } else {
        (gap, delta_v)
    }
}

fn apply_vehicle_physics(
    vehicle: &mut Vehicle,
    accel: f32,
    real_dt_s: f32,
    map: &MapData,
    intersections: &IntersectionManager,
    speed_cfg: &SpeedConfig,
) {
    vehicle.accel = accel;
    vehicle.speed = (vehicle.speed + accel * real_dt_s).max(0.0);

    // ── Frustration update ─────────────────────────────────────────────────
    let pi = SpeedConfig::profile_idx(vehicle.driver_profile);
    let rage = &speed_cfg.rage;

    if vehicle.is_stopped() {
        // Vehicle is standing still
        vehicle.standstill_time_real_s += real_dt_s;
        vehicle.crawl_time_real_s = 0.0;

        let threshold = rage.standstill_threshold_s[pi];
        let rate = if vehicle.standstill_time_real_s < threshold {
            rage.decay_rate_linear[pi]
        } else {
            let excess = (vehicle.standstill_time_real_s - threshold) / threshold;
            rage.decay_rate_linear[pi] * (1.0 + excess * excess)
        };
        vehicle.frustration = (vehicle.frustration + rate * real_dt_s).min(100.0);
    } else {
        // Vehicle is moving
        vehicle.standstill_time_real_s = 0.0;

        // Check for crawling (very slow movement)
        let crawl_speed = if vehicle.route_pos < vehicle.route.len() {
            let road_max = map.graph.edge_weight(vehicle.route[vehicle.route_pos])
                .map(|e| e.max_speed)
                .unwrap_or(14.0);
            road_max * rage.crawl_fraction
        } else {
            0.5
        };

        if vehicle.speed < crawl_speed.max(0.5) {
            vehicle.crawl_time_real_s += real_dt_s;
            if vehicle.crawl_time_real_s > rage.crawl_threshold_s {
                vehicle.frustration =
                    (vehicle.frustration + rage.crawl_rate[pi] * real_dt_s).min(100.0);
            }
        } else {
            vehicle.crawl_time_real_s = 0.0;
            // Recover frustration while moving normally
            vehicle.frustration =
                (vehicle.frustration - rage.recovery_rate[pi] * real_dt_s).max(0.0);
        }
    }

    // Despawn at rage (frustration = 100)
    if vehicle.frustration >= 100.0 {
        vehicle.despawned = true;
        return;
    }

    // ── Stop-sign: mark when fully stopped near the stop line ─────────────
    if !vehicle.has_stopped_at_stop_sign && vehicle.speed < 0.3 {
        if vehicle.route_pos < vehicle.route.len() {
            let edge_idx = vehicle.route[vehicle.route_pos];
            if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
                if matches!(map.graph[tgt].intersection_type, IntersectionType::Stop) {
                    let dist_to_end = map.graph
                        .edge_weight(edge_idx)
                        .map(|e| e.length_m * (1.0 - vehicle.edge_progress))
                        .unwrap_or(f32::MAX);
                    if dist_to_end <= 15.0 {
                        vehicle.has_stopped_at_stop_sign = true;
                    }
                }
            }
        }
    }

    // ── Lane change cooldown ───────────────────────────────────────────────
    if vehicle.lane_change_cooldown > 0.0 {
        vehicle.lane_change_cooldown = (vehicle.lane_change_cooldown - real_dt_s).max(0.0);
    }

    // ── Lateral-offset smoothing (GTA-style lane glide) ───────────────────
    // target_lateral_offset tracks the desired lane (== target_lane as f32).
    // current_lateral_offset glides toward it at LANE_CHANGE_SPEED lane/s.
    // Using real-time dt keeps the animation speed independent of sim speed.
    vehicle.target_lateral_offset = vehicle.target_lane as f32;
    {
        const LANE_CHANGE_SPEED: f32 = 0.35; // lane-widths per real second (~2.9 s full change)
        let diff = vehicle.target_lateral_offset - vehicle.current_lateral_offset;
        let step = LANE_CHANGE_SPEED * real_dt_s;
        if diff.abs() <= step {
            vehicle.current_lateral_offset = vehicle.target_lateral_offset;
        } else {
            vehicle.current_lateral_offset += step * diff.signum();
        }
    }

    // ── Advance along route ────────────────────────────────────────────────
    if vehicle.route_pos >= vehicle.route.len() {
        vehicle.despawned = true;
        return;
    }

    let edge_idx = vehicle.route[vehicle.route_pos];
    let (edge_len, src_idx, tgt_idx) = {
        let edge = match map.graph.edge_weight(edge_idx) {
            Some(e) => e,
            None    => { vehicle.despawned = true; return; }
        };
        let endpoints = match map.graph.edge_endpoints(edge_idx) {
            Some(e) => e,
            None    => { vehicle.despawned = true; return; }
        };
        (edge.length_m, endpoints.0, endpoints.1)
    };

    // ── Turn connector (Bezier) state machine ──────────────────────────────
    if vehicle.on_turn_connector {
        let turn_len = vehicle.turn_length_m.max(0.5);
        let dt = (vehicle.speed * real_dt_s / turn_len).max(0.0);
        vehicle.turn_t = (vehicle.turn_t + dt).min(1.0);

        // Keep edge_progress monotonic for leader logic while traversing the connector.
        vehicle.edge_progress =
            vehicle.turn_entry_progress + (1.0 - vehicle.turn_entry_progress) * vehicle.turn_t;

        let (lat, lng) = bezier_point_lat_lng(
            vehicle.turn_p1_lat,
            vehicle.turn_p1_lng,
            vehicle.turn_ctrl_lat,
            vehicle.turn_ctrl_lng,
            vehicle.turn_p2_lat,
            vehicle.turn_p2_lng,
            vehicle.turn_t,
        );
        vehicle.lat = lat;
        vehicle.lng = lng;

        let look_t = (vehicle.turn_t + 0.03).min(1.0);
        let (lat2, lng2) = bezier_point_lat_lng(
            vehicle.turn_p1_lat,
            vehicle.turn_p1_lng,
            vehicle.turn_ctrl_lat,
            vehicle.turn_ctrl_lng,
            vehicle.turn_p2_lat,
            vehicle.turn_p2_lng,
            look_t,
        );
        let dlng = (lng2 - lng) as f32;
        let dlat = (lat2 - lat) as f32;
        if dlng.abs() > 1e-6 || dlat.abs() > 1e-6 {
            vehicle.angle = dlng.atan2(dlat);
        }

        if vehicle.turn_t >= 1.0 {
            vehicle.on_turn_connector = false;
            vehicle.turn_t = 0.0;
            vehicle.route_pos += 1;
            vehicle.edge_progress = vehicle.turn_exit_progress;
            vehicle.has_stopped_at_stop_sign = false;

            if vehicle.route_pos >= vehicle.route.len() {
                vehicle.despawned = true;
                return;
            }

            vehicle.target_lane = compute_vehicle_target_lane(vehicle, map);
        }
        return;
    }

    if edge_len > 0.0 {
        vehicle.edge_progress += vehicle.speed * real_dt_s / edge_len;
    }

    // Hard red-line guard: never let a vehicle cross the stop line on red/yellow.
    // IDM does the smooth braking; this guard prevents rare frame-step overshoot.
    if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
        let tgt_osm_id = map.graph[tgt].osm_id;
        let itype = &map.graph[tgt].intersection_type;
        if matches!(itype, IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing)
            && !intersections.can_vehicle_proceed(
                tgt_osm_id,
                vehicle.has_stopped_at_stop_sign,
                vehicle,
                map,
            )
        {
            let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge_len.max(1.0)).clamp(0.0, 1.0);
            if vehicle.edge_progress >= stop_t {
                vehicle.edge_progress = stop_t;
                vehicle.speed = vehicle.speed.min(0.2);
            }
        }
    }

    if let Some(conn) = planned_turn_connector(vehicle, map) {
        if vehicle.edge_progress >= conn.entry_progress {
            vehicle.on_turn_connector = true;
            vehicle.turn_t = 0.0;
            vehicle.turn_entry_progress = conn.entry_progress;
            vehicle.turn_exit_progress = conn.exit_progress;
            vehicle.turn_length_m = conn.length_m.max(0.5);
            vehicle.turn_p1_lat = conn.p1_lat;
            vehicle.turn_p1_lng = conn.p1_lng;
            vehicle.turn_ctrl_lat = conn.ctrl_lat;
            vehicle.turn_ctrl_lng = conn.ctrl_lng;
            vehicle.turn_p2_lat = conn.p2_lat;
            vehicle.turn_p2_lng = conn.p2_lng;
            vehicle.edge_progress = conn.entry_progress;
        }
    }

    if vehicle.edge_progress >= 1.0 {
        vehicle.route_pos             += 1;
        vehicle.edge_progress          = 0.0;
        vehicle.has_stopped_at_stop_sign = false; // reset for next edge

        if vehicle.route_pos >= vehicle.route.len() {
            vehicle.despawned = true;
            return;
        }

        // Recompute which lane to target based on the upcoming turn.
        vehicle.target_lane = compute_vehicle_target_lane(vehicle, map);
    }

    // Interpolate position
    let src = &map.graph[src_idx];
    let tgt = &map.graph[tgt_idx];
    let t   = vehicle.edge_progress as f64;
    vehicle.lat = src.lat + (tgt.lat - src.lat) * t;
    vehicle.lng = src.lng + (tgt.lng - src.lng) * t;

    // Heading
    let dlng = tgt.lng - src.lng;
    let dlat = tgt.lat - src.lat;
    vehicle.angle = (dlng as f32).atan2(dlat as f32);
}

struct PlannedTurnConnector {
    entry_progress: f32,
    exit_progress: f32,
    length_m: f32,
    p1_lat: f64,
    p1_lng: f64,
    ctrl_lat: f64,
    ctrl_lng: f64,
    p2_lat: f64,
    p2_lng: f64,
}

fn planned_turn_connector(vehicle: &Vehicle, map: &MapData) -> Option<PlannedTurnConnector> {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return None;
    }
    let curr_edge = vehicle.route[vehicle.route_pos];
    let next_edge = vehicle.route[vehicle.route_pos + 1];
    let (src, junction) = map.graph.edge_endpoints(curr_edge)?;
    let (next_src, next_tgt) = map.graph.edge_endpoints(next_edge)?;
    if junction != next_src {
        return None;
    }

    let curr_len = map.graph.edge_weight(curr_edge)?.length_m.max(1.0);
    let next_len = map.graph.edge_weight(next_edge)?.length_m.max(1.0);

    let src_n = &map.graph[src];
    let jn = &map.graph[junction];
    let tgt_n = &map.graph[next_tgt];

    // Incoming direction toward the node, outgoing direction from the node.
    let in_x = (jn.lng - src_n.lng) as f32;
    let in_y = (jn.lat - src_n.lat) as f32;
    let out_x = (tgt_n.lng - jn.lng) as f32;
    let out_y = (tgt_n.lat - jn.lat) as f32;
    let in_len = (in_x * in_x + in_y * in_y).sqrt().max(1e-6);
    let out_len = (out_x * out_x + out_y * out_y).sqrt().max(1e-6);
    let dot = ((in_x / in_len) * (out_x / out_len) + (in_y / in_len) * (out_y / out_len))
        .clamp(-1.0, 1.0);
    let angle = dot.acos();
    if angle < TURN_CONNECTOR_MIN_ANGLE_RAD {
        return None;
    }

    let entry_progress = (1.0 - TURN_CONNECTOR_ENTRY_M / curr_len).clamp(0.0, 1.0);
    let exit_progress = (TURN_CONNECTOR_EXIT_M / next_len).clamp(0.0, 1.0);
    if entry_progress <= vehicle.edge_progress {
        return None;
    }

    let p1_lat = src_n.lat + (jn.lat - src_n.lat) * entry_progress as f64;
    let p1_lng = src_n.lng + (jn.lng - src_n.lng) * entry_progress as f64;
    let p2_lat = jn.lat + (tgt_n.lat - jn.lat) * exit_progress as f64;
    let p2_lng = jn.lng + (tgt_n.lng - jn.lng) * exit_progress as f64;
    let ctrl_lat = jn.lat;
    let ctrl_lng = jn.lng;

    let length_m = bezier_length_m(p1_lat, p1_lng, ctrl_lat, ctrl_lng, p2_lat, p2_lng, 16);

    Some(PlannedTurnConnector {
        entry_progress,
        exit_progress,
        length_m,
        p1_lat,
        p1_lng,
        ctrl_lat,
        ctrl_lng,
        p2_lat,
        p2_lng,
    })
}

#[inline]
fn bezier_point_lat_lng(
    p1_lat: f64,
    p1_lng: f64,
    ctrl_lat: f64,
    ctrl_lng: f64,
    p2_lat: f64,
    p2_lng: f64,
    t: f32,
) -> (f64, f64) {
    let u = (1.0 - t) as f64;
    let tt = t as f64;
    let lat = u * u * p1_lat + 2.0 * u * tt * ctrl_lat + tt * tt * p2_lat;
    let lng = u * u * p1_lng + 2.0 * u * tt * ctrl_lng + tt * tt * p2_lng;
    (lat, lng)
}

fn bezier_length_m(
    p1_lat: f64,
    p1_lng: f64,
    ctrl_lat: f64,
    ctrl_lng: f64,
    p2_lat: f64,
    p2_lng: f64,
    segments: usize,
) -> f32 {
    let segs = segments.max(4);
    let mut total = 0.0f32;
    let mut prev = bezier_point_lat_lng(p1_lat, p1_lng, ctrl_lat, ctrl_lng, p2_lat, p2_lng, 0.0);
    for i in 1..=segs {
        let t = i as f32 / segs as f32;
        let curr = bezier_point_lat_lng(p1_lat, p1_lng, ctrl_lat, ctrl_lng, p2_lat, p2_lng, t);
        total += geo_dist_approx(prev.0, prev.1, curr.0, curr.1);
        prev = curr;
    }
    total
}

// ── Serialisation ─────────────────────────────────────────────────────────────

/// Serialise all vehicles (including trams) into a packed binary buffer.
///
/// Per-vehicle layout (32 bytes, 4-byte aligned):
/// ```text
///   [0..3]   id:              u32  LE
///   [4..7]   lat:             f32  LE
///   [8..11]  lng:             f32  LE
///   [12..15] angle:           f32  LE
///   [16..19] speed:           f32  LE
///   [20]     type:            u8   (0=Car, 1=Van, 2=Bus, 3=Truck, 4=Tram)
///   [21]     profile:         u8
///   [22]     trip_kind:       u8   (0=local_od, 1=transit, 2=ext_in, 3=ext_out)
///   [23]     lane_flags:      u8   (bits 0..6 lane index, bit7 on_turn_connector)
///   [24..27] frustration:     f32  LE  (0=calm, 100=rage)
///   [28..31] lateral_offset:  f32  LE  (smooth lane pos: 0.0=lane-0 centre, 1.0=lane-1 …)
/// ```
fn serialize_vehicles(vehicles: &[Vehicle], tram_sim: &TramSim) -> Vec<u8> {
    let total = vehicles.len() + tram_sim.trams.len();
    let mut buf = Vec::with_capacity(total * 32);

    for v in vehicles {
        push_vehicle_packet(
            &mut buf,
            v.id,
            v.lat,
            v.lng,
            v.angle,
            v.speed,
            v.vehicle_type as u8,
            v.driver_profile as u8,
            v.trip_kind,
            v.current_lane,
            v.on_turn_connector,
            v.frustration,
            v.current_lateral_offset,
        );
    }

    for t in &tram_sim.trams {
        push_vehicle_packet(
            &mut buf,
            t.id,
            t.lat,
            t.lng,
            t.angle,
            t.speed,
            crate::vehicles::types::VehicleType::Tram as u8,
            0, // Normal profile placeholder
            t.trip_kind,
            0, // Trams have fixed track, always lane 0
            false,
            t.frustration,
            0.0, // Trams stay on fixed track, no lateral offset
        );
    }

    buf
}

/// Per-vehicle binary packet layout (32 bytes):
/// ```text
///   [0..3]   id:              u32 LE
///   [4..7]   lat:             f32 LE
///   [8..11]  lng:             f32 LE
///   [12..15] angle:           f32 LE
///   [16..19] speed:           f32 LE
///   [20]     vehicle_type:    u8
///   [21]     driver_profile:  u8
///   [22]     trip_kind:       u8
///   [23]     lane_flags:      u8   (bits 0..6 lane index, bit7 on_turn_connector)
///   [24..27] frustration:     f32 LE
///   [28..31] lateral_offset:  f32 LE (smooth: 0.0=lane-0, 1.0=lane-1, …)
/// ```
#[inline]
fn push_vehicle_packet(
    buf: &mut Vec<u8>,
    id: u32,
    lat: f64,
    lng: f64,
    angle: f32,
    speed: f32,
    vtype: u8,
    profile: u8,
    trip_kind: u8,
    current_lane: u8,
    on_turn_connector: bool,
    frustration: f32,
    lateral_offset: f32,
) {
    buf.extend_from_slice(&id.to_le_bytes());                  // [0..3]
    buf.extend_from_slice(&(lat as f32).to_le_bytes());        // [4..7]
    buf.extend_from_slice(&(lng as f32).to_le_bytes());        // [8..11]
    buf.extend_from_slice(&angle.to_le_bytes());               // [12..15]
    buf.extend_from_slice(&speed.to_le_bytes());               // [16..19]
    buf.push(vtype);                                           // [20]
    buf.push(profile);                                         // [21]
    buf.push(trip_kind);                                       // [22]
    let lane_flags = (current_lane & 0x7f) | if on_turn_connector { 0x80 } else { 0 };
    buf.push(lane_flags);                                      // [23] lane + turn flag
    buf.extend_from_slice(&frustration.to_le_bytes());         // [24..27]
    buf.extend_from_slice(&lateral_offset.to_le_bytes());      // [28..31]
}
