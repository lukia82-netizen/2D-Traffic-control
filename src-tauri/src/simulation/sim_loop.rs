use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};
use std::collections::HashMap;
use std::cmp::Ordering;
use parking_lot::RwLock;
use std::sync::Arc;
use rayon::prelude::*;
use tauri::ipc::Channel;
use tauri::{AppHandle, Emitter};
use base64::Engine;
use serde::Serialize;
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;

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
use crate::simulation::bezier_smooth::BezierPath;
use glam::DVec2;
use rstar::{AABB, PointDistance, RTree, RTreeObject};
use parry2d::shape::Segment;
use parry2d::query::intersection_test;

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
const TURN_CONNECTOR_ENTRY_M: f32 = 30.0;
const TURN_CONNECTOR_EXIT_M: f32 = 30.0;
const TURN_CONNECTOR_MIN_ANGLE_RAD: f32 = 0.35;
const TURN_CONNECTOR_TARGET_SPEED_MPS: f32 = 25.0 / 3.6; // 6.94 m/s = 25 km/h on arc
const GEO_LAT_M: f64 = 111_320.0;
const GEO_LNG_M: f64 = 71_700.0;
const CONFLICT_LOOKAHEAD_M: f32 = 45.0;
const CONFLICT_PRIORITY_ACTIVATION_M: f32 = 35.0;
const CONFLICT_RELEASE_BUFFER_M: f32 = 3.0;
const CONFLICT_SHAPE_BUFFER_M: f32 = 1.0;
const RIGHT_HAND_CHECK_DIST_M: f32 = 15.0;
const DEADLOCK_BREAK_S: f32 = 4.0;
/// Physical lane half-width in metres used to offset Bezier P1/P2 from the

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
    desired_speed: f32,
    acceleration: f32,
    /// Arc / IDM braking gap to the dominant obstacle for this timestep.
    distance_to_leader_m: f32,
    leader_vehicle_id: Option<u32>,
    conflict_reserver_id: Option<u32>,
    dist_to_stop_line: f32,
    red_blocking: bool,
    on_curve: bool,
    turn_t: f32,
    shape_length_m: f32,
    shape_width_m: f32,
    shape_radius_m: f32,
    threat_kind: String,
    threat_line_style: String,
    threat_point: Option<[f64; 2]>,
    hood_lng_lat: [f64; 2],
    rear_bumper_lng_lat: [f64; 2],
    route_points: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct DebugConflictPointPayload {
    lng: f64,
    lat: f64,
    reserved_by: Option<u32>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct DebugVehicleThreatPayload {
    vehicle_id: u32,
    hood_lng_lat: [f64; 2],
    rear_bumper_lng_lat: [f64; 2],
    threat_lng_lat: Option<[f64; 2]>,
    line_style: String,
    threat_kind: String,
    leader_vehicle_id: Option<u32>,
    conflict_reserver_id: Option<u32>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct DebugVisualizationPayload {
    conflict_points: Vec<DebugConflictPointPayload>,
    vehicle_threats: Vec<DebugVehicleThreatPayload>,
}

#[derive(Debug, Clone)]
struct ConflictPoint {
    pos: DVec2,
    distance_on_path: f32,
    reserved_by: Option<u32>,
}

#[derive(Debug, Clone)]
struct ConflictPath {
    bezier: PlannedTurnConnector,
    points: Vec<ConflictPoint>,
}

#[derive(Debug, Clone)]
struct IntersectionConflictData {
    by_movement: HashMap<(EdgeIndex, EdgeIndex), ConflictPath>,
    incoming_cw: Vec<EdgeIndex>,
    deadlock_timer_s: f32,
    deadlock_first_move: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PriorityState {
    Wait,
    Clear,
    FirstMove,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ObstacleKind {
    Vehicle,
    ConflictPoint,
    PriorityStopLine,
    TrafficSignalStopLine,
    StopSignStopLine,
    YieldTarget,
}

#[derive(Debug, Clone, Copy)]
struct ClosestObstacle {
    kind: ObstacleKind,
    gap_m: f32,
    delta_v: f32,
    point_lng_lat: Option<[f64; 2]>,
    /// Same-lane leader (rear bumper coords in `point_lng_lat` when present).
    leader_vehicle_id: Option<u32>,
    /// Vehicle id that owns a reserved conflict patch ahead of ego.
    conflict_reserver_id: Option<u32>,
}

#[derive(Debug, Clone, Copy)]
struct IdmStepResult {
    accel: f32,
    desired_speed: f32,
    obstacle: ClosestObstacle,
}

#[derive(Debug, Clone, Default)]
struct ConflictSystem {
    nodes: HashMap<NodeIndex, IntersectionConflictData>,
}

#[derive(Clone, Copy)]
struct ApproachIndexItem {
    pos: [f64; 2],
    vehicle_idx: usize,
}

impl RTreeObject for ApproachIndexItem {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.pos)
    }
}

impl PointDistance for ApproachIndexItem {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.pos[0] - point[0];
        let dy = self.pos[1] - point[1];
        dx * dx + dy * dy
    }
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
    let mut idm_env_log_timer = 0.0f32;
    let mut idm_overlay_timer = 0.0f32;
    let mut selected_debug_vehicle: Option<u32> = None;
    let mut debug_visualization_enabled = false;

    // ── Build subsystems from map ────────────────────────────────────────────
    let (mut intersections, mut spawn_system, mut od_model, mut tram_sim, mut conflict_system) = {
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
            build_conflict_system(map),
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
                    &mut debug_visualization_enabled,
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
                conflict_system = build_conflict_system(map);
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
            idm_env_log_timer += PHYSICS_DT;
            idm_overlay_timer += PHYSICS_DT;

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
            let approach_items: Vec<ApproachIndexItem> = vehicles
                .iter()
                .enumerate()
                .filter(|(_, v)| !v.on_turn_connector && v.route_pos < v.route.len())
                .map(|(i, v)| {
                    let (x, y) = geo_to_m_xy(v.lat, v.lng);
                    ApproachIndexItem { pos: [x, y], vehicle_idx: i }
                })
                .collect();
            let approach_tree = RTree::bulk_load(approach_items);
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    conflict_system.update_deadlock_state(
                        map,
                        &vehicles,
                        &vehicles_by_target_node,
                        &approach_tree,
                        PHYSICS_DT,
                    );
                }
            }

            // Parallel IDM / closest-obstacle computation (read-only parallel part)
            let tram_snapshot: Vec<(f64, f64, f32)> = tram_sim.trams.iter()
                .map(|t| (t.lat, t.lng, t.speed))
                .collect();

            let idm_steps: Vec<IdmStepResult> = {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    vehicles
                        .par_iter()
                        .enumerate()
                        .map(|(i, v)| {
                            compute_vehicle_idm_step(
                                i,
                                v,
                                &vehicles,
                                &edge_lane_vehicles,
                                &vehicles_by_target_node,
                                &tram_snapshot,
                                map,
                                &intersections,
                                &conflict_system,
                                &approach_tree,
                            )
                        })
                        .collect()
                } else {
                    vec![]
                }
            };
            let accel_inputs: Vec<f32> = if idm_steps.len() == vehicles.len() {
                idm_steps.iter().map(|s| s.accel).collect()
            } else {
                vec![0.0f32; vehicles.len()]
            };

            if debug_visualization_enabled
                && !idm_steps.is_empty()
                && idm_steps.len() == vehicles.len()
            {
                let conflict_points = conflict_system.debug_conflict_points_snapshot();
                let vehicle_threats: Vec<DebugVehicleThreatPayload> = vehicles
                    .iter()
                    .zip(idm_steps.iter())
                    .map(|(v, s)| DebugVehicleThreatPayload {
                        vehicle_id: v.id,
                        hood_lng_lat: hood_lng_lat_m(v),
                        rear_bumper_lng_lat: rear_bumper_lng_lat_vehicle(v),
                        threat_lng_lat: s.obstacle.point_lng_lat,
                        line_style: threat_line_style_label(s.obstacle.kind).to_string(),
                        threat_kind: obstacle_kind_label(s.obstacle.kind).to_string(),
                        leader_vehicle_id: s.obstacle.leader_vehicle_id,
                        conflict_reserver_id: s.obstacle.conflict_reserver_id,
                    })
                    .collect();
                let _ = app_handle.emit(
                    "debug_visualization",
                    &DebugVisualizationPayload {
                        conflict_points,
                        vehicle_threats,
                    },
                );
            }

            if idm_env_log_timer >= debug_log_interval_s {
                let guard = graph_lock.read();
                if let (Some(_map), true) = (
                    guard.as_ref(),
                    idm_steps.len() == vehicles.len() && !vehicles.is_empty(),
                ) {
                    if let Some((ego_idx, ego)) = vehicles
                        .iter()
                        .enumerate()
                        .find(|(_, v)| v.id == debug_vehicle_id && !v.despawned)
                    {
                        let desired = idm_steps[ego_idx].desired_speed;
                        let obstacle = idm_steps[ego_idx].obstacle;
                        let params = ego.driver_profile.params();
                        let vtype = ego.vehicle_type.params();
                        let (s_clamped, s_star, accel_raw, accel_clamped) =
                            idm_debug_snapshot(
                                ego.speed,
                                desired,
                                obstacle.gap_m,
                                obstacle.delta_v,
                                &params,
                                &vtype,
                            );
                        let s_ratio = s_star / s_clamped;
                        let leader_speed = (ego.speed - obstacle.delta_v).max(0.0);
                        log::info!(
                            "IDM_DEBUG id={} v={:.2} v0={:.2} v_leader={:.2} dv={:.2} s={:.2} s*={:.2} s_ratio={:.2} accel_raw={:.2} accel={:.2}",
                            ego.id,
                            ego.speed,
                            desired,
                            leader_speed,
                            obstacle.delta_v,
                            s_clamped,
                            s_star,
                            s_ratio,
                            accel_raw,
                            accel_clamped,
                        );
                    }
                }
                idm_env_log_timer = 0.0;
            }

            // Apply physics — always uses PHYSICS_DT (fixed step)
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    let vehicles_snapshot = vehicles.clone();
                    for (i, vehicle) in vehicles.iter_mut().enumerate() {
                        apply_vehicle_physics(
                            vehicle,
                            accel_inputs[i],
                            PHYSICS_DT,
                            map,
                            &intersections,
                            &mut conflict_system,
                            &vehicles_by_target_node,
                            &approach_tree,
                            &vehicles_snapshot,
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
            for v in vehicles.iter().filter(|v| v.despawned) {
                conflict_system.release_all_for_vehicle(v.id);
            }
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

            // IDM debug snapshot (~5 Hz): selected or first car — feed HUD + route overlay.
            if idm_overlay_timer >= 0.2 {
                idm_overlay_timer = 0.0;
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
                if let Some((i, ego)) = debug_target {
                    let guard = graph_lock.read();
                    if let Some(map) = guard.as_ref() {
                        if i < idm_steps.len() {
                            let step = idm_steps[i];
                            let obstacle = step.obstacle;
                            let (dist_to_stop_line, red_blocking) =
                                stop_line_debug(ego, map, &intersections).unwrap_or((1000.0, false));
                            let vp = ego.vehicle_type.params();
                            let accel_i = accel_inputs.get(i).copied().unwrap_or(0.0);
                            let payload = IdmDebugPayload {
                                vehicle_id: ego.id,
                                speed: ego.speed,
                                gap: obstacle.gap_m,
                                delta_v: obstacle.delta_v,
                                desired_speed: step.desired_speed,
                                acceleration: accel_i,
                                distance_to_leader_m: obstacle.gap_m,
                                leader_vehicle_id: obstacle.leader_vehicle_id,
                                conflict_reserver_id: obstacle.conflict_reserver_id,
                                dist_to_stop_line,
                                red_blocking,
                                on_curve: ego.on_turn_connector,
                                turn_t: (ego.turn_dist_m / ego.turn_length_m.max(0.001) as f64)
                                    .clamp(0.0, 1.0) as f32,
                                shape_length_m: vp.length_m,
                                shape_width_m: vp.width_m,
                                shape_radius_m: vehicle_path_radius_m(ego),
                                threat_kind: obstacle_kind_label(obstacle.kind).to_string(),
                                threat_line_style: threat_line_style_label(obstacle.kind).to_string(),
                                threat_point: obstacle.point_lng_lat,
                                hood_lng_lat: hood_lng_lat_m(ego),
                                rear_bumper_lng_lat: rear_bumper_lng_lat_vehicle(ego),
                                route_points: build_route_points(ego, map),
                            };
                            let _ = app_handle.emit("idm_debug", payload);
                        }
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
    debug_visualization_enabled: &mut bool,
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
        SimCommand::SetDebugVisualization(on) => {
            *debug_visualization_enabled = on;
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

/// Find the dominant same-lane / look-ahead leader as an IDM obstacle.
///
/// Gap is bumper-to-bumper arc length (`MIN_IDM_GAP_M` clamped).
/// Uses the leader's **rear bumper** as the visualised threat anchor.
fn find_leader_obstacle_arc(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    edge_lane_vehicles: &HashMap<(EdgeIndex, u8), Vec<usize>>,
    map: &MapData,
) -> ClosestObstacle {
    if ego.route_pos >= ego.route.len() {
        return free_leader_obstacle();
    }

    let ego_edge = ego.route[ego.route_pos];
    let ego_lane = ego.current_lane;
    let ego_edge_len = map.graph.edge_weight(ego_edge).map(|e| e.length_m).unwrap_or(50.0);

    if let Some(bucket) = edge_lane_vehicles.get(&(ego_edge, ego_lane)) {
        if let Some(pos) = bucket.iter().position(|&idx| idx == ego_idx) {
            if let Some(&leader_idx) = bucket.get(pos + 1) {
                let leader = &vehicles[leader_idx];
                let center_dist_m = (leader.edge_progress - ego.edge_progress) * ego_edge_len;
                let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
                let rear_ll = rear_bumper_lng_lat_vehicle(leader);
                return ClosestObstacle {
                    kind: ObstacleKind::Vehicle,
                    gap_m: gap,
                    delta_v: ego.speed - leader.speed,
                    point_lng_lat: Some(rear_ll),
                    leader_vehicle_id: Some(leader.id),
                    conflict_reserver_id: None,
                };
            }
        }
    }

    if ego.edge_progress >= 0.60 && ego.route_pos + 1 < ego.route.len() {
        let next_edge = ego.route[ego.route_pos + 1];
        let next_edge_len = map.graph.edge_weight(next_edge).map(|e| e.length_m).unwrap_or(50.0);
        let dist_to_end = (1.0 - ego.edge_progress) * ego_edge_len;

        let lanes_to_check: [u8; 3] = [
            ego_lane,
            ego_lane.saturating_sub(1),
            ego_lane.saturating_add(1),
        ];
        let mut best: Option<(f32, f32, u32, [f64; 2])> = None;

        for &lane in &lanes_to_check {
            if let Some(bucket) = edge_lane_vehicles.get(&(next_edge, lane)) {
                if let Some(&leader_idx) = bucket.first() {
                    let leader = &vehicles[leader_idx];
                    let leader_from_start = leader.edge_progress * next_edge_len;
                    let center_dist_m = dist_to_end + leader_from_start;
                    let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
                    if best.map_or(true, |(g, _, _, _)| gap < g) {
                        let rear_ll = rear_bumper_lng_lat_vehicle(leader);
                        best = Some((gap, ego.speed - leader.speed, leader.id, rear_ll));
                    }
                }
            }
        }

        if let Some((gap, dv, lid, rear_ll)) = best {
            return ClosestObstacle {
                kind: ObstacleKind::Vehicle,
                gap_m: gap,
                delta_v: dv,
                point_lng_lat: Some(rear_ll),
                leader_vehicle_id: Some(lid),
                conflict_reserver_id: None,
            };
        }
    }

    free_leader_obstacle()
}

#[inline]
fn bumper_gap(center_dist_m: f32, ego: &Vehicle, leader: &Vehicle) -> f32 {
    let ego_len = ego.vehicle_type.params().length_m;
    let leader_len = leader.vehicle_type.params().length_m;
    center_dist_m - 0.5 * (ego_len + leader_len)
}

#[inline]
fn offset_vehicle_center_geo(v: &Vehicle, forward_m: f32) -> [f64; 2] {
    let north = v.angle.cos() as f64 * forward_m as f64;
    let east = v.angle.sin() as f64 * forward_m as f64;
    [v.lng + east / GEO_LNG_M, v.lat + north / GEO_LAT_M]
}

#[inline]
fn hood_lng_lat_m(v: &Vehicle) -> [f64; 2] {
    let half = v.vehicle_type.params().length_m * 0.5;
    offset_vehicle_center_geo(v, half)
}

#[inline]
fn rear_bumper_lng_lat_vehicle(v: &Vehicle) -> [f64; 2] {
    let half = v.vehicle_type.params().length_m * 0.5;
    offset_vehicle_center_geo(v, -half)
}

#[inline]
fn free_leader_obstacle() -> ClosestObstacle {
    ClosestObstacle {
        kind: ObstacleKind::Vehicle,
        gap_m: 1000.0,
        delta_v: 0.0,
        point_lng_lat: None,
        leader_vehicle_id: None,
        conflict_reserver_id: None,
    }
}

#[inline]
fn geo_dist_approx(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f32 {
    let dlat = (lat2 - lat1) * GEO_LAT_M;
    let dlng = (lng2 - lng1) * GEO_LNG_M;
    ((dlat * dlat + dlng * dlng) as f32).sqrt()
}

#[inline]
fn geo_to_m_xy(lat: f64, lng: f64) -> (f64, f64) {
    (lng * GEO_LNG_M, lat * GEO_LAT_M)
}

#[inline]
fn m_xy_to_geo(x_m: f64, y_m: f64) -> (f64, f64) {
    (y_m / GEO_LAT_M, x_m / GEO_LNG_M)
}

/// Build a `BezierPath` from three geographic control points (lat/lng).
/// The curve is constructed in local Cartesian metres (x = east, y = north).
#[inline]
fn bezier_path_from_geo(
    p1_lat: f64, p1_lng: f64,
    ctrl_lat: f64, ctrl_lng: f64,
    p2_lat: f64, p2_lng: f64,
) -> BezierPath {
    BezierPath::new(
        glam::DVec2::new(p1_lng  * GEO_LNG_M, p1_lat  * GEO_LAT_M),
        glam::DVec2::new(ctrl_lng * GEO_LNG_M, ctrl_lat * GEO_LAT_M),
        glam::DVec2::new(p2_lng  * GEO_LNG_M, p2_lat  * GEO_LAT_M),
    )
}

/// Convert a BezierPath `CarState` to (lat, lng, angle) using our angle convention.
/// BezierPath returns rotation = atan2(north, east); we need atan2(east, north).
#[inline]
fn bezier_state_to_geo(state: &crate::simulation::bezier_smooth::CarState) -> (f64, f64, f32) {
    use std::f64::consts::{FRAC_PI_2, PI, TAU};
    let lat = state.position.y / GEO_LAT_M;
    let lng = state.position.x / GEO_LNG_M;
    let raw = FRAC_PI_2 - state.rotation;
    let angle = (if raw > PI { raw - TAU } else { raw }) as f32;
    (lat, lng, angle)
}

#[inline]
fn normalize_xy(x: f64, y: f64) -> (f64, f64) {
    let len = (x * x + y * y).sqrt();
    if len <= 1e-9 {
        (0.0, 0.0)
    } else {
        (x / len, y / len)
    }
}

#[inline]
fn line_intersection(
    p1x: f64,
    p1y: f64,
    d1x: f64,
    d1y: f64,
    p2x: f64,
    p2y: f64,
    d2x: f64,
    d2y: f64,
) -> Option<(f64, f64)> {
    let det = d1x * d2y - d1y * d2x;
    if det.abs() < 1e-9 {
        return None;
    }
    let dx = p2x - p1x;
    let dy = p2y - p1y;
    let t = (dx * d2y - dy * d2x) / det;
    Some((p1x + t * d1x, p1y + t * d1y))
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

fn apply_connector_conflict_obstacle(
    vehicle: &Vehicle,
    base_obstacle: ClosestObstacle,
    conflict_system: &ConflictSystem,
    map: &MapData,
) -> ClosestObstacle {
    if !vehicle.on_turn_connector {
        return base_obstacle;
    }
    let in_e = EdgeIndex::new(vehicle.turn_from_edge);
    let out_e = EdgeIndex::new(vehicle.turn_to_edge);
    let Some((_, node_idx)) = map.graph.edge_endpoints(in_e) else {
        return base_obstacle;
    };
    let movement = (in_e, out_e);
    let half_len = vehicle.vehicle_type.params().length_m * 0.5;
    let s_front_arc = (vehicle.turn_dist_m as f32 + half_len).max(0.0);
    let vr = vehicle_path_radius_m(vehicle);
    let Some((block_dist, pt, owner)) = conflict_system.first_blocking_conflict_on_arc(
        movement,
        s_front_arc,
        CONFLICT_LOOKAHEAD_M,
        vehicle.id,
        vr,
        node_idx,
    ) else {
        return base_obstacle;
    };
    let threat = ClosestObstacle {
        kind: ObstacleKind::ConflictPoint,
        gap_m: block_dist,
        delta_v: vehicle.speed,
        point_lng_lat: Some(pt),
        leader_vehicle_id: None,
        conflict_reserver_id: Some(owner),
    };
    min_obstacle(base_obstacle, threat)
}

/// Apply traffic-light, stop-sign, and yield-sign braking effect.
///
/// - **Traffic light (red/yellow):** treat the stop-line as a stationary obstacle.
/// - **Stop sign:** slow to zero if not yet stopped; once stopped, allow proceed.
/// - **Yield sign:** cap approach speed to a low value within 20 m.
fn apply_intersection_effect(
    vehicle: &Vehicle,
    base_obstacle: ClosestObstacle,
    intersections: &IntersectionManager,
    conflict_system: &ConflictSystem,
    approach_tree: &RTree<ApproachIndexItem>,
    vehicles: &[Vehicle],
    map: &MapData,
) -> ClosestObstacle {
    if vehicle.route_pos >= vehicle.route.len() { return base_obstacle; }
    // Vehicle already past the stop line — Bezier path owns its motion.
    if vehicle.on_turn_connector { return base_obstacle; }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = match map.graph.edge_weight(edge_idx) {
        Some(e) => e,
        None    => return base_obstacle,
    };
    let dist_to_end = edge.length_m * (1.0 - vehicle.edge_progress);
    // IDM must see free space from the FRONT BUMPER to the stop line.
    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(vehicle, dist_to_end);

    let (tgt_node_idx, tgt_osm_id) = match map.graph.edge_endpoints(edge_idx) {
        Some((_, tgt)) => (tgt, map.graph[tgt].osm_id),
        None           => return base_obstacle,
    };
    let intersection_type = &map.graph[tgt_node_idx].intersection_type;
    let mut best = base_obstacle;

    if let Some((movement, conn)) = vehicle_next_movement(vehicle, map) {
        let dist_to_entry_center = ((conn.entry_progress - vehicle.edge_progress).max(0.0)
            * edge.length_m)
            .max(0.0);
        let half_len = vehicle.vehicle_type.params().length_m * 0.5;
        let dist_to_entry = (dist_to_entry_center - half_len).max(MIN_IDM_GAP_M);
        if dist_to_entry <= CONFLICT_PRIORITY_ACTIVATION_M {
            let pstate = conflict_system.check_priority(
                vehicle,
                movement.0,
                tgt_node_idx,
                map,
                vehicles,
                approach_tree,
            );
            if pstate == PriorityState::Wait {
                log::debug!(
                    "CP_WAIT veh={} node={} in_edge={} dist_to_entry={:.1}m",
                    vehicle.id,
                    map.graph[tgt_node_idx].osm_id,
                    edge_idx.index(),
                    dist_to_entry
                );
                let threat = ClosestObstacle {
                    kind: ObstacleKind::PriorityStopLine,
                    gap_m: dist_to_entry.min(best.gap_m).max(MIN_IDM_GAP_M),
                    delta_v: vehicle.speed,
                    point_lng_lat: Some([conn.p1_lng, conn.p1_lat]),
                    leader_vehicle_id: None,
                    conflict_reserver_id: None,
                };
                best = min_obstacle(best, threat);
            }
            if let Some((block_dist, pt, owner)) = conflict_system.first_blocking_conflict_distance(
                movement,
                dist_to_entry,
                CONFLICT_LOOKAHEAD_M,
                vehicle.id,
                vehicle_path_radius_m(vehicle),
                tgt_node_idx,
            ) {
                log::debug!(
                    "CP_BLOCKED veh={} node={} movement=({}->{}) block_dist={:.1}m",
                    vehicle.id,
                    map.graph[tgt_node_idx].osm_id,
                    movement.0.index(),
                    movement.1.index(),
                    block_dist
                );
                let threat = ClosestObstacle {
                    kind: ObstacleKind::ConflictPoint,
                    gap_m: block_dist.min(best.gap_m).max(MIN_IDM_GAP_M),
                    delta_v: vehicle.speed,
                    point_lng_lat: Some(pt),
                    leader_vehicle_id: None,
                    conflict_reserver_id: Some(owner),
                };
                best = min_obstacle(best, threat);
            }
        }
    }

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
            let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge.length_m.max(1.0)).clamp(0.0, 1.0) as f64;
            let src = map.graph.edge_endpoints(edge_idx).map(|e| e.0).unwrap_or(tgt_node_idx);
            let src_n = &map.graph[src];
            let tgt_n = &map.graph[tgt_node_idx];
            let stop_lat = src_n.lat + (tgt_n.lat - src_n.lat) * stop_t;
            let stop_lng = src_n.lng + (tgt_n.lng - src_n.lng) * stop_t;
            let threat = ClosestObstacle {
                kind: ObstacleKind::TrafficSignalStopLine,
                gap_m: dist_to_stop_line.max(MIN_IDM_GAP_M),
                delta_v: vehicle.speed,
                point_lng_lat: Some([stop_lng, stop_lat]),
                leader_vehicle_id: None,
                conflict_reserver_id: None,
            };
            best = min_obstacle(best, threat);
        }
    }

    // ── Stop sign ──────────────────────────────────────────────────────────
    // Must decelerate to full stop within 8 m of the stop line.
    // `has_stopped_at_stop_sign` is set by `apply_vehicle_physics` once the
    // vehicle reaches speed < 0.3 m/s.  After stopping, the vehicle may proceed.
    if matches!(intersection_type, IntersectionType::Stop) {
        if dist_to_end <= 15.0 && !vehicle.has_stopped_at_stop_sign {
            let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge.length_m.max(1.0)).clamp(0.0, 1.0) as f64;
            let src = map.graph.edge_endpoints(edge_idx).map(|e| e.0).unwrap_or(tgt_node_idx);
            let src_n = &map.graph[src];
            let tgt_n = &map.graph[tgt_node_idx];
            let stop_lat = src_n.lat + (tgt_n.lat - src_n.lat) * stop_t;
            let stop_lng = src_n.lng + (tgt_n.lng - src_n.lng) * stop_t;
            let threat = ClosestObstacle {
                kind: ObstacleKind::StopSignStopLine,
                gap_m: dist_to_stop_line.max(MIN_IDM_GAP_M),
                delta_v: vehicle.speed,
                point_lng_lat: Some([stop_lng, stop_lat]),
                leader_vehicle_id: None,
                conflict_reserver_id: None,
            };
            best = min_obstacle(best, threat);
        }
    }

    // ── Yield / give-way sign ──────────────────────────────────────────────
    // Slow to ≤ 5 km/h (1.39 m/s) within 20 m of the junction.
    if matches!(intersection_type, IntersectionType::Yield) {
        const YIELD_SPEED: f32 = 1.39; // 5 km/h
        if dist_to_end <= 20.0 && vehicle.speed > YIELD_SPEED {
            // Treat the junction entry as a slow virtual leader.
            let jn = &map.graph[tgt_node_idx];
            let threat = ClosestObstacle {
                kind: ObstacleKind::YieldTarget,
                gap_m: dist_to_stop_line.max(0.5),
                delta_v: vehicle.speed - YIELD_SPEED,
                point_lng_lat: Some([jn.lng, jn.lat]),
                leader_vehicle_id: None,
                conflict_reserver_id: None,
            };
            best = min_obstacle(best, threat);
        }
    }

    best
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
    // Already on the connector — cross-traffic braking not applicable.
    if ego.on_turn_connector { return (gap, delta_v); }
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
    conflict_system: &mut ConflictSystem,
    _by_target_node: &HashMap<NodeIndex, Vec<usize>>,
    approach_tree: &RTree<ApproachIndexItem>,
    vehicles: &[Vehicle],
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
    // Frozen on turn connector: target_lane may change mid-turn (compute for
    // next segment) and would cause current_lateral_offset to oscillate,
    // which the renderer then shows as perpendicular flicker.
    vehicle.target_lateral_offset = vehicle.target_lane as f32;
    if !vehicle.on_turn_connector {
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

    // ── Turn connector (Bezier arc-length) state machine ──────────────────
    if vehicle.on_turn_connector {
        conflict_system.release_passed_for_vehicle(
            vehicle.id,
            vehicle.turn_from_edge,
            vehicle.turn_to_edge,
            vehicle.turn_dist_m as f32,
            vehicle_path_radius_m(vehicle),
        );
        // Build arc-length-parameterised path in local Cartesian metres.
        let path = bezier_path_from_geo(
            vehicle.turn_p1_lat, vehicle.turn_p1_lng,
            vehicle.turn_ctrl_lat, vehicle.turn_ctrl_lng,
            vehicle.turn_p2_lat, vehicle.turn_p2_lng,
        );

        // Advance by exact physical distance — constant speed along the arc.
        vehicle.turn_dist_m = (vehicle.turn_dist_m
            + vehicle.speed as f64 * real_dt_s as f64)
            .min(path.total_length);

        // Query position + heading at the new arc-length distance.
        let state = path.get_state(vehicle.turn_dist_m);
        let (lat, lng, angle) = bezier_state_to_geo(&state);
        vehicle.lat   = lat;
        vehicle.lng   = lng;
        vehicle.angle = angle;

        // Keep edge_progress monotonic for leader logic while on the connector.
        let frac = (vehicle.turn_dist_m / path.total_length) as f32;
        vehicle.edge_progress =
            vehicle.turn_entry_progress + (1.0 - vehicle.turn_entry_progress) * frac;

        if vehicle.turn_dist_m >= path.total_length {
            conflict_system.release_all_for_vehicle(vehicle.id);
            vehicle.on_turn_connector = false;
            vehicle.turn_dist_m = 0.0;
            vehicle.turn_from_edge = 0;
            vehicle.turn_to_edge = 0;
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
    // Skip entirely when the vehicle is already on a turn connector (past the stop line).
    if !vehicle.on_turn_connector {
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
    } // end !on_turn_connector guard

    if let Some(conn) = planned_turn_connector(vehicle, map) {
        if let Some((movement, _)) = vehicle_next_movement(vehicle, map) {
            let pstate = conflict_system.check_priority(
                vehicle,
                movement.0,
                tgt_idx,
                map,
                vehicles,
                approach_tree,
            );
            if pstate != PriorityState::Wait {
                conflict_system.reserve_for_vehicle(
                    vehicle.id,
                    movement,
                    0.0,
                    CONFLICT_LOOKAHEAD_M,
                    vehicle_path_radius_m(vehicle),
                    tgt_idx,
                );
            }
        }
        if vehicle.edge_progress >= conn.entry_progress {
            let p = vehicle.vehicle_type.params();
            let shape_radius = vehicle_path_radius_m(vehicle);
            log::debug!(
                "CP_SHAPE veh={} type={:?} L={:.2}m W={:.2}m radius={:.2}m",
                vehicle.id,
                vehicle.vehicle_type,
                p.length_m,
                p.width_m,
                shape_radius
            );
            vehicle.on_turn_connector = true;
            vehicle.turn_dist_m = 0.0;
            vehicle.turn_from_edge = edge_idx.index();
            if vehicle.route_pos + 1 < vehicle.route.len() {
                vehicle.turn_to_edge = vehicle.route[vehicle.route_pos + 1].index();
            }
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

            // Snap lateral offset to target immediately so it doesn't drift
            // perpendicularly to the Bezier while animating during the turn.
            vehicle.current_lateral_offset = vehicle.target_lateral_offset;

            // Immediately place vehicle at arc start using the same BezierPath so
            // position/heading are consistent with the traversal logic (avoids 1-frame snap).
            let path0 = bezier_path_from_geo(
                conn.p1_lat, conn.p1_lng,
                conn.ctrl_lat, conn.ctrl_lng,
                conn.p2_lat, conn.p2_lng,
            );
            let state0 = path0.get_state(0.0);
            let (lat0, lng0, angle0) = bezier_state_to_geo(&state0);
            vehicle.lat   = lat0;
            vehicle.lng   = lng0;
            vehicle.angle = angle0;
            // Do NOT fall through to linear interpolation — Bezier takes over next tick.
            return;
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

    // Linear position interpolation (straight road segments only).
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

#[derive(Debug, Clone)]
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

#[inline]
fn is_major_road(road_type: &str) -> bool {
    matches!(road_type, "motorway" | "trunk" | "primary" | "secondary")
}

#[inline]
fn right_incoming_edge(incoming_cw: &[EdgeIndex], ego_in: EdgeIndex) -> Option<EdgeIndex> {
    let idx = incoming_cw.iter().position(|&e| e == ego_in)?;
    if incoming_cw.is_empty() {
        return None;
    }
    let right_idx = if idx == 0 { incoming_cw.len() - 1 } else { idx - 1 };
    incoming_cw.get(right_idx).copied()
}

#[inline]
fn vehicle_path_radius_m(vehicle: &Vehicle) -> f32 {
    let p = vehicle.vehicle_type.params();
    0.5 * (p.length_m * p.length_m + p.width_m * p.width_m).sqrt()
}

#[inline]
fn obstacle_kind_label(kind: ObstacleKind) -> &'static str {
    match kind {
        ObstacleKind::Vehicle => "vehicle",
        ObstacleKind::ConflictPoint => "conflict_point",
        ObstacleKind::PriorityStopLine => "priority_stop_line",
        ObstacleKind::TrafficSignalStopLine => "traffic_signal_stop_line",
        ObstacleKind::StopSignStopLine => "stop_sign_stop_line",
        ObstacleKind::YieldTarget => "yield_target",
    }
}

/// UI line style from hood toward IDM braking target (`solid`|`dashed`|`thick`).
#[inline]
fn threat_line_style_label(kind: ObstacleKind) -> &'static str {
    match kind {
        ObstacleKind::Vehicle => "solid",
        ObstacleKind::ConflictPoint => "dashed",
        ObstacleKind::PriorityStopLine
        | ObstacleKind::TrafficSignalStopLine
        | ObstacleKind::StopSignStopLine
        | ObstacleKind::YieldTarget => "thick",
    }
}

#[inline]
fn min_obstacle(a: ClosestObstacle, b: ClosestObstacle) -> ClosestObstacle {
    if b.gap_m < a.gap_m { b } else { a }
}

#[inline]
fn compute_idm_accel_with_obstacle(vehicle: &Vehicle, desired: f32, obs: ClosestObstacle) -> f32 {
    let params = vehicle.driver_profile.params();
    let vtype  = vehicle.vehicle_type.params();
    idm_acceleration(vehicle.speed, desired, obs.gap_m, obs.delta_v, &params, &vtype)
}

fn compute_vehicle_idm_step(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    edge_lane_vehicles: &HashMap<(EdgeIndex, u8), Vec<usize>>,
    vehicles_by_target_node: &HashMap<NodeIndex, Vec<usize>>,
    tram_snapshot: &[(f64, f64, f32)],
    map: &MapData,
    intersections: &IntersectionManager,
    conflict_system: &ConflictSystem,
    approach_tree: &RTree<ApproachIndexItem>,
) -> IdmStepResult {
    let desired = compute_desired_speed(ego, map);

    if ego.on_turn_connector {
        let mut base = free_leader_obstacle();
        let (gap, dv) =
            apply_tram_leader_effect(ego, base.gap_m, base.delta_v, tram_snapshot);
        base.gap_m = gap;
        base.delta_v = dv;
        let obstacle =
            apply_connector_conflict_obstacle(ego, base, conflict_system, map);
        let accel = compute_idm_accel_with_obstacle(ego, desired, obstacle);
        return IdmStepResult {
            accel,
            desired_speed: desired,
            obstacle,
        };
    }

    let mut base_obstacle =
        find_leader_obstacle_arc(ego_idx, ego, vehicles, edge_lane_vehicles, map);
    let (gap, dv) = apply_tram_leader_effect(
        ego,
        base_obstacle.gap_m,
        base_obstacle.delta_v,
        tram_snapshot,
    );
    base_obstacle.gap_m = gap;
    base_obstacle.delta_v = dv;
    let (gap, dv) = apply_cross_traffic_leader_effect(
        ego_idx,
        ego,
        vehicles,
        vehicles_by_target_node,
        map,
        intersections,
        base_obstacle.gap_m,
        base_obstacle.delta_v,
    );
    base_obstacle.gap_m = gap;
    base_obstacle.delta_v = dv;

    let obstacle = apply_intersection_effect(
        ego,
        base_obstacle,
        intersections,
        conflict_system,
        approach_tree,
        vehicles,
        map,
    );
    let accel = compute_idm_accel_with_obstacle(ego, desired, obstacle);

    IdmStepResult {
        accel,
        desired_speed: desired,
        obstacle,
    }
}

#[inline]
fn vehicle_next_movement(vehicle: &Vehicle, map: &MapData) -> Option<((EdgeIndex, EdgeIndex), PlannedTurnConnector)> {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return None;
    }
    let in_edge = vehicle.route[vehicle.route_pos];
    let out_edge = vehicle.route[vehicle.route_pos + 1];
    let conn = planned_turn_connector(vehicle, map)?;
    Some(((in_edge, out_edge), conn))
}

impl ConflictSystem {
    fn update_deadlock_state(
        &mut self,
        map: &MapData,
        vehicles: &[Vehicle],
        by_target_node: &HashMap<NodeIndex, Vec<usize>>,
        approach_tree: &RTree<ApproachIndexItem>,
        dt_s: f32,
    ) {
        for (node_idx, data) in self.nodes.iter_mut() {
            let Some(candidates) = by_target_node.get(node_idx) else {
                data.deadlock_timer_s = 0.0;
                data.deadlock_first_move = None;
                continue;
            };
            let waiting: Vec<u32> = candidates
                .iter()
                .filter_map(|&i| {
                    let v = vehicles.get(i)?;
                    if v.route_pos + 1 >= v.route.len() || v.on_turn_connector {
                        return None;
                    }
                    let in_edge = v.route[v.route_pos];
                    if Self::check_priority_for_node(data, v, in_edge, *node_idx, map, vehicles, approach_tree)
                        == PriorityState::Wait
                    {
                        Some(v.id)
                    } else {
                        None
                    }
                })
                .collect();
            if waiting.len() >= 4 {
                data.deadlock_timer_s += dt_s;
                if data.deadlock_timer_s >= DEADLOCK_BREAK_S {
                    let next = waiting.iter().copied().min();
                    if data.deadlock_first_move != next {
                        log::info!(
                            "CP_DEADLOCK_BREAK node={} waited={:.1}s first_move={:?} waiting_count={}",
                            map.graph[*node_idx].osm_id,
                            data.deadlock_timer_s,
                            next,
                            waiting.len()
                        );
                    }
                    data.deadlock_first_move = next;
                }
            } else {
                data.deadlock_timer_s = 0.0;
                data.deadlock_first_move = None;
            }
        }
    }

    fn check_priority(
        &self,
        vehicle: &Vehicle,
        in_edge: EdgeIndex,
        node_idx: NodeIndex,
        map: &MapData,
        vehicles: &[Vehicle],
        approach_tree: &RTree<ApproachIndexItem>,
    ) -> PriorityState {
        let Some(node_data) = self.nodes.get(&node_idx) else {
            return PriorityState::Clear;
        };
        Self::check_priority_for_node(node_data, vehicle, in_edge, node_idx, map, vehicles, approach_tree)
    }

    fn check_priority_for_node(
        node_data: &IntersectionConflictData,
        vehicle: &Vehicle,
        in_edge: EdgeIndex,
        node_idx: NodeIndex,
        map: &MapData,
        vehicles: &[Vehicle],
        approach_tree: &RTree<ApproachIndexItem>,
    ) -> PriorityState {
        if node_data.deadlock_first_move == Some(vehicle.id) {
            return PriorityState::FirstMove;
        }
        let Some(right_edge) = right_incoming_edge(&node_data.incoming_cw, in_edge) else {
            return PriorityState::Clear;
        };
        let in_road = map.graph.edge_weight(in_edge).map(|e| e.road_type.as_str()).unwrap_or("tertiary");
        let ego_major = is_major_road(in_road);
        let center = &map.graph[node_idx];
        let (cx, cy) = geo_to_m_xy(center.lat, center.lng);
        let search = [cx, cy];
        for item in approach_tree.locate_within_distance(search, RIGHT_HAND_CHECK_DIST_M as f64) {
            let Some(other) = vehicles.get(item.vehicle_idx) else { continue; };
            if other.id == vehicle.id || other.route_pos >= other.route.len() || other.on_turn_connector {
                continue;
            }
            let other_edge = other.route[other.route_pos];
            if other_edge != right_edge {
                continue;
            }
            let other_road = map.graph.edge_weight(other_edge).map(|e| e.road_type.as_str()).unwrap_or("tertiary");
            let other_major = is_major_road(other_road);
            if ego_major && !other_major {
                continue;
            }
            if !ego_major && other_major {
                return PriorityState::Wait;
            }
            return PriorityState::Wait;
        }
        PriorityState::Clear
    }

    fn first_blocking_conflict_distance(
        &self,
        movement: (EdgeIndex, EdgeIndex),
        dist_to_entry: f32,
        look_ahead: f32,
        vehicle_id: u32,
        vehicle_radius_m: f32,
        node_idx: NodeIndex,
    ) -> Option<(f32, [f64; 2], u32)> {
        let node = self.nodes.get(&node_idx)?;
        let path = node.by_movement.get(&movement)?;
        path.points
            .iter()
            .filter_map(|p| {
                let owner = match p.reserved_by {
                    Some(o) if o != vehicle_id => o,
                    _ => return None,
                };
                let d_raw = dist_to_entry + p.distance_on_path;
                let d = (d_raw - vehicle_radius_m - CONFLICT_SHAPE_BUFFER_M).max(MIN_IDM_GAP_M);
                if d > look_ahead { return None; }
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                Some((d, [lng, lat], owner))
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
    }

    /// Bezier-turn connector: ego front-bumper arc length from Bezier start (p1).
    fn first_blocking_conflict_on_arc(
        &self,
        movement: (EdgeIndex, EdgeIndex),
        s_front_arc_m: f32,
        look_ahead: f32,
        vehicle_id: u32,
        vehicle_radius_m: f32,
        node_idx: NodeIndex,
    ) -> Option<(f32, [f64; 2], u32)> {
        let node = self.nodes.get(&node_idx)?;
        let path = node.by_movement.get(&movement)?;
        path.points
            .iter()
            .filter_map(|p| {
                let owner = match p.reserved_by {
                    Some(o) if o != vehicle_id => o,
                    _ => return None,
                };
                let d_raw = p.distance_on_path - s_front_arc_m;
                if d_raw < 0.0 {
                    return None;
                }
                let d = (d_raw - vehicle_radius_m - CONFLICT_SHAPE_BUFFER_M).max(MIN_IDM_GAP_M);
                if d > look_ahead {
                    return None;
                }
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                Some((d, [lng, lat], owner))
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
    }

    fn debug_conflict_points_snapshot(&self) -> Vec<DebugConflictPointPayload> {
        let mut out = Vec::new();
        for node in self.nodes.values() {
            for path in node.by_movement.values() {
                for p in &path.points {
                    let lng = p.pos.x / GEO_LNG_M;
                    let lat = p.pos.y / GEO_LAT_M;
                    out.push(DebugConflictPointPayload {
                        lng,
                        lat,
                        reserved_by: p.reserved_by,
                    });
                }
            }
        }
        out
    }

    fn reserve_for_vehicle(
        &mut self,
        vehicle_id: u32,
        movement: (EdgeIndex, EdgeIndex),
        current_dist_on_path: f32,
        look_ahead: f32,
        vehicle_radius_m: f32,
        node_idx: NodeIndex,
    ) {
        let Some(node) = self.nodes.get_mut(&node_idx) else { return; };
        let Some(path) = node.by_movement.get_mut(&movement) else { return; };
        let reserve_from = (current_dist_on_path - vehicle_radius_m - CONFLICT_SHAPE_BUFFER_M).max(0.0);
        let reserve_to = current_dist_on_path + look_ahead + vehicle_radius_m + CONFLICT_SHAPE_BUFFER_M;
        for p in &mut path.points {
            if p.distance_on_path < reserve_from {
                continue;
            }
            if p.distance_on_path > reserve_to {
                break;
            }
            if p.reserved_by.is_none() || p.reserved_by == Some(vehicle_id) {
                if p.reserved_by != Some(vehicle_id) {
                    log::debug!(
                        "CP_RESERVE veh={} node={} movement=({}->{}) at={:.1}m",
                        vehicle_id,
                        node_idx.index(),
                        movement.0.index(),
                        movement.1.index(),
                        p.distance_on_path
                    );
                }
                p.reserved_by = Some(vehicle_id);
            }
        }
    }

    fn release_passed_for_vehicle(
        &mut self,
        vehicle_id: u32,
        from_edge: usize,
        to_edge: usize,
        current_dist_on_path: f32,
        vehicle_radius_m: f32,
    ) {
        let in_edge = EdgeIndex::new(from_edge);
        let out_edge = EdgeIndex::new(to_edge);
        for node in self.nodes.values_mut() {
            if let Some(path) = node.by_movement.get_mut(&(in_edge, out_edge)) {
                for p in &mut path.points {
                    if p.reserved_by == Some(vehicle_id)
                        && (current_dist_on_path - vehicle_radius_m)
                            > p.distance_on_path + CONFLICT_RELEASE_BUFFER_M
                    {
                        log::debug!(
                            "CP_RELEASE veh={} movement=({}->{}) passed={:.1}m point={:.1}m",
                            vehicle_id,
                            from_edge,
                            to_edge,
                            current_dist_on_path,
                            p.distance_on_path
                        );
                        p.reserved_by = None;
                    }
                }
            }
        }
    }

    fn release_all_for_vehicle(&mut self, vehicle_id: u32) {
        for node in self.nodes.values_mut() {
            for path in node.by_movement.values_mut() {
                for p in &mut path.points {
                    if p.reserved_by == Some(vehicle_id) {
                        p.reserved_by = None;
                    }
                }
            }
        }
    }
}

fn planned_turn_connector(vehicle: &Vehicle, map: &MapData) -> Option<PlannedTurnConnector> {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return None;
    }
    connector_for_movement(
        map,
        vehicle.route[vehicle.route_pos],
        vehicle.route[vehicle.route_pos + 1],
    )
}

fn connector_for_movement(map: &MapData, in_edge: EdgeIndex, out_edge: EdgeIndex) -> Option<PlannedTurnConnector> {
    let (src, junction) = map.graph.edge_endpoints(in_edge)?;
    let (next_src, next_tgt) = map.graph.edge_endpoints(out_edge)?;
    if junction != next_src {
        return None;
    }
    let curr_len = map.graph.edge_weight(in_edge)?.length_m.max(1.0);
    let next_len = map.graph.edge_weight(out_edge)?.length_m.max(1.0);
    let src_n = &map.graph[src];
    let jn = &map.graph[junction];
    let tgt_n = &map.graph[next_tgt];
    let in_x = (jn.lng - src_n.lng) as f32;
    let in_y = (jn.lat - src_n.lat) as f32;
    let out_x = (tgt_n.lng - jn.lng) as f32;
    let out_y = (tgt_n.lat - jn.lat) as f32;
    let in_len = (in_x * in_x + in_y * in_y).sqrt().max(1e-6);
    let out_len = (out_x * out_x + out_y * out_y).sqrt().max(1e-6);
    let dot = ((in_x / in_len) * (out_x / out_len) + (in_y / in_len) * (out_y / out_len)).clamp(-1.0, 1.0);
    let angle = dot.acos();
    if angle < TURN_CONNECTOR_MIN_ANGLE_RAD {
        return None;
    }
    let entry_progress = (1.0 - TURN_CONNECTOR_ENTRY_M / curr_len).clamp(0.0, 1.0);
    let exit_progress = (TURN_CONNECTOR_EXIT_M / next_len).clamp(0.0, 1.0);
    let p1_lat = src_n.lat + (jn.lat - src_n.lat) * entry_progress as f64;
    let p1_lng = src_n.lng + (jn.lng - src_n.lng) * entry_progress as f64;
    let p2_lat = jn.lat + (tgt_n.lat - jn.lat) * exit_progress as f64;
    let p2_lng = jn.lng + (tgt_n.lng - jn.lng) * exit_progress as f64;
    let (in_fx, in_fy) = normalize_xy((jn.lng - src_n.lng) * GEO_LNG_M, (jn.lat - src_n.lat) * GEO_LAT_M);
    let (out_fx, out_fy) = normalize_xy((tgt_n.lng - jn.lng) * GEO_LNG_M, (tgt_n.lat - jn.lat) * GEO_LAT_M);
    let (p1x, p1y) = geo_to_m_xy(p1_lat, p1_lng);
    let (p2x, p2y) = geo_to_m_xy(p2_lat, p2_lng);
    let (ctrl_lat, ctrl_lng) = if let Some((cx, cy)) =
        line_intersection(p1x, p1y, in_fx, in_fy, p2x, p2y, -out_fx, -out_fy)
    {
        m_xy_to_geo(cx, cy)
    } else {
        (jn.lat, jn.lng)
    };
    let length_m = bezier_length_m(p1_lat, p1_lng, ctrl_lat, ctrl_lng, p2_lat, p2_lng, 24);
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

fn build_conflict_system(map: &MapData) -> ConflictSystem {
    let mut nodes: HashMap<NodeIndex, IntersectionConflictData> = HashMap::new();
    for node in map.graph.node_indices() {
        let incoming: Vec<EdgeIndex> = map.graph
            .edges_directed(node, petgraph::Direction::Incoming)
            .map(|e| e.id())
            .collect();
        let outgoing: Vec<EdgeIndex> = map.graph
            .edges_directed(node, petgraph::Direction::Outgoing)
            .map(|e| e.id())
            .collect();
        if incoming.len() < 2 || outgoing.is_empty() {
            continue;
        }
        let mut incoming_cw = incoming.clone();
        incoming_cw.sort_by(|a, b| {
            let aa = approach_angle_to_node(map, *a, node);
            let bb = approach_angle_to_node(map, *b, node);
            bb.partial_cmp(&aa).unwrap_or(Ordering::Equal)
        });
        let mut by_movement: HashMap<(EdgeIndex, EdgeIndex), ConflictPath> = HashMap::new();
        for &in_edge in &incoming {
            for &out_edge in &outgoing {
                if let Some(c) = connector_for_movement(map, in_edge, out_edge) {
                    by_movement.insert((in_edge, out_edge), ConflictPath {
                        bezier: c,
                        points: Vec::new(),
                    });
                }
            }
        }
        build_conflicts_for_node(&mut by_movement);
        let conflict_count: usize = by_movement.values().map(|p| p.points.len()).sum();
        if !by_movement.is_empty() {
            log::info!(
                "CP_INIT node={} movements={} conflict_points={}",
                map.graph[node].osm_id,
                by_movement.len(),
                conflict_count
            );
        }
        nodes.insert(node, IntersectionConflictData {
            by_movement,
            incoming_cw,
            deadlock_timer_s: 0.0,
            deadlock_first_move: None,
        });
    }
    ConflictSystem { nodes }
}

fn approach_angle_to_node(map: &MapData, in_edge: EdgeIndex, node: NodeIndex) -> f64 {
    let Some((src, tgt)) = map.graph.edge_endpoints(in_edge) else { return 0.0; };
    if tgt != node {
        return 0.0;
    }
    let s = &map.graph[src];
    let n = &map.graph[node];
    let (dx, dy) = (n.lng - s.lng, n.lat - s.lat);
    dy.atan2(dx)
}

fn build_conflicts_for_node(by_movement: &mut HashMap<(EdgeIndex, EdgeIndex), ConflictPath>) {
    let keys: Vec<(EdgeIndex, EdgeIndex)> = by_movement.keys().copied().collect();
    for i in 0..keys.len() {
        for j in (i + 1)..keys.len() {
            let k1 = keys[i];
            let k2 = keys[j];
            let (poly1, dist1) = sample_connector_polyline(&by_movement[&k1].bezier, 20);
            let (poly2, dist2) = sample_connector_polyline(&by_movement[&k2].bezier, 20);
            for a in 0..(poly1.len().saturating_sub(1)) {
                for b in 0..(poly2.len().saturating_sub(1)) {
                    if let Some((p, da, db)) = polyline_segment_intersection(
                        poly1[a], poly1[a + 1], dist1[a], dist1[a + 1],
                        poly2[b], poly2[b + 1], dist2[b], dist2[b + 1],
                    ) {
                        if let Some(path) = by_movement.get_mut(&k1) {
                            path.points.push(ConflictPoint { pos: p, distance_on_path: da, reserved_by: None });
                        }
                        if let Some(path) = by_movement.get_mut(&k2) {
                            path.points.push(ConflictPoint { pos: p, distance_on_path: db, reserved_by: None });
                        }
                    }
                }
            }
        }
    }
    for path in by_movement.values_mut() {
        path.points.sort_by(|a, b| a.distance_on_path.partial_cmp(&b.distance_on_path).unwrap_or(Ordering::Equal));
        path.points.dedup_by(|a, b| {
            (a.distance_on_path - b.distance_on_path).abs() < 0.5
                && (a.pos - b.pos).length_squared() < 1.0
        });
    }
}

fn sample_connector_polyline(conn: &PlannedTurnConnector, samples: usize) -> (Vec<DVec2>, Vec<f32>) {
    let n = samples.max(8);
    let mut pts = Vec::with_capacity(n + 1);
    let mut dists = Vec::with_capacity(n + 1);
    let path = bezier_path_from_geo(
        conn.p1_lat, conn.p1_lng,
        conn.ctrl_lat, conn.ctrl_lng,
        conn.p2_lat, conn.p2_lng,
    );
    let total = path.total_length.max(0.1) as f32;
    for i in 0..=n {
        let d = total * (i as f32 / n as f32);
        let state = path.get_state(d as f64);
        let p = DVec2::new(state.position.x, state.position.y);
        pts.push(p);
        dists.push(d);
    }
    (pts, dists)
}

fn polyline_segment_intersection(
    a0: DVec2, a1: DVec2, da0: f32, da1: f32,
    b0: DVec2, b1: DVec2, db0: f32, db1: f32,
) -> Option<(DVec2, f32, f32)> {
    let sa = Segment::new(parry2d::na::Point2::new(a0.x as f32, a0.y as f32), parry2d::na::Point2::new(a1.x as f32, a1.y as f32));
    let sb = Segment::new(parry2d::na::Point2::new(b0.x as f32, b0.y as f32), parry2d::na::Point2::new(b1.x as f32, b1.y as f32));
    if !intersection_test(&parry2d::na::Isometry2::identity(), &sa, &parry2d::na::Isometry2::identity(), &sb).ok()? {
        return None;
    }
    let ad = a1 - a0;
    let bd = b1 - b0;
    let det = ad.x * bd.y - ad.y * bd.x;
    if det.abs() < 1e-9 {
        return None;
    }
    let rel = b0 - a0;
    let ta = ((rel.x * bd.y - rel.y * bd.x) / det).clamp(0.0, 1.0);
    let tb = ((rel.x * ad.y - rel.y * ad.x) / det).clamp(0.0, 1.0);
    let p = a0 + ad * ta;
    let da = da0 + (da1 - da0) * ta as f32;
    let db = db0 + (db1 - db0) * tb as f32;
    Some((p, da, db))
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
/// Per-vehicle layout (40 bytes):
/// ```text
///   [0..3]   id:              u32  LE
///   [4..11]  lat:             f64  LE  ← full double precision
///   [12..19] lng:             f64  LE  ← full double precision
///   [20..23] angle:           f32  LE
///   [24..27] speed:           f32  LE
///   [28]     type:            u8   (0=Car, 1=Van, 2=Bus, 3=Truck, 4=Tram)
///   [29]     profile:         u8
///   [30]     trip_kind:       u8   (0=local_od, 1=transit, 2=ext_in, 3=ext_out)
///   [31]     lane_flags:      u8   (bits 0..6 lane index, bit7 on_turn_connector)
///   [32..35] frustration:     f32  LE  (0=calm, 100=rage)
///   [36..39] lateral_offset:  f32  LE  (smooth lane pos: 0.0=lane-0 centre, 1.0=lane-1 …)
/// ```
fn serialize_vehicles(vehicles: &[Vehicle], tram_sim: &TramSim) -> Vec<u8> {
    let total = vehicles.len() + tram_sim.trams.len();
    let mut buf = Vec::with_capacity(total * 40);

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

/// Per-vehicle binary packet layout (40 bytes):
/// ```text
///   [0..3]   id:              u32 LE
///   [4..11]  lat:             f64 LE   ← full double precision, eliminates ~0.4 m f32 quantisation noise
///   [12..19] lng:             f64 LE   ← full double precision
///   [20..23] angle:           f32 LE
///   [24..27] speed:           f32 LE
///   [28]     vehicle_type:    u8
///   [29]     driver_profile:  u8
///   [30]     trip_kind:       u8
///   [31]     lane_flags:      u8   (bits 0..6 lane index, bit7 on_turn_connector)
///   [32..35] frustration:     f32 LE
///   [36..39] lateral_offset:  f32 LE (smooth: 0.0=lane-0, 1.0=lane-1, …)
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
    buf.extend_from_slice(&id.to_le_bytes());          // [0..3]
    buf.extend_from_slice(&lat.to_le_bytes());         // [4..11]  f64
    buf.extend_from_slice(&lng.to_le_bytes());         // [12..19] f64
    buf.extend_from_slice(&angle.to_le_bytes());       // [20..23]
    buf.extend_from_slice(&speed.to_le_bytes());       // [24..27]
    buf.push(vtype);                                   // [28]
    buf.push(profile);                                 // [29]
    buf.push(trip_kind);                               // [30]
    let lane_flags = (current_lane & 0x7f) | if on_turn_connector { 0x80 } else { 0 };
    buf.push(lane_flags);                              // [31]
    buf.extend_from_slice(&frustration.to_le_bytes()); // [32..35]
    buf.extend_from_slice(&lateral_offset.to_le_bytes()); // [36..39]
}
