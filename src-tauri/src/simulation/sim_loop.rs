use base64::Engine;
use parking_lot::RwLock;
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rayon::prelude::*;
use serde::Serialize;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tauri::ipc::Channel;
use tauri::{AppHandle, Emitter};

use crate::map::road_network::{
    lane_connector_cubic, IntersectionType, Lane, LaneId, MapData, CONNECTOR_ARCLEN_ACC,
};
use crate::simulation::bezier_smooth::BezierPath;
use crate::simulation::congestion::compute_congestion;
use crate::simulation::idm::idm_acceleration;
use crate::simulation::lane_change::{compute_vehicle_target_lane, decide_lane_change};
use crate::simulation::od_model::OdModel;
use crate::simulation::spawn::SpawnSystem;
use crate::simulation::speed_config::SpeedConfig;
use crate::simulation::tram_sim::TramSim;
use crate::state::SimCommand;
use crate::time::day_cycle::DayCycle;
use crate::time::game_clock::GameClock;
use crate::traffic::intersection::IntersectionManager;
use crate::vehicles::vehicle::Vehicle;
use glam::DVec2;
use kurbo::{ParamCurve, ParamCurveArclen, ParamCurveDeriv};
use parry2d::query::intersection_test;
use parry2d::shape::Segment;


const TARGET_TICK_S: f32 = 1.0 / 60.0;
const CONGESTION_INTERVAL_S: f32 = 0.5;

/// Distance along an incoming edge from the **intersection node** to the stop line / virtual leader.
/// Larger value = stop farther **before** the junction (vehicles do not enter the crossing on red).
const STOP_LINE_OFFSET_M: f32 = 16.0;

/// Smallest admissible IDM free gap to avoid divide-by-zero and explosive braking.
const MIN_IDM_GAP_M: f32 = 0.1;
/// Additional spawn breathing room after bumper gap has been computed.
const SPAWN_BUFFER_M: f32 = 2.0;

/// Only consider cross-traffic IDM within this distance to the node (meters).
const CROSS_TRAFFIC_MAX_DIST_M: f32 = 70.0;
// Must be at/after stop line (STOP_LINE_OFFSET_M = 16 m) so we never enter a turn arc on red.
const TURN_CONNECTOR_ENTRY_M: f32 = 12.0;
const TURN_CONNECTOR_EXIT_M: f32 = 30.0;
const TURN_CONNECTOR_MIN_ANGLE_RAD: f32 = 0.35;
const TURN_CONNECTOR_TARGET_SPEED_MPS: f32 = 25.0 / 3.6; // 6.94 m/s = 25 km/h on arc
/// After leaving a geometric connector, `closest_arclength_m_on_lane_path` can snap to the **end**
/// of the outbound lane polyline (`lane_progress ≈ length_m` ⇒ `edge_progress ≈ 1`). The next
/// physics tick then hits `edge_progress >= 1` route-advance logic while the chassis is still on
/// the first post-junction segment — route/lane-route desync → standing still / “zero headway”.
const POST_CONNECTOR_LANE_END_MARGIN_M: f32 = 2.8;
/// Beyond this perpendicular distance to the lane polyline we still keep the vehicle attached by
/// snapping arclength to the nearer endpoint ("wide road bounds") instead of returning `None`.
const LANE_PATH_PROJECTION_CONFIDENCE_M: f64 = 2.5;
const STEERING_LOOKAHEAD_M: f32 = 7.0;
/// Comfortable deceleration for UI “required braking distance” (m/s²).
const IDM_UI_COMFORT_DECEL_MPS2: f32 = 3.5;
/// Speed below which we label the manoeuvre STOP in the HUD.
const IDM_UI_STOP_SPEED_MPS: f32 = 0.2;
const IDM_UI_BRAKE_ACCEL_THRESHOLD: f32 = -0.45;
const IDM_UI_COAST_ACCEL_THRESHOLD: f32 = -0.12;
const IDM_UI_TTC_MIN_CLOSING_MPS: f32 = 0.08;
const GEO_LAT_M: f64 = 111_320.0;
const GEO_LNG_M: f64 = 71_700.0;
const CONFLICT_LOOKAHEAD_M: f32 = 45.0;
const CONFLICT_PRIORITY_ACTIVATION_M: f32 = 35.0;
const CONFLICT_SHAPE_BUFFER_M: f32 = 1.0;
/// On **lane approach** (pre-connector arc length metric): guard hard overlap stops to small `d_raw`.
const CONFLICT_OVERLAP_HARD_STOP_D_RAW_CAP_APPROACH_M: f32 = 14.0;
/// On the **Bezier/cubic connector** `turn_dist_m` can disagree with conflict `distance_on_path`
/// sampling; allow larger `d_raw` before coupling overlap → MIN gap. Hood-only overlap still
/// prevents phantom stops from Rear-of-OBB grazing a patch aft of the nose.
const CONFLICT_OVERLAP_HARD_STOP_D_RAW_CAP_CONNECTOR_M: f32 = 28.0;
/// Extra metres past [`conflict_scan_distance_m`] before connector conflict/yield probes engage.
/// A bad projected `entry_progress` can hug 0 along a long edge so `dist_to_entry` clips to MIN_IDM
/// even though the junction is far — IDM sees a phantom `ConflictPoint`.
const CONFLICT_APPROACH_TAIL_M: f32 = 60.0;
const CONFLICT_SCAN_SAFETY_MARGIN_M: f32 = 12.0;
/// Euclidean hood→rear cap beyond arc gap before we drop a bogus lane-route match (large gaps).
const LEADER_ARC_GEO_MAX_SLACK_M: f32 = 36.0;
/// Below this nominal bumper gap, require hood→rear Euclidean distance to sit close to the path
/// metric. Large `slack = gap + MAX` alone keeps ~36 m elbow room — enough to accept phantom
/// “0 m leaders” whose polyline/arclength glues wrong at connectors while cars are tens of metres
/// apart in the map.
const LEADER_ARC_GEO_NEAR_GAP_THRESHOLD_M: f32 = 18.0;
const LEADER_ARC_GEO_NEAR_SLACK_M: f32 = 6.5;
/// Past connector entry fraction: Plain cross-traffic `d_ego` surrogate must not imitate a phantom leader.
const CROSS_TRAFFIC_ENTRY_COMMIT_EPS: f32 = 0.0008;
/// IDM may run earlier in the frame than physics flips [`Vehicle::on_turn_connector`]; if `edge_progress`
/// already crossed connector entry along the inbound edge but the flag is still false, scanning
/// [`find_leader_obstacle_arc`] invents phantom “Vehicle ahead”. Match physics’ connector conflict scan.
const IDM_CONNECTOR_ENTRY_COMMIT_EPS: f32 = 0.001;
/// Reject buggy `planned_turn_connector.entry_progress≈0` from P1 / lane mismatch — otherwise synthetic
/// IDM thinks the vehicle is \"past connector entry\" for whole blocks and freezes mid-leg.
const IDM_CONNECTOR_SYNTHETIC_ENTRY_MIN_FRAC: f32 = 0.04;
/// Synthetic connector-IDM only within this distance (m) of the **downstream** junction of the current route edge.
const IDM_CONNECTOR_SYNTHETIC_APPROACH_GATE_M: f32 = 130.0;
const CONFLICT_TTL_STALLED_S: f32 = 10.0;
/// Release [`ConflictPoint`]s reserved by ego once the vehicle **rear bumper** arc distance on the
/// connector exceeds the patch arc position by at least this many metres (`s_rear > distance + …`).
/// (Intersection conflict reservations live on [`ConflictSystem`], not [`IntersectionManager`].)
const CONFLICT_CLEAN_REAR_PAST_POINT_M: f32 = 1.0;
/// At the stop line: only **claim** patches this far along the connector arc (m). Claiming the
/// whole intersection at once reserves the geometric centre for the whole turn and deadlocks followers.
const CONFLICT_RESERVE_INITIAL_ARC_M: f32 = 18.0;
/// While on the connector: extend reservations to this distance **ahead of vehicle centre** (m).
const CONFLICT_RESERVE_HORIZON_AHEAD_M: f32 = 22.0;
/// Release ego's patches once vehicle **centre** passes `distance_on_path` by this margin (stricter clear than rear-only).
const CONFLICT_CLEAN_CENTER_PAST_POINT_M: f32 = 0.35;
/// Extra envelope when deciding that a foreign vehicle centroid already occupies a conflict patch —
/// clears stale reservations (`reserved_by ≠ occupant`) so the occupant IDM yield does not deadlock.
const CONFLICT_PHYSICAL_PATCH_ENVELOPE_M: f32 = 0.85;
/// If centre-to-patch Euclidean distance clears this envelope, ignore 1-D arc blocker (Bezier vs sampled path mismatch mid-turn).
const CONFLICT_EUCLIDEAN_CLEAR_EXTRA_M: f32 = 3.25;
/// Trams behind ego must not shorten IDM gaps (false “Vehicle ahead”) — require forward metres along ego heading ≥ this.
const TRAM_LEADER_FORWARD_MIN_PROJ_M: f32 = 2.0;
const DEADLOCK_BREAK_S: f32 = 3.0;
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
    stop_line_point: Option<[f64; 2]>,
    turn_entry_point: Option<[f64; 2]>,
    hood_lng_lat: [f64; 2],
    rear_bumper_lng_lat: [f64; 2],
    look_ahead_point: Option<[f64; 2]>,
    look_ahead_distance_m: f32,
    current_lane_id: Option<u64>,
    target_lane: u8,
    next_turn_intent: String,
    idm_focus: String,
    route_points: Vec<[f64; 2]>,
    /// Quadratic Bezier P1 → control → P2 in \[lng, lat\] while `on_curve` (for debug arrows).
    bezier_control_path_lng_lat: Vec<[f64; 2]>,
    /// Planned lane graph segment ids from current lane onward (includes connectors).
    lane_route_ids: Vec<u64>,
    /// Short human-readable braking cause when acceleration is strongly negative.
    brake_reason: Option<String>,
    /// Discrete IDM/UI mode: GO / COAST / BRAKE / YIELD / STOP.
    idm_decision: String,
    /// Time-to-collision style metric: gap / closing speed when closing; None if not closing.
    ttc_seconds: Option<f32>,
    /// Comfortable stop distance v² / (2 b) using [`IDM_UI_COMFORT_DECEL_MPS2`].
    comfort_braking_distance_m: f32,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct LeaderDebugEntry {
    vehicle_id: u32,
    /// Vehicle leader encoded in the dominant IDM obstacle when it is a same-lane car (`vehicle`).
    idm_leader_vehicle_id: Option<u32>,
    /// Closest vehicle ahead in the same edge+lane bucket (physical queue predecessor).
    lane_leader_vehicle_id: Option<u32>,
    /// `true` when IDM follows a vehicle leader that is not the immediate predecessor — lane-route / sensor inconsistency.
    sensor_mismatch: bool,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct LeaderDebugPayload {
    entries: Vec<LeaderDebugEntry>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct LaneMovementKey {
    in_edge: EdgeIndex,
    out_edge: EdgeIndex,
    in_lane: u8,
    out_lane: u8,
}

#[derive(Debug, Clone)]
struct ConflictPoint {
    #[allow(dead_code)]
    id: u64,
    pos: DVec2,
    distance_on_path: f32,
    radius_m: f32,
    reserved_by: Option<u32>,
    reserved_at_game_s: Option<f32>,
    reserved_last_progress_m: Option<f32>,
    reserved_last_motion_s: Option<f32>,
}

#[derive(Debug, Clone)]
struct ConflictPath {
    bezier: PlannedTurnConnector,
    points: Vec<ConflictPoint>,
}

#[derive(Debug, Clone)]
struct IntersectionConflictData {
    by_movement: HashMap<LaneMovementKey, ConflictPath>,
    deadlock_timer_s: f32,
    deadlock_first_move: Option<u32>,
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
    /// Precomputed [`ConflictPoint::id`] for the dominating conflict threat (`None` unless `ConflictPoint`).
    conflict_point_id: Option<u64>,
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

pub fn run_simulation(
    graph_lock: Arc<RwLock<Option<MapData>>>,
    command_rx: Receiver<SimCommand>,
    vehicle_channel: Channel<String>,
    app_handle: AppHandle,
) {
    let mut clock = GameClock::new();
    let mut vehicles: Vec<Vehicle> = Vec::with_capacity(512);
    let mut congestion_timer = 0.0f32;
    let mut high_frustration_timer = 0.0f32;
    let mut idm_overlay_timer = 0.0f32;
    let mut selected_debug_vehicle: Option<u32> = None;

    // ── Build subsystems from map ────────────────────────────────────────────
    let (mut intersections, mut spawn_system, mut od_model, mut tram_sim, mut conflict_system) = {
        let guard = graph_lock.read();
        let map = guard
            .as_ref()
            .expect("map must be loaded before starting simulation");

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
    log::info!(
        "Simulation loop started (fixed-step dt = {:.4} s)",
        PHYSICS_DT
    );

    loop {
        // ── Wall-clock time measurement ──────────────────────────────────────
        let now = Instant::now();
        // Cap at 250 ms to prevent "spiral of death" on very slow machines
        let real_elapsed = now.duration_since(last_tick).as_secs_f32().min(0.25);
        last_tick = now;

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
                intersections =
                    IntersectionManager::from_graph(&map.graph, map.sandbox_simple_cross_tl);
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

            let game_dt_s = clock.tick(PHYSICS_DT);
            let game_hour = clock.game_hour();
            let spawn_multiplier = DayCycle::spawn_multiplier(game_hour);
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
                let mut guard = graph_lock.write();
                if let Some(map) = guard.as_mut() {
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
                            let edge_len = map
                                .graph
                                .edge_weight(first_edge)
                                .map(|e| e.length_m)
                                .unwrap_or(100.0);
                            let blocked = vehicles.iter().any(|v| {
                                let new_len = nv.vehicle_type.params().length_m;
                                let existing_len = v.vehicle_type.params().length_m;
                                let center_dist_m = v.edge_progress * edge_len;
                                let bumper_gap_m = center_dist_m - 0.5 * (new_len + existing_len);
                                let min_spawn_gap_m = new_len.max(existing_len) + SPAWN_BUFFER_M;
                                v.route_pos < v.route.len()
                                    && v.route[v.route_pos] == first_edge
                                    && v.current_lane == nv.current_lane
                                    && bumper_gap_m < min_spawn_gap_m
                            });
                            if blocked {
                                continue;
                            } // no room, skip this tick
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
                    vehicles[a]
                        .edge_progress
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
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    conflict_system.update_deadlock_state(
                        map,
                        &vehicles,
                        &vehicles_by_target_node,
                        &intersections,
                        PHYSICS_DT,
                    );
                }
            }

            conflict_system.release_reservations_overridden_by_foreign_occupants(&vehicles);

            // Parallel IDM / closest-obstacle computation (read-only parallel part)
            let tram_snapshot: Vec<(f64, f64, f32)> = tram_sim
                .trams
                .iter()
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
                            &vehicles_snapshot,
                            &spawn_system.speed_config,
                            clock.game_time_s as f32,
                        );
                    }
                }
            }
            conflict_system.expire_stale_reservations(&vehicles, clock.game_time_s as f32);

            // Lane changes
            {
                let guard = graph_lock.read();
                if let Some(map) = guard.as_ref() {
                    let snapshot: Vec<&Vehicle> = vehicles.iter().collect();
                    let changes: Vec<(u32, u8)> = vehicles
                        .iter()
                        .filter_map(|v| {
                            if v.route_pos >= v.route.len() {
                                return None;
                            }
                            let edge_idx = v.route[v.route_pos];
                            let edge = map.graph.edge_weight(edge_idx)?;
                            let same_edge: Vec<&Vehicle> = snapshot
                                .iter()
                                .filter(|o| {
                                    o.route_pos < o.route.len() && o.route[o.route_pos] == edge_idx
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
                            avg_frustration,
                            high_frustration_timer
                        );
                        let _ = app_handle.emit(
                            "game_over",
                            GameOverPayload {
                                reason: "avg_frustration".to_string(),
                                value: avg_frustration,
                                timestamp_game: clock.game_time_s as f32,
                            },
                        );
                        high_frustration_timer = 0.0;
                    }
                } else if mass_rage_fraction >= cfg.mass_rage_fraction {
                    log::warn!(
                        "GAME OVER: mass rage – {:.0}% vehicles at 100 frustration",
                        mass_rage_fraction * 100.0
                    );
                    let _ = app_handle.emit(
                        "game_over",
                        GameOverPayload {
                            reason: "mass_rage".to_string(),
                            value: mass_rage_fraction * 100.0,
                            timestamp_game: clock.game_time_s as f32,
                        },
                    );
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
                                stop_line_debug(ego, map, &intersections)
                                    .unwrap_or((1000.0, false));
                            let vp = ego.vehicle_type.params();
                            let accel_i = accel_inputs.get(i).copied().unwrap_or(0.0);
                            let look_ahead_point = vehicle_lookahead_point_lng_lat(ego, map);
                            let look_ahead_distance_m = look_ahead_point
                                .map(|p| {
                                    let hood = hood_lng_lat_m(ego);
                                    geo_dist_approx(hood[1], hood[0], p[1], p[0])
                                })
                                .unwrap_or(0.0)
                                .max(0.0);
                            let brake_reason =
                                idm_brake_caption(accel_i, &obstacle, ego, map);
                            let comfort_stop_m = comfort_braking_distance_m(ego.speed);
                            let ttc_seconds = idm_ttc_seconds(&obstacle, ego.speed);
                            let idm_decision =
                                idm_ui_decision(ego.speed, accel_i, &obstacle, red_blocking)
                                    .to_string();
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
                                threat_line_style: threat_line_style_label(obstacle.kind)
                                    .to_string(),
                                threat_point: obstacle.point_lng_lat,
                                stop_line_point: stop_line_point(ego, map),
                                turn_entry_point: vehicle_next_movement(ego, map)
                                    .map(|(_, conn)| [conn.p1_lng, conn.p1_lat]),
                                hood_lng_lat: hood_lng_lat_m(ego),
                                rear_bumper_lng_lat: rear_bumper_lng_lat_vehicle(ego),
                                look_ahead_point,
                                look_ahead_distance_m,
                                current_lane_id: ego.current_lane_id,
                                target_lane: ego.target_lane,
                                next_turn_intent: next_turn_intent_label_for_vehicle(ego, map),
                                idm_focus: idm_focus_caption(&obstacle),
                                route_points: build_debug_target_path_points(ego, map),
                                bezier_control_path_lng_lat: bezier_debug_control_polyline_lng_lat(
                                    ego,
                                ),
                                lane_route_ids: ego.lane_route.clone(),
                                brake_reason,
                                idm_decision,
                                ttc_seconds,
                                comfort_braking_distance_m: comfort_stop_m,
                            };
                            let _ = app_handle.emit("idm_debug", payload);
                        }
                    }
                }
                let guard_ld = graph_lock.read();
                if let Some(map_ld) = guard_ld.as_ref() {
                    if idm_steps.len() == vehicles.len() {
                        let mut entries: Vec<LeaderDebugEntry> = Vec::new();
                        for (idx, v) in vehicles.iter().enumerate() {
                            if v.route_pos >= v.route.len() || v.vehicle_type as u8 == 4 {
                                continue;
                            }
                            if v.speed > 3.5 {
                                continue;
                            }
                            let Some(step) = idm_steps.get(idx) else {
                                continue;
                            };
                            let obs = step.obstacle;
                            let idm_leader = if matches!(obs.kind, ObstacleKind::Vehicle) {
                                obs.leader_vehicle_id
                            } else {
                                None
                            };
                            let lane_leader = immediate_same_lane_leader_id(
                                idx,
                                v,
                                &vehicles,
                                &edge_lane_vehicles,
                                map_ld,
                            );
                            let sensor_mismatch = !v.on_turn_connector
                                && matches!(obs.kind, ObstacleKind::Vehicle)
                                && idm_leader != lane_leader;
                            entries.push(LeaderDebugEntry {
                                vehicle_id: v.id,
                                idm_leader_vehicle_id: idm_leader,
                                lane_leader_vehicle_id: lane_leader,
                                sensor_mismatch,
                            });
                        }
                        let _ = app_handle.emit("leader_debug", &LeaderDebugPayload { entries });
                    }
                }
            }
        } // end while physics_accumulator >= PHYSICS_DT

        // ── Render: serialise once per outer loop iteration ──────────────────
        // Decoupled from physics steps: the render fires at wall-clock rate
        // (~60 Hz target) even if physics ran 0 or 2 steps this iteration.
        if !vehicles.is_empty() || !tram_sim.is_empty() {
            let frame = serialize_vehicles(&vehicles, &tram_sim);
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
        let elapsed = last_tick.elapsed().as_secs_f32();
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

    if !matches!(
        itype,
        IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing
    ) {
        return None;
    }

    let dist_to_end = edge.length_m * (1.0 - vehicle.edge_progress);
    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(vehicle, dist_to_end);
    let red_blocking = !intersections.can_vehicle_proceed(
        tgt_osm_id,
        vehicle.has_stopped_at_stop_sign,
        vehicle,
        map,
    );
    Some((dist_to_stop_line, red_blocking))
}

fn stop_line_point(vehicle: &Vehicle, map: &MapData) -> Option<[f64; 2]> {
    if vehicle.route_pos >= vehicle.route.len() {
        return None;
    }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = map.graph.edge_weight(edge_idx)?;
    let (_, tgt) = map.graph.edge_endpoints(edge_idx)?;
    let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge.length_m.max(1.0)).clamp(0.0, 1.0) as f64;
    let src = map
        .graph
        .edge_endpoints(edge_idx)
        .map(|e| e.0)
        .unwrap_or(tgt);
    let src_n = &map.graph[src];
    let tgt_n = &map.graph[tgt];
    let stop_lat = src_n.lat + (tgt_n.lat - src_n.lat) * stop_t;
    let stop_lng = src_n.lng + (tgt_n.lng - src_n.lng) * stop_t;
    Some([stop_lng, stop_lat])
}

fn current_map_signature(
    graph_lock: &Arc<RwLock<Option<MapData>>>,
) -> Option<(usize, usize, usize, usize)> {
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
        SimCommand::Pause => clock.pause(),
        SimCommand::Resume => clock.resume(),
        SimCommand::SetTimeScale(s) => clock.set_time_scale(s),
        SimCommand::SetSpeedConfig(cfg) => spawn_system.set_speed_config(cfg),
        SimCommand::SetMaxVehicles(n) => spawn_system.max_vehicles = n,
        SimCommand::SetLightMode {
            intersection_id,
            mode,
        } => {
            intersections.set_mode(intersection_id, mode);
        }
        SimCommand::SetLightPhase {
            intersection_id,
            phase,
        } => {
            intersections.set_phase(intersection_id, phase);
        }
        SimCommand::SetLightDurations {
            intersection_id,
            green_s,
            red_s,
        } => {
            intersections.set_durations(intersection_id, green_s, red_s);
        }
        SimCommand::SetDebugVehicle(id) => {
            *selected_debug_vehicle = id;
        }
        SimCommand::Stop => {}
    }
}

/// Lane polyline as \[lng, lat\] samples (same order as frontend `lane.points`).
fn lane_centerline_lng_lat(lane: &crate::map::road_network::Lane) -> Vec<[f64; 2]> {
    lane.path.points.iter().map(|p| [p[1], p[0]]).collect()
}

/// Knot spacing (~0.5 m chord along the cubic) for overlays + reservation polylines; clamp for perf.
const CONNECTOR_ARC_SAMPLE_STEP_M: f64 = 0.55;
/// Minimum number of **uniform arc-length subintervals** on a lane connector in `route_points` / conflict
/// polylines (≥ **10 interior** samples between endpoints ⇒ 11 intervals ⇒ 12 vertices).
const MIN_LANE_CONNECTOR_ROUTE_INTERVALS: usize = 11;
const CONNECTOR_ROUTE_MAX_INTERVALS: usize = 140;

#[inline]
fn connector_polyline_segment_count_for_arc_m(total_m: f32) -> usize {
    let t = total_m.max(0.0) as f64;
    let by_len = ((t / CONNECTOR_ARC_SAMPLE_STEP_M).ceil()) as usize;
    by_len
        .max(MIN_LANE_CONNECTOR_ROUTE_INTERVALS)
        .min(CONNECTOR_ROUTE_MAX_INTERVALS)
}

/// Uniform resample of a piecewise-linear \[\[lng,lat\],…\] polyline in metre space (for connector fallback).
fn resample_lng_lat_polyline_min_intervals(
    pts: &[[f64; 2]],
    min_subintervals: usize,
) -> Vec<[f64; 2]> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let mut cum = vec![0.0_f64];
    let mut total = 0.0_f64;
    for i in 0..pts.len() - 1 {
        let d = lng_lat_metre_delta(pts[i], pts[i + 1]);
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        total += len;
        cum.push(total);
    }
    if total < 1e-6 {
        return pts.to_vec();
    }
    let n = min_subintervals.max(1);
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let target = total * (k as f64 / n as f64);
        let mut j = 0usize;
        while j + 1 < cum.len() && cum[j + 1] < target - 1e-9 {
            j += 1;
        }
        let j = j.min(pts.len() - 2);
        let seg_lo = cum[j];
        let seg_hi = cum[j + 1];
        let local = if (seg_hi - seg_lo).abs() < 1e-9 {
            0.0
        } else {
            ((target - seg_lo) / (seg_hi - seg_lo)).clamp(0.0, 1.0)
        };
        let a = pts[j];
        let b = pts[j + 1];
        out.push([
            a[0] + (b[0] - a[0]) * local,
            a[1] + (b[1] - a[1]) * local,
        ]);
    }
    out
}

#[inline]
fn push_route_ll_dedupe(points: &mut Vec<[f64; 2]>, p: [f64; 2]) {
    const EPS: f64 = 2e-7;
    if let Some(last) = points.last() {
        if (last[0] - p[0]).abs() < EPS && (last[1] - p[1]).abs() < EPS {
            return;
        }
    }
    points.push(p);
}

/// When `lane_route` is missing/short, synthesize the remainder from graph edges — **dense lane cubics**
/// between `(edge_i, edge_{i+1})` only. Does not append the final routed edge’s far node (would be a
/// spurious straight chord). If no lane-graph connector exists, uses [`connector_for_movement_lane`]
/// quadratic samples — never a single hop through the junction node (that skewed the purple path).
fn append_remaining_route_via_lane_connectors(
    vehicle: &Vehicle,
    map: &MapData,
    points: &mut Vec<[f64; 2]>,
    skip_first_movement_connector: bool,
) {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return;
    }

    let route = vehicle.route.as_slice();
    let mut ref_en = if points.len() >= 2 {
        normalize_metre_vec(lng_lat_metre_delta(
            points[points.len() - 2],
            points[points.len() - 1],
        ))
        .unwrap_or_else(|| vehicle_forward_metres_en(vehicle))
    } else {
        vehicle_forward_metres_en(vehicle)
    };

    let pair_start_i = vehicle.route_pos + if skip_first_movement_connector { 1 } else { 0 };
    if pair_start_i + 1 >= route.len() {
        return;
    }

    let mut lane_from = vehicle.current_lane;
    let mut lane_from_out = lane_from;

    for i in pair_start_i..route.len().saturating_sub(1) {
        let in_e = route[i];
        let out_e = route[i + 1];
        let in_ln = map
            .graph
            .edge_weight(in_e)
            .map(|e| e.lanes.max(1))
            .unwrap_or(1);
        let out_ln = map
            .graph
            .edge_weight(out_e)
            .map(|e| e.lanes.max(1))
            .unwrap_or(1);

        lane_from = if i == vehicle.route_pos {
            vehicle.current_lane
        } else {
            lane_from_out
        }
        .min(in_ln.saturating_sub(1));

        let lane_to_hint = if i == vehicle.route_pos {
            vehicle.target_lane
        } else {
            lane_from
        }
        .min(out_ln.saturating_sub(1));

        let mut resolved_conn: Option<(LaneId, u8)> = None;

        let try_lane_pairs: Vec<u8> = {
            let mut v = vec![lane_to_hint];
            for alt in 0..out_ln {
                if alt != lane_to_hint {
                    v.push(alt);
                }
            }
            v
        };

        'scan: for &lo in &try_lane_pairs {
            if let Some(cid) = find_connector_lane_id(map, in_e, lane_from, out_e, lo) {
                resolved_conn = Some((cid, lo));
                break 'scan;
            }
        }

        if let Some((conn_id, lo_used)) = resolved_conn {
            if let Some(conn_lane) = map.lanes.get(&conn_id) {
                let mut seg = connector_lane_dense_polyline_lng_lat(conn_lane);
                if !seg.is_empty() {
                    if seg.len() >= 2 {
                        orient_lng_lat_polyline_forward(&mut seg, Some(ref_en));
                    }
                    for p in seg {
                        push_route_ll_dedupe(points, p);
                    }
                    if points.len() >= 2 {
                        if let Some(te) = normalize_metre_vec(lng_lat_metre_delta(
                            points[points.len() - 2],
                            points[points.len() - 1],
                        )) {
                            ref_en = te;
                        }
                    }
                }
            }
            lane_from_out = lo_used;
            continue;
        }

        if let Some(planned) = connector_for_movement_lane(map, in_e, out_e, lane_from, lane_to_hint)
        {
            let n_lin = ((((planned.length_m / 1.2).ceil()) as usize).clamp(16, 80)).max(16);
            let (poly_xy, _dists) = sample_connector_polyline(&planned, n_lin);
            let mut seg: Vec<[f64; 2]> = poly_xy
                .iter()
                .map(|p| {
                    let (lat, lng) = m_xy_to_geo(p.x, p.y);
                    [lng, lat]
                })
                .collect();
            if seg.len() >= 2 {
                orient_lng_lat_polyline_forward(&mut seg, Some(ref_en));
            }
            for p in seg {
                push_route_ll_dedupe(points, p);
            }
            if points.len() >= 2 {
                if let Some(te) = normalize_metre_vec(lng_lat_metre_delta(
                    points[points.len() - 2],
                    points[points.len() - 1],
                )) {
                    ref_en = te;
                }
            }
        }
        lane_from_out = lane_to_hint;
    }

    // Do **not** append the last route edge's target graph node: this function only stitches
    // Kubro lane connectors between consecutive edges. Pushing `tgt` drew an extra straight segment
    // from the last connector exit to the far end of the road (no centreline samples here).
}

/// Dense \[\[lng,lat\],…\] for a **lane connector** (`edge_id == u64::MAX`): Kurbo arc-length samples,
/// or equidistant resample of stored polyline so the frontend never gets a 2-point chord through the junction.
fn connector_lane_dense_polyline_lng_lat(lane: &Lane) -> Vec<[f64; 2]> {
    let poly_ll = lane_centerline_lng_lat(lane);
    let Some(cubic) = lane_connector_cubic(lane) else {
        return resample_lng_lat_polyline_min_intervals(&poly_ll, MIN_LANE_CONNECTOR_ROUTE_INTERVALS);
    };
    let total_m = cubic.arclen(CONNECTOR_ARCLEN_ACC) as f32;
    if total_m <= 0.06 {
        return resample_lng_lat_polyline_min_intervals(&poly_ll, MIN_LANE_CONNECTOR_ROUTE_INTERVALS);
    }
    let steps = connector_polyline_segment_count_for_arc_m(total_m);
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let dist = total_m * i as f32 / steps as f32;
        if let Some((lat, lng, _)) =
            sample_connector_kurbo_at(lane, dist).or_else(|| sample_lane_path_at(lane, dist))
        {
            out.push([lng, lat]);
        }
    }
    if out.len() >= MIN_LANE_CONNECTOR_ROUTE_INTERVALS + 1 {
        out
    } else if out.len() >= 2 {
        resample_lng_lat_polyline_min_intervals(&out, MIN_LANE_CONNECTOR_ROUTE_INTERVALS)
    } else {
        resample_lng_lat_polyline_min_intervals(&poly_ll, MIN_LANE_CONNECTOR_ROUTE_INTERVALS)
    }
}

/// Planner-quality samples in **metre XY** (`geo_to_m_xy` space). `dists\[i\]` equals true Kurbo
/// arc-length from the connector entry — aligned with [`Vehicle::turn_dist_m`] on lane connectors.
fn kurbo_lane_connector_meter_samples(lane: &Lane) -> Option<(Vec<DVec2>, Vec<f32>)> {
    let cubic = lane_connector_cubic(lane)?;
    let acc = CONNECTOR_ARCLEN_ACC;
    let total = cubic.arclen(acc) as f32;
    if total <= 0.06 {
        return None;
    }
    let n = connector_polyline_segment_count_for_arc_m(total);
    let mut pts = Vec::with_capacity(n + 1);
    let mut dists = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let s = total * i as f32 / n as f32;
        let t = cubic.inv_arclen(s as f64, acc);
        let p = cubic.eval(t);
        pts.push(DVec2::new(p.x, p.y));
        dists.push(s);
    }
    Some((pts, dists))
}

/// Vehicle forward unit vector in planar east / north metres (matches [`offset_vehicle_center_geo`]).
#[inline]
fn vehicle_forward_metres_en(vehicle: &Vehicle) -> (f64, f64) {
    (vehicle.angle.sin() as f64, vehicle.angle.cos() as f64)
}

#[inline]
fn lng_lat_metre_delta(from_ll: [f64; 2], to_ll: [f64; 2]) -> (f64, f64) {
    (
        (to_ll[0] - from_ll[0]) * GEO_LNG_M,
        (to_ll[1] - from_ll[1]) * GEO_LAT_M,
    )
}

#[inline]
fn normalize_metre_vec(v: (f64, f64)) -> Option<(f64, f64)> {
    let len = (v.0 * v.0 + v.1 * v.1).sqrt();
    if len < 0.06 {
        return None;
    }
    Some((v.0 / len, v.1 / len))
}

/// Rotate polyline 180° if its start tangent fights the reference (`connectors` often stored either way).
fn orient_lng_lat_polyline_forward(seg: &mut Vec<[f64; 2]>, reference_en: Option<(f64, f64)>) {
    let Some((rx, ry)) = reference_en else {
        return;
    };
    if seg.len() < 2 {
        return;
    }
    let dv = lng_lat_metre_delta(seg[0], seg[1]);
    let Some((tx, ty)) = normalize_metre_vec(dv) else {
        return;
    };
    if tx * rx + ty * ry < -0.02 {
        seg.reverse();
    }
}

/// Drop leading samples that jog backward relative to nominal travel along the oriented segment.
fn trim_lng_lat_polyline_backtrack_from_anchor(
    seg: &mut Vec<[f64; 2]>,
    anchor_ll: [f64; 2],
    mut max_trim: usize,
) {
    while max_trim > 0 && seg.len() >= 2 {
        let to_first = lng_lat_metre_delta(anchor_ll, seg[0]);
        let along = lng_lat_metre_delta(seg[0], seg[1]);
        let Some(tf) = normalize_metre_vec(to_first) else {
            break;
        };
        let Some(ta) = normalize_metre_vec(along) else {
            break;
        };
        if tf.0 * ta.0 + tf.1 * ta.1 < -0.25 {
            seg.remove(0);
            max_trim -= 1;
        } else {
            break;
        }
    }
}

/// After a Kubro connector, the outbound road centreline often begins with OSM/junction vertices that
/// lie **behind** the connector exit relative to travel — a visible spike toward the intersection node.
/// Drop those leading samples (keep at least one point).
fn trim_lng_lat_polyline_outbound_spike_after_connector(
    seg: &mut Vec<[f64; 2]>,
    join_ll: [f64; 2],
    ref_en: (f64, f64),
    mut max_trim: usize,
) {
    let (rx, ry) = ref_en;
    while max_trim > 0 && seg.len() > 1 {
        let to_first = lng_lat_metre_delta(join_ll, seg[0]);
        let dot = to_first.0 * rx + to_first.1 * ry;
        if dot < -0.35 {
            seg.remove(0);
            max_trim -= 1;
        } else {
            break;
        }
    }
}

/// Index into `lane_route` to begin the purple debug polyline (handles post-connector frame lag).
fn lane_route_polyline_start_index(vehicle: &Vehicle, map: &MapData) -> usize {
    if vehicle.lane_route.is_empty() {
        return 0;
    }
    let fallback = || {
        vehicle
            .lane_route_pos
            .min(vehicle.lane_route.len().saturating_sub(1))
    };

    if vehicle.on_turn_connector {
        return vehicle
            .connector_lane_id
            .and_then(|cid| vehicle.lane_route.iter().position(|&id| id == cid))
            .or_else(|| {
                vehicle
                    .current_lane_id
                    .and_then(|cid| vehicle.lane_route.iter().position(|&id| id == cid))
            })
            .unwrap_or_else(fallback);
    }

    let from_lane_id = vehicle
        .current_lane_id
        .and_then(|cid| vehicle.lane_route.iter().position(|&id| id == cid));

    let from_edge = vehicle.route.get(vehicle.route_pos).and_then(|edge_idx| {
        let eid = edge_idx.index() as u64;
        vehicle
            .lane_route
            .iter()
            .position(|&lid| map.lanes.get(&lid).is_some_and(|l| l.edge_id == eid))
    });

    from_lane_id.or(from_edge).unwrap_or_else(fallback)
}

fn sync_vehicle_lane_route_state(vehicle: &mut Vehicle, map: &MapData) -> bool {
    if vehicle.lane_route.is_empty() {
        return false;
    }

    let route_edge = vehicle
        .route
        .get(vehicle.route_pos)
        .map(|e| e.index() as u64);
    let preferred = vehicle
        .current_lane_id
        .and_then(|cid| vehicle.lane_route.iter().position(|&id| id == cid));
    let fallback = route_edge.and_then(|eid| {
        vehicle.lane_route.iter().position(|&lid| {
            map.lanes
                .get(&lid)
                .is_some_and(|lane| lane.edge_id == eid && lane.edge_id != u64::MAX)
        })
    });
    let Some(idx) = preferred.or(fallback) else {
        return false;
    };
    vehicle.lane_route_pos = idx;
    let lane_id = vehicle.lane_route[idx];
    vehicle.current_lane_id = Some(lane_id);
    if let Some(lane) = map.lanes.get(&lane_id) {
        if lane.edge_id != u64::MAX {
            vehicle.current_lane = lane.lane_index;
        }
    }
    true
}

/// Remaining path along **`lane_route`** (vehicle → dense connectors → road centreline), without
/// stitching through the junction graph node; falls back to edge-pair connectors if `lane_route` is empty.
fn build_debug_target_path_points(vehicle: &Vehicle, map: &MapData) -> Vec<[f64; 2]> {
    let mut points: Vec<[f64; 2]> = vec![[vehicle.lng, vehicle.lat]];

    if !vehicle.lane_route.is_empty() {
        let start_i = lane_route_polyline_start_index(vehicle, map);
        let veh_ll = points[0];
        let mut ref_en = Some(vehicle_forward_metres_en(vehicle));
        let mut prev_lane_was_connector = false;

        for &lane_id in vehicle.lane_route.iter().skip(start_i) {
            let Some(lane) = map.lanes.get(&lane_id) else {
                continue;
            };
            let is_connector = lane.edge_id == u64::MAX;
            // Road lane: centreline polyline. **LaneConnector:** never push only endpoints — always
            // `connector_lane_dense_polyline_lng_lat` (Kurbo ≥ `MIN_LANE_CONNECTOR_ROUTE_INTERVALS` or resample).
            let mut seg = if is_connector {
                connector_lane_dense_polyline_lng_lat(lane)
            } else {
                lane_centerline_lng_lat(lane)
            };
            if seg.is_empty() {
                continue;
            }
            if seg.len() >= 2 {
                orient_lng_lat_polyline_forward(&mut seg, ref_en);
                let join_ll = points.last().copied().unwrap_or(veh_ll);
                if !is_connector {
                    trim_lng_lat_polyline_backtrack_from_anchor(&mut seg, join_ll, 96);
                    if prev_lane_was_connector {
                        if let Some(re) = ref_en {
                            trim_lng_lat_polyline_outbound_spike_after_connector(
                                &mut seg, join_ll, re, 64,
                            );
                        }
                    }
                }
                // Connectors: keep full Kubro samples for purple line / debug — no trim (avoids zig-zag).
            }
            for p in seg {
                if let Some(last) = points.last() {
                    let dup = (last[0] - p[0]).abs() < 1e-8 && (last[1] - p[1]).abs() < 1e-8;
                    if dup {
                        continue;
                    }
                }
                points.push(p);
            }
            if points.len() >= 2 {
                if let Some(te) = normalize_metre_vec(lng_lat_metre_delta(
                    points[points.len() - 2],
                    points[points.len() - 1],
                )) {
                    ref_en = Some(te);
                }
            }
            prev_lane_was_connector = is_connector;
        }

        if points.len() >= 2 {
            return points;
        }
    }

    // Fallback: no stitched lane-route (or degenerate stitching) → sample lane-graph Kubro connectors
    // between routed edges instead of straight graph-node hops (which zig-zag through junction centres).
    points.clear();
    points.push([vehicle.lng, vehicle.lat]);
    let mut skip_dup_connector = false;
    if vehicle.on_turn_connector {
        let mut kubro_dense = false;
        if let Some(cid) = vehicle.connector_lane_id {
            if let Some(lane) = map.lanes.get(&cid) {
                if lane_connector_cubic(lane).is_some() {
                    let mut seg = connector_lane_dense_polyline_lng_lat(lane);
                    if seg.len() >= 2 {
                        orient_lng_lat_polyline_forward(&mut seg, Some(vehicle_forward_metres_en(vehicle)));
                    }
                    for p in seg {
                        push_route_ll_dedupe(&mut points, p);
                    }
                    kubro_dense = true;
                }
            }
        }
        if !kubro_dense {
            let samples = 26usize;
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
                push_route_ll_dedupe(&mut points, [lng, lat]);
            }
            if points.len() >= 3 {
                let mut bez_tail: Vec<[f64; 2]> = points[1..].to_vec();
                orient_lng_lat_polyline_forward(
                    &mut bez_tail,
                    Some(vehicle_forward_metres_en(vehicle)),
                );
                points.truncate(1);
                for q in bez_tail {
                    push_route_ll_dedupe(&mut points, q);
                }
            }
        }
        skip_dup_connector = true;
    }
    append_remaining_route_via_lane_connectors(vehicle, map, &mut points, skip_dup_connector);
    points
}

fn vehicle_lookahead_point_lng_lat(vehicle: &Vehicle, map: &MapData) -> Option<[f64; 2]> {
    if vehicle.on_turn_connector {
        if let Some(cid) = vehicle.connector_lane_id {
            if let Some(lane) = map.lanes.get(&cid) {
                let dist = vehicle.turn_dist_m as f32 + STEERING_LOOKAHEAD_M;
                if let Some((lat, lng, _)) =
                    sample_connector_kurbo_at(lane, dist).or_else(|| sample_lane_path_at(lane, dist))
                {
                    return Some([lng, lat]);
                }
            }
        }
    }
    if let Some(lid) = vehicle.current_lane_id {
        if let Some(lane) = map.lanes.get(&lid) {
            let dist = vehicle.lane_progress_m + STEERING_LOOKAHEAD_M;
            if let Some((lat, lng, _)) = sample_lane_path_at(lane, dist) {
                return Some([lng, lat]);
            }
            if let Some(last) = lane.path.points.last() {
                return Some([last[1], last[0]]);
            }
        }
    }
    None
}

#[inline]
fn turn_intent_label(intent: TurnIntent) -> &'static str {
    match intent {
        TurnIntent::Straight => "straight",
        TurnIntent::Left => "left",
        TurnIntent::Right => "right",
        TurnIntent::UTurn => "uturn",
    }
}

fn next_turn_intent_label_for_vehicle(vehicle: &Vehicle, map: &MapData) -> String {
    let Some((in_edge, out_edge)) = vehicle_next_movement(vehicle, map).map(|(m, _)| m) else {
        return "end_of_route".to_string();
    };
    let Some((_, node_idx)) = map.graph.edge_endpoints(in_edge) else {
        return "unknown".to_string();
    };
    turn_intent_label(movement_turn_intent(map, (in_edge, out_edge), node_idx)).to_string()
}

fn bezier_debug_control_polyline_lng_lat(vehicle: &Vehicle) -> Vec<[f64; 2]> {
    if !vehicle.on_turn_connector {
        return Vec::new();
    }
    vec![
        [vehicle.turn_p1_lng, vehicle.turn_p1_lat],
        [vehicle.turn_ctrl_lng, vehicle.turn_ctrl_lat],
        [vehicle.turn_p2_lng, vehicle.turn_p2_lat],
    ]
}

/// Within this distance (m) of the current edge's downstream node we treat braking on a vehicle
/// obstacle as intersection-related slowdown (distinct caption from mid-block following).
const INTERSECTION_VEHICLE_BRAKE_CAPTION_DIST_M: f32 = 72.0;

fn ego_near_downstream_route_junction(ego: &Vehicle, map: &MapData, dist_m: f32) -> bool {
    if ego.on_turn_connector || ego.route_pos >= ego.route.len() {
        return false;
    }
    if vehicle_next_movement(ego, map).is_none() {
        return false;
    }
    let edge_idx = ego.route[ego.route_pos];
    let edge_len = map
        .graph
        .edge_weight(edge_idx)
        .map(|e| e.length_m)
        .unwrap_or(0.0);
    edge_len > 0.0 && edge_len * (1.0 - ego.edge_progress) <= dist_m
}

fn idm_brake_caption(
    accel: f32,
    obstacle: &ClosestObstacle,
    ego: &Vehicle,
    map: &MapData,
) -> Option<String> {
    if accel > -0.45 {
        return None;
    }
    if ego.route_pos.saturating_add(1) >= ego.route.len()
        && matches!(obstacle.kind, ObstacleKind::Vehicle)
        && obstacle.gap_m > 300.0
    {
        return Some("End of path (no next edge)".to_string());
    }
    let near_ix = ego_near_downstream_route_junction(ego, map, INTERSECTION_VEHICLE_BRAKE_CAPTION_DIST_M);
    match obstacle.kind {
        ObstacleKind::Vehicle => obstacle
            .leader_vehicle_id
            .map(|id| {
                if near_ix {
                    format!("Intersection approach — queued leader #{id}")
                } else {
                    format!("Leader #{id}")
                }
            })
            .or_else(|| {
                Some(if near_ix {
                    "Intersection approach — vehicle in path".to_string()
                } else {
                    "Vehicle ahead".to_string()
                })
            }),
        ObstacleKind::ConflictPoint => Some(
            obstacle
                .conflict_reserver_id
                .map(|id| format!("Conflict patch — reserver #{id}"))
                .unwrap_or_else(|| "Conflict geometry".to_string()),
        ),
        ObstacleKind::PriorityStopLine => {
            if let Some(id) = obstacle.leader_vehicle_id {
                Some(format!("Yield / priority (vehicle #{id})"))
            } else {
                Some("Yield / priority".to_string())
            }
        }
        ObstacleKind::TrafficSignalStopLine => Some("Traffic signal".to_string()),
        ObstacleKind::StopSignStopLine => Some("Stop sign".to_string()),
        ObstacleKind::YieldTarget => Some("Yield sign".to_string()),
    }
}

fn idm_focus_caption(obstacle: &ClosestObstacle) -> String {
    match obstacle.kind {
        ObstacleKind::Vehicle => obstacle
            .leader_vehicle_id
            .map(|id| format!("follow_vehicle#{id}"))
            .unwrap_or_else(|| "follow_vehicle".to_string()),
        ObstacleKind::ConflictPoint => obstacle
            .conflict_reserver_id
            .map(|id| format!("yield_conflict#{id}"))
            .unwrap_or_else(|| "yield_conflict".to_string()),
        ObstacleKind::PriorityStopLine => obstacle
            .leader_vehicle_id
            .map(|id| format!("yield_priority#{id}"))
            .unwrap_or_else(|| "yield_priority".to_string()),
        ObstacleKind::TrafficSignalStopLine => "obey_signal".to_string(),
        ObstacleKind::StopSignStopLine => "obey_stop_sign".to_string(),
        ObstacleKind::YieldTarget => "obey_yield_sign".to_string(),
    }
}

#[inline]
fn comfort_braking_distance_m(speed: f32) -> f32 {
    let b = IDM_UI_COMFORT_DECEL_MPS2.max(0.15);
    speed * speed / (2.0 * b)
}

fn idm_yield_context(kind: ObstacleKind, red_blocking: bool) -> bool {
    match kind {
        ObstacleKind::Vehicle => false,
        ObstacleKind::TrafficSignalStopLine => red_blocking,
        ObstacleKind::ConflictPoint | ObstacleKind::PriorityStopLine
        | ObstacleKind::YieldTarget
        | ObstacleKind::StopSignStopLine => true,
    }
}

/// HUD TTC: gap / positive closing speed — vehicle leader uses `delta_v` (= v_ego − v_leader);
/// non-vehicle threats use `ego_speed` as closing on a stationary constraint.
fn idm_ttc_seconds(obstacle: &ClosestObstacle, ego_speed: f32) -> Option<f32> {
    let closing = match obstacle.kind {
        ObstacleKind::Vehicle => obstacle.delta_v.max(0.0),
        _ => ego_speed.max(0.0),
    };
    if closing <= IDM_UI_TTC_MIN_CLOSING_MPS {
        return None;
    }
    let t = obstacle.gap_m / closing;
    if t.is_finite() && t > 0.0 && t < 600.0 {
        Some(t)
    } else {
        None
    }
}

fn idm_ui_decision(
    ego_speed: f32,
    accel: f32,
    obstacle: &ClosestObstacle,
    red_blocking: bool,
) -> &'static str {
    if ego_speed < IDM_UI_STOP_SPEED_MPS {
        return "STOP";
    }
    if accel < IDM_UI_BRAKE_ACCEL_THRESHOLD {
        return "BRAKE";
    }
    if idm_yield_context(obstacle.kind, red_blocking) {
        return "YIELD";
    }
    if accel < IDM_UI_COAST_ACCEL_THRESHOLD {
        return "COAST";
    }
    "GO"
}

#[inline]
fn node_is_intersection_like(map: &MapData, node: NodeIndex) -> bool {
    let deg = map.graph.edges(node).count()
        + map
            .graph
            .edges_directed(node, petgraph::Direction::Incoming)
            .count();
    !matches!(map.graph[node].intersection_type, IntersectionType::Plain) || deg >= 4
}

// ── Physics helpers ────────────────────────────────────────────────────────────

/// Prefer [`Vehicle::connector_lane_id`] while on a turn arc; otherwise `current_lane_id` only when
/// it lies on [`Vehicle::route`]\[`route_pos`\]. Using a stale outbound lane while the route edge is
/// still the approach places ego on the wrong polyline and pairs it with unrelated leaders
/// (~1–2 m “ghost vehicle ahead”).
fn vehicle_arc_scan_lane_id(vehicle: &Vehicle, map: &MapData) -> Option<LaneId> {
    if vehicle.on_turn_connector {
        return vehicle.connector_lane_id;
    }
    if vehicle.route_pos >= vehicle.route.len() {
        return None;
    }
    let route_edge = vehicle.route[vehicle.route_pos];
    let route_eid = route_edge.index() as u64;
    if let Some(lid) = vehicle.current_lane_id {
        if let Some(lane) = map.lanes.get(&lid) {
            if lane.edge_id != u64::MAX && lane.edge_id == route_eid {
                return Some(lid);
            }
        }
    }
    map.lane_by_edge_lane
        .get(&(route_edge.index(), vehicle.current_lane))
        .copied()
}

#[inline]
fn leader_arc_lane_matches_route(leader: &Vehicle, lane: &Lane) -> bool {
    if leader.route_pos >= leader.route.len() {
        return false;
    }
    if leader.on_turn_connector {
        return lane.edge_id == u64::MAX && leader.connector_lane_id == Some(lane.id);
    }
    if lane.edge_id == u64::MAX {
        return false;
    }
    lane.edge_id == leader.route[leader.route_pos].index() as u64
}

/// Find the dominant same-lane / look-ahead leader as an IDM obstacle.
///
/// Gap is bumper-to-bumper arc length (`MIN_IDM_GAP_M` clamped).
/// Uses the leader's **rear bumper** as the visualised threat anchor.
fn find_leader_obstacle_same_edge_lane(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    edge_lane_vehicles: &HashMap<(EdgeIndex, u8), Vec<usize>>,
    map: &MapData,
) -> ClosestObstacle {
    let Some(&edge_idx) = ego.route.get(ego.route_pos) else {
        return free_leader_obstacle();
    };
    let Some(bucket) = edge_lane_vehicles.get(&(edge_idx, ego.current_lane)) else {
        return free_leader_obstacle();
    };
    let lane_len = map
        .lane_by_edge_lane
        .get(&(edge_idx.index(), ego.current_lane))
        .and_then(|lid| map.lanes.get(lid))
        .map(|lane| lane.path.length_m)
        .unwrap_or(0.0);
    if lane_len <= 0.01 {
        return free_leader_obstacle();
    }
    let ego_progress = (ego.edge_progress * lane_len).clamp(0.0, lane_len);
    let mut best: Option<(f32, f32, u32, [f64; 2])> = None;
    for &leader_idx in bucket {
        if leader_idx == ego_idx {
            continue;
        }
        let Some(leader) = vehicles.get(leader_idx) else {
            continue;
        };
        if leader.despawned || leader.route_pos >= leader.route.len() || leader.route[leader.route_pos] != edge_idx
        {
            continue;
        }
        let leader_progress = (leader.edge_progress * lane_len).clamp(0.0, lane_len);
        let center_dist_m = leader_progress - ego_progress;
        if center_dist_m <= 0.0 {
            continue;
        }
        let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
        if best.map_or(true, |(g, _, _, _)| gap < g) {
            best = Some((
                gap,
                ego.speed - leader.speed,
                leader.id,
                rear_bumper_lng_lat_vehicle(leader),
            ));
        }
    }
    if let Some((gap, dv, leader_id, rear_ll)) = best {
        return ClosestObstacle {
            kind: ObstacleKind::Vehicle,
            gap_m: gap,
            delta_v: dv,
            point_lng_lat: Some(rear_ll),
            leader_vehicle_id: Some(leader_id),
            conflict_reserver_id: None,
            conflict_point_id: None,
        };
    }
    free_leader_obstacle()
}

/// Immediate predecessor on the same directed edge and lane index (sorted bucket),
/// ignoring lane-graph continuation — ground truth for a simple queue on one lane.
fn immediate_same_lane_leader_id(
    ego_idx: usize,
    ego: &Vehicle,
    vehicles: &[Vehicle],
    edge_lane_vehicles: &HashMap<(EdgeIndex, u8), Vec<usize>>,
    map: &MapData,
) -> Option<u32> {
    if ego.on_turn_connector {
        return None;
    }
    let edge_idx = *ego.route.get(ego.route_pos)?;
    let bucket = edge_lane_vehicles.get(&(edge_idx, ego.current_lane))?;
    let lane_len = map
        .lane_by_edge_lane
        .get(&(edge_idx.index(), ego.current_lane))
        .and_then(|lid| map.lanes.get(lid))
        .map(|lane| lane.path.length_m)
        .unwrap_or(0.0);
    if lane_len <= 0.01 {
        return None;
    }
    let ego_progress = (ego.edge_progress * lane_len).clamp(0.0, lane_len);
    let mut best: Option<(f32, u32)> = None;
    for &leader_idx in bucket {
        if leader_idx == ego_idx {
            continue;
        }
        let leader = vehicles.get(leader_idx)?;
        if leader.despawned
            || leader.route_pos >= leader.route.len()
            || leader.route[leader.route_pos] != edge_idx
        {
            continue;
        }
        let leader_progress = (leader.edge_progress * lane_len).clamp(0.0, lane_len);
        let center_dist_m = leader_progress - ego_progress;
        if center_dist_m <= 0.0 {
            continue;
        }
        let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
        if best.map_or(true, |(g, _)| gap < g) {
            best = Some((gap, leader.id));
        }
    }
    best.map(|(_, id)| id)
}

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
    if ego.lane_route.is_empty() {
        return find_leader_obstacle_same_edge_lane(ego_idx, ego, vehicles, edge_lane_vehicles, map);
    }

    let ego_lane_id = vehicle_arc_scan_lane_id(ego, map);
    let Some(ego_lane_id) = ego_lane_id else {
        return find_leader_obstacle_same_edge_lane(ego_idx, ego, vehicles, edge_lane_vehicles, map);
    };
    let Some(start_idx) = ego.lane_route.iter().position(|&lid| lid == ego_lane_id) else {
        if ego.route_pos + 1 < ego.route.len() {
            log::warn!(
                "AUTO {}: brak ciaglosci lane_route (active_lane={} route_pos={})",
                ego.id,
                ego_lane_id,
                ego.route_pos
            );
        }
        return find_leader_obstacle_same_edge_lane(ego_idx, ego, vehicles, edge_lane_vehicles, map);
    };
    if start_idx + 1 >= ego.lane_route.len()
        && ego.route_pos + 1 < ego.route.len()
        && ego.edge_progress >= 0.60
    {
        log::warn!(
            "AUTO {}: IDM brak kontynuacji lane_route przed skrzyzowaniem (route_pos={})",
            ego.id,
            ego.route_pos
        );
    }

    let lane_segment: Vec<LaneId> = ego.lane_route.iter().skip(start_idx).copied().collect();
    if lane_segment.is_empty() {
        return find_leader_obstacle_same_edge_lane(ego_idx, ego, vehicles, edge_lane_vehicles, map);
    }
    let mut segment_offsets = Vec::with_capacity(lane_segment.len());
    let mut acc_len = 0.0f32;
    for lane_id in &lane_segment {
        segment_offsets.push(acc_len);
        if let Some(lane) = map.lanes.get(lane_id) {
            acc_len += lane.path.length_m.max(0.0);
        }
        if acc_len >= 220.0 {
            break;
        }
    }
    let lane_segment = &lane_segment[..segment_offsets.len()];
    let ego_lane_len = map
        .lanes
        .get(&ego_lane_id)
        .map(|l| l.path.length_m)
        .unwrap_or(0.0);
    let ego_progress = ego
        .lane_progress_m
        .clamp(0.0, ego_lane_len.max(0.0))
        .max((ego.edge_progress * ego_lane_len).clamp(0.0, ego_lane_len.max(0.0)));

    let mut best: Option<(f32, f32, u32, [f64; 2])> = None;
    for (idx, leader) in vehicles.iter().enumerate() {
        if idx == ego_idx || leader.despawned {
            continue;
        }
        let Some(leader_lane_id) = vehicle_arc_scan_lane_id(leader, map) else {
            continue;
        };
        let Some(seg_idx) = lane_segment.iter().position(|&lid| lid == leader_lane_id) else {
            continue;
        };
        let lane_len = map
            .lanes
            .get(&leader_lane_id)
            .filter(|lane| leader_arc_lane_matches_route(leader, lane))
            .map(|l| l.path.length_m)
            .unwrap_or(0.0);
        if lane_len <= 0.01 {
            continue;
        }
        let leader_progress = if leader.on_turn_connector {
            (leader.turn_dist_m as f32).clamp(0.0, lane_len.max(0.0))
        } else {
            leader
                .lane_progress_m
                .clamp(0.0, lane_len.max(0.0))
                .max((leader.edge_progress * lane_len).clamp(0.0, lane_len.max(0.0)))
        };
        let center_dist_m = segment_offsets[seg_idx] + leader_progress - ego_progress;
        if center_dist_m <= 0.0 {
            continue;
        }
        let gap = bumper_gap(center_dist_m, ego, leader).max(MIN_IDM_GAP_M);
        let rear_ll = rear_bumper_lng_lat_vehicle(leader);
        let hood = hood_lng_lat_m(ego);
        let geo_sep = geo_dist_approx(hood[1], hood[0], rear_ll[1], rear_ll[0]);
        let geo_allow = if gap < LEADER_ARC_GEO_NEAR_GAP_THRESHOLD_M {
            gap + LEADER_ARC_GEO_NEAR_SLACK_M
        } else {
            gap + LEADER_ARC_GEO_MAX_SLACK_M
        };
        if geo_sep > geo_allow {
            continue;
        }
        if best.map_or(true, |(g, _, _, _)| gap < g) {
            best = Some((gap, ego.speed - leader.speed, leader.id, rear_ll));
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
            conflict_point_id: None,
        };
    }
    find_leader_obstacle_same_edge_lane(ego_idx, ego, vehicles, edge_lane_vehicles, map)
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
        conflict_point_id: None,
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

/// Sample the lane centreline in arc-length metres from the start.
/// Distances past the polyline length clamp to the endpoint (no failure on overshoot).
fn sample_lane_path_at(lane: &Lane, dist_m: f32) -> Option<(f64, f64, f32)> {
    let pts = &lane.path.points;
    if pts.len() < 2 {
        return None;
    }
    if lane.path.length_m <= 0.01 {
        let a = pts[0];
        let b = pts[1];
        let (dx, dy) = normalize_xy((b[1] - a[1]) * GEO_LNG_M, (b[0] - a[0]) * GEO_LAT_M);
        return Some((a[0], a[1], (dx as f32).atan2(dy as f32)));
    }
    let mut rem = dist_m.clamp(0.0, lane.path.length_m) as f64;
    for seg in pts.windows(2) {
        let a = seg[0];
        let b = seg[1];
        let seg_len = geo_dist_approx(a[0], a[1], b[0], b[1]) as f64;
        if seg_len <= 1e-6 {
            continue;
        }
        if rem <= seg_len {
            let t = rem / seg_len;
            let lat = a[0] + (b[0] - a[0]) * t;
            let lng = a[1] + (b[1] - a[1]) * t;
            let (dx, dy) = normalize_xy((b[1] - a[1]) * GEO_LNG_M, (b[0] - a[0]) * GEO_LAT_M);
            return Some((lat, lng, (dx as f32).atan2(dy as f32)));
        }
        rem -= seg_len;
    }
    let last = pts[pts.len() - 1];
    let prev = pts[pts.len() - 2];
    let (dx, dy) = normalize_xy(
        (last[1] - prev[1]) * GEO_LNG_M,
        (last[0] - prev[0]) * GEO_LAT_M,
    );
    Some((last[0], last[1], (dx as f32).atan2(dy as f32)))
}

/// Cumulative arc length along `lane.path` (same metric as [`sample_lane_path_at`])
/// from the lane start through the orthogonal projection of `(lat,lng)` onto the polyline.
fn closest_arclength_m_on_lane_path(lane: &Lane, lat: f64, lng: f64) -> Option<f32> {
    let pts = &lane.path.points;
    if pts.len() < 2 {
        return None;
    }
    let (px, py) = geo_to_m_xy(lat, lng);
    let mut accumulated = 0.0_f64;
    let mut best_s = 0.0_f64;
    let mut best_d2 = f64::MAX;

    for seg in pts.windows(2) {
        let a = seg[0];
        let b = seg[1];
        let (ax, ay) = geo_to_m_xy(a[0], a[1]);
        let (bx, by) = geo_to_m_xy(b[0], b[1]);
        let abx = bx - ax;
        let aby = by - ay;
        let seg_len2 = abx * abx + aby * aby;
        if seg_len2 <= 1e-12 {
            continue;
        }
        let seg_len = seg_len2.sqrt();
        let apx = px - ax;
        let apy = py - ay;
        let t = (apx * abx + apy * aby) / seg_len2;
        let t_clamped = t.clamp(0.0, 1.0);
        let qx = ax + abx * t_clamped;
        let qy = ay + aby * t_clamped;
        let d2 = (px - qx).powi(2) + (py - qy).powi(2);
        if d2 < best_d2 {
            best_d2 = d2;
            best_s = accumulated + seg_len * t_clamped;
        }
        accumulated += seg_len;
    }

    if best_d2 >= f64::MAX {
        None
    } else {
        let len_m = lane.path.length_m as f64;
        let mut s = best_s.clamp(0.0, len_m);
        if best_d2.sqrt() > LANE_PATH_PROJECTION_CONFIDENCE_M {
            let start = pts[0];
            let end = pts[pts.len() - 1];
            let (sx, sy) = geo_to_m_xy(start[0], start[1]);
            let (ex, ey) = geo_to_m_xy(end[0], end[1]);
            let d_start_sq = (px - sx).powi(2) + (py - sy).powi(2);
            let d_end_sq = (px - ex).powi(2) + (py - ey).powi(2);
            s = if d_start_sq <= d_end_sq { 0.0 } else { len_m };
        }
        Some(s as f32)
    }
}

/// First index to scan along `lane_route` after exiting this connector lane (advance past the spline).
#[inline]
fn lane_route_handover_scan_start(vehicle: &Vehicle, connector_just_finished: Option<LaneId>) -> usize {
    if let Some(cid) = connector_just_finished {
        if let Some(p) = vehicle.lane_route.iter().position(|&id| id == cid) {
            return (p + 1).min(vehicle.lane_route.len());
        }
    }
    vehicle.lane_route_pos.min(vehicle.lane_route.len())
}

/// Handover after a junction / edge rollover: anchor to planned outbound lane — keep continuity vs
/// `exit_hint`/projection (`s=0` alone caused visible snap-back down the outbound arm).
fn apply_lane_route_handover_teleport_from_queue(
    vehicle: &mut Vehicle,
    map: &MapData,
    scan_from: usize,
    route_edge_id: u64,
    preferred_lane_index: u8,
    outbound_edge_progress_hint_frac: Option<f32>,
) -> bool {
    if vehicle.lane_route.is_empty() || route_edge_id == u64::MAX {
        return false;
    }
    let from = scan_from.min(vehicle.lane_route.len());
    let mut fallback: Option<(usize, LaneId)> = None;
    for j in from..vehicle.lane_route.len() {
        let lid = vehicle.lane_route[j];
        let Some(lane) = map.lanes.get(&lid) else {
            continue;
        };
        if lane.edge_id == u64::MAX || lane.edge_id != route_edge_id {
            continue;
        }
        if lane.lane_index == preferred_lane_index {
            return snap_vehicle_to_lane_route_physical_start(
                vehicle,
                j,
                lane,
                outbound_edge_progress_hint_frac,
            );
        }
        if fallback.is_none() {
            fallback = Some((j, lid));
        }
    }
    if let Some((j, lid)) = fallback {
        map.lanes
            .get(&lid)
            .is_some_and(|lane| {
                snap_vehicle_to_lane_route_physical_start(
                    vehicle,
                    j,
                    lane,
                    outbound_edge_progress_hint_frac,
                )
            })
    } else {
        false
    }
}

#[inline]
fn snap_vehicle_to_lane_route_physical_start(
    vehicle: &mut Vehicle,
    idx: usize,
    lane: &Lane,
    outbound_edge_progress_hint_frac: Option<f32>,
) -> bool {
    vehicle.lane_route_pos = idx;
    vehicle.current_lane_id = Some(lane.id);
    vehicle.current_lane = lane.lane_index;

    let len = lane.path.length_m.max(0.001);

    let proj_s = closest_arclength_m_on_lane_path(lane, vehicle.lat, vehicle.lng)
        .unwrap_or(0.0)
        .clamp(0.0, len);

    let floor_from_exit = outbound_edge_progress_hint_frac
        .filter(|t| *t > 1e-4_f32 && *t < 1.0_f32)
        .map(|t| (t * len).clamp(0.0, len * 0.49))
        .unwrap_or(0.0_f32);

    let s_full = proj_s.max(floor_from_exit).clamp(0.0, len);
    // Keep margin before polyline tip (same rationale as geometric post-connector snap).
    let margin_m = POST_CONNECTOR_LANE_END_MARGIN_M
        .max((len * 0.018_f32).clamp(0.45_f32, 3.5_f32))
        .min(len * 0.22_f32);
    let max_s = (len - margin_m).max(0.0);
    let s = s_full.min(max_s);

    vehicle.lane_progress_m = s;
    vehicle.edge_progress = (vehicle.lane_progress_m / len).clamp(0.0, 1.0);

    if let Some((lat, lng, angle)) = sample_lane_path_at(lane, vehicle.lane_progress_m) {
        vehicle.lat = lat;
        vehicle.lng = lng;
        vehicle.angle = angle;
        return true;
    }
    let Some(first) = lane.path.points.first().copied() else {
        return false;
    };
    vehicle.lat = first[0];
    vehicle.lng = first[1];
    if let Some(sec) = lane.path.points.get(1).copied() {
        let (dx, dy) =
            normalize_xy((sec[1] - first[1]) * GEO_LNG_M, (sec[0] - first[0]) * GEO_LAT_M);
        vehicle.angle = (dx as f32).atan2(dy as f32);
    }
    true
}

/// Connector motion along the stored kurbo [`CubicBez`] (arc-length parameterisation).
/// Requested distances beyond arc length clamp to the curve end (`t`/arclength overshoot tolerated).
fn sample_connector_kurbo_at(lane: &Lane, dist_m: f32) -> Option<(f64, f64, f32)> {
    let cubic = lane_connector_cubic(lane)?;
    let acc = CONNECTOR_ARCLEN_ACC;
    let total = cubic.arclen(acc);
    if total <= 1e-9 {
        return None;
    }
    let s = (dist_m as f64).clamp(0.0, total);
    let t = cubic.inv_arclen(s, acc);
    let p = cubic.eval(t);
    let d = cubic.deriv().eval(t);
    let angle = (d.x as f32).atan2(d.y as f32);
    let (lat, lng) = m_xy_to_geo(p.x, p.y);
    Some((lat, lng, angle))
}

/// Build a `BezierPath` from three geographic control points (lat/lng).
/// The curve is constructed in local Cartesian metres (x = east, y = north).
#[inline]
fn bezier_path_from_geo(
    p1_lat: f64,
    p1_lng: f64,
    ctrl_lat: f64,
    ctrl_lng: f64,
    p2_lat: f64,
    p2_lng: f64,
) -> BezierPath {
    BezierPath::new(
        glam::DVec2::new(p1_lng * GEO_LNG_M, p1_lat * GEO_LAT_M),
        glam::DVec2::new(ctrl_lng * GEO_LNG_M, ctrl_lat * GEO_LAT_M),
        glam::DVec2::new(p2_lng * GEO_LNG_M, p2_lat * GEO_LAT_M),
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

    let v0_road = road_max * vehicle.personal_compliance;
    let vtype_max = vehicle.vehicle_type.params().max_speed;
    let mut desired = v0_road.min(vtype_max);
    if vehicle.on_turn_connector {
        desired = desired.min(TURN_CONNECTOR_TARGET_SPEED_MPS);
    }
    desired
}

#[inline]
fn ego_idm_past_connector_entry_for_idm(
    ego: &Vehicle,
    map: &MapData,
    intersections: &IntersectionManager,
) -> bool {
    if ego.route_pos + 1 >= ego.route.len() {
        return false;
    }
    let edge_idx = ego.route[ego.route_pos];
    let edge_len = match map.graph.edge_weight(edge_idx) {
        Some(e) => e.length_m,
        None => return false,
    };
    let dist_to_downstream_junction_m = edge_len * (1.0 - ego.edge_progress.clamp(0.0, 1.0));
    if dist_to_downstream_junction_m > IDM_CONNECTOR_SYNTHETIC_APPROACH_GATE_M {
        return false;
    }
    let Some(conn) = planned_turn_connector(ego, map) else {
        return false;
    };
    if conn.entry_progress < IDM_CONNECTOR_SYNTHETIC_ENTRY_MIN_FRAC {
        return false;
    }
    if ego.edge_progress + IDM_CONNECTOR_ENTRY_COMMIT_EPS < conn.entry_progress {
        return false;
    }
    if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
        let itype = &map.graph[tgt].intersection_type;
        if matches!(
            itype,
            IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing
        ) {
            let tgt_osm_id = map.graph[tgt].osm_id;
            if !intersections.can_vehicle_proceed(
                tgt_osm_id,
                ego.has_stopped_at_stop_sign,
                ego,
                map,
            ) {
                return false;
            }
        }
    }
    true
}

#[inline]
fn synthetic_turn_centre_dist_m(edge_progress: f32, conn: &PlannedTurnConnector) -> f32 {
    if conn.entry_progress >= 1.0 - 1e-6 {
        return 0.0;
    }
    let overshoot_ratio = ((edge_progress - conn.entry_progress)
        / (1.0 - conn.entry_progress).max(1e-6))
        .clamp(0.0, 1.0);
    conn.length_m * overshoot_ratio
}

fn apply_connector_conflict_obstacle(
    vehicle: &Vehicle,
    base_obstacle: ClosestObstacle,
    conflict_system: &ConflictSystem,
    map: &MapData,
    intersections: &IntersectionManager,
) -> ClosestObstacle {
    let use_connector_conflict_scan =
        vehicle.on_turn_connector || ego_idm_past_connector_entry_for_idm(vehicle, map, intersections);
    if !use_connector_conflict_scan {
        return base_obstacle;
    }
    let (in_e, out_e): (EdgeIndex, EdgeIndex) = if vehicle.on_turn_connector {
        (
            EdgeIndex::new(vehicle.turn_from_edge),
            EdgeIndex::new(vehicle.turn_to_edge),
        )
    } else {
        (
            vehicle.route[vehicle.route_pos],
            vehicle.route[vehicle.route_pos + 1],
        )
    };
    let Some((_, node_idx)) = map.graph.edge_endpoints(in_e) else {
        return base_obstacle;
    };
    let movement = (in_e, out_e);
    let lane_key = lane_movement_key_for_vehicle(vehicle, movement);
    let half_len = vehicle.vehicle_type.params().length_m * 0.5;
    let s_center_arc = if vehicle.on_turn_connector {
        vehicle.turn_dist_m as f32
    } else if let Some(ref conn) = planned_turn_connector(vehicle, map) {
        synthetic_turn_centre_dist_m(vehicle.edge_progress, conn)
    } else {
        0.0
    };
    let s_front_arc = (s_center_arc + half_len).max(0.0);
    let vr = vehicle_path_radius_m(vehicle);
    let look_ahead = conflict_scan_distance_m(vehicle).max(CONFLICT_LOOKAHEAD_M);
    let Some((block_dist, pt, owner, pt_id)) = conflict_system.first_blocking_conflict_on_arc(
        vehicle,
        lane_key,
        s_front_arc,
        look_ahead,
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
        conflict_point_id: Some(pt_id),
    };
    min_obstacle(base_obstacle, threat)
}

#[inline]
fn conflict_scan_distance_m(vehicle: &Vehicle) -> f32 {
    let comfort_b = vehicle.driver_profile.params().comfort_decel.max(0.6);
    // Physics-based look-ahead: v^2 / (2*b) + safety margin.
    let stopping = (vehicle.speed * vehicle.speed) / (2.0 * comfort_b);
    (stopping + CONFLICT_SCAN_SAFETY_MARGIN_M).max(CONFLICT_PRIORITY_ACTIVATION_M)
}

#[inline]
fn stopping_distance_m(speed: f32, decel: f32) -> f32 {
    if speed <= 0.0 {
        0.0
    } else {
        (speed * speed) / (2.0 * decel.max(0.2))
    }
}

#[inline]
fn emergency_decel_mps2(vehicle: &Vehicle) -> f32 {
    let params = vehicle.driver_profile.params();
    let vtype = vehicle.vehicle_type.params();
    (params.comfort_decel * 1.8)
        .max(vtype.max_decel * 1.25)
        .max(3.0)
}

/// Sphere around the hood (front reference) overlapping the conflict ball — tighter than full OBB
/// for manoeuvre envelopes: aft chassis can still intersect a roadside patch while the nose has left.
#[inline]
fn hood_conflicts_conflict_ball(vehicle: &Vehicle, point_pos_m: DVec2, point_radius_m: f32) -> bool {
    let h = hood_lng_lat_m(vehicle);
    let (hx, hy) = geo_to_m_xy(h[1], h[0]);
    let dx = (hx - point_pos_m.x) as f32;
    let dy = (hy - point_pos_m.y) as f32;
    let half_w = vehicle.vehicle_type.params().width_m * 0.5;
    let nose_r = half_w.max(0.65);
    let r = nose_r + point_radius_m.max(0.1) + CONFLICT_SHAPE_BUFFER_M * 0.5;
    dx * dx + dy * dy <= r * r
}

#[inline]
fn vehicle_centroid_engulfs_conflict_patch(vehicle: &Vehicle, patch: &ConflictPoint) -> bool {
    let (vx, vy) = geo_to_m_xy(vehicle.lat, vehicle.lng);
    let dx = (vx - patch.pos.x) as f32;
    let dy = (vy - patch.pos.y) as f32;
    let sep = (dx * dx + dy * dy).sqrt();
    sep <= vehicle_path_radius_m(vehicle)
        + patch.radius_m.max(0.1)
        + CONFLICT_SHAPE_BUFFER_M
        + CONFLICT_PHYSICAL_PATCH_ENVELOPE_M
}

/// Connector turns: centroid can lag behind the nose; overlap via hood must count as occupying the patch,
/// otherwise IDM clamps to MIN gap on foreign `reserved_by` and freezes (e.g. vehicle 40 vs 46 on same arc).
#[inline]
fn vehicle_body_physically_overlaps_conflict_patch(vehicle: &Vehicle, patch: &ConflictPoint) -> bool {
    vehicle_centroid_engulfs_conflict_patch(vehicle, patch)
        || hood_conflicts_conflict_ball(vehicle, patch.pos, patch.radius_m)
}

#[inline]
fn conflict_patch_physical_occupied_by_other(
    vehicles: &[Vehicle],
    requesting_id: u32,
    patch: &ConflictPoint,
) -> Option<u32> {
    for v in vehicles {
        if v.id == requesting_id || v.despawned {
            continue;
        }
        if vehicle_body_physically_overlaps_conflict_patch(v, patch) {
            return Some(v.id);
        }
    }
    None
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum TurnIntent {
    Straight,
    Left,
    Right,
    UTurn,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum IntersectionControl {
    Lights,
    Signs,
    Uncontrolled,
}

#[inline]
fn intersection_control_from_type(itype: &IntersectionType) -> IntersectionControl {
    match itype {
        IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing => {
            IntersectionControl::Lights
        }
        IntersectionType::Stop | IntersectionType::Yield => IntersectionControl::Signs,
        _ => IntersectionControl::Uncontrolled,
    }
}

fn movement_turn_intent(
    map: &MapData,
    movement: (EdgeIndex, EdgeIndex),
    node_idx: NodeIndex,
) -> TurnIntent {
    let Some((in_src, in_tgt)) = map.graph.edge_endpoints(movement.0) else {
        return TurnIntent::Straight;
    };
    let Some((out_src, out_tgt)) = map.graph.edge_endpoints(movement.1) else {
        return TurnIntent::Straight;
    };
    if in_tgt != node_idx || out_src != node_idx {
        return TurnIntent::Straight;
    }
    if out_tgt == in_src {
        return TurnIntent::UTurn;
    }
    let n = &map.graph[node_idx];
    let s = &map.graph[in_src];
    let t = &map.graph[out_tgt];
    let in_x = (n.lng - s.lng) as f32;
    let in_y = (n.lat - s.lat) as f32;
    let out_x = (t.lng - n.lng) as f32;
    let out_y = (t.lat - n.lat) as f32;
    let in_len = (in_x * in_x + in_y * in_y).sqrt().max(1e-6);
    let out_len = (out_x * out_x + out_y * out_y).sqrt().max(1e-6);
    let dot = ((in_x / in_len) * (out_x / out_len) + (in_y / in_len) * (out_y / out_len))
        .clamp(-1.0, 1.0);
    let angle = dot.acos();
    if angle < 0.25 {
        return TurnIntent::Straight;
    }
    let cross = in_x * out_y - in_y * out_x;
    if cross > 0.0 {
        TurnIntent::Left
    } else {
        TurnIntent::Right
    }
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
    vehicles: &[Vehicle],
    map: &MapData,
) -> ClosestObstacle {
    if vehicle.route_pos >= vehicle.route.len() {
        return base_obstacle;
    }
    // Vehicle already past the stop line — Bezier path owns its motion.
    if vehicle.on_turn_connector {
        return base_obstacle;
    }
    let edge_idx = vehicle.route[vehicle.route_pos];
    let edge = match map.graph.edge_weight(edge_idx) {
        Some(e) => e,
        None => return base_obstacle,
    };
    let dist_to_end = edge.length_m * (1.0 - vehicle.edge_progress);
    // IDM must see free space from the FRONT BUMPER to the stop line.
    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(vehicle, dist_to_end);

    let (tgt_node_idx, tgt_osm_id) = match map.graph.edge_endpoints(edge_idx) {
        Some((_, tgt)) => (tgt, map.graph[tgt].osm_id),
        None => return base_obstacle,
    };
    let intersection_type = &map.graph[tgt_node_idx].intersection_type;
    let mut best = base_obstacle;
    let scan_dist = conflict_scan_distance_m(vehicle);

    if let Some((movement, conn)) = vehicle_next_movement(vehicle, map) {
        let lane_key = lane_movement_key_for_vehicle(vehicle, movement);
        let dist_to_entry_center =
            ((conn.entry_progress - vehicle.edge_progress).max(0.0) * edge.length_m).max(0.0);
        let half_len = vehicle.vehicle_type.params().length_m * 0.5;
        let dist_to_entry = (dist_to_entry_center - half_len).max(MIN_IDM_GAP_M);
        let conflict_zone_depth_m = scan_dist + CONFLICT_APPROACH_TAIL_M;
        if dist_to_entry <= scan_dist && dist_to_end <= conflict_zone_depth_m {
            let deadlock_first = conflict_system
                .nodes
                .get(&tgt_node_idx)
                .and_then(|d| d.deadlock_first_move);
            if let Some((yield_id, yield_pos)) = conflict_system.cross_traffic_yield_target(
                vehicle,
                map,
                vehicles,
                intersections,
                tgt_node_idx,
                movement,
                scan_dist,
                deadlock_first,
            ) {
                let threat = ClosestObstacle {
                    kind: ObstacleKind::PriorityStopLine,
                    gap_m: dist_to_stop_line.max(MIN_IDM_GAP_M),
                    delta_v: vehicle.speed,
                    point_lng_lat: Some(yield_pos),
                    leader_vehicle_id: Some(yield_id),
                    conflict_reserver_id: None,
                    conflict_point_id: None,
                };
                best = min_obstacle(best, threat);
            }
            // Foreign conflict reservations: rely on `first_blocking_conflict_distance` only.
            // A separate path scan with `gap_m = dist_to_stop_line` falsely brakes the vehicle
            // (stop-line distance to the *node ahead*) even when the blocking conflict is far
            // along the connector or already stale — e.g. shortly after leaving a junction.
            if let Some((block_dist, pt, owner, pt_id)) =
                conflict_system.first_blocking_conflict_distance(
                vehicle,
                lane_key,
                dist_to_entry,
                scan_dist.max(CONFLICT_LOOKAHEAD_M),
                vehicle.id,
                vehicle_path_radius_m(vehicle),
                tgt_node_idx,
            ) {
                let threat = ClosestObstacle {
                    kind: ObstacleKind::ConflictPoint,
                    gap_m: block_dist.min(best.gap_m).max(MIN_IDM_GAP_M),
                    delta_v: vehicle.speed,
                    point_lng_lat: Some(pt),
                    leader_vehicle_id: None,
                    conflict_reserver_id: Some(owner),
                    conflict_point_id: Some(pt_id),
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
            let src = map
                .graph
                .edge_endpoints(edge_idx)
                .map(|e| e.0)
                .unwrap_or(tgt_node_idx);
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
                conflict_point_id: None,
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
            let src = map
                .graph
                .edge_endpoints(edge_idx)
                .map(|e| e.0)
                .unwrap_or(tgt_node_idx);
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
                conflict_point_id: None,
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
                conflict_point_id: None,
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
    let mut best_dv = delta_v;
    let fwd_e = vehicle.angle.sin() as f64;
    let fwd_n = vehicle.angle.cos() as f64;

    for &(tlat, tlng, tspeed) in trams {
        let dist = geo_dist_approx(vehicle.lat, vehicle.lng, tlat, tlng) - TRAM_LENGTH_M;
        let dist = dist.max(0.1);

        let east_m = (tlng - vehicle.lng) * GEO_LNG_M;
        let north_m = (tlat - vehicle.lat) * GEO_LAT_M;
        let forward_m = fwd_e * east_m + fwd_n * north_m;
        if forward_m < TRAM_LEADER_FORWARD_MIN_PROJ_M as f64 {
            continue;
        }

        // Only treat a tram as our leader if it is in front of us and close.
        if dist < best_gap && dist < 150.0 {
            let dv = vehicle.speed - tspeed;
            if dv > 0.0 || dist < 20.0 {
                best_gap = dist;
                best_dv = dv;
            }
        }
    }

    (best_gap, best_dv)
}

/// Index vehicles by the graph node at the **end** of their current edge
/// (the intersection / lane-merge node they are driving toward).
fn build_vehicles_by_target_node(
    vehicles: &[Vehicle],
    map: &MapData,
) -> HashMap<NodeIndex, Vec<usize>> {
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
    if ego.on_turn_connector {
        return (gap, delta_v);
    }
    // After the planned connector entry, `on_turn_connector` may still be false for a tick; using
    // `d_ego` as a synthetic closing gap falsely pins IDM (~2 m → “vehicle ahead” + hard stop).
    if let Some((_, conn)) = vehicle_next_movement(ego, map) {
        if ego.edge_progress + CROSS_TRAFFIC_ENTRY_COMMIT_EPS >= conn.entry_progress {
            return (gap, delta_v);
        }
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

#[inline]
fn vehicle_mark_despawned(vehicle: &mut Vehicle, reason: &'static str) {
    if vehicle.despawned {
        return;
    }
    vehicle.despawned = true;
    let lane = vehicle
        .current_lane_id
        .map(|id| id.to_string())
        .unwrap_or_else(|| "none".to_string());
    println!(
        "Vehicle {} despawned at lane {} due to: {}",
        vehicle.id, lane, reason
    );
    log::warn!(
        "Vehicle {} despawned at lane {} due to: {}",
        vehicle.id, lane, reason
    );
}

fn apply_vehicle_physics(
    vehicle: &mut Vehicle,
    accel: f32,
    real_dt_s: f32,
    map: &MapData,
    intersections: &IntersectionManager,
    conflict_system: &mut ConflictSystem,
    _by_target_node: &HashMap<NodeIndex, Vec<usize>>,
    vehicles: &[Vehicle],
    speed_cfg: &SpeedConfig,
    now_game_s: f32,
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
            let road_max = map
                .graph
                .edge_weight(vehicle.route[vehicle.route_pos])
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
        vehicle_mark_despawned(vehicle, "Frustration (rage)");
        return;
    }

    // ── Stop-sign: mark when fully stopped near the stop line ─────────────
    if !vehicle.has_stopped_at_stop_sign && vehicle.speed < 0.3 {
        if vehicle.route_pos < vehicle.route.len() {
            let edge_idx = vehicle.route[vehicle.route_pos];
            if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
                if matches!(map.graph[tgt].intersection_type, IntersectionType::Stop) {
                    let dist_to_end = map
                        .graph
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
        vehicle_mark_despawned(
            vehicle,
            "End of Route (tick start — route exhausted or stale route_pos)",
        );
        return;
    }

    let edge_idx = vehicle.route[vehicle.route_pos];
    let (edge_len, src_idx, tgt_idx) = {
        let edge = match map.graph.edge_weight(edge_idx) {
            Some(e) => e,
            None => {
                vehicle_mark_despawned(vehicle, "Out of Path (missing edge weight)");
                return;
            }
        };
        let endpoints = match map.graph.edge_endpoints(edge_idx) {
            Some(e) => e,
            None => {
                vehicle_mark_despawned(vehicle, "Out of Path (missing edge endpoints)");
                return;
            }
        };
        (edge.length_m, endpoints.0, endpoints.1)
    };

    // ── Turn connector state machine ──────────────────────────────────────
    if vehicle.on_turn_connector {
        // When a precomputed connector lane is available, follow its polyline exactly
        // so the vehicle tracks the same path shown by the visual lane lines.
        let (total_len, new_pos) = if let Some(cid) = vehicle.connector_lane_id {
            if let Some(clane) = map.lanes.get(&cid) {
                let total = lane_connector_cubic(clane)
                    .map(|c| c.arclen(CONNECTOR_ARCLEN_ACC))
                    .unwrap_or(clane.path.length_m as f64);
                let dist =
                    (vehicle.turn_dist_m + vehicle.speed as f64 * real_dt_s as f64).min(total);
                let pos = sample_connector_kurbo_at(clane, dist as f32)
                    .or_else(|| sample_lane_path_at(clane, dist as f32));
                if let Some((lat, lng, angle)) = pos {
                    vehicle.lat = lat;
                    vehicle.lng = lng;
                    vehicle.angle = angle;
                }
                (total, dist)
            } else {
                // Lane was removed; fall back gracefully.
                vehicle.connector_lane_id = None;
                let path = bezier_path_from_geo(
                    vehicle.turn_p1_lat,
                    vehicle.turn_p1_lng,
                    vehicle.turn_ctrl_lat,
                    vehicle.turn_ctrl_lng,
                    vehicle.turn_p2_lat,
                    vehicle.turn_p2_lng,
                );
                let dist = (vehicle.turn_dist_m + vehicle.speed as f64 * real_dt_s as f64)
                    .min(path.total_length);
                let state = path.get_state(dist);
                let (lat, lng, angle) = bezier_state_to_geo(&state);
                vehicle.lat = lat;
                vehicle.lng = lng;
                vehicle.angle = angle;
                (path.total_length, dist)
            }
        } else {
            // No precomputed lane — use quadratic Bezier (legacy / straight-through).
            let path = bezier_path_from_geo(
                vehicle.turn_p1_lat,
                vehicle.turn_p1_lng,
                vehicle.turn_ctrl_lat,
                vehicle.turn_ctrl_lng,
                vehicle.turn_p2_lat,
                vehicle.turn_p2_lng,
            );
            let dist = (vehicle.turn_dist_m + vehicle.speed as f64 * real_dt_s as f64)
                .min(path.total_length);
            let state = path.get_state(dist);
            let (lat, lng, angle) = bezier_state_to_geo(&state);
            vehicle.lat = lat;
            vehicle.lng = lng;
            vehicle.angle = angle;
            (path.total_length, dist)
        };

        vehicle.turn_dist_m = new_pos;

        // Keep edge_progress monotonic for leader logic while on the connector.
        let frac =
            ((vehicle.turn_dist_m / total_len.max(1e-9)).min(1.0)).clamp(0.0_f64, 1.0_f64) as f32;
        vehicle.edge_progress =
            vehicle.turn_entry_progress + (1.0 - vehicle.turn_entry_progress) * frac;
        conflict_system.update_reservation_motion_for_vehicle(
            vehicle.id,
            vehicle.turn_from_edge,
            vehicle.turn_to_edge,
            vehicle.turn_dist_m as f32,
            now_game_s,
        );
        conflict_system.clean_passed_points_for_vehicle(
            vehicle.id,
            vehicle.turn_from_edge,
            vehicle.turn_to_edge,
            vehicle.turn_dist_m as f32,
            vehicle.vehicle_type.params().length_m * 0.5,
        );
        // Progressive reservation: only near-field patches were claimed at the stop line; extend the
        // claim window as the vehicle actually moves along the Kubro arc (avoids pinning the whole junction).
        if let Some((movement, _)) = vehicle_next_movement(vehicle, map) {
            let in_e = EdgeIndex::new(vehicle.turn_from_edge);
            if let Some((_, node_idx)) = map.graph.edge_endpoints(in_e) {
                let lane_key = lane_movement_key_for_vehicle(vehicle, movement);
                if conflict_system
                    .expand_connector_conflict_reservations_along_arc(
                        vehicle.id,
                        lane_key,
                        node_idx,
                        vehicle.turn_dist_m as f32,
                        vehicle.vehicle_type.params().length_m * 0.5,
                        now_game_s,
                        vehicles,
                    )
                    .is_err()
                {
                    vehicle.speed = vehicle.speed.min(2.0);
                }
            }
        }

        if vehicle.turn_dist_m >= total_len {
            let finished_connector_id = vehicle.connector_lane_id;
            conflict_system.release_all_for_vehicle(vehicle.id);
            vehicle.on_turn_connector = false;
            vehicle.turn_dist_m = 0.0;
            vehicle.turn_from_edge = 0;
            vehicle.turn_to_edge = 0;
            vehicle.route_pos += 1;
            let exit_hint = vehicle.turn_exit_progress;
            vehicle.edge_progress = exit_hint;
            vehicle.has_stopped_at_stop_sign = false;

            if vehicle.route_pos >= vehicle.route.len() {
                vehicle_mark_despawned(
                    vehicle,
                    "End of Route (after connector — no next edge in planned route)",
                );
                return;
            }

            vehicle.target_lane = compute_vehicle_target_lane(vehicle, map);

            let outbound_eid = vehicle.route[vehicle.route_pos].index() as u64;
            let scan_start = lane_route_handover_scan_start(vehicle, finished_connector_id);
            let mut queue_snapped = apply_lane_route_handover_teleport_from_queue(
                vehicle,
                map,
                scan_start,
                outbound_eid,
                vehicle.target_lane,
                Some(exit_hint),
            );

        if !queue_snapped {
            if let Some(conn_id) = finished_connector_id {
                if let Some(conn_lane) = map.lanes.get(&conn_id) {
                    let tgt_idx = vehicle.target_lane;
                    let next_lane_opt = conn_lane
                        .connections
                        .iter()
                        .copied()
                        .find(|&lid| {
                            map.lanes.get(&lid).is_some_and(|nl| {
                                nl.edge_id != u64::MAX
                                    && nl.edge_id == outbound_eid
                                    && nl.lane_index == tgt_idx
                            })
                        })
                        .or_else(|| {
                            conn_lane.connections.iter().copied().find(|&lid| {
                                map.lanes.get(&lid).is_some_and(|nl| {
                                    nl.edge_id != u64::MAX && nl.edge_id == outbound_eid
                                })
                            })
                        })
                        .or_else(|| conn_lane.connections.first().copied());

                    if let Some(next_lane_id) = next_lane_opt {
                        let snapped_graph =
                            vehicle
                                .lane_route
                                .iter()
                                .position(|&id| id == next_lane_id)
                                .and_then(|lpos| {
                                    map.lanes.get(&next_lane_id).map(|nl| (lpos, nl))
                                })
                                .map_or(false, |(lpos, nl)| {
                                    nl.edge_id != u64::MAX
                                        && nl.edge_id == outbound_eid
                                        && snap_vehicle_to_lane_route_physical_start(
                                            vehicle,
                                            lpos,
                                            nl,
                                            Some(exit_hint),
                                        )
                                });
                        if snapped_graph {
                            queue_snapped = true;
                        } else {
                            vehicle.current_lane_id = Some(next_lane_id);
                            if let Some(next_lane) = map.lanes.get(&next_lane_id) {
                                if next_lane.edge_id != u64::MAX {
                                    vehicle.current_lane = next_lane.lane_index;
                                }
                            }
                        }
                    } else {
                        log::warn!(
                            "AUTO {} STRACILO CEL NA SKRZYZOWANIU: connector {} bez lane wyjazdowego",
                            vehicle.id,
                            conn_id
                        );
                    }
                }
            }

            if !queue_snapped {
                queue_snapped = apply_lane_route_handover_teleport_from_queue(
                    vehicle,
                    map,
                    vehicle.lane_route_pos.min(vehicle.lane_route.len()),
                    outbound_eid,
                    vehicle.target_lane,
                    Some(exit_hint),
                );
            }
        }

        if !queue_snapped {
            if !sync_vehicle_lane_route_state(vehicle, map) {
                let active_lane = vehicle.current_lane_id;
                let next_edge = vehicle.route.get(vehicle.route_pos).map(|e| e.index());
                log::warn!(
                    "AUTO {} STRACILO CEL NA SKRZYZOWANIU: brak lane-route handover (route_pos={}, active_lane_id={:?}, next_edge={:?})",
                    vehicle.id,
                    vehicle.route_pos,
                    active_lane,
                    next_edge
                );
            }
        } else {
            let _ = sync_vehicle_lane_route_state(vehicle, map);
        }
        vehicle.connector_lane_id = None;
        // If queue handover did not anchor us, infer along-lane offsets from posture (tolerant bbox).
        if !queue_snapped {
            if let Some(lid) = vehicle.current_lane_id {
                if let Some(lane) = map.lanes.get(&lid).filter(|l| l.edge_id != u64::MAX) {
                    let len = lane.path.length_m.max(0.001);
                    let margin_m = POST_CONNECTOR_LANE_END_MARGIN_M
                        .max((len * 0.018_f32).clamp(0.45_f32, 3.5_f32))
                        .min(len * 0.22_f32);
                    let max_s = (len - margin_m).max(0.0);
                    if let Some(s) = closest_arclength_m_on_lane_path(lane, vehicle.lat, vehicle.lng)
                    {
                        vehicle.lane_progress_m = s.clamp(0.0, max_s);
                        vehicle.edge_progress = (vehicle.lane_progress_m / len).clamp(0.0, 1.0);
                    } else {
                        vehicle.edge_progress =
                            exit_hint.clamp(0.0, (max_s / len).clamp(0.0, 1.0));
                        vehicle.lane_progress_m = (vehicle.edge_progress * len).clamp(0.0, max_s);
                    }
                }
            }
        }
        }
        return;
    }

    if edge_len > 0.0 {
        vehicle.edge_progress += vehicle.speed * real_dt_s / edge_len;
    }

    // Final edge destination handling: use per-vehicle virtual exit point on approach.
    let is_last_edge = vehicle.route_pos + 1 >= vehicle.route.len();
    if is_last_edge && node_is_intersection_like(map, tgt_idx) {
        let exit_t = vehicle.final_edge_exit_progress.clamp(0.0, 1.0);
        if vehicle.edge_progress >= exit_t {
            vehicle.edge_progress = exit_t;
            vehicle_mark_despawned(vehicle, "End of Route (destination reached)");
            return;
        }
    }

    // Hard red-line guard: never let a vehicle cross the stop line on red/yellow.
    // IDM does the smooth braking; this guard prevents rare frame-step overshoot.
    // Skip entirely when the vehicle is already on a turn connector (past the stop line).
    if !vehicle.on_turn_connector {
        if let Some((_, tgt)) = map.graph.edge_endpoints(edge_idx) {
            let tgt_osm_id = map.graph[tgt].osm_id;
            let itype = &map.graph[tgt].intersection_type;
            if matches!(
                itype,
                IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing
            ) && !intersections.can_vehicle_proceed(
                tgt_osm_id,
                vehicle.has_stopped_at_stop_sign,
                vehicle,
                map,
            ) {
                let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge_len.max(1.0)).clamp(0.0, 1.0);
                if vehicle.edge_progress >= stop_t {
                    vehicle.edge_progress = stop_t;
                    vehicle.speed = vehicle.speed.min(0.2);
                }
            }
        }
    } // end !on_turn_connector guard

    if let Some(conn) = planned_turn_connector(vehicle, map) {
        let mut can_enter_connector = true;
        if matches!(
            map.graph[tgt_idx].intersection_type,
            IntersectionType::TrafficLight | IntersectionType::PedestrianCrossing
        ) {
            let tgt_osm_id = map.graph[tgt_idx].osm_id;
            if !intersections.can_vehicle_proceed(
                tgt_osm_id,
                vehicle.has_stopped_at_stop_sign,
                vehicle,
                map,
            ) {
                can_enter_connector = false;
            }
        }
        if let Some((movement, _)) = vehicle_next_movement(vehicle, map) {
            let lane_key = lane_movement_key_for_vehicle(vehicle, movement);
            let deadlock_first = conflict_system
                .nodes
                .get(&tgt_idx)
                .and_then(|d| d.deadlock_first_move);
            if conflict_system.waiting_on_cross_traffic_yield(
                vehicle,
                map,
                vehicles,
                intersections,
                tgt_idx,
                movement,
                deadlock_first,
            ) {
                can_enter_connector = false;
            } else if conflict_system
                .try_reserve_all_for_vehicle(
                    vehicle.id,
                    lane_key,
                    tgt_idx,
                    now_game_s,
                    vehicles,
                )
                .is_err()
            {
                can_enter_connector = false;
            }
        }
        if !can_enter_connector {
            let stop_t = (1.0 - STOP_LINE_OFFSET_M / edge_len.max(1.0)).clamp(0.0, 1.0);
            if vehicle.edge_progress >= stop_t {
                vehicle.edge_progress = stop_t;
                vehicle.speed = vehicle.speed.min(0.2);
                return;
            }
        }
        if can_enter_connector && vehicle.edge_progress >= conn.entry_progress {
            let edge_progress_before_entry = vehicle.edge_progress;
            vehicle.on_turn_connector = true;
            // If we crossed entry within this frame, start the connector with
            // matching distance to avoid snapping backward to connector start.
            let overshoot_ratio = if conn.entry_progress < 1.0 {
                ((edge_progress_before_entry - conn.entry_progress) / (1.0 - conn.entry_progress))
                    .clamp(0.0, 1.0)
            } else {
                0.0
            };
            vehicle.turn_dist_m = (conn.length_m as f64 * overshoot_ratio as f64)
                .clamp(0.0, conn.length_m as f64);
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
            // Store the precomputed connector lane ID so traversal uses the same path.
            if vehicle.route_pos + 1 < vehicle.route.len() {
                vehicle.connector_lane_id = find_connector_lane_id(
                    map,
                    edge_idx,
                    vehicle.current_lane,
                    vehicle.route[vehicle.route_pos + 1],
                    vehicle.target_lane,
                );
                if vehicle.connector_lane_id.is_none() {
                    log::warn!(
                        "AUTO {} STRACILO CEL NA SKRZYZOWANIU: brak connectora edge {} lane {} -> edge {} lane {}",
                        vehicle.id,
                        edge_idx.index(),
                        vehicle.current_lane,
                        vehicle.route[vehicle.route_pos + 1].index(),
                        vehicle.target_lane
                    );
                }
            }

            // Snap lateral offset to target immediately so it doesn't drift
            // perpendicularly to the Bezier while animating during the turn.
            vehicle.current_lateral_offset = vehicle.target_lateral_offset;

            // Immediately place vehicle at connector start so there is no 1-frame positional snap.
            // Use the polyline when we have a precomputed connector lane, else fall back to bezier.
            let placed = if let Some(cid) = vehicle.connector_lane_id {
                map.lanes
                    .get(&cid)
                    .and_then(|cl| {
                        sample_connector_kurbo_at(cl, vehicle.turn_dist_m as f32)
                            .or_else(|| sample_lane_path_at(cl, vehicle.turn_dist_m as f32))
                    })
                    .map(|(lat0, lng0, angle0)| {
                        vehicle.lat = lat0;
                        vehicle.lng = lng0;
                        vehicle.angle = angle0;
                    })
                    .is_some()
            } else {
                false
            };
            if !placed {
                let path0 = bezier_path_from_geo(
                    conn.p1_lat,
                    conn.p1_lng,
                    conn.ctrl_lat,
                    conn.ctrl_lng,
                    conn.p2_lat,
                    conn.p2_lng,
                );
                let state0 = path0.get_state(vehicle.turn_dist_m);
                let (lat0, lng0, angle0) = bezier_state_to_geo(&state0);
                vehicle.lat = lat0;
                vehicle.lng = lng0;
                vehicle.angle = angle0;
            }
            // Do NOT fall through to linear interpolation — connector takes over next tick.
            return;
        }
    }

    if vehicle.edge_progress >= 1.0 {
        vehicle.route_pos += 1;
        vehicle.edge_progress = 0.0;
        vehicle.has_stopped_at_stop_sign = false; // reset for next edge

        if vehicle.route_pos >= vehicle.route.len() {
            vehicle_mark_despawned(
                vehicle,
                "End of Route (rolled past final edge — route had no successor)",
            );
            return;
        }

        vehicle.target_lane = compute_vehicle_target_lane(vehicle, map);
        let outbound_eid = vehicle.route[vehicle.route_pos].index() as u64;
        let scan_from = vehicle.lane_route_pos.min(vehicle.lane_route.len());
        let queue_snapped = apply_lane_route_handover_teleport_from_queue(
            vehicle,
            map,
            scan_from,
            outbound_eid,
            vehicle.target_lane,
            None,
        );
        if !queue_snapped {
            if !sync_vehicle_lane_route_state(vehicle, map) {
                let active_lane = vehicle.current_lane_id;
                let next_edge = vehicle.route.get(vehicle.route_pos).map(|e| e.index());
                let lane_conn_count = active_lane
                    .and_then(|lid| map.lanes.get(&lid).map(|l| l.connections.len()))
                    .unwrap_or(0);
                log::warn!(
                    "AUTO {} EDGE-END: brak sukcesora lane na nowym odcinku (route_pos={}, active_lane_id={:?}, active_lane_connections={}, next_edge={:?})",
                    vehicle.id,
                    vehicle.route_pos,
                    active_lane,
                    lane_conn_count,
                    next_edge
                );
            }
            vehicle.lane_progress_m = 0.0;
        } else {
            let _ = sync_vehicle_lane_route_state(vehicle, map);
        }
    }

    let active_edge_idx = vehicle
        .route
        .get(vehicle.route_pos)
        .copied()
        .unwrap_or(edge_idx);

    // Full lane graph path following: sample directly from physical lane path.
    let lane_id = vehicle.current_lane_id.or_else(|| {
        map.lanes
            .values()
            .find(|l| {
                l.edge_id == active_edge_idx.index() as u64 && l.lane_index == vehicle.current_lane
            })
            .map(|l| l.id)
    });
    if let Some(lid) = lane_id {
        if let Some(lane) = map.lanes.get(&lid) {
            vehicle.current_lane_id = Some(lid);
            vehicle.lane_progress_m =
                (vehicle.edge_progress * lane.path.length_m).clamp(0.0, lane.path.length_m);
            if let Some((lat, lng, angle)) = sample_lane_path_at(lane, vehicle.lane_progress_m) {
                vehicle.lat = lat;
                vehicle.lng = lng;
                vehicle.angle = angle;
                return;
            }
        }
    }
    // Fallback for malformed lane graph data.
    let src = &map.graph[src_idx];
    let tgt = &map.graph[tgt_idx];
    let t = vehicle.edge_progress as f64;
    vehicle.lat = src.lat + (tgt.lat - src.lat) * t;
    vehicle.lng = src.lng + (tgt.lng - src.lng) * t;
    let (dx, dy) = normalize_xy(
        (tgt.lng - src.lng) * GEO_LNG_M,
        (tgt.lat - src.lat) * GEO_LAT_M,
    );
    vehicle.angle = (dx as f32).atan2(dy as f32);
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
    if b.gap_m < a.gap_m {
        b
    } else {
        a
    }
}

/// Brake diagnostics (strong IDM slowdown): identify conflict-patch vs phantom vehicle obstacle.
#[inline]
fn idm_conflict_brake_diagnostic_log(accel: f32, ego: &Vehicle, obs: &ClosestObstacle) {
    if accel > IDM_UI_BRAKE_ACCEL_THRESHOLD {
        return;
    }
    match obs.kind {
        ObstacleKind::ConflictPoint => {
            log::debug!(
                "Vehicle {} braking due to ConflictPoint {} (reserved by {:?}); gap {:.2} m; on_turn_connector={}",
                ego.id,
                obs.conflict_point_id.unwrap_or(0),
                obs.conflict_reserver_id,
                obs.gap_m,
                ego.on_turn_connector,
            );
            if obs.conflict_reserver_id == Some(ego.id) {
                log::warn!(
                    "BUG: ego {} braking on ConflictPoint {} reserved by self (filters should skip)",
                    ego.id,
                    obs.conflict_point_id.unwrap_or(0),
                );
            }
        }
        ObstacleKind::Vehicle
            if obs.leader_vehicle_id.is_none()
                && obs.gap_m < 25.0
                && obs.conflict_point_id.is_none() =>
        {
            log::warn!(
                "Vehicle {} strong brake: vehicle-class obstacle gap={:.2} m without leader id (tram/proxy/overlap cue) on_turn_connector={}",
                ego.id,
                obs.gap_m,
                ego.on_turn_connector
            );
        }
        _ => {}
    }
}

#[inline]
fn compute_idm_accel_with_obstacle(vehicle: &Vehicle, desired: f32, obs: ClosestObstacle) -> f32 {
    let params = vehicle.driver_profile.params();
    let vtype = vehicle.vehicle_type.params();
    let mut accel = idm_acceleration(
        vehicle.speed,
        desired,
        obs.gap_m,
        obs.delta_v,
        &params,
        &vtype,
    );
    if emergency_braking_needed(vehicle, obs) {
        let emergency_b = emergency_decel_mps2(vehicle);
        accel = accel.min(-emergency_b);
    }
    accel
}

#[inline]
fn emergency_braking_needed(vehicle: &Vehicle, obs: ClosestObstacle) -> bool {
    if !matches!(
        obs.kind,
        ObstacleKind::ConflictPoint | ObstacleKind::PriorityStopLine
    ) {
        return false;
    }
    let comfort_stop =
        stopping_distance_m(vehicle.speed, vehicle.driver_profile.params().comfort_decel);
    obs.gap_m < comfort_stop + 1.0
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
) -> IdmStepResult {
    let desired_base = compute_desired_speed(ego, map);

    let idm_connector_conflict_arc = ego.on_turn_connector
        || ego_idm_past_connector_entry_for_idm(ego, map, intersections);
    if idm_connector_conflict_arc {
        let mut desired = desired_base;
        if !ego.on_turn_connector {
            desired = desired.min(TURN_CONNECTOR_TARGET_SPEED_MPS);
        }
        let mut base = free_leader_obstacle();
        let (gap, dv) = apply_tram_leader_effect(ego, base.gap_m, base.delta_v, tram_snapshot);
        base.gap_m = gap;
        base.delta_v = dv;
        let obstacle = apply_connector_conflict_obstacle(ego, base, conflict_system, map, intersections);
        let accel = compute_idm_accel_with_obstacle(ego, desired, obstacle);
        idm_conflict_brake_diagnostic_log(accel, ego, &obstacle);
        return IdmStepResult {
            accel,
            desired_speed: desired,
            obstacle,
        };
    }

    let desired = desired_base;

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
        vehicles,
        map,
    );
    let accel = compute_idm_accel_with_obstacle(ego, desired, obstacle);
    idm_conflict_brake_diagnostic_log(accel, ego, &obstacle);

    IdmStepResult {
        accel,
        desired_speed: desired,
        obstacle,
    }
}

#[inline]
fn vehicle_next_movement(
    vehicle: &Vehicle,
    map: &MapData,
) -> Option<((EdgeIndex, EdgeIndex), PlannedTurnConnector)> {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return None;
    }
    let in_edge = vehicle.route[vehicle.route_pos];
    let out_edge = vehicle.route[vehicle.route_pos + 1];
    let conn = planned_turn_connector(vehicle, map)?;
    Some(((in_edge, out_edge), conn))
}

#[inline]
fn lane_movement_key_for_vehicle(
    vehicle: &Vehicle,
    movement: (EdgeIndex, EdgeIndex),
) -> LaneMovementKey {
    LaneMovementKey {
        in_edge: movement.0,
        out_edge: movement.1,
        in_lane: vehicle.current_lane,
        out_lane: vehicle.target_lane,
    }
}

impl ConflictSystem {
    fn movements_conflict(
        &self,
        movement_a: (EdgeIndex, EdgeIndex),
        movement_b: (EdgeIndex, EdgeIndex),
        node_idx: NodeIndex,
    ) -> bool {
        let Some(node) = self.nodes.get(&node_idx) else {
            return false;
        };
        for (ka, path_a) in &node.by_movement {
            if ka.in_edge != movement_a.0 || ka.out_edge != movement_a.1 {
                continue;
            }
            if path_a.points.is_empty() {
                continue;
            }
            for (kb, path_b) in &node.by_movement {
                if kb.in_edge != movement_b.0 || kb.out_edge != movement_b.1 {
                    continue;
                }
                if path_b.points.is_empty() {
                    continue;
                }
                for pa in &path_a.points {
                    for pb in &path_b.points {
                        if (pa.pos - pb.pos).length_squared() < 1.0 {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    fn is_yielding_to_right(
        &self,
        ego: &Vehicle,
        vehicles: &[Vehicle],
        map: &MapData,
        node_idx: NodeIndex,
        movement: (EdgeIndex, EdgeIndex),
        scan_dist_m: f32,
    ) -> Option<(u32, [f64; 2])> {
        let _node = self.nodes.get(&node_idx)?;
        let (ego_src, ego_tgt) = map.graph.edge_endpoints(movement.0)?;
        if ego_tgt != node_idx {
            return None;
        }
        let n = &map.graph[node_idx];
        let e_src = &map.graph[ego_src];
        let ego_in_x = (n.lng - e_src.lng) as f32;
        let ego_in_y = (n.lat - e_src.lat) as f32;
        let ego_in_len = (ego_in_x * ego_in_x + ego_in_y * ego_in_y).sqrt().max(1e-6);
        let ego_in_ux = ego_in_x / ego_in_len;
        let ego_in_uy = ego_in_y / ego_in_len;
        let right_x = ego_in_uy;
        let right_y = -ego_in_ux;
        let my_intent = movement_turn_intent(map, movement, node_idx);

        let mut best: Option<(f32, u32, [f64; 2])> = None;
        for other in vehicles {
            if other.id == ego.id || other.despawned || other.on_turn_connector {
                continue;
            }
            if other.route_pos + 1 >= other.route.len() {
                continue;
            }
            let in_edge = other.route[other.route_pos];
            let out_edge = other.route[other.route_pos + 1];
            let Some((o_src, o_tgt)) = map.graph.edge_endpoints(in_edge) else {
                continue;
            };
            if o_tgt != node_idx {
                continue;
            }
            let movement_o = (in_edge, out_edge);
            if !self.movements_conflict(movement, movement_o, node_idx) {
                continue;
            }
            let o_edge_len = map
                .graph
                .edge_weight(in_edge)
                .map(|e| e.length_m)
                .unwrap_or(100.0);
            let d_other = (1.0 - other.edge_progress) * o_edge_len;
            if d_other > scan_dist_m {
                continue;
            }

            // Right-side sector in ego local frame (dot with right normal > 0).
            let o_src_n = &map.graph[o_src];
            let rel_x = (o_src_n.lng - n.lng) as f32;
            let rel_y = (o_src_n.lat - n.lat) as f32;
            let from_right = rel_x * right_x + rel_y * right_y > 0.0;

            // Exception: ego straight has priority over opposite left-turn.
            let o_src_nx = (n.lng - o_src_n.lng) as f32;
            let o_src_ny = (n.lat - o_src_n.lat) as f32;
            let o_len = (o_src_nx * o_src_nx + o_src_ny * o_src_ny).sqrt().max(1e-6);
            let dot_opp =
                (ego_in_ux * (o_src_nx / o_len) + ego_in_uy * (o_src_ny / o_len)).clamp(-1.0, 1.0);
            let opposite_approach = dot_opp < -0.8;
            let other_intent = movement_turn_intent(map, movement_o, node_idx);
            if my_intent == TurnIntent::Straight
                && opposite_approach
                && other_intent == TurnIntent::Left
            {
                continue;
            }
            if !from_right {
                continue;
            }
            let other_pos = [other.lng, other.lat];
            if best.map_or(true, |(bd, _, _)| d_other < bd) {
                best = Some((d_other, other.id, other_pos));
            }
        }
        best.map(|(_, id, pos)| (id, pos))
    }

    fn is_yielding_to_opposite_straight(
        &self,
        ego: &Vehicle,
        vehicles: &[Vehicle],
        map: &MapData,
        node_idx: NodeIndex,
        movement: (EdgeIndex, EdgeIndex),
        scan_dist_m: f32,
    ) -> Option<(u32, [f64; 2])> {
        let (ego_src, ego_tgt) = map.graph.edge_endpoints(movement.0)?;
        if ego_tgt != node_idx {
            return None;
        }
        let n = &map.graph[node_idx];
        let e_src = &map.graph[ego_src];
        let ego_in_x = (n.lng - e_src.lng) as f32;
        let ego_in_y = (n.lat - e_src.lat) as f32;
        let ego_in_len = (ego_in_x * ego_in_x + ego_in_y * ego_in_y).sqrt().max(1e-6);
        let ego_in_ux = ego_in_x / ego_in_len;
        let ego_in_uy = ego_in_y / ego_in_len;

        let mut best: Option<(f32, u32, [f64; 2])> = None;
        for other in vehicles {
            if other.id == ego.id || other.despawned || other.on_turn_connector {
                continue;
            }
            if other.route_pos + 1 >= other.route.len() {
                continue;
            }
            let in_edge = other.route[other.route_pos];
            let out_edge = other.route[other.route_pos + 1];
            let Some((o_src, o_tgt)) = map.graph.edge_endpoints(in_edge) else {
                continue;
            };
            if o_tgt != node_idx {
                continue;
            }
            let movement_o = (in_edge, out_edge);
            if !self.movements_conflict(movement, movement_o, node_idx) {
                continue;
            }
            let other_intent = movement_turn_intent(map, movement_o, node_idx);
            if other_intent != TurnIntent::Straight {
                continue;
            }
            let o_edge_len = map
                .graph
                .edge_weight(in_edge)
                .map(|e| e.length_m)
                .unwrap_or(100.0);
            let d_other = (1.0 - other.edge_progress) * o_edge_len;
            if d_other > scan_dist_m {
                continue;
            }
            let o_src_n = &map.graph[o_src];
            let o_src_nx = (n.lng - o_src_n.lng) as f32;
            let o_src_ny = (n.lat - o_src_n.lat) as f32;
            let o_len = (o_src_nx * o_src_nx + o_src_ny * o_src_ny).sqrt().max(1e-6);
            let dot_opp =
                (ego_in_ux * (o_src_nx / o_len) + ego_in_uy * (o_src_ny / o_len)).clamp(-1.0, 1.0);
            let opposite_approach = dot_opp < -0.8;
            if !opposite_approach {
                continue;
            }
            let other_pos = [other.lng, other.lat];
            if best.map_or(true, |(bd, _, _)| d_other < bd) {
                best = Some((d_other, other.id, other_pos));
            }
        }
        best.map(|(_, id, pos)| (id, pos))
    }

    /// Single rule for “give way to cross traffic” at a node:
    /// - **Uncontrolled:** yield to traffic from the right (`is_yielding_to_right`).
    /// - **Lights (green):** left turn yields to opposite straight only.
    /// - **Stop / yield signs:** no extra cross-traffic scan here (handled by stop/yield IDM).
    ///
    /// `deadlock_first_mover`: when set to this vehicle’s id, skip yield (deadlock breaker).
    fn cross_traffic_yield_target(
        &self,
        vehicle: &Vehicle,
        map: &MapData,
        vehicles: &[Vehicle],
        intersections: &IntersectionManager,
        node_idx: NodeIndex,
        movement: (EdgeIndex, EdgeIndex),
        scan_dist_m: f32,
        deadlock_first_mover: Option<u32>,
    ) -> Option<(u32, [f64; 2])> {
        if deadlock_first_mover == Some(vehicle.id) {
            return None;
        }
        let intersection_type = &map.graph[node_idx].intersection_type;
        let control = intersection_control_from_type(intersection_type);
        let look = scan_dist_m.max(CONFLICT_LOOKAHEAD_M);
        match control {
            IntersectionControl::Uncontrolled => {
                self.is_yielding_to_right(vehicle, vehicles, map, node_idx, movement, look)
            }
            IntersectionControl::Lights => {
                if movement_turn_intent(map, movement, node_idx) != TurnIntent::Left {
                    return None;
                }
                let tgt_osm_id = map.graph[node_idx].osm_id;
                if !intersections.can_vehicle_proceed(
                    tgt_osm_id,
                    vehicle.has_stopped_at_stop_sign,
                    vehicle,
                    map,
                ) {
                    return None;
                }
                self.is_yielding_to_opposite_straight(
                    vehicle, vehicles, map, node_idx, movement, look,
                )
            }
            IntersectionControl::Signs => None,
        }
    }

    #[inline]
    fn waiting_on_cross_traffic_yield(
        &self,
        vehicle: &Vehicle,
        map: &MapData,
        vehicles: &[Vehicle],
        intersections: &IntersectionManager,
        node_idx: NodeIndex,
        movement: (EdgeIndex, EdgeIndex),
        deadlock_first_mover: Option<u32>,
    ) -> bool {
        self.cross_traffic_yield_target(
            vehicle,
            map,
            vehicles,
            intersections,
            node_idx,
            movement,
            conflict_scan_distance_m(vehicle),
            deadlock_first_mover,
        )
        .is_some()
    }

    fn release_reservations_overridden_by_foreign_occupants(&mut self, vehicles: &[Vehicle]) {
        for node in self.nodes.values_mut() {
            for path in node.by_movement.values_mut() {
                for p in &mut path.points {
                    let Some(owner) = p.reserved_by else {
                        continue;
                    };
                    for v in vehicles {
                        if v.id == owner || v.despawned {
                            continue;
                        }
                        if vehicle_body_physically_overlaps_conflict_patch(v, p) {
                            p.reserved_by = None;
                            p.reserved_at_game_s = None;
                            p.reserved_last_progress_m = None;
                            p.reserved_last_motion_s = None;
                            break;
                        }
                    }
                }
            }
        }
    }

    fn try_reserve_all_for_vehicle(
        &mut self,
        vehicle_id: u32,
        movement: LaneMovementKey,
        node_idx: NodeIndex,
        now_game_s: f32,
        vehicles: &[Vehicle],
    ) -> Result<(), ([f64; 2], u32)> {
        let Some(node) = self.nodes.get_mut(&node_idx) else {
            return Ok(());
        };
        let Some(path) = node.by_movement.get_mut(&movement) else {
            return Ok(());
        };

        // Do not claim patches whose centroid or hood overlaps another vehicle on the junction.
        for p in &path.points {
            if let Some(occupant_id) =
                conflict_patch_physical_occupied_by_other(vehicles, vehicle_id, p)
            {
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                return Err(([lng, lat], occupant_id));
            }
        }

        for p in &path.points {
            if let Some(owner) = p.reserved_by {
                if owner != vehicle_id {
                    let lng = p.pos.x / GEO_LNG_M;
                    let lat = p.pos.y / GEO_LAT_M;
                    return Err(([lng, lat], owner));
                }
            }
        }
        // Claim only the **near** arc window — not every patch to the far side of the junction (that
        // pins the node centre for the whole manoeuvre and causes deadlocks). Deeper patches are
        // claimed in [`expand_connector_conflict_reservations_along_arc`] each tick on the connector.
        for p in &mut path.points {
            if p.distance_on_path > CONFLICT_RESERVE_INITIAL_ARC_M {
                continue;
            }
            p.reserved_by = Some(vehicle_id);
            p.reserved_at_game_s = Some(now_game_s);
            p.reserved_last_progress_m = Some(0.0);
            p.reserved_last_motion_s = Some(now_game_s);
        }
        Ok(())
    }

    /// Extend reservations along the connector as the vehicle advances — keeps claims aligned with
    /// physical occupancy instead of holding the entire precomputed CP list from t=0.
    fn expand_connector_conflict_reservations_along_arc(
        &mut self,
        vehicle_id: u32,
        movement: LaneMovementKey,
        node_idx: NodeIndex,
        s_center_arc_m: f32,
        vehicle_half_length_m: f32,
        now_game_s: f32,
        vehicles: &[Vehicle],
    ) -> Result<(), ([f64; 2], u32)> {
        let horizon_end =
            s_center_arc_m + vehicle_half_length_m + CONFLICT_RESERVE_HORIZON_AHEAD_M;
        let Some(node) = self.nodes.get_mut(&node_idx) else {
            return Ok(());
        };
        let Some(path) = node.by_movement.get_mut(&movement) else {
            return Ok(());
        };
        for p in &mut path.points {
            if p.distance_on_path > horizon_end {
                continue;
            }
            if p.reserved_by == Some(vehicle_id) {
                continue;
            }
            if let Some(occupant_id) =
                conflict_patch_physical_occupied_by_other(vehicles, vehicle_id, p)
            {
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                return Err(([lng, lat], occupant_id));
            }
            if let Some(owner) = p.reserved_by {
                if owner != vehicle_id {
                    let lng = p.pos.x / GEO_LNG_M;
                    let lat = p.pos.y / GEO_LAT_M;
                    return Err(([lng, lat], owner));
                }
            }
            p.reserved_by = Some(vehicle_id);
            p.reserved_at_game_s = Some(now_game_s);
            p.reserved_last_progress_m = Some(s_center_arc_m);
            p.reserved_last_motion_s = Some(now_game_s);
        }
        Ok(())
    }

    fn update_deadlock_state(
        &mut self,
        map: &MapData,
        vehicles: &[Vehicle],
        by_target_node: &HashMap<NodeIndex, Vec<usize>>,
        intersections: &IntersectionManager,
        dt_s: f32,
    ) {
        let node_indices: Vec<NodeIndex> = self.nodes.keys().copied().collect();
        for node_idx in node_indices {
            let forced_first = self
                .nodes
                .get(&node_idx)
                .and_then(|d| d.deadlock_first_move);
            let Some(candidates) = by_target_node.get(&node_idx) else {
                if let Some(data) = self.nodes.get_mut(&node_idx) {
                    data.deadlock_timer_s = 0.0;
                    data.deadlock_first_move = None;
                }
                continue;
            };
            let waiting: Vec<u32> = candidates
                .iter()
                .filter_map(|&i| {
                    let v = vehicles.get(i)?;
                    if v.route_pos + 1 >= v.route.len() || v.on_turn_connector {
                        return None;
                    }
                    let movement = (v.route[v.route_pos], v.route[v.route_pos + 1]);
                    let in_edge = movement.0;
                    let edge = map.graph.edge_weight(in_edge)?;
                    let dist_to_end = edge.length_m * (1.0 - v.edge_progress);
                    let dist_to_stop_line = distance_to_stop_line_from_front_bumper(v, dist_to_end);
                    let waiting_on_stop_line = v.speed < 0.35 && dist_to_stop_line <= 2.0;
                    let waits_yield = self.waiting_on_cross_traffic_yield(
                        v,
                        map,
                        vehicles,
                        intersections,
                        node_idx,
                        movement,
                        forced_first,
                    );
                    if waiting_on_stop_line && waits_yield {
                        Some(v.id)
                    } else {
                        None
                    }
                })
                .collect();
            if let Some(data) = self.nodes.get_mut(&node_idx) {
                if waiting.len() >= 2 {
                    data.deadlock_timer_s += dt_s;
                    if data.deadlock_timer_s >= DEADLOCK_BREAK_S {
                        let next = waiting.iter().copied().min();
                        if data.deadlock_first_move != next {
                            if let Some(force_id) = next {
                                log::info!("DEADLOCK DETECTED - Forcing Car {} to move", force_id);
                            }
                        }
                        data.deadlock_first_move = next;
                    }
                } else {
                    data.deadlock_timer_s = 0.0;
                    data.deadlock_first_move = None;
                }
            }
        }
    }

    fn first_blocking_conflict_distance(
        &self,
        vehicle: &Vehicle,
        movement: LaneMovementKey,
        dist_to_entry: f32,
        look_ahead: f32,
        vehicle_id: u32,
        vehicle_radius_m: f32,
        node_idx: NodeIndex,
    ) -> Option<(f32, [f64; 2], u32, u64)> {
        let node = self.nodes.get(&node_idx)?;
        let path = node.by_movement.get(&movement)?;
        path.points
            .iter()
            .filter_map(|p| {
                // Self-ignore: our own reservations are never obstacles for IDM path scan.
                let owner_id = match p.reserved_by {
                    None => return None,
                    Some(id) if id == vehicle_id => return None,
                    Some(id) => id,
                };
                // Hull on the patch (nose turns before centroid catches up): ignore alien `reserved_by`.
                if vehicle_body_physically_overlaps_conflict_patch(vehicle, p) {
                    return None;
                }
                let d_raw = dist_to_entry + p.distance_on_path;
                let mut d = (d_raw - vehicle_radius_m - CONFLICT_SHAPE_BUFFER_M).max(MIN_IDM_GAP_M);
                if d_raw <= CONFLICT_OVERLAP_HARD_STOP_D_RAW_CAP_APPROACH_M
                    && hood_conflicts_conflict_ball(vehicle, p.pos, p.radius_m)
                {
                    d = MIN_IDM_GAP_M;
                }
                if d > look_ahead {
                    return None;
                }
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                Some((d, [lng, lat], owner_id, p.id))
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
    }

    /// Bezier-turn connector: ego front-bumper arc length from Bezier start (p1).
    fn first_blocking_conflict_on_arc(
        &self,
        vehicle: &Vehicle,
        movement: LaneMovementKey,
        s_front_arc_m: f32,
        look_ahead: f32,
        vehicle_id: u32,
        vehicle_radius_m: f32,
        node_idx: NodeIndex,
    ) -> Option<(f32, [f64; 2], u32, u64)> {
        let node = self.nodes.get(&node_idx)?;
        let path = node.by_movement.get(&movement)?;
        path.points
            .iter()
            .filter_map(|p| {
                // Self-ignore: never brake on conflict patches still attributed to ourselves.
                let owner_id = match p.reserved_by {
                    None => return None,
                    Some(id) if id == vehicle_id => return None,
                    Some(id) => id,
                };
                // Hull overlap pre-empts a remote reservation (fixes arc freeze nose-on-patch cases).
                if vehicle_body_physically_overlaps_conflict_patch(vehicle, p) {
                    return None;
                }
                let (vx, vy) = geo_to_m_xy(vehicle.lat, vehicle.lng);
                let rdx = vx - p.pos.x;
                let rdy = vy - p.pos.y;
                let centre_sep = ((rdx * rdx + rdy * rdy) as f32).sqrt();
                let radial_cap = vehicle_radius_m
                    + p.radius_m.max(0.1)
                    + CONFLICT_SHAPE_BUFFER_M
                    + CONFLICT_EUCLIDEAN_CLEAR_EXTRA_M;
                // Bezier traversal vs sampled conflict path can disagree on arc length; if the hull
                // is clearly clear in the plane, do not latch a phantom "ahead" blocker (self/other).
                if centre_sep > radial_cap {
                    return None;
                }
                let d_raw = p.distance_on_path - s_front_arc_m;
                if d_raw < 0.0 {
                    return None;
                }
                let mut d = (d_raw - vehicle_radius_m - CONFLICT_SHAPE_BUFFER_M).max(MIN_IDM_GAP_M);
                if d_raw <= CONFLICT_OVERLAP_HARD_STOP_D_RAW_CAP_CONNECTOR_M
                    && hood_conflicts_conflict_ball(vehicle, p.pos, p.radius_m)
                {
                    d = MIN_IDM_GAP_M;
                }
                if d > look_ahead {
                    return None;
                }
                let lng = p.pos.x / GEO_LNG_M;
                let lat = p.pos.y / GEO_LAT_M;
                Some((d, [lng, lat], owner_id, p.id))
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
    }

    /// Clear conflict patches ego has cleared along the connector: **rear** past arc depth, or **centre** past (incremental release).
    /// (`turn_dist_m` tracks vehicle centre; `s_rear = centre − half_length`).
    fn clean_passed_points_for_vehicle(
        &mut self,
        vehicle_id: u32,
        from_edge: usize,
        to_edge: usize,
        center_dist_on_connector_m: f32,
        vehicle_half_length_m: f32,
    ) {
        let s_rear = (center_dist_on_connector_m - vehicle_half_length_m).max(0.0);
        let s_center = center_dist_on_connector_m.max(0.0);
        let in_edge = EdgeIndex::new(from_edge);
        let out_edge = EdgeIndex::new(to_edge);
        for node in self.nodes.values_mut() {
            for (k, path) in node.by_movement.iter_mut() {
                if k.in_edge != in_edge || k.out_edge != out_edge {
                    continue;
                }
                for p in &mut path.points {
                    if p.reserved_by == Some(vehicle_id)
                        && (s_center > p.distance_on_path + CONFLICT_CLEAN_CENTER_PAST_POINT_M
                            || s_rear > p.distance_on_path + CONFLICT_CLEAN_REAR_PAST_POINT_M)
                    {
                        p.reserved_by = None;
                        p.reserved_at_game_s = None;
                        p.reserved_last_progress_m = None;
                        p.reserved_last_motion_s = None;
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
                        p.reserved_at_game_s = None;
                        p.reserved_last_progress_m = None;
                        p.reserved_last_motion_s = None;
                    }
                }
            }
        }
    }

    fn update_reservation_motion_for_vehicle(
        &mut self,
        vehicle_id: u32,
        from_edge: usize,
        to_edge: usize,
        current_dist_on_path: f32,
        now_game_s: f32,
    ) {
        let in_edge = EdgeIndex::new(from_edge);
        let out_edge = EdgeIndex::new(to_edge);
        for node in self.nodes.values_mut() {
            for (k, path) in node.by_movement.iter_mut() {
                if k.in_edge != in_edge || k.out_edge != out_edge {
                    continue;
                }
                for p in &mut path.points {
                    if p.reserved_by != Some(vehicle_id) {
                        continue;
                    }
                    let moved = p
                        .reserved_last_progress_m
                        .map(|prev| (current_dist_on_path - prev).abs() > 0.15)
                        .unwrap_or(true);
                    p.reserved_last_progress_m = Some(current_dist_on_path);
                    if moved {
                        p.reserved_last_motion_s = Some(now_game_s);
                    }
                }
            }
        }
    }

    fn expire_stale_reservations(&mut self, vehicles: &[Vehicle], now_game_s: f32) {
        for node in self.nodes.values_mut() {
            for path in node.by_movement.values_mut() {
                for p in &mut path.points {
                    let Some(owner) = p.reserved_by else {
                        continue;
                    };
                    let Some(owner_vehicle) = vehicles.iter().find(|v| v.id == owner) else {
                        p.reserved_by = None;
                        p.reserved_at_game_s = None;
                        p.reserved_last_progress_m = None;
                        p.reserved_last_motion_s = None;
                        continue;
                    };
                    if owner_vehicle.despawned {
                        p.reserved_by = None;
                        p.reserved_at_game_s = None;
                        p.reserved_last_progress_m = None;
                        p.reserved_last_motion_s = None;
                        continue;
                    }
                    let age_s = p.reserved_at_game_s.map(|t| now_game_s - t).unwrap_or(0.0);
                    if age_s < CONFLICT_TTL_STALLED_S {
                        continue;
                    }
                    let last_motion = p
                        .reserved_last_motion_s
                        .unwrap_or_else(|| p.reserved_at_game_s.unwrap_or(now_game_s));
                    let stalled_too_long = now_game_s - last_motion >= CONFLICT_TTL_STALLED_S;
                    if owner_vehicle.on_turn_connector && stalled_too_long {
                        p.reserved_by = None;
                        p.reserved_at_game_s = None;
                        p.reserved_last_progress_m = None;
                        p.reserved_last_motion_s = None;
                    }
                }
            }
        }
    }
}

/// Find the precomputed connector lane ID for the given movement.
/// Returns `None` when no connector lane was built (e.g. straight-through on a single road).
fn find_connector_lane_id(
    map: &MapData,
    in_edge: EdgeIndex,
    in_lane: u8,
    out_edge: EdgeIndex,
    out_lane: u8,
) -> Option<LaneId> {
    let in_lane_id = match map.lane_by_edge_lane.get(&(in_edge.index(), in_lane)) {
        Some(id) => *id,
        None => {
            log::warn!(
                "Brak lane_by_edge_lane dla in_edge={} in_lane={}",
                in_edge.index(),
                in_lane
            );
            return None;
        }
    };
    let out_lane_id = match map.lane_by_edge_lane.get(&(out_edge.index(), out_lane)) {
        Some(id) => *id,
        None => {
            log::warn!(
                "Brak lane_by_edge_lane dla out_edge={} out_lane={}",
                out_edge.index(),
                out_lane
            );
            return None;
        }
    };
    let in_lane_obj = map.lanes.get(&in_lane_id)?;
    let mut fallback_same_edge: Option<LaneId> = None;
    let out_eid = out_edge.index() as u64;
    for &conn_id in &in_lane_obj.connections {
        if let Some(conn) = map.lanes.get(&conn_id) {
            if conn.edge_id != u64::MAX {
                continue;
            }
            // Match exact outbound lane (do not use `connections.first()` — order is not guaranteed).
            if conn.connections.contains(&out_lane_id) {
                return Some(conn_id);
            }
            if conn.connections.iter().any(|&next_id| {
                map.lanes
                    .get(&next_id)
                    .is_some_and(|next_lane| next_lane.edge_id == out_eid)
            }) {
                // Last matching connector in `in_lane_obj.connections` order (legacy behaviour).
                fallback_same_edge = Some(conn_id);
            }
        }
    }
    if let Some(conn_id) = fallback_same_edge {
        log::warn!(
            "Connector fallback: in_edge={} in_lane={} -> out_edge={} requested lane {} unavailable",
            in_edge.index(),
            in_lane,
            out_edge.index(),
            out_lane
        );
        return Some(conn_id);
    }
    None
}

fn planned_turn_connector(vehicle: &Vehicle, map: &MapData) -> Option<PlannedTurnConnector> {
    if vehicle.route_pos + 1 >= vehicle.route.len() {
        return None;
    }
    let in_edge = vehicle.route[vehicle.route_pos];
    let out_edge = vehicle.route[vehicle.route_pos + 1];

    // Prefer the precomputed connector lane so vehicles follow the same path as the visuals.
    if let Some(cid) = find_connector_lane_id(
        map,
        in_edge,
        vehicle.current_lane,
        out_edge,
        vehicle.target_lane,
    ) {
        if let Some(clane) = map.lanes.get(&cid) {
            let pts = &clane.path.points;
            if pts.len() >= 2 {
                let curr_len = map
                    .graph
                    .edge_weight(in_edge)
                    .map(|e| e.length_m.max(1.0))
                    .unwrap_or(1.0);
                let next_len = map
                    .graph
                    .edge_weight(out_edge)
                    .map(|e| e.length_m.max(1.0))
                    .unwrap_or(1.0);
                let p1 = pts[0];
                let p2 = *pts.last().unwrap();
                let mid = pts[pts.len() / 2];
                let length_m = lane_connector_cubic(clane)
                    .map(|c| c.arclen(CONNECTOR_ARCLEN_ACC) as f32)
                    .unwrap_or(clane.path.length_m);
                let entry_progress = map
                    .lane_by_edge_lane
                    .get(&(in_edge.index(), vehicle.current_lane))
                    .and_then(|lid| map.lanes.get(lid))
                    .and_then(|in_lane| {
                        closest_arclength_m_on_lane_path(in_lane, p1[0], p1[1])
                            .map(|s| (s / in_lane.path.length_m.max(0.001)).clamp(0.0, 1.0))
                    })
                    .unwrap_or_else(|| {
                        (1.0 - TURN_CONNECTOR_ENTRY_M / curr_len).clamp(0.0, 1.0)
                    });
                let exit_progress = map
                    .lane_by_edge_lane
                    .get(&(out_edge.index(), vehicle.target_lane))
                    .and_then(|lid| map.lanes.get(lid))
                    .and_then(|out_lane| {
                        closest_arclength_m_on_lane_path(out_lane, p2[0], p2[1])
                            .map(|s| (s / out_lane.path.length_m.max(0.001)).clamp(0.0, 1.0))
                    })
                    .unwrap_or_else(|| (TURN_CONNECTOR_EXIT_M / next_len).clamp(0.0, 1.0));
                return Some(PlannedTurnConnector {
                    entry_progress,
                    exit_progress,
                    length_m,
                    p1_lat: p1[0],
                    p1_lng: p1[1],
                    ctrl_lat: mid[0],
                    ctrl_lng: mid[1],
                    p2_lat: p2[0],
                    p2_lng: p2[1],
                });
            }
        }
    }

    // Fallback: dynamic computation for routes without a precomputed connector.
    connector_for_movement_lane(
        map,
        in_edge,
        out_edge,
        vehicle.current_lane,
        vehicle.target_lane,
    )
}

fn lane_center_offset_m(lane: u8, lanes_total: u8, oneway: bool, lane_width_m: f64) -> f64 {
    // Must stay in sync with road_network::lane_center_offset_m.
    // No direction_sign: the perpendicular normal already flips for the reverse edge.
    let centered = ((lane as f64) - ((lanes_total as f64 - 1.0) * 0.5)) * lane_width_m;
    if oneway {
        centered
    } else {
        centered + (lanes_total as f64 * lane_width_m * 0.5)
    }
}

fn connector_for_movement_lane(
    map: &MapData,
    in_edge: EdgeIndex,
    out_edge: EdgeIndex,
    in_lane: u8,
    out_lane: u8,
) -> Option<PlannedTurnConnector> {
    let (src, junction) = map.graph.edge_endpoints(in_edge)?;
    let (next_src, next_tgt) = map.graph.edge_endpoints(out_edge)?;
    if junction != next_src {
        return None;
    }
    let in_w = map.graph.edge_weight(in_edge)?;
    let out_w = map.graph.edge_weight(out_edge)?;
    let curr_len = in_w.length_m.max(1.0);
    let next_len = out_w.length_m.max(1.0);
    let src_n = &map.graph[src];
    let jn = &map.graph[junction];
    let tgt_n = &map.graph[next_tgt];
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
    let p1_lat = src_n.lat + (jn.lat - src_n.lat) * entry_progress as f64;
    let p1_lng = src_n.lng + (jn.lng - src_n.lng) * entry_progress as f64;
    let p2_lat = jn.lat + (tgt_n.lat - jn.lat) * exit_progress as f64;
    let p2_lng = jn.lng + (tgt_n.lng - jn.lng) * exit_progress as f64;
    let (in_fx, in_fy) = normalize_xy(
        (jn.lng - src_n.lng) * GEO_LNG_M,
        (jn.lat - src_n.lat) * GEO_LAT_M,
    );
    let (out_fx, out_fy) = normalize_xy(
        (tgt_n.lng - jn.lng) * GEO_LNG_M,
        (tgt_n.lat - jn.lat) * GEO_LAT_M,
    );
    // Build per-lane physical entry/exit anchors once (lane-based geometry).
    let right_in_x = in_fy;
    let right_in_y = -in_fx;
    let right_out_x = out_fy;
    let right_out_y = -out_fx;
    let (mut p1x, mut p1y) = geo_to_m_xy(p1_lat, p1_lng);
    let (mut p2x, mut p2y) = geo_to_m_xy(p2_lat, p2_lng);
    let lane_width_m = map.lane_width_m.max(2.5) as f64;
    let in_shift = lane_center_offset_m(in_lane, in_w.lanes.max(1), in_w.oneway, lane_width_m);
    let out_shift = lane_center_offset_m(out_lane, out_w.lanes.max(1), out_w.oneway, lane_width_m);
    p1x += right_in_x * in_shift;
    p1y += right_in_y * in_shift;
    p2x += right_out_x * out_shift;
    p2y += right_out_y * out_shift;
    let (p1_lat, p1_lng) = m_xy_to_geo(p1x, p1y);
    let (p2_lat, p2_lng) = m_xy_to_geo(p2x, p2y);
    let (ctrl_lat, ctrl_lng) = if let Some((cx, cy)) =
        line_intersection(p1x, p1y, in_fx, in_fy, p2x, p2y, -out_fx, -out_fy)
    {
        m_xy_to_geo(cx, cy)
    } else {
        // Never use the raw junction node as the quadratic control — it pulls the sampled path through
        // the intersection centre (same bug as the old graph-node hop).
        m_xy_to_geo((p1x + p2x) * 0.5, (p1y + p2y) * 0.5)
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

/// Returns which (in_lane, out_lane) pairs are valid for this movement,
/// following Cities: Skylines-style rules based on turn intent:
///   Right  → rightmost in-lane  → rightmost out-lane
///   Straight → each lane maps to the same index (clamped)
///   Left   → leftmost  in-lane  → leftmost  out-lane
///   UTurn  → not allowed at intersections
fn valid_lane_pairs_for_movement(
    map: &MapData,
    in_edge: EdgeIndex,
    out_edge: EdgeIndex,
    node: NodeIndex,
) -> Vec<(u8, u8)> {
    let in_lanes = map
        .graph
        .edge_weight(in_edge)
        .map(|e| e.lanes.max(1))
        .unwrap_or(1);
    let out_lanes = map
        .graph
        .edge_weight(out_edge)
        .map(|e| e.lanes.max(1))
        .unwrap_or(1);
    let intent = movement_turn_intent(map, (in_edge, out_edge), node);
    match intent {
        TurnIntent::Right => vec![(in_lanes - 1, out_lanes - 1)],
        TurnIntent::Straight => (0..in_lanes).map(|i| (i, i.min(out_lanes - 1))).collect(),
        TurnIntent::Left => vec![(0, 0)],
        TurnIntent::UTurn => vec![], // u-turns disabled at intersections
    }
}

/// Conflict geometry must follow the lane-graph Kurbo cubic when available so `distance_on_path`
/// matches [`Vehicle::turn_dist_m`]. Quadratic [`PlannedTurnConnector`] is fallback only.
fn sample_lane_movement_conflict_polyline_meter(
    map: &MapData,
    movement: LaneMovementKey,
    planned: &PlannedTurnConnector,
) -> (Vec<DVec2>, Vec<f32>) {
    if let Some(conn_id) = find_connector_lane_id(
        map,
        movement.in_edge,
        movement.in_lane,
        movement.out_edge,
        movement.out_lane,
    ) {
        if let Some(lane) = map.lanes.get(&conn_id) {
            if let Some(samples) = kurbo_lane_connector_meter_samples(lane) {
                return samples;
            }
        }
    }
    let n_lin = ((((planned.length_m / 1.2).ceil()) as usize).clamp(16, 80)).max(16);
    sample_connector_polyline(planned, n_lin)
}

fn build_conflict_system(map: &MapData) -> ConflictSystem {
    let mut nodes: HashMap<NodeIndex, IntersectionConflictData> = HashMap::new();
    for node in map.graph.node_indices() {
        let incoming: Vec<EdgeIndex> = map
            .graph
            .edges_directed(node, petgraph::Direction::Incoming)
            .map(|e| e.id())
            .collect();
        let outgoing: Vec<EdgeIndex> = map
            .graph
            .edges_directed(node, petgraph::Direction::Outgoing)
            .map(|e| e.id())
            .collect();
        if incoming.len() < 2 || outgoing.is_empty() {
            continue;
        }
        let mut by_movement: HashMap<LaneMovementKey, ConflictPath> = HashMap::new();
        let mut next_cp_id: u64 = 1;
        for &in_edge in &incoming {
            for &out_edge in &outgoing {
                // Only generate connectors for valid lane pairs (turn-rule filtered).
                for (in_lane, out_lane) in
                    valid_lane_pairs_for_movement(map, in_edge, out_edge, node)
                {
                    if let Some(c) =
                        connector_for_movement_lane(map, in_edge, out_edge, in_lane, out_lane)
                    {
                        by_movement.insert(
                            LaneMovementKey {
                                in_edge,
                                out_edge,
                                in_lane,
                                out_lane,
                            },
                            ConflictPath {
                                bezier: c,
                                points: Vec::new(),
                            },
                        );
                    }
                }
            }
        }
        build_conflicts_for_node(map, &mut by_movement, &mut next_cp_id);
        nodes.insert(
            node,
            IntersectionConflictData {
                by_movement,
                deadlock_timer_s: 0.0,
                deadlock_first_move: None,
            },
        );
    }
    ConflictSystem { nodes }
}

fn build_conflicts_for_node(
    map: &MapData,
    by_movement: &mut HashMap<LaneMovementKey, ConflictPath>,
    next_cp_id: &mut u64,
) {
    let keys: Vec<LaneMovementKey> = by_movement.keys().copied().collect();
    for i in 0..keys.len() {
        for j in (i + 1)..keys.len() {
            let k1 = keys[i];
            let k2 = keys[j];
            let b1 = by_movement[&k1].bezier.clone();
            let b2 = by_movement[&k2].bezier.clone();
            let (poly1, dist1) = sample_lane_movement_conflict_polyline_meter(map, k1, &b1);
            let (poly2, dist2) = sample_lane_movement_conflict_polyline_meter(map, k2, &b2);
            for a in 0..(poly1.len().saturating_sub(1)) {
                for b in 0..(poly2.len().saturating_sub(1)) {
                    if let Some((p, da, db)) = polyline_segment_intersection(
                        poly1[a],
                        poly1[a + 1],
                        dist1[a],
                        dist1[a + 1],
                        poly2[b],
                        poly2[b + 1],
                        dist2[b],
                        dist2[b + 1],
                    ) {
                        if let Some(path) = by_movement.get_mut(&k1) {
                            path.points.push(ConflictPoint {
                                id: *next_cp_id,
                                pos: p,
                                distance_on_path: da,
                                radius_m: 2.0,
                                reserved_by: None,
                                reserved_at_game_s: None,
                                reserved_last_progress_m: None,
                                reserved_last_motion_s: None,
                            });
                            *next_cp_id += 1;
                        }
                        if let Some(path) = by_movement.get_mut(&k2) {
                            path.points.push(ConflictPoint {
                                id: *next_cp_id,
                                pos: p,
                                distance_on_path: db,
                                radius_m: 2.0,
                                reserved_by: None,
                                reserved_at_game_s: None,
                                reserved_last_progress_m: None,
                                reserved_last_motion_s: None,
                            });
                            *next_cp_id += 1;
                        }
                    }
                }
            }
        }
    }
    for path in by_movement.values_mut() {
        path.points.sort_by(|a, b| {
            a.distance_on_path
                .partial_cmp(&b.distance_on_path)
                .unwrap_or(Ordering::Equal)
        });
        path.points.dedup_by(|a, b| {
            (a.distance_on_path - b.distance_on_path).abs() < 0.5
                && (a.pos - b.pos).length_squared() < 1.0
        });
    }
}

fn sample_connector_polyline(
    conn: &PlannedTurnConnector,
    samples: usize,
) -> (Vec<DVec2>, Vec<f32>) {
    let n = samples.max(16);
    let mut pts = Vec::with_capacity(n + 1);
    let mut dists = Vec::with_capacity(n + 1);
    let path = bezier_path_from_geo(
        conn.p1_lat,
        conn.p1_lng,
        conn.ctrl_lat,
        conn.ctrl_lng,
        conn.p2_lat,
        conn.p2_lng,
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
    a0: DVec2,
    a1: DVec2,
    da0: f32,
    da1: f32,
    b0: DVec2,
    b1: DVec2,
    db0: f32,
    db1: f32,
) -> Option<(DVec2, f32, f32)> {
    let sa = Segment::new(
        parry2d::na::Point2::new(a0.x as f32, a0.y as f32),
        parry2d::na::Point2::new(a1.x as f32, a1.y as f32),
    );
    let sb = Segment::new(
        parry2d::na::Point2::new(b0.x as f32, b0.y as f32),
        parry2d::na::Point2::new(b1.x as f32, b1.y as f32),
    );
    if !intersection_test(
        &parry2d::na::Isometry2::identity(),
        &sa,
        &parry2d::na::Isometry2::identity(),
        &sb,
    )
    .ok()?
    {
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
/// Per-vehicle layout (48 bytes):
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
///   [40..47] current_lane_id: u64  LE  (`u64::MAX` = unknown)
/// ```
fn serialize_vehicles(vehicles: &[Vehicle], tram_sim: &TramSim) -> Vec<u8> {
    let total = vehicles.len() + tram_sim.trams.len();
    let mut buf = Vec::with_capacity(total * 48);

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
            v.current_lane_id.unwrap_or(u64::MAX),
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
            u64::MAX,
        );
    }

    buf
}

/// Per-vehicle binary packet layout (48 bytes):
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
///   [40..47] current_lane_id: u64 LE (`u64::MAX` when unknown)
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
    current_lane_id: u64,
) {
    buf.extend_from_slice(&id.to_le_bytes()); // [0..3]
    buf.extend_from_slice(&lat.to_le_bytes()); // [4..11]  f64
    buf.extend_from_slice(&lng.to_le_bytes()); // [12..19] f64
    buf.extend_from_slice(&angle.to_le_bytes()); // [20..23]
    buf.extend_from_slice(&speed.to_le_bytes()); // [24..27]
    buf.push(vtype); // [28]
    buf.push(profile); // [29]
    buf.push(trip_kind); // [30]
    let lane_flags = (current_lane & 0x7f) | if on_turn_connector { 0x80 } else { 0 };
    buf.push(lane_flags); // [31]
    buf.extend_from_slice(&frustration.to_le_bytes()); // [32..35]
    buf.extend_from_slice(&lateral_offset.to_le_bytes()); // [36..39]
    buf.extend_from_slice(&current_lane_id.to_le_bytes()); // [40..47]
}
