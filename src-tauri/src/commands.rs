use serde::{Deserialize, Serialize};
use petgraph::visit::EdgeRef;
use tauri::ipc::Channel;
use tauri::{command, AppHandle, State};

use crate::map::city_map::build_city_map_from_bbox;
use crate::map::road_network::{
    build_demo_road_network, build_single_intersection_network, build_single_road_network,
    InfraType, IntersectionType, LaneDirection, MapData,
    RestrictionKind,
};
use crate::map::world_editor::{EditorTool, GraphChange, MapOverrides};
use crate::simulation::sim_loop::run_simulation;
use crate::simulation::speed_config::SpeedConfig;
use crate::state::{AppState, LightControlMode, SimCommand, SimControl};

// ── Response DTOs ─────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize)]
pub struct BBox {
    pub west: f64,
    pub south: f64,
    pub east: f64,
    pub north: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct NodeData {
    pub id: u64,
    pub lat: f64,
    pub lng: f64,
    pub intersection_type: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct EdgeData {
    pub id: u64,
    pub from: u64,
    pub to: u64,
    pub lanes: u8,
    pub max_speed: f32,
    pub oneway: bool,
    pub infra_type: String,
    pub layer: i8,
    pub length_m: f32,
    pub road_type: String,
    /// Per-lane direction hints: "left" | "straight" | "right" | "uturn"
    pub lane_directions: Vec<String>,
}

/// Building DTO sent once at startup via `buildings_data` or `load_map` response.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct BuildingData {
    pub id: u64,
    /// Polygon vertices as \[lng, lat\] pairs (GeoJSON convention).
    pub polygon: Vec<[f64; 2]>,
    pub building_type: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct TurnRestrictionData {
    pub from_way_id: u64,
    pub via_node_id: u64,
    pub to_way_id: u64,
    pub kind: String,
}

/// Tram stop DTO – sent once at startup as part of the map response.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct TramStopData {
    pub id: u64,
    pub lat: f64,
    pub lng: f64,
    pub dwell_s: f32,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct ConflictAreaData {
    pub id: u64,
    pub center_lat: f64,
    pub center_lng: f64,
    pub radius_m: f32,
    pub lane_ids: Vec<u64>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct LaneData {
    pub id: u64,
    pub width: f32,
    pub connections: Vec<u64>,
    pub conflict_areas: Vec<u64>,
    pub points: Vec<[f64; 2]>,
    pub length_m: f32,
    pub from_node_osm_id: u64,
    pub to_node_osm_id: u64,
    pub lane_index: u8,
    /// True for connector lanes (junction crossing arcs); false for straight road lanes.
    pub is_connector: bool,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapDataResponse {
    pub nodes: Vec<NodeData>,
    pub edges: Vec<EdgeData>,
    pub spawn_points: Vec<[f64; 2]>,
    pub bbox: [f64; 4],
    pub buildings: Vec<BuildingData>,
    pub restrictions: Vec<TurnRestrictionData>,
    pub tram_stops: Vec<TramStopData>,
    pub turn_connectors: Vec<TurnConnectorData>,
    pub lanes: Vec<LaneData>,
    pub conflict_areas: Vec<ConflictAreaData>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct TurnConnectorData {
    pub from_node_id: u64,
    pub via_node_id: u64,
    pub to_node_id: u64,
    pub bezier_lut: Vec<[f64; 2]>,
}

// ── Commands ──────────────────────────────────────────────────────────────────

/// Load a map from Overpass API or build a sandbox grid.
/// `force_sandbox`: when `Some`, skip Overpass.
///   Values: `"mixed"` | `"one_lane"` | `"two_lane"` | `"three_lane"` | `"single_road"` | `"single_intersection"`
/// `lane_width_m`: optional lane width override used for lane offset generation.
#[command]
pub async fn load_map(
    bbox: BBox,
    force_sandbox: Option<String>,
    lane_width_m: Option<f32>,
    state: State<'_, AppState>,
) -> Result<MapDataResponse, String> {
    log::info!(
        "load_map called with bbox: west={}, south={}, east={}, north={}",
        bbox.west,
        bbox.south,
        bbox.east,
        bbox.north
    );

    let (_city_map, mut map_data) = if let Some(mode) = force_sandbox.as_deref() {
        log::info!("load_map forced sandbox mode={}", mode);
        let map = if mode == "single_road" {
            build_single_road_network([bbox.west, bbox.south, bbox.east, bbox.north])
        } else if mode == "single_intersection" {
            build_single_intersection_network([bbox.west, bbox.south, bbox.east, bbox.north])
        } else {
            build_demo_road_network(mode, [bbox.west, bbox.south, bbox.east, bbox.north])
        };
        (
            crate::map::city_map::CityMap {
                lanes: Vec::new(),
                intersections: Vec::new(),
            },
            map,
        )
    } else {
        build_city_map_from_bbox([bbox.west, bbox.south, bbox.east, bbox.north])
            .await
            .unwrap_or_else(|err| {
                log::warn!(
                    "osm2streets pipeline failed, fallback to sandbox map: {}",
                    err
                );
                (
                    crate::map::city_map::CityMap {
                        lanes: Vec::new(),
                        intersections: Vec::new(),
                    },
                    build_single_intersection_network([
                        bbox.west, bbox.south, bbox.east, bbox.north,
                    ]),
                )
            })
    };
    if let Some(width) = lane_width_m {
        map_data.lane_width_m = width.clamp(2.0, 6.0);
    }

    apply_overrides_from_disk(&mut map_data)?;
    map_data.rebuild_all_geometry();
    crate::map::road_network::populate_lane_graph(&mut map_data);

    let response = build_map_response(&map_data);
    let mut guard = state.road_graph.write();
    *guard = Some(map_data);
    Ok(response)
}

#[command]
pub fn start_simulation(
    on_vehicle_frame: Channel<String>,
    app: AppHandle,
    state: State<AppState>,
) -> Result<(), String> {
    log::info!("start_simulation called");

    let graph_arc = state.road_graph.clone();
    {
        let guard = graph_arc.read();
        if guard.is_none() {
            return Err("Map not loaded. Call load_map first.".to_string());
        }
    }

    let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<SimCommand>();

    let graph_arc_for_thread = graph_arc.clone();
    let channel = on_vehicle_frame;
    let app_handle = app;

    std::thread::Builder::new()
        .name("sim_loop".to_string())
        .spawn(move || {
            run_simulation(graph_arc_for_thread, cmd_rx, channel, app_handle);
        })
        .map_err(|e| format!("Failed to spawn simulation thread: {}", e))?;

    let mut sim_guard = state
        .sim_control
        .lock()
        .map_err(|e| format!("Lock poisoned: {}", e))?;
    *sim_guard = Some(SimControl { command_tx: cmd_tx });

    Ok(())
}

#[command]
pub fn pause_simulation(state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::Pause)
}

#[command]
pub fn resume_simulation(state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::Resume)
}

#[command]
pub fn set_time_scale(scale: f32, state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetTimeScale(scale))
}

#[command]
pub fn set_traffic_light_mode(
    intersection_id: u64,
    mode: String,
    state: State<AppState>,
) -> Result<(), String> {
    let light_mode = match mode.as_str() {
        "manual" => LightControlMode::Manual,
        "semi_auto" => LightControlMode::SemiAuto,
        "auto" => LightControlMode::Auto,
        "adaptive" => LightControlMode::Adaptive,
        _ => return Err(format!("Unknown light mode: {}", mode)),
    };
    send_sim_command(
        &state,
        SimCommand::SetLightMode {
            intersection_id,
            mode: light_mode,
        },
    )
}

#[command]
pub fn set_traffic_light_phase(
    intersection_id: u64,
    phase: u8,
    state: State<AppState>,
) -> Result<(), String> {
    send_sim_command(
        &state,
        SimCommand::SetLightPhase {
            intersection_id,
            phase,
        },
    )
}

/// Update the speed / compliance / route / rage configuration at runtime.
/// Changes affect newly spawned vehicles; existing vehicles keep their
/// `personal_compliance` and `route_alpha` for the duration of their trip.
#[command]
pub fn set_speed_config(config: SpeedConfig, state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetSpeedConfig(config))
}

/// Set the maximum number of simultaneously active (non-tram) vehicles.
/// Takes effect immediately; excess vehicles already on the road are not removed.
#[command]
pub fn set_max_vehicles(count: usize, state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetMaxVehicles(count))
}

/// Set the green and red phase durations for a traffic light.
/// Effective in SemiAuto and Auto modes; ignored in Manual and Adaptive.
#[command]
pub fn set_light_durations(
    intersection_id: u64,
    green_s: f32,
    red_s: f32,
    state: State<AppState>,
) -> Result<(), String> {
    send_sim_command(
        &state,
        SimCommand::SetLightDurations {
            intersection_id,
            green_s,
            red_s,
        },
    )
}

/// Set the vehicle tracked by debug overlay (`None` clears selection).
#[command]
pub fn set_debug_vehicle(vehicle_id: Option<u32>, state: State<AppState>) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetDebugVehicle(vehicle_id))
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MoveNodePayload {
    pub node_id: u64,
    pub lat: f64,
    pub lng: f64,
    pub final_commit: bool,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectPayload {
    pub from_node_id: u64,
    pub to_node_id: u64,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExtrudePayload {
    pub from_node_id: u64,
    pub new_node_id: u64,
    pub lat: f64,
    pub lng: f64,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UpdateEdgeTagsPayload {
    pub from_node_id: u64,
    pub to_node_id: u64,
    pub lanes: u8,
    pub oneway: bool,
    pub lane_directions: Vec<String>,
}

#[command]
pub fn set_editor_tool(tool: EditorTool, state: State<AppState>) -> Result<(), String> {
    *state.editor_tool.write() = tool;
    Ok(())
}

#[command]
pub fn editor_move_node(payload: MoveNodePayload, state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut guard = state.road_graph.write();
    let map = guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    let Some(&idx) = map.node_index_map.get(&payload.node_id) else {
        return Err("Unknown node id".to_string());
    };
    let before = (map.graph[idx].lat, map.graph[idx].lng);
    map.update_node_position(payload.node_id, payload.lat, payload.lng)?;
    if payload.final_commit {
        let mut hist = state.editor_history.write();
        hist.undo.push(GraphChange::NodePosition {
            node_id: payload.node_id,
            before_lat: before.0,
            before_lng: before.1,
            after_lat: payload.lat,
            after_lng: payload.lng,
        });
        hist.redo.clear();
    }
    Ok(build_map_response(map))
}

#[command]
pub fn editor_extrude(payload: ExtrudePayload, state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut guard = state.road_graph.write();
    let map = guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    if map.node_index_map.contains_key(&payload.new_node_id) {
        return Err("new_node_id already exists".to_string());
    }
    let new_idx = map.graph.add_node(crate::map::road_network::RoadNode {
        osm_id: payload.new_node_id,
        lat: payload.lat,
        lng: payload.lng,
        intersection_type: crate::map::road_network::IntersectionType::Plain,
    });
    map.node_index_map.insert(payload.new_node_id, new_idx);
    map.add_edge_default(payload.from_node_id, payload.new_node_id)?;
    let mut hist = state.editor_history.write();
    hist.undo.push(GraphChange::EdgeAdded {
        from_node_id: payload.from_node_id,
        to_node_id: payload.new_node_id,
    });
    hist.redo.clear();
    Ok(build_map_response(map))
}

#[command]
pub fn editor_connect(payload: ConnectPayload, state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut guard = state.road_graph.write();
    let map = guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    map.add_edge_default(payload.from_node_id, payload.to_node_id)?;
    let mut hist = state.editor_history.write();
    hist.undo.push(GraphChange::EdgeAdded {
        from_node_id: payload.from_node_id,
        to_node_id: payload.to_node_id,
    });
    hist.redo.clear();
    Ok(build_map_response(map))
}

#[command]
pub fn editor_delete_edge(payload: ConnectPayload, state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut guard = state.road_graph.write();
    let map = guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    let mut removed = false;
    let edge_ids: Vec<_> = map
        .graph
        .edge_references()
        .filter(|e| map.graph[e.source()].osm_id == payload.from_node_id && map.graph[e.target()].osm_id == payload.to_node_id)
        .map(|e| e.id())
        .collect();
    for id in edge_ids {
        map.graph.remove_edge(id);
        removed = true;
    }
    if !removed {
        return Err("Edge not found".to_string());
    }
    map.rebuild_geometry(payload.from_node_id);
    map.rebuild_geometry(payload.to_node_id);
    let mut hist = state.editor_history.write();
    hist.undo.push(GraphChange::EdgeDeleted {
        from_node_id: payload.from_node_id,
        to_node_id: payload.to_node_id,
    });
    hist.redo.clear();
    Ok(build_map_response(map))
}

#[command]
pub fn editor_undo(state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut graph_guard = state.road_graph.write();
    let map = graph_guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    let mut hist = state.editor_history.write();
    let Some(change) = hist.undo.pop() else {
        return Ok(build_map_response(map));
    };
    apply_inverse_change(map, &change)?;
    hist.redo.push(change);
    Ok(build_map_response(map))
}

#[command]
pub fn editor_redo(state: State<AppState>) -> Result<MapDataResponse, String> {
    let mut graph_guard = state.road_graph.write();
    let map = graph_guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    let mut hist = state.editor_history.write();
    let Some(change) = hist.redo.pop() else {
        return Ok(build_map_response(map));
    };
    apply_change(map, &change)?;
    hist.undo.push(change);
    Ok(build_map_response(map))
}

#[command]
pub fn save_map_overrides(state: State<AppState>) -> Result<(), String> {
    let graph_guard = state.road_graph.read();
    let map = graph_guard.as_ref().ok_or_else(|| "Map not loaded".to_string())?;
    let overrides = build_overrides(map);
    let body = serde_json::to_string_pretty(&overrides).map_err(|e| e.to_string())?;
    std::fs::write("map_overrides.json", body).map_err(|e| e.to_string())
}

#[command]
pub fn editor_update_edge_tags(
    payload: UpdateEdgeTagsPayload,
    state: State<AppState>,
) -> Result<MapDataResponse, String> {
    let mut graph_guard = state.road_graph.write();
    let map = graph_guard.as_mut().ok_or_else(|| "Map not loaded".to_string())?;
    let mut updated = false;
    let lanes = payload.lanes.max(1);
    let lane_directions = payload
        .lane_directions
        .iter()
        .map(|d| parse_lane_direction(d))
        .collect::<Vec<_>>();
    let edge_ids: Vec<_> = map
        .graph
        .edge_references()
        .filter(|e| map.graph[e.source()].osm_id == payload.from_node_id && map.graph[e.target()].osm_id == payload.to_node_id)
        .map(|e| e.id())
        .collect();
    for edge_id in edge_ids {
        if let Some(edge) = map.graph.edge_weight_mut(edge_id) {
            edge.lanes = lanes;
            edge.oneway = payload.oneway;
            edge.lane_directions = lane_directions.clone();
            updated = true;
        }
    }
    if !updated {
        return Err("Edge not found".to_string());
    }
    map.rebuild_geometry(payload.from_node_id);
    map.rebuild_geometry(payload.to_node_id);
    Ok(build_map_response(map))
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn send_sim_command(state: &State<AppState>, cmd: SimCommand) -> Result<(), String> {
    let guard = state
        .sim_control
        .lock()
        .map_err(|e| format!("Lock poisoned: {}", e))?;
    match guard.as_ref() {
        Some(ctrl) => ctrl
            .command_tx
            .send(cmd)
            .map_err(|e| format!("Failed to send command: {}", e)),
        None => Err("Simulation not started".to_string()),
    }
}

fn build_map_response(map_data: &MapData) -> MapDataResponse {
    let mut nodes = Vec::new();
    for node_idx in map_data.graph.node_indices() {
        let node = &map_data.graph[node_idx];
        nodes.push(NodeData {
            id: node.osm_id,
            lat: node.lat,
            lng: node.lng,
            intersection_type: match node.intersection_type {
                IntersectionType::Plain => "plain",
                IntersectionType::TrafficLight => "traffic_light",
                IntersectionType::PedestrianCrossing => "pedestrian_crossing",
                IntersectionType::Stop => "stop",
                IntersectionType::Yield => "yield",
                IntersectionType::Roundabout => "roundabout",
            }
            .to_string(),
        });
    }

    let mut edges = Vec::new();
    for edge_ref in map_data.graph.edge_references() {
        let edge = edge_ref.weight();
        let from_node = &map_data.graph[edge_ref.source()];
        let to_node = &map_data.graph[edge_ref.target()];
        let lane_directions: Vec<String> = edge
            .lane_directions
            .iter()
            .map(|d| {
                match d {
                    LaneDirection::Left => "left",
                    LaneDirection::Straight => "straight",
                    LaneDirection::Right => "right",
                    LaneDirection::UTurn => "uturn",
                }
                .to_string()
            })
            .collect();

        edges.push(EdgeData {
            id: edge_ref.id().index() as u64,
            from: from_node.osm_id,
            to: to_node.osm_id,
            lanes: edge.lanes,
            max_speed: edge.max_speed,
            oneway: edge.oneway,
            infra_type: match edge.infra_type {
                InfraType::Normal => "normal",
                InfraType::Bridge => "bridge",
                InfraType::Tunnel => "tunnel",
            }
            .to_string(),
            layer: edge.layer,
            length_m: edge.length_m,
            road_type: edge.road_type.clone(),
            lane_directions,
        });
    }

    let spawn_points: Vec<[f64; 2]> = map_data
        .spawn_points
        .iter()
        .map(|&idx| {
            let n = &map_data.graph[idx];
            [n.lat, n.lng]
        })
        .collect();

    let buildings: Vec<BuildingData> = map_data
        .od_buildings
        .iter()
        .map(|b| BuildingData {
            id: b.id,
            polygon: b.polygon.clone(),
            building_type: b.building_type.as_str().to_string(),
        })
        .collect();

    let restrictions: Vec<TurnRestrictionData> = map_data
        .restrictions
        .iter()
        .map(|r| TurnRestrictionData {
            from_way_id: r.from_way_id,
            via_node_id: r.via_node_id,
            to_way_id: r.to_way_id,
            kind: match r.kind {
                RestrictionKind::NoLeftTurn => "no_left_turn",
                RestrictionKind::NoRightTurn => "no_right_turn",
                RestrictionKind::NoStraightOn => "no_straight_on",
                RestrictionKind::NoUTurn => "no_u_turn",
                RestrictionKind::OnlyLeftTurn => "only_left_turn",
                RestrictionKind::OnlyRightTurn => "only_right_turn",
                RestrictionKind::OnlyStraightOn => "only_straight_on",
                RestrictionKind::NoEntry => "no_entry",
            }
            .to_string(),
        })
        .collect();

    // ── Tram stops ──────────────────────────────────────────────────────────
    let tram_stops: Vec<TramStopData> = map_data
        .tram_data
        .graph
        .node_indices()
        .filter_map(|idx| {
            let node = &map_data.tram_data.graph[idx];
            if node.is_stop {
                Some(TramStopData {
                    id: node.id,
                    lat: node.lat,
                    lng: node.lng,
                    dwell_s: node.stop_dwell_s,
                })
            } else {
                None
            }
        })
        .collect();

    let turn_connectors: Vec<TurnConnectorData> = map_data
        .turn_connectors
        .iter()
        .map(|c| TurnConnectorData {
            from_node_id: c.from_node_id,
            via_node_id: c.via_node_id,
            to_node_id: c.to_node_id,
            bezier_lut: c.bezier_lut.clone(),
        })
        .collect();

    MapDataResponse {
        nodes,
        edges,
        spawn_points,
        bbox: map_data.bbox,
        buildings,
        restrictions,
        tram_stops,
        turn_connectors,
        lanes: map_data
            .lanes
            .values()
            .map(|l| LaneData {
                id: l.id,
                width: l.width,
                connections: l.connections.clone(),
                conflict_areas: l.conflict_areas.clone(),
                points: l.path.points.clone(),
                length_m: l.path.length_m,
                from_node_osm_id: l.from_node_osm_id,
                to_node_osm_id: l.to_node_osm_id,
                lane_index: l.lane_index,
                // edge_id == u64::MAX marks connector (junction crossing) lanes.
                is_connector: l.edge_id == u64::MAX,
            })
            .collect(),
        conflict_areas: map_data
            .conflict_areas
            .values()
            .map(|c| ConflictAreaData {
                id: c.id,
                center_lat: c.center_lat,
                center_lng: c.center_lng,
                radius_m: c.radius_m,
                lane_ids: c.lane_ids.clone(),
            })
            .collect(),
    }
}

fn apply_change(map: &mut MapData, change: &GraphChange) -> Result<(), String> {
    match *change {
        GraphChange::NodePosition { node_id, after_lat, after_lng, .. } => {
            map.update_node_position(node_id, after_lat, after_lng)
        }
        GraphChange::EdgeAdded { from_node_id, to_node_id } => map.add_edge_default(from_node_id, to_node_id),
        GraphChange::EdgeDeleted { from_node_id, to_node_id } => {
            let payload = ConnectPayload { from_node_id, to_node_id };
            let edge_ids: Vec<_> = map
                .graph
                .edge_references()
                .filter(|e| map.graph[e.source()].osm_id == payload.from_node_id && map.graph[e.target()].osm_id == payload.to_node_id)
                .map(|e| e.id())
                .collect();
            for id in edge_ids {
                map.graph.remove_edge(id);
            }
            map.rebuild_geometry(from_node_id);
            map.rebuild_geometry(to_node_id);
            Ok(())
        }
    }
}

fn apply_inverse_change(map: &mut MapData, change: &GraphChange) -> Result<(), String> {
    match *change {
        GraphChange::NodePosition { node_id, before_lat, before_lng, .. } => {
            map.update_node_position(node_id, before_lat, before_lng)
        }
        GraphChange::EdgeAdded { from_node_id, to_node_id } => {
            apply_change(map, &GraphChange::EdgeDeleted { from_node_id, to_node_id })
        }
        GraphChange::EdgeDeleted { from_node_id, to_node_id } => {
            apply_change(map, &GraphChange::EdgeAdded { from_node_id, to_node_id })
        }
    }
}

fn apply_overrides_from_disk(map: &mut MapData) -> Result<(), String> {
    let Ok(raw) = std::fs::read_to_string("map_overrides.json") else {
        return Ok(());
    };
    let parsed: MapOverrides = serde_json::from_str(&raw).map_err(|e| e.to_string())?;
    for (id, [lat, lng]) in parsed.node_positions {
        let _ = map.update_node_position(id, lat, lng);
    }
    for [from, to] in parsed.added_edges {
        let _ = map.add_edge_default(from, to);
    }
    for [from, to] in parsed.deleted_edges {
        let edge_ids: Vec<_> = map
            .graph
            .edge_references()
            .filter(|e| map.graph[e.source()].osm_id == from && map.graph[e.target()].osm_id == to)
            .map(|e| e.id())
            .collect();
        for id in edge_ids {
            map.graph.remove_edge(id);
        }
    }
    for (key, tags) in parsed.edge_tag_overrides {
        let Some((from, to)) = decode_edge_key(&key) else {
            continue;
        };
        let lane_directions = tags
            .lane_directions
            .iter()
            .map(|d| parse_lane_direction(d))
            .collect::<Vec<_>>();
        let edge_ids: Vec<_> = map
            .graph
            .edge_references()
            .filter(|e| map.graph[e.source()].osm_id == from && map.graph[e.target()].osm_id == to)
            .map(|e| e.id())
            .collect();
        for edge_id in edge_ids {
            if let Some(edge) = map.graph.edge_weight_mut(edge_id) {
                edge.lanes = tags.lanes.max(1);
                edge.oneway = tags.oneway;
                edge.lane_directions = lane_directions.clone();
            }
        }
    }
    Ok(())
}

fn build_overrides(map: &MapData) -> MapOverrides {
    let mut node_positions = std::collections::HashMap::new();
    for idx in map.graph.node_indices() {
        let n = &map.graph[idx];
        node_positions.insert(n.osm_id, [n.lat, n.lng]);
    }
    MapOverrides {
        node_positions,
        added_edges: map
            .graph
            .edge_references()
            .filter(|e| e.weight().osm_id == 0)
            .map(|e| [map.graph[e.source()].osm_id, map.graph[e.target()].osm_id])
            .collect(),
        deleted_edges: Vec::new(),
        edge_tag_overrides: map
            .graph
            .edge_references()
            .map(|e| {
                let from = map.graph[e.source()].osm_id;
                let to = map.graph[e.target()].osm_id;
                let lane_directions = e
                    .weight()
                    .lane_directions
                    .iter()
                    .map(|d| match d {
                        LaneDirection::Left => "left",
                        LaneDirection::Straight => "straight",
                        LaneDirection::Right => "right",
                        LaneDirection::UTurn => "uturn",
                    }
                    .to_string())
                    .collect::<Vec<_>>();
                (
                    encode_edge_key(from, to),
                    crate::map::world_editor::EdgeTagOverride {
                        lanes: e.weight().lanes,
                        oneway: e.weight().oneway,
                        lane_directions,
                    },
                )
            })
            .collect(),
    }
}

fn parse_lane_direction(value: &str) -> LaneDirection {
    match value {
        "left" => LaneDirection::Left,
        "right" => LaneDirection::Right,
        "uturn" => LaneDirection::UTurn,
        _ => LaneDirection::Straight,
    }
}

fn encode_edge_key(from: u64, to: u64) -> String {
    format!("{}->{}", from, to)
}

fn decode_edge_key(key: &str) -> Option<(u64, u64)> {
    let (a, b) = key.split_once("->")?;
    Some((a.parse().ok()?, b.parse().ok()?))
}
