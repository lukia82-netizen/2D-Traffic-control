use tauri::{command, State};
use tauri::ipc::Channel;
use serde::{Deserialize, Serialize};

use crate::state::{AppState, SimCommand, SimControl, LightControlMode};
use crate::map::osm_loader::fetch_osm_data;
use crate::map::road_network::{
    build_road_network, build_demo_road_network, MapData, IntersectionType, InfraType,
    RestrictionKind,
};
use crate::simulation::sim_loop::run_simulation;
use crate::simulation::congestion::CongestionData;
use crate::simulation::speed_config::SpeedConfig;
use crate::traffic::traffic_light::LightStateUpdate;

// ── Response DTOs ─────────────────────────────────────────────────────────────

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
    pub from: u64,
    pub to: u64,
    pub lanes: u8,
    pub max_speed: f32,
    pub oneway: bool,
    pub infra_type: String,
    pub layer: i8,
    pub length_m: f32,
    pub road_type: String,
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

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapDataResponse {
    pub nodes: Vec<NodeData>,
    pub edges: Vec<EdgeData>,
    pub spawn_points: Vec<[f64; 2]>,
    pub bbox: [f64; 4],
    pub buildings: Vec<BuildingData>,
    pub restrictions: Vec<TurnRestrictionData>,
}

// ── Commands ──────────────────────────────────────────────────────────────────

#[command]
pub async fn load_map(
    bbox: [f64; 4],
    state: State<'_, AppState>,
) -> Result<MapDataResponse, String> {
    log::info!("load_map called with bbox: {:?}", bbox);

    let map_data = match fetch_osm_data(bbox).await {
        Ok(osm_data) => {
            log::info!("OSM data fetched, building road network");
            build_road_network(osm_data)
        }
        Err(e) => {
            log::warn!("Overpass API unavailable ({}), using demo network", e);
            build_demo_road_network()
        }
    };

    let response = build_map_response(&map_data);
    let mut guard = state.road_graph.write();
    *guard = Some(map_data);
    Ok(response)
}

#[command]
pub fn start_simulation(
    on_vehicle_frame: Channel<String>,
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
    let (congestion_tx, _congestion_rx) = std::sync::mpsc::channel::<Vec<CongestionData>>();
    let (light_tx, _light_rx) = std::sync::mpsc::channel::<Vec<LightStateUpdate>>();

    let graph_arc_for_thread = graph_arc.clone();
    let channel = on_vehicle_frame;

    std::thread::Builder::new()
        .name("sim_loop".to_string())
        .spawn(move || {
            run_simulation(
                graph_arc_for_thread,
                cmd_rx,
                channel,
                congestion_tx,
                light_tx,
            );
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
        "manual"    => LightControlMode::Manual,
        "semi_auto" => LightControlMode::SemiAuto,
        "auto"      => LightControlMode::Auto,
        "adaptive"  => LightControlMode::Adaptive,
        _ => return Err(format!("Unknown light mode: {}", mode)),
    };
    send_sim_command(&state, SimCommand::SetLightMode { intersection_id, mode: light_mode })
}

#[command]
pub fn set_traffic_light_phase(
    intersection_id: u64,
    phase: u8,
    state: State<AppState>,
) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetLightPhase { intersection_id, phase })
}

/// Update the speed / compliance / route / rage configuration at runtime.
/// Changes affect newly spawned vehicles; existing vehicles keep their
/// `personal_compliance` and `route_alpha` for the duration of their trip.
#[command]
pub fn set_speed_config(
    config: SpeedConfig,
    state: State<AppState>,
) -> Result<(), String> {
    send_sim_command(&state, SimCommand::SetSpeedConfig(config))
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
    use petgraph::visit::EdgeRef;

    let mut nodes = Vec::new();
    for node_idx in map_data.graph.node_indices() {
        let node = &map_data.graph[node_idx];
        nodes.push(NodeData {
            id: node.osm_id,
            lat: node.lat,
            lng: node.lng,
            intersection_type: match node.intersection_type {
                IntersectionType::Plain        => "plain",
                IntersectionType::TrafficLight => "traffic_light",
                IntersectionType::Stop         => "stop",
                IntersectionType::Yield        => "yield",
                IntersectionType::Roundabout   => "roundabout",
            }.to_string(),
        });
    }

    let mut edges = Vec::new();
    for edge_ref in map_data.graph.edge_references() {
        let edge     = edge_ref.weight();
        let from_node = &map_data.graph[edge_ref.source()];
        let to_node   = &map_data.graph[edge_ref.target()];
        edges.push(EdgeData {
            from: from_node.osm_id,
            to: to_node.osm_id,
            lanes: edge.lanes,
            max_speed: edge.max_speed,
            oneway: edge.oneway,
            infra_type: match edge.infra_type {
                InfraType::Normal => "normal",
                InfraType::Bridge => "bridge",
                InfraType::Tunnel => "tunnel",
            }.to_string(),
            layer: edge.layer,
            length_m: edge.length_m,
            road_type: edge.road_type.clone(),
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
                RestrictionKind::NoLeftTurn     => "no_left_turn",
                RestrictionKind::NoRightTurn    => "no_right_turn",
                RestrictionKind::NoStraightOn   => "no_straight_on",
                RestrictionKind::NoUTurn        => "no_u_turn",
                RestrictionKind::OnlyLeftTurn   => "only_left_turn",
                RestrictionKind::OnlyRightTurn  => "only_right_turn",
                RestrictionKind::OnlyStraightOn => "only_straight_on",
                RestrictionKind::NoEntry        => "no_entry",
            }.to_string(),
        })
        .collect();

    MapDataResponse { nodes, edges, spawn_points, bbox: map_data.bbox, buildings, restrictions }
}
