use tauri::{command, State};
use tauri::ipc::Channel;
use serde::{Deserialize, Serialize};

use crate::state::{AppState, SimCommand, SimControl, LightControlMode};
use crate::map::osm_loader::fetch_osm_data;
use crate::map::road_network::{build_road_network, build_demo_road_network, MapData};
use crate::simulation::sim_loop::run_simulation;
use crate::simulation::congestion::CongestionData;
use crate::traffic::traffic_light::LightStateUpdate;

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
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MapDataResponse {
    pub nodes: Vec<NodeData>,
    pub edges: Vec<EdgeData>,
    pub spawn_points: Vec<[f64; 2]>,
    pub bbox: [f64; 4],
}

#[command]
pub async fn load_map(
    bbox: [f64; 4],
    state: State<'_, AppState>,
) -> Result<MapDataResponse, String> {
    log::info!("load_map called with bbox: {:?}", bbox);

    let map_data = match fetch_osm_data(bbox).await {
        Ok(osm_data) => {
            log::info!("OSM data fetched successfully, building road network");
            build_road_network(osm_data)
        }
        Err(e) => {
            log::warn!("Overpass API unavailable ({}), using demo road network", e);
            build_demo_road_network()
        }
    };

    let response = build_map_response(&map_data);

    let mut guard = state.road_graph.write();
    *guard = Some(map_data);

    Ok(response)
}

fn build_map_response(map_data: &MapData) -> MapDataResponse {
    use petgraph::visit::EdgeRef;
    use crate::map::road_network::IntersectionType;

    let mut nodes = Vec::new();
    for node_idx in map_data.graph.node_indices() {
        let node = &map_data.graph[node_idx];
        nodes.push(NodeData {
            id: node.osm_id,
            lat: node.lat,
            lng: node.lng,
            intersection_type: match node.intersection_type {
                IntersectionType::Plain => "plain".to_string(),
                IntersectionType::TrafficLight => "traffic_light".to_string(),
                IntersectionType::Stop => "stop".to_string(),
                IntersectionType::Yield => "yield".to_string(),
            },
        });
    }

    let mut edges = Vec::new();
    for edge_ref in map_data.graph.edge_references() {
        let edge = edge_ref.weight();
        let from_node = &map_data.graph[edge_ref.source()];
        let to_node = &map_data.graph[edge_ref.target()];
        edges.push(EdgeData {
            from: from_node.osm_id,
            to: to_node.osm_id,
            lanes: edge.lanes,
            max_speed: edge.max_speed,
            oneway: edge.oneway,
            infra_type: match edge.infra_type {
                crate::map::road_network::InfraType::Normal => "normal".to_string(),
                crate::map::road_network::InfraType::Bridge => "bridge".to_string(),
                crate::map::road_network::InfraType::Tunnel => "tunnel".to_string(),
            },
            layer: edge.layer,
            length_m: edge.length_m,
        });
    }

    let spawn_points: Vec<[f64; 2]> = map_data
        .spawn_points
        .iter()
        .map(|&idx| {
            let node = &map_data.graph[idx];
            [node.lat, node.lng]
        })
        .collect();

    MapDataResponse {
        nodes,
        edges,
        spawn_points,
        bbox: map_data.bbox,
    }
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
            run_simulation(graph_arc_for_thread, cmd_rx, channel, congestion_tx, light_tx);
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
    send_sim_command(&state, SimCommand::SetLightPhase { intersection_id, phase })
}

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
