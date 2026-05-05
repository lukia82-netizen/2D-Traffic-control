use std::collections::HashMap;
use petgraph::graph::{DiGraph, NodeIndex};
use serde::{Deserialize, Serialize};

use crate::map::osm_loader::OsmData;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IntersectionType {
    Plain,
    TrafficLight,
    Stop,
    Yield,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InfraType {
    Normal,
    Bridge,
    Tunnel,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LaneDirection {
    Left,
    Straight,
    Right,
    UTurn,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RoadNode {
    pub osm_id: u64,
    pub lat: f64,
    pub lng: f64,
    pub intersection_type: IntersectionType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RoadEdge {
    pub osm_id: u64,
    pub lanes: u8,
    pub max_speed: f32,
    pub oneway: bool,
    pub infra_type: InfraType,
    pub layer: i8,
    pub length_m: f32,
    pub lane_directions: Vec<LaneDirection>,
    pub decision_points: [f32; 3],
}

pub type RoadGraph = DiGraph<RoadNode, RoadEdge>;

pub struct MapData {
    pub graph: RoadGraph,
    pub node_index_map: HashMap<u64, NodeIndex>,
    pub bbox: [f64; 4],
    pub spawn_points: Vec<NodeIndex>,
}

pub fn build_road_network(osm_data: OsmData) -> MapData {
    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    // Collect all node ids that appear in highway ways
    let mut used_node_ids: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for way in &osm_data.ways {
        for &node_id in &way.node_refs {
            used_node_ids.insert(node_id);
        }
    }

    // Add graph nodes
    for &node_id in &used_node_ids {
        if let Some(osm_node) = osm_data.nodes.get(&node_id) {
            let intersection_type = determine_intersection_type(&osm_node.tags);
            let node_idx = graph.add_node(RoadNode {
                osm_id: node_id,
                lat: osm_node.lat,
                lng: osm_node.lng,
                intersection_type,
            });
            node_index_map.insert(node_id, node_idx);
        }
    }

    // Count how many ways reference each node to find junctions
    let mut node_way_count: HashMap<u64, u32> = HashMap::new();
    for way in &osm_data.ways {
        for &nid in &way.node_refs {
            *node_way_count.entry(nid).or_insert(0) += 1;
        }
    }

    // Add graph edges
    for way in &osm_data.ways {
        let tags = &way.tags;
        let oneway = parse_oneway(tags.get("oneway").map(|s| s.as_str()));
        let lanes = parse_lanes(tags.get("lanes").map(|s| s.as_str()));
        let max_speed = parse_max_speed(tags.get("maxspeed").map(|s| s.as_str()),
                                        tags.get("highway").map(|s| s.as_str()));
        let infra_type = parse_infra_type(tags);
        let layer = parse_layer(tags.get("layer").map(|s| s.as_str()));
        let lane_directions = build_lane_directions(lanes);

        for window in way.node_refs.windows(2) {
            let from_id = window[0];
            let to_id = window[1];

            let (from_idx, to_idx) = match (node_index_map.get(&from_id), node_index_map.get(&to_id)) {
                (Some(&a), Some(&b)) => (a, b),
                _ => continue,
            };

            let from_node = &graph[from_idx];
            let to_node = &graph[to_idx];

            let length_m = haversine_distance_m(from_node.lat, from_node.lng, to_node.lat, to_node.lng);
            let decision_points = [length_m * 0.25, length_m * 0.50, length_m * 0.75];

            let edge = RoadEdge {
                osm_id: way.id,
                lanes,
                max_speed,
                oneway: oneway != 0,
                infra_type: infra_type.clone(),
                layer,
                length_m,
                lane_directions: lane_directions.clone(),
                decision_points,
            };

            match oneway {
                1 => {
                    graph.add_edge(from_idx, to_idx, edge);
                }
                -1 => {
                    graph.add_edge(to_idx, from_idx, edge);
                }
                _ => {
                    graph.add_edge(from_idx, to_idx, edge.clone());
                    let rev_edge = RoadEdge {
                        lane_directions: build_lane_directions_reversed(lanes),
                        ..edge
                    };
                    graph.add_edge(to_idx, from_idx, rev_edge);
                }
            }
        }
    }

    // Determine bbox from all used nodes
    let bbox = compute_bbox(&graph);

    // Identify spawn points: nodes on the boundary or high-degree nodes
    let spawn_points = find_spawn_points(&graph, &bbox);

    log::info!(
        "Built road graph: {} nodes, {} edges, {} spawn points",
        graph.node_count(),
        graph.edge_count(),
        spawn_points.len()
    );

    MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
    }
}

fn determine_intersection_type(tags: &HashMap<String, String>) -> IntersectionType {
    if let Some(highway) = tags.get("highway") {
        if highway == "traffic_signals" {
            return IntersectionType::TrafficLight;
        }
        if highway == "stop" {
            return IntersectionType::Stop;
        }
        if highway == "give_way" {
            return IntersectionType::Yield;
        }
    }
    if tags.contains_key("traffic_signals") {
        return IntersectionType::TrafficLight;
    }
    if tags.get("highway").map(|s| s.as_str()) == Some("crossing") {
        return IntersectionType::TrafficLight;
    }
    IntersectionType::Plain
}

fn parse_oneway(value: Option<&str>) -> i8 {
    match value {
        Some("yes") | Some("true") | Some("1") => 1,
        Some("-1") | Some("reverse") => -1,
        _ => 0,
    }
}

fn parse_lanes(value: Option<&str>) -> u8 {
    value
        .and_then(|s| s.parse::<u8>().ok())
        .unwrap_or(1)
        .max(1)
        .min(8)
}

fn parse_max_speed(maxspeed: Option<&str>, highway: Option<&str>) -> f32 {
    if let Some(s) = maxspeed {
        let s = s.trim();
        let kmh = if let Some(stripped) = s.strip_suffix(" mph") {
            stripped.trim().parse::<f32>().ok().map(|v| v * 1.60934)
        } else if let Some(stripped) = s.strip_suffix("mph") {
            stripped.trim().parse::<f32>().ok().map(|v| v * 1.60934)
        } else {
            s.split_whitespace().next().and_then(|v| v.parse::<f32>().ok())
        };
        if let Some(kmh_val) = kmh {
            return (kmh_val / 3.6).max(1.0);
        }
    }

    // Defaults based on highway type (km/h → m/s)
    let kmh = match highway {
        Some("motorway") | Some("motorway_link") => 120.0,
        Some("trunk") | Some("trunk_link") => 90.0,
        Some("primary") | Some("primary_link") => 70.0,
        Some("secondary") | Some("secondary_link") => 60.0,
        Some("tertiary") | Some("tertiary_link") => 50.0,
        Some("residential") => 30.0,
        Some("living_street") => 10.0,
        Some("service") => 20.0,
        Some("pedestrian") | Some("footway") | Some("path") => 10.0,
        _ => 50.0,
    };
    kmh / 3.6
}

fn parse_infra_type(tags: &HashMap<String, String>) -> InfraType {
    if tags.get("bridge").map(|s| s.as_str()) == Some("yes") {
        return InfraType::Bridge;
    }
    if tags.get("tunnel").map(|s| s.as_str()) == Some("yes") {
        return InfraType::Tunnel;
    }
    InfraType::Normal
}

fn parse_layer(value: Option<&str>) -> i8 {
    value
        .and_then(|s| s.parse::<i8>().ok())
        .unwrap_or(0)
}

fn build_lane_directions(lanes: u8) -> Vec<LaneDirection> {
    match lanes {
        0 | 1 => vec![LaneDirection::Straight],
        2 => vec![LaneDirection::Left, LaneDirection::Straight],
        3 => vec![LaneDirection::Left, LaneDirection::Straight, LaneDirection::Right],
        n => {
            let mut dirs = vec![LaneDirection::Left];
            for _ in 1..(n - 1) {
                dirs.push(LaneDirection::Straight);
            }
            dirs.push(LaneDirection::Right);
            dirs
        }
    }
}

fn build_lane_directions_reversed(lanes: u8) -> Vec<LaneDirection> {
    let mut dirs = build_lane_directions(lanes);
    dirs.reverse();
    dirs
}

pub fn haversine_distance_m(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f32 {
    const R: f64 = 6_371_000.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlng = (lng2 - lng1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlng / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    (R * c) as f32
}

fn compute_bbox(graph: &RoadGraph) -> [f64; 4] {
    let mut min_lat = f64::MAX;
    let mut max_lat = f64::MIN;
    let mut min_lng = f64::MAX;
    let mut max_lng = f64::MIN;

    for idx in graph.node_indices() {
        let n = &graph[idx];
        if n.lat < min_lat { min_lat = n.lat; }
        if n.lat > max_lat { max_lat = n.lat; }
        if n.lng < min_lng { min_lng = n.lng; }
        if n.lng > max_lng { max_lng = n.lng; }
    }

    if min_lat == f64::MAX {
        [0.0, 0.0, 0.0, 0.0]
    } else {
        [min_lat, min_lng, max_lat, max_lng]
    }
}

fn find_spawn_points(graph: &RoadGraph, bbox: &[f64; 4]) -> Vec<NodeIndex> {
    let [min_lat, min_lng, max_lat, max_lng] = *bbox;
    let lat_range = max_lat - min_lat;
    let lng_range = max_lng - min_lng;
    let margin = 0.05; // 5% inward from boundary

    let mut spawn_points = Vec::new();

    for idx in graph.node_indices() {
        let n = &graph[idx];

        let near_boundary = n.lat < min_lat + lat_range * margin
            || n.lat > max_lat - lat_range * margin
            || n.lng < min_lng + lng_range * margin
            || n.lng > max_lng - lng_range * margin;

        let degree = graph.edges(idx).count() + graph.edges_directed(idx, petgraph::Direction::Incoming).count();
        let is_junction = degree >= 3;

        if near_boundary || is_junction {
            spawn_points.push(idx);
        }
    }

    // If too few, fall back to all nodes
    if spawn_points.len() < 4 {
        spawn_points = graph.node_indices().collect();
    }

    spawn_points
}
