use std::collections::HashMap;
use petgraph::graph::{DiGraph, NodeIndex};
use serde::{Deserialize, Serialize};

use crate::map::osm_loader::{OsmData, OsmRelation};

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
    /// OSM highway tag value: "primary", "residential", etc.
    pub road_type: String,
}

/// A building polygon represented as an ordered list of [lat, lng] vertices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildingPolygon {
    pub polygon: Vec<[f64; 2]>,
}

/// OSM turn-restriction kinds derived from the `restriction` tag.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RestrictionKind {
    NoLeftTurn,
    NoRightTurn,
    NoStraightOn,
    NoUTurn,
    OnlyLeftTurn,
    OnlyRightTurn,
    OnlyStraightOn,
    NoEntry,
}

/// A resolved turn restriction: vehicles arriving via `from_way_id` and
/// passing through `via_node_id` may not (or may only) proceed onto
/// `to_way_id` according to `kind`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurnRestriction {
    pub from_way_id: u64,
    pub via_node_id: u64,
    pub to_way_id: u64,
    pub kind: RestrictionKind,
}

pub type RoadGraph = DiGraph<RoadNode, RoadEdge>;

pub struct MapData {
    pub graph: RoadGraph,
    pub node_index_map: HashMap<u64, NodeIndex>,
    pub bbox: [f64; 4],
    pub spawn_points: Vec<NodeIndex>,
    pub buildings: Vec<BuildingPolygon>,
    pub restrictions: Vec<TurnRestriction>,
}

/// Build a simple 5×5 grid road network centred on Kraków.
/// Used as a fallback when the Overpass API is not reachable.
pub fn build_demo_road_network() -> MapData {
    const CX: f64 = 19.940;       // centre longitude
    const CY: f64 = 50.060;       // centre latitude
    const STEP_LNG: f64 = 0.004;  // ~300 m
    const STEP_LAT: f64 = 0.003;
    const COLS: usize = 5;
    const ROWS: usize = 5;

    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    let nid = |r: usize, c: usize| -> u64 { (r * COLS + c) as u64 };

    for r in 0..ROWS {
        for c in 0..COLS {
            let lat = CY + (r as f64 - (ROWS / 2) as f64) * STEP_LAT;
            let lng = CX + (c as f64 - (COLS / 2) as f64) * STEP_LNG;
            let idx = graph.add_node(RoadNode {
                osm_id: nid(r, c),
                lat,
                lng,
                intersection_type: IntersectionType::TrafficLight,
            });
            node_index_map.insert(nid(r, c), idx);
        }
    }

    let add_edge = |graph: &mut RoadGraph, a: NodeIndex, b: NodeIndex| {
        let src = &graph[a];
        let tgt = &graph[b];
        let length_m = haversine_distance_m(src.lat, src.lng, tgt.lat, tgt.lng);
        let edge = RoadEdge {
            osm_id: 0,
            lanes: 2,
            max_speed: 50.0 / 3.6,
            oneway: false,
            infra_type: InfraType::Normal,
            layer: 0,
            length_m,
            lane_directions: build_lane_directions(2),
            decision_points: [length_m * 0.25, length_m * 0.5, length_m * 0.75],
            road_type: "residential".to_string(),
        };
        let rev = RoadEdge {
            lane_directions: build_lane_directions_reversed(2),
            ..edge.clone()
        };
        graph.add_edge(a, b, edge);
        graph.add_edge(b, a, rev);
    };

    // Horizontal edges
    for r in 0..ROWS {
        for c in 0..(COLS - 1) {
            let a = node_index_map[&nid(r, c)];
            let b = node_index_map[&nid(r, c + 1)];
            add_edge(&mut graph, a, b);
        }
    }
    // Vertical edges
    for r in 0..(ROWS - 1) {
        for c in 0..COLS {
            let a = node_index_map[&nid(r, c)];
            let b = node_index_map[&nid(r + 1, c)];
            add_edge(&mut graph, a, b);
        }
    }

    let bbox = compute_bbox(&graph);
    let spawn_points = find_spawn_points(&graph, &bbox);

    log::info!(
        "Built DEMO road grid: {} nodes, {} edges, {} spawn points",
        graph.node_count(),
        graph.edge_count(),
        spawn_points.len()
    );

    MapData { graph, node_index_map, bbox, spawn_points, buildings: Vec::new(), restrictions: Vec::new() }
}

pub fn build_road_network(osm_data: OsmData) -> MapData {
    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    // Collect node ids from HIGHWAY ways only (not buildings).
    // Building polygon nodes are handled separately and must not pollute the
    // road graph (they would add ~12 000 extra nodes and slow everything down).
    let mut used_node_ids: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for way in &osm_data.ways {
        if !way.tags.contains_key("highway") { continue; }
        for &node_id in &way.node_refs {
            used_node_ids.insert(node_id);
        }
    }

    // Add road-graph nodes
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

    // Count how many highway ways reference each node (to detect junctions)
    let mut node_way_count: HashMap<u64, u32> = HashMap::new();
    for way in &osm_data.ways {
        if !way.tags.contains_key("highway") { continue; }
        for &nid in &way.node_refs {
            *node_way_count.entry(nid).or_insert(0) += 1;
        }
    }

    // Add graph edges
    for way in &osm_data.ways {
        let tags = &way.tags;
        // Skip building ways — they are handled separately below
        if !tags.contains_key("highway") { continue; }

        let oneway = parse_oneway(tags.get("oneway").map(|s| s.as_str()));
        let highway_type = tags.get("highway").map(|s| s.as_str()).unwrap_or("unclassified");
        let lanes = parse_lanes(tags.get("lanes").map(|s| s.as_str()),
                                highway_type);
        let max_speed = parse_max_speed(tags.get("maxspeed").map(|s| s.as_str()),
                                        Some(highway_type));
        let infra_type = parse_infra_type(tags);
        let layer = parse_layer(tags.get("layer").map(|s| s.as_str()));
        let lane_directions = tags
            .get("turn:lanes")
            .map(|s| parse_turn_lanes(s))
            .unwrap_or_else(|| build_lane_directions(lanes));
        let road_type = highway_type.to_string();

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
                road_type: road_type.clone(),
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

    // ── Parse building polygons ───────────────────────────────────────────────
    let mut buildings: Vec<BuildingPolygon> = Vec::new();
    for way in &osm_data.ways {
        if !way.tags.contains_key("building") { continue; }
        let polygon: Vec<[f64; 2]> = way.node_refs.iter()
            .filter_map(|&nid| osm_data.nodes.get(&nid))
            .map(|n| [n.lat, n.lng])
            .collect();
        if polygon.len() >= 3 {
            buildings.push(BuildingPolygon { polygon });
        }
    }

    // ── Parse turn restrictions ───────────────────────────────────────────────
    let restrictions = build_turn_restrictions(&osm_data.relations);

    log::info!(
        "Built road graph: {} nodes, {} edges, {} spawn points, {} buildings, {} restrictions",
        graph.node_count(),
        graph.edge_count(),
        spawn_points.len(),
        buildings.len(),
        restrictions.len(),
    );

    MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        buildings,
        restrictions,
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

fn parse_lanes(value: Option<&str>, highway: &str) -> u8 {
    if let Some(n) = value.and_then(|s| s.parse::<u8>().ok()) {
        return n.max(1).min(8);
    }
    // Defaults by road type when OSM tag is absent
    match highway {
        "motorway" | "trunk"                       => 3,
        "primary"                                  => 2,
        "secondary" | "tertiary"                   => 2,
        "residential" | "living_street" | "service"=> 1,
        _                                          => 1,
    }
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

/// Parse a `turn:lanes` tag value such as `"left|through|right"` into a
/// per-lane direction list.  Multiple directions per lane (e.g. `left;through`)
/// resolve to the first listed direction.  Falls back to `Straight` for
/// unrecognised values.
fn parse_turn_lanes(tag: &str) -> Vec<LaneDirection> {
    tag.split('|')
        .map(|lane| {
            let first = lane.split(';').next().unwrap_or("through").trim();
            match first {
                "left" | "sharp_left" | "slight_left" => LaneDirection::Left,
                "right" | "sharp_right" | "slight_right" => LaneDirection::Right,
                "reverse" => LaneDirection::UTurn,
                _ => LaneDirection::Straight,
            }
        })
        .collect()
}

fn parse_restriction_kind(s: &str) -> Option<RestrictionKind> {
    match s {
        "no_left_turn"     => Some(RestrictionKind::NoLeftTurn),
        "no_right_turn"    => Some(RestrictionKind::NoRightTurn),
        "no_straight_on"   => Some(RestrictionKind::NoStraightOn),
        "no_u_turn"        => Some(RestrictionKind::NoUTurn),
        "only_left_turn"   => Some(RestrictionKind::OnlyLeftTurn),
        "only_right_turn"  => Some(RestrictionKind::OnlyRightTurn),
        "only_straight_on" => Some(RestrictionKind::OnlyStraightOn),
        "no_entry"         => Some(RestrictionKind::NoEntry),
        _                  => None,
    }
}

fn build_turn_restrictions(relations: &[OsmRelation]) -> Vec<TurnRestriction> {
    relations
        .iter()
        .filter_map(|rel| {
            // Only handle simple node-via restrictions
            let restriction_tag = rel.tags.get("restriction")?;
            let kind = parse_restriction_kind(restriction_tag)?;

            let from_way_id = rel.members.iter()
                .find(|m| m.role == "from" && m.member_type == "way")
                .map(|m| m.ref_id)?;

            let via_node_id = rel.members.iter()
                .find(|m| m.role == "via" && m.member_type == "node")
                .map(|m| m.ref_id)?;

            let to_way_id = rel.members.iter()
                .find(|m| m.role == "to" && m.member_type == "way")
                .map(|m| m.ref_id)?;

            Some(TurnRestriction { from_way_id, via_node_id, to_way_id, kind })
        })
        .collect()
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
