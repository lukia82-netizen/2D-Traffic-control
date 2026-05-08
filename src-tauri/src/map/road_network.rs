use std::collections::HashMap;
use petgraph::graph::{DiGraph, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use parry2d::na::Point2;
use parry2d::query::intersection_test;
use parry2d::shape::Segment;

use crate::map::osm_loader::{OsmData, OsmRelation};
use crate::map::building_loader::OdBuilding;
use crate::map::tram_network::TramData;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IntersectionType {
    Plain,
    TrafficLight,
    /// Mid-road pedestrian crossing with traffic signals.
    /// Vehicles treat it like a TrafficLight; the frontend renders a zebra + pedestrian signal.
    PedestrianCrossing,
    Stop,
    Yield,
    /// Roundabout node (junction=roundabout) – always one-way.
    Roundabout,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InfraType {
    Normal,
    Bridge,
    Tunnel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
    /// True when at least one lane of this edge is shared with a tram track.
    /// Vehicles must not change into tram-dedicated lanes on such edges.
    pub has_tram_track: bool,
}

/// A building polygon represented as an ordered list of \[lat, lng\] vertices.
/// Kept for backward-compat with the frontend buildings response.
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

/// A resolved turn restriction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurnRestriction {
    pub from_way_id: u64,
    pub via_node_id: u64,
    pub to_way_id: u64,
    pub kind: RestrictionKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurnConnector {
    pub from_node_id: u64,
    pub via_node_id: u64,
    pub to_node_id: u64,
    /// LUT samples of a quadratic Bezier in [lng, lat].
    pub bezier_lut: Vec<[f64; 2]>,
}

pub type RoadGraph = DiGraph<RoadNode, RoadEdge>;

pub type LaneId = u64;
pub type ConflictAreaId = u64;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BezierPath {
    pub points: Vec<[f64; 2]>,
    pub length_m: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConflictArea {
    pub id: ConflictAreaId,
    pub center_lat: f64,
    pub center_lng: f64,
    pub radius_m: f32,
    pub lane_ids: Vec<LaneId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Lane {
    pub id: LaneId,
    pub path: BezierPath,
    pub width: f32,
    pub connections: Vec<LaneId>,
    pub conflict_areas: Vec<ConflictAreaId>,
    /// Compatibility metadata for edge-oriented systems during migration.
    pub from_node_osm_id: u64,
    pub to_node_osm_id: u64,
    pub edge_id: u64,
    pub lane_index: u8,
}

pub struct MapData {
    pub graph: RoadGraph,
    pub node_index_map: HashMap<u64, NodeIndex>,
    pub bbox: [f64; 4],
    /// All spawn points (boundary + junctions) – used as transit boundary nodes.
    pub spawn_points: Vec<NodeIndex>,
    /// Nodes strictly on the bbox boundary (subset of spawn_points) – for transit spawning.
    pub boundary_nodes: Vec<NodeIndex>,
    /// OD buildings with type, centroid, access_node.
    pub od_buildings: Vec<OdBuilding>,
    pub restrictions: Vec<TurnRestriction>,
    /// Tram network (empty when no tram data in OSM).
    pub tram_data: TramData,
    /// True when this is the built-in sandbox demo map (3×3 grid).
    /// The spawn system uses this to restrict vehicles to a single type.
    pub is_sandbox: bool,
    /// Single-intersection sandbox (+ cross): 2-phase manual TL (N–S / E–W, no lefts) at init.
    pub sandbox_simple_cross_tl: bool,
    pub turn_connectors: Vec<TurnConnector>,
    /// Full lane graph used by lane-based movement and conflict reservation.
    pub lanes: HashMap<LaneId, Lane>,
    pub conflict_areas: HashMap<ConflictAreaId, ConflictArea>,
}

// ── Demo network ─────────────────────────────────────────────────────────────

/// Build the **sandbox** 3×3 grid road network centred on the supplied bbox.
///
/// `grid_type` selects the lane layout:
///
/// | value         | lanes per segment          |
/// |---|---|
/// | `"mixed"`     | cycles 1/2/3 per row/col   |
/// | `"one_lane"`  | 1                          |
/// | `"two_lane"`  | 2                          |
/// | `"three_lane"`| 3                          |
///
/// All roads bidirectional, all intersections = TrafficLight.
pub fn build_demo_road_network(grid_type: &str, bbox: [f64; 4]) -> MapData {
    // Centre the demo grid on the requested bbox so it's always in view.
    // bbox = [west, south, east, north]
    let cx = (bbox[0] + bbox[2]) / 2.0;
    let cy = (bbox[1] + bbox[3]) / 2.0;

    const COLS: usize = 3;
    const ROWS: usize = 3;

    // ~400 m spacing, scaled for latitude
    let step_lat: f64 = 0.0036;
    let cos_lat = (cy * std::f64::consts::PI / 180.0).cos().max(0.01);
    let step_lng: f64 = step_lat / cos_lat;

    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    let nid = |r: usize, c: usize| -> u64 { (r * COLS + c) as u64 };

    // Create 9 intersection nodes
    for r in 0..ROWS {
        for c in 0..COLS {
            let lat = cy + (r as f64 - 1.0) * step_lat;
            let lng = cx + (c as f64 - 1.0) * step_lng;
            let idx = graph.add_node(RoadNode {
                osm_id: nid(r, c),
                lat,
                lng,
                intersection_type: IntersectionType::TrafficLight,
            });
            node_index_map.insert(nid(r, c), idx);
        }
    }

    // Adds one bidirectional road segment with the given lane count.
    let add_road = |graph: &mut RoadGraph,
                    a: NodeIndex,
                    b: NodeIndex,
                    lanes: u8,
                    max_speed_kmh: f32,
                    road_type: &str| {
        let length_m = {
            let src = &graph[a];
            let tgt = &graph[b];
            haversine_distance_m(src.lat, src.lng, tgt.lat, tgt.lng)
        };
        let max_speed = max_speed_kmh / 3.6;
        let edge = RoadEdge {
            osm_id: 0,
            lanes,
            max_speed,
            oneway: false,
            infra_type: InfraType::Normal,
            layer: 0,
            length_m,
            lane_directions: build_lane_directions(lanes),
            decision_points: [length_m * 0.25, length_m * 0.5, length_m * 0.75],
            road_type: road_type.to_string(),
            has_tram_track: false,
        };
        let rev = RoadEdge {
            lane_directions: build_lane_directions_reversed(lanes),
            ..edge.clone()
        };
        graph.add_edge(a, b, edge);
        graph.add_edge(b, a, rev);
    };

    // Helper: resolve lane count for a given axis index.
    let resolve_lanes = |idx: usize| -> u8 {
        match grid_type {
            "one_lane"   => 1,
            "two_lane"   => 2,
            "three_lane" => 3,
            _            => (idx % 3 + 1) as u8, // "mixed": 1, 2, 3
        }
    };
    let spec = |lanes: u8| -> (f32, &'static str) {
        match lanes {
            1 => (50.0_f32, "tertiary"),
            2 => (70.0_f32, "secondary"),
            _ => (70.0_f32, "primary"),
        }
    };

    // Horizontal segments — lanes determined by row (or grid_type override)
    for r in 0..ROWS {
        let lanes = resolve_lanes(r);
        let (speed_kmh, road_type) = spec(lanes);
        for c in 0..(COLS - 1) {
            let a = node_index_map[&nid(r, c)];
            let b = node_index_map[&nid(r, c + 1)];
            add_road(&mut graph, a, b, lanes, speed_kmh, road_type);
        }
    }

    // Vertical segments — lanes determined by col (or grid_type override)
    for c in 0..COLS {
        let lanes = resolve_lanes(c);
        let (speed_kmh, road_type) = spec(lanes);
        for r in 0..(ROWS - 1) {
            let a = node_index_map[&nid(r, c)];
            let b = node_index_map[&nid(r + 1, c)];
            add_road(&mut graph, a, b, lanes, speed_kmh, road_type);
        }
    }

    let bbox = compute_bbox(&graph);

    // All 9 nodes are spawn points (each is a junction)
    let spawn_points: Vec<NodeIndex> = graph.node_indices().collect();

    // Boundary nodes = the 8 outer nodes (all except centre (1,1))
    let centre_id = nid(1, 1);
    let boundary_nodes: Vec<NodeIndex> = graph
        .node_indices()
        .filter(|&idx| graph[idx].osm_id != centre_id)
        .collect();

    log::info!(
        "Built SANDBOX 3×3 grid: {} nodes, {} directed edges, mixed-lane roads",
        graph.node_count(),
        graph.edge_count(),
    );

    let tram_data = TramData {
        graph: crate::map::tram_network::TramGraph::new(),
        node_index_map: HashMap::new(),
        stops: Vec::new(),
        lines: Vec::new(),
    };

    let mut map = MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        boundary_nodes,
        od_buildings: Vec::new(),
        restrictions: Vec::new(),
        tram_data,
        is_sandbox: true,
        sandbox_simple_cross_tl: false,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
    };
    populate_lane_graph(&mut map);
    map
}

/// Build the **single-road** test map: one straight 600 m one-way road.
///
/// Used to verify IDM fundamentals: vehicles should queue up without
/// overlapping, accelerate freely, and despawn at the far end.
///
/// Layout:  [START] ──────────────────── [END]
///                      600 m oneway →
///
/// START is the only spawn point; END is the only boundary (despawn) node.
pub fn build_single_road_network(bbox: [f64; 4]) -> MapData {
    let cx = (bbox[0] + bbox[2]) / 2.0;
    let cy = (bbox[1] + bbox[3]) / 2.0;
    // 600 m east along latitude cy
    let cos_lat = (cy * std::f64::consts::PI / 180.0).cos().max(0.01);
    let half_lng = 0.003 / cos_lat; // ~300 m east and west of centre

    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    let start_node = graph.add_node(RoadNode {
        osm_id: 0,
        lat:    cy,
        lng:    cx - half_lng,
        intersection_type: IntersectionType::Plain,
    });
    // Middle node — pedestrian crossing with traffic signals
    let mid_node = graph.add_node(RoadNode {
        osm_id: 100,
        lat:    cy,
        lng:    cx,
        intersection_type: IntersectionType::PedestrianCrossing,
    });
    let end_node = graph.add_node(RoadNode {
        osm_id: 1,
        lat:    cy,
        lng:    cx + half_lng,
        intersection_type: IntersectionType::Plain,
    });
    node_index_map.insert(0,   start_node);
    node_index_map.insert(100, mid_node);
    node_index_map.insert(1,   end_node);

    let half_m = haversine_distance_m(cy, cx - half_lng, cy, cx);
    let make_edge = |len: f32| RoadEdge {
        osm_id: 0,
        lanes:  1,
        max_speed: 13.89, // 50 km/h
        oneway: true,
        infra_type: InfraType::Normal,
        layer: 0,
        length_m: len,
        lane_directions: vec![LaneDirection::Straight],
        decision_points: [len * 0.25, len * 0.5, len * 0.75],
        road_type: "secondary".to_string(),
        has_tram_track: false,
    };
    graph.add_edge(start_node, mid_node, make_edge(half_m));
    graph.add_edge(mid_node,   end_node, make_edge(half_m));

    let bbox = compute_bbox(&graph);
    let spawn_points   = vec![start_node];
    let boundary_nodes = vec![end_node];

    log::info!("Built SINGLE-ROAD test: 3 nodes, 2 edges, pedestrian crossing at centre ({:.0} m each)", half_m);

    let tram_data = TramData {
        graph: crate::map::tram_network::TramGraph::new(),
        node_index_map: HashMap::new(),
        stops: Vec::new(),
        lines: Vec::new(),
    };

    let mut map = MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        boundary_nodes,
        od_buildings: Vec::new(),
        restrictions: Vec::new(),
        tram_data,
        is_sandbox: true,
        sandbox_simple_cross_tl: false,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
    };
    populate_lane_graph(&mut map);
    map
}

/// Build the default **single-intersection** sandbox map:
/// one unsignalized crossing with one lane per approach.
///
/// Layout:
///            N
///            |
///      W ----+---- E
///            |
///            S
pub fn build_single_intersection_network(bbox: [f64; 4]) -> MapData {
    let cx = (bbox[0] + bbox[2]) / 2.0;
    let cy = (bbox[1] + bbox[3]) / 2.0;

    // ~250 m from center to each approach node.
    let arm_lat = 0.00225;
    let cos_lat = (cy * std::f64::consts::PI / 180.0).cos().max(0.01);
    let arm_lng = arm_lat / cos_lat;

    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    let center = graph.add_node(RoadNode {
        osm_id: 100,
        lat: cy,
        lng: cx,
        intersection_type: IntersectionType::Plain,
    });
    let north = graph.add_node(RoadNode {
        osm_id: 101,
        lat: cy + arm_lat,
        lng: cx,
        intersection_type: IntersectionType::Plain,
    });
    let south = graph.add_node(RoadNode {
        osm_id: 102,
        lat: cy - arm_lat,
        lng: cx,
        intersection_type: IntersectionType::Plain,
    });
    let west = graph.add_node(RoadNode {
        osm_id: 103,
        lat: cy,
        lng: cx - arm_lng,
        intersection_type: IntersectionType::Plain,
    });
    let east = graph.add_node(RoadNode {
        osm_id: 104,
        lat: cy,
        lng: cx + arm_lng,
        intersection_type: IntersectionType::Plain,
    });

    node_index_map.insert(100, center);
    node_index_map.insert(101, north);
    node_index_map.insert(102, south);
    node_index_map.insert(103, west);
    node_index_map.insert(104, east);

    let make_edge = |len: f32| RoadEdge {
        osm_id: 0,
        lanes: 1,
        max_speed: 11.11, // 40 km/h
        oneway: true,
        infra_type: InfraType::Normal,
        layer: 0,
        length_m: len,
        lane_directions: vec![LaneDirection::Straight],
        decision_points: [len * 0.25, len * 0.5, len * 0.75],
        road_type: "secondary".to_string(),
        has_tram_track: false,
    };
    let add_two_way = |graph: &mut RoadGraph, a: NodeIndex, b: NodeIndex| {
        let len = {
            let na = &graph[a];
            let nb = &graph[b];
            haversine_distance_m(na.lat, na.lng, nb.lat, nb.lng)
        };
        graph.add_edge(a, b, make_edge(len));
        graph.add_edge(b, a, make_edge(len));
    };

    add_two_way(&mut graph, north, center);
    add_two_way(&mut graph, south, center);
    add_two_way(&mut graph, west, center);
    add_two_way(&mut graph, east, center);

    let bbox = compute_bbox(&graph);
    let spawn_points = vec![north, south, west, east];
    let boundary_nodes = spawn_points.clone();

    log::info!("Built SINGLE-INTERSECTION sandbox: 5 nodes, 8 directed edges");

    let tram_data = TramData {
        graph: crate::map::tram_network::TramGraph::new(),
        node_index_map: HashMap::new(),
        stops: Vec::new(),
        lines: Vec::new(),
    };

    let mut map = MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        boundary_nodes,
        od_buildings: Vec::new(),
        restrictions: Vec::new(),
        tram_data,
        is_sandbox: true,
        sandbox_simple_cross_tl: false,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
    };
    populate_lane_graph(&mut map);
    map
}

// ── Real OSM network ─────────────────────────────────────────────────────────

pub fn build_road_network(osm_data: OsmData) -> MapData {
    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    // Collect node ids used by highway ways only (buildings handled separately)
    let mut used_node_ids: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for way in &osm_data.ways {
        if !is_backbone_road(way.tags.get("highway").map(|s| s.as_str())) {
            continue;
        }
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

    // Add graph edges
    for way in &osm_data.ways {
        let tags = &way.tags;
        if !is_backbone_road(tags.get("highway").map(|s| s.as_str())) {
            continue;
        }

        let oneway = parse_oneway(tags);
        let highway_type = tags.get("highway").map(String::as_str).unwrap_or("unclassified");
        let lanes = parse_lanes(tags.get("lanes").map(String::as_str), highway_type);
        let max_speed = parse_max_speed(
            tags.get("maxspeed").map(String::as_str),
            Some(highway_type),
        );
        let infra_type = parse_infra_type(tags);
        let layer = parse_layer(tags.get("layer").map(String::as_str));
        let lane_directions = tags
            .get("turn:lanes")
            .map(|s| parse_turn_lanes(s))
            .unwrap_or_else(|| build_lane_directions(lanes));
        let road_type = highway_type.to_string();
        // A way tagged with both highway and railway=tram is a shared tram/road segment.
        let has_tram_track = tags.get("railway").map(String::as_str) == Some("tram");

        for window in way.node_refs.windows(2) {
            let from_id = window[0];
            let to_id   = window[1];

            let (from_idx, to_idx) =
                match (node_index_map.get(&from_id), node_index_map.get(&to_id)) {
                    (Some(&a), Some(&b)) => (a, b),
                    _ => continue,
                };

            let from_node = &graph[from_idx];
            let to_node   = &graph[to_idx];
            let length_m  = haversine_distance_m(
                from_node.lat, from_node.lng,
                to_node.lat,   to_node.lng,
            );
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
                has_tram_track,
            };

            match oneway {
                1  => { graph.add_edge(from_idx, to_idx, edge); }
                -1 => { graph.add_edge(to_idx, from_idx, edge); }
                _  => {
                    graph.add_edge(from_idx, to_idx, edge.clone());
                    let rev = RoadEdge {
                        lane_directions: build_lane_directions_reversed(lanes),
                        ..edge
                    };
                    graph.add_edge(to_idx, from_idx, rev);
                }
            }
        }
    }

    // ── Demote mid-road TrafficLight nodes to Plain ───────────────────────────
    // A real signalised intersection is where ≥ 2 distinct OSM ways meet.
    // Pedestrian crossings and standalone `highway=traffic_signals` nodes that
    // sit in the middle of a single way should not show signal heads.
    {
        use std::collections::HashSet;
        let mut ways_per_node: HashMap<NodeIndex, HashSet<u64>> = HashMap::new();
        for edge_idx in graph.edge_indices() {
            if let (Some(w), Some((from, to))) = (
                graph.edge_weight(edge_idx).map(|e| e.osm_id),
                graph.edge_endpoints(edge_idx),
            ) {
                ways_per_node.entry(from).or_default().insert(w);
                ways_per_node.entry(to).or_default().insert(w);
            }
        }
        for node_idx in graph.node_indices() {
            if matches!(graph[node_idx].intersection_type, IntersectionType::TrafficLight) {
                let way_count = ways_per_node.get(&node_idx).map(|s| s.len()).unwrap_or(0);
                if way_count < 2 {
                    graph[node_idx].intersection_type = IntersectionType::Plain;
                }
            }
        }
    }

    let bbox           = compute_bbox(&graph);
    let spawn_points   = find_spawn_points(&graph, &bbox);
    let boundary_nodes = find_boundary_nodes(&graph, &bbox);

    // ── OD buildings ─────────────────────────────────────────────────────────
    let mut od_buildings =
        crate::map::building_loader::extract_od_buildings(&osm_data);
    crate::map::building_network::link_to_road_nodes(&mut od_buildings, &graph);

    // ── Turn restrictions ────────────────────────────────────────────────────
    let restrictions = build_turn_restrictions(&osm_data.relations);

    // ── Tram network ─────────────────────────────────────────────────────────
    let tram_data = crate::map::tram_network::build_tram_network(&osm_data, &graph);

    log::info!(
        "Built road graph: {} nodes, {} edges, {} spawn, {} boundary, {} buildings, {} tram-nodes",
        graph.node_count(),
        graph.edge_count(),
        spawn_points.len(),
        boundary_nodes.len(),
        od_buildings.len(),
        tram_data.graph.node_count()
    );

    let mut map = MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        boundary_nodes,
        od_buildings,
        restrictions,
        tram_data,
        is_sandbox: false,
        sandbox_simple_cross_tl: false,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
    };
    populate_lane_graph(&mut map);
    map
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Returns `true` only for the three-tier backbone hierarchy that forms the
/// visible skeleton of a city.  Filtering to these road types reduces the
/// graph from ~14 000 edges to ~500, giving PixiJS stable 60 FPS and
/// keeping intersections clearly separated for traffic-light gameplay.
#[inline]
fn is_backbone_road(highway: Option<&str>) -> bool {
    matches!(highway, Some("primary") | Some("secondary") | Some("tertiary"))
}

fn determine_intersection_type(tags: &HashMap<String, String>) -> IntersectionType {
    // Roundabout (junction tag on the way, propagated to nodes in some OSM extracts)
    if tags.get("junction").map(String::as_str) == Some("roundabout") {
        return IntersectionType::Roundabout;
    }
    if let Some(highway) = tags.get("highway") {
        match highway.as_str() {
            "traffic_signals" => return IntersectionType::TrafficLight,
            "stop"            => return IntersectionType::Stop,
            "give_way"        => return IntersectionType::Yield,
            _                 => {}
        }
    }
    if tags.contains_key("traffic_signals") {
        return IntersectionType::TrafficLight;
    }
    // highway=crossing is a pedestrian crossing — not a car traffic signal.
    IntersectionType::Plain
}

/// Parse the `oneway` direction from the **full** tag map of a way.
///
/// Returns:
/// - `1`  – forward (from → to node order)
/// - `-1` – reverse (to → from node order)
/// - `0`  – bidirectional
fn parse_oneway(tags: &HashMap<String, String>) -> i8 {
    // Explicit override: oneway=no forces bidirectional even on motorways
    if tags.get("oneway").map(String::as_str) == Some("no") {
        return 0;
    }
    // Explicit oneway tag
    match tags.get("oneway").map(String::as_str) {
        Some("yes") | Some("true") | Some("1") => return 1,
        Some("-1")  | Some("reverse")           => return -1,
        _ => {}
    }
    // Roundabouts are always one-way (forward)
    if tags.get("junction").map(String::as_str) == Some("roundabout") {
        return 1;
    }
    // Motorways and motorway_links are drawn as separate one-way carriageways
    match tags.get("highway").map(String::as_str) {
        Some("motorway") | Some("motorway_link") => return 1,
        _ => {}
    }
    0
}

fn parse_lanes(value: Option<&str>, highway: &str) -> u8 {
    if let Some(n) = value.and_then(|s| s.parse::<u8>().ok()) {
        return n.max(1).min(8);
    }
    match highway {
        "motorway" | "trunk"                        => 3,
        "primary"                                   => 2,
        "secondary" | "tertiary"                    => 2,
        "residential" | "living_street" | "service" => 1,
        _                                           => 1,
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
    let kmh: f32 = match highway {
        Some("motorway") | Some("motorway_link") => 120.0,
        Some("trunk") | Some("trunk_link")       => 90.0,
        Some("primary") | Some("primary_link")   => 70.0,
        Some("secondary") | Some("secondary_link") => 60.0,
        Some("tertiary") | Some("tertiary_link") => 50.0,
        Some("residential")                      => 30.0,
        Some("living_street")                    => 10.0,
        Some("service")                          => 20.0,
        Some("pedestrian") | Some("footway") | Some("path") => 10.0,
        _                                        => 50.0,
    };
    kmh / 3.6
}

fn parse_infra_type(tags: &HashMap<String, String>) -> InfraType {
    if tags.get("bridge").map(String::as_str) == Some("yes") {
        return InfraType::Bridge;
    }
    if tags.get("tunnel").map(String::as_str) == Some("yes") {
        return InfraType::Tunnel;
    }
    InfraType::Normal
}

fn parse_layer(value: Option<&str>) -> i8 {
    value.and_then(|s| s.parse::<i8>().ok()).unwrap_or(0)
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

fn parse_turn_lanes(tag: &str) -> Vec<LaneDirection> {
    tag.split('|')
        .map(|lane| {
            let first = lane.split(';').next().unwrap_or("through").trim();
            match first {
                "left"  | "sharp_left"  | "slight_left"  => LaneDirection::Left,
                "right" | "sharp_right" | "slight_right" => LaneDirection::Right,
                "reverse"                                 => LaneDirection::UTurn,
                _                                         => LaneDirection::Straight,
            }
        })
        .collect()
}

fn parse_restriction_kind(s: &str) -> Option<RestrictionKind> {
    match s {
        "no_left_turn"      => Some(RestrictionKind::NoLeftTurn),
        "no_right_turn"     => Some(RestrictionKind::NoRightTurn),
        "no_straight_on"    => Some(RestrictionKind::NoStraightOn),
        "no_u_turn"         => Some(RestrictionKind::NoUTurn),
        "only_left_turn"    => Some(RestrictionKind::OnlyLeftTurn),
        "only_right_turn"   => Some(RestrictionKind::OnlyRightTurn),
        "only_straight_on"  => Some(RestrictionKind::OnlyStraightOn),
        "no_entry"          => Some(RestrictionKind::NoEntry),
        _                   => None,
    }
}

fn build_turn_restrictions(relations: &[OsmRelation]) -> Vec<TurnRestriction> {
    relations
        .iter()
        .filter_map(|rel| {
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
    let margin = 0.05;

    let mut spawn_points = Vec::new();
    for idx in graph.node_indices() {
        let n = &graph[idx];
        let near_boundary = n.lat < min_lat + lat_range * margin
            || n.lat > max_lat - lat_range * margin
            || n.lng < min_lng + lng_range * margin
            || n.lng > max_lng - lng_range * margin;

        let degree = graph.edges(idx).count()
            + graph.edges_directed(idx, petgraph::Direction::Incoming).count();
        let is_junction = degree >= 3;

        if near_boundary || is_junction {
            spawn_points.push(idx);
        }
    }
    if spawn_points.len() < 4 {
        spawn_points = graph.node_indices().collect();
    }
    spawn_points
}

/// Nodes within the outermost 3 % of the bounding box – used for transit spawning.
fn find_boundary_nodes(graph: &RoadGraph, bbox: &[f64; 4]) -> Vec<NodeIndex> {
    let [min_lat, min_lng, max_lat, max_lng] = *bbox;
    let lat_range = max_lat - min_lat;
    let lng_range = max_lng - min_lng;
    let margin = 0.03;

    let boundary: Vec<NodeIndex> = graph
        .node_indices()
        .filter(|&idx| {
            let n = &graph[idx];
            n.lat < min_lat + lat_range * margin
                || n.lat > max_lat - lat_range * margin
                || n.lng < min_lng + lng_range * margin
                || n.lng > max_lng - lng_range * margin
        })
        .collect();

    if boundary.is_empty() {
        graph.node_indices().take(4).collect()
    } else {
        boundary
    }
}

pub fn populate_lane_graph(map: &mut MapData) {
    const LANE_WIDTH_M: f32 = 3.2;
    let mut lanes: HashMap<LaneId, Lane> = HashMap::new();
    let mut by_edge_lane: HashMap<(usize, u8), LaneId> = HashMap::new();
    let mut next_lane_id: LaneId = 1;

    for edge_ref in map.graph.edge_references() {
        let edge_id = edge_ref.id();
        let edge = edge_ref.weight();
        let from = &map.graph[edge_ref.source()];
        let to = &map.graph[edge_ref.target()];
        let (sx, sy) = geo_to_m_xy(from.lat, from.lng);
        let (tx, ty) = geo_to_m_xy(to.lat, to.lng);
        let dx = tx - sx;
        let dy = ty - sy;
        let len = (dx * dx + dy * dy).sqrt().max(1.0);
        let ux = dx / len;
        let uy = dy / len;
        let nx = uy;
        let ny = -ux;

        for lane_idx in 0..edge.lanes.max(1) {
            let offset_m = lane_center_offset_m(lane_idx, edge.lanes.max(1), edge.oneway);
            let psx = sx + nx * offset_m;
            let psy = sy + ny * offset_m;
            let ptx = tx + nx * offset_m;
            let pty = ty + ny * offset_m;
            let (plat, plng) = m_xy_to_geo(psx, psy);
            let (qlat, qlng) = m_xy_to_geo(ptx, pty);

            let lane_id = next_lane_id;
            next_lane_id += 1;
            by_edge_lane.insert((edge_id.index(), lane_idx), lane_id);
            lanes.insert(lane_id, Lane {
                id: lane_id,
                path: BezierPath {
                    points: vec![[plat, plng], [qlat, qlng]],
                    length_m: edge.length_m,
                },
                width: LANE_WIDTH_M,
                connections: Vec::new(),
                conflict_areas: Vec::new(),
                from_node_osm_id: from.osm_id,
                to_node_osm_id: to.osm_id,
                edge_id: edge_id.index() as u64,
                lane_index: lane_idx,
            });
        }
    }

    // Build lane connections at each node using lane-index pairing and
    // create physical Bezier connector lanes.
    for node in map.graph.node_indices() {
        let incoming: Vec<_> = map.graph
            .edges_directed(node, petgraph::Direction::Incoming)
            .map(|e| (e.id(), e.weight().lanes.max(1)))
            .collect();
        let outgoing: Vec<_> = map.graph
            .edges_directed(node, petgraph::Direction::Outgoing)
            .map(|e| (e.id(), e.weight().lanes.max(1)))
            .collect();
        if incoming.is_empty() || outgoing.is_empty() {
            continue;
        }
        for (in_edge, in_lanes) in &incoming {
            for in_lane in 0..*in_lanes {
                let Some(&in_lane_id) = by_edge_lane.get(&(in_edge.index(), in_lane)) else { continue; };
                let mut conns = Vec::new();
                for (out_edge, out_lanes) in &outgoing {
                    if in_edge == out_edge {
                        continue;
                    }
                    let out_lane = in_lane.min(out_lanes.saturating_sub(1));
                    if let Some(&out_lane_id) = by_edge_lane.get(&(out_edge.index(), out_lane)) {
                        let Some(connector) = build_lane_connector(&lanes, in_lane_id, out_lane_id, next_lane_id) else {
                            conns.push(out_lane_id);
                            continue;
                        };
                        next_lane_id += 1;
                        let connector_id = connector.id;
                        conns.push(connector_id);
                        lanes.insert(connector_id, connector);
                        if let Some(out) = lanes.get_mut(&connector_id) {
                            out.connections.push(out_lane_id);
                        }
                    }
                }
                if let Some(l) = lanes.get_mut(&in_lane_id) {
                    l.connections = conns;
                }
            }
        }
    }

    // Build physical conflict areas from lane path intersections.
    let lane_items: Vec<(LaneId, Vec<[f64; 2]>)> = lanes
        .iter()
        .map(|(id, lane)| (*id, lane.path.points.clone()))
        .collect();
    let mut areas: HashMap<ConflictAreaId, ConflictArea> = HashMap::new();
    let mut next_conflict_id: ConflictAreaId = 1;
    for i in 0..lane_items.len() {
        for j in (i + 1)..lane_items.len() {
            let (a_id, a_pts) = &lane_items[i];
            let (b_id, b_pts) = &lane_items[j];
            if a_pts.len() < 2 || b_pts.len() < 2 {
                continue;
            }
            for wa in a_pts.windows(2) {
                for wb in b_pts.windows(2) {
                    let (a0x, a0y) = geo_to_m_xy(wa[0][0], wa[0][1]);
                    let (a1x, a1y) = geo_to_m_xy(wa[1][0], wa[1][1]);
                    let (b0x, b0y) = geo_to_m_xy(wb[0][0], wb[0][1]);
                    let (b1x, b1y) = geo_to_m_xy(wb[1][0], wb[1][1]);
                    let sa = Segment::new(Point2::new(a0x as f32, a0y as f32), Point2::new(a1x as f32, a1y as f32));
                    let sb = Segment::new(Point2::new(b0x as f32, b0y as f32), Point2::new(b1x as f32, b1y as f32));
                    let hits = intersection_test(
                        &parry2d::na::Isometry2::identity(),
                        &sa,
                        &parry2d::na::Isometry2::identity(),
                        &sb,
                    ).unwrap_or(false);
                    if !hits {
                        continue;
                    }
                    let cx = (a0x + a1x + b0x + b1x) * 0.25;
                    let cy = (a0y + a1y + b0y + b1y) * 0.25;
                    let (clat, clng) = m_xy_to_geo(cx, cy);
                    let cid = next_conflict_id;
                    next_conflict_id += 1;
                    areas.insert(cid, ConflictArea {
                        id: cid,
                        center_lat: clat,
                        center_lng: clng,
                        radius_m: 2.0,
                        lane_ids: vec![*a_id, *b_id],
                    });
                    if let Some(la) = lanes.get_mut(a_id) {
                        la.conflict_areas.push(cid);
                    }
                    if let Some(lb) = lanes.get_mut(b_id) {
                        lb.conflict_areas.push(cid);
                    }
                }
            }
        }
    }

    map.lanes = lanes;
    map.conflict_areas = areas;
}

#[inline]
fn lane_center_offset_m(lane: u8, lanes_total: u8, oneway: bool) -> f64 {
    const LANE_WIDTH_M: f64 = 3.2;
    if oneway {
        ((lane as f64 + 0.5) - (lanes_total as f64) * 0.5) * LANE_WIDTH_M
    } else {
        (lane as f64 + 0.5) * LANE_WIDTH_M
    }
}

#[inline]
fn geo_to_m_xy(lat: f64, lng: f64) -> (f64, f64) {
    (lng * 71_700.0, lat * 111_320.0)
}

#[inline]
fn m_xy_to_geo(x: f64, y: f64) -> (f64, f64) {
    (y / 111_320.0, x / 71_700.0)
}

fn build_lane_connector(
    lanes: &HashMap<LaneId, Lane>,
    from_lane_id: LaneId,
    to_lane_id: LaneId,
    lane_id: LaneId,
) -> Option<Lane> {
    let from = lanes.get(&from_lane_id)?;
    let to = lanes.get(&to_lane_id)?;
    let p1 = *from.path.points.last()?;
    let p2 = *to.path.points.first()?;
    if p1 == p2 {
        return None;
    }
    let prev = if from.path.points.len() >= 2 {
        from.path.points[from.path.points.len() - 2]
    } else {
        p1
    };
    let next = if to.path.points.len() >= 2 {
        to.path.points[1]
    } else {
        p2
    };
    let (p1x, p1y) = geo_to_m_xy(p1[0], p1[1]);
    let (p2x, p2y) = geo_to_m_xy(p2[0], p2[1]);
    let (prx, pry) = geo_to_m_xy(prev[0], prev[1]);
    let (nx, ny) = geo_to_m_xy(next[0], next[1]);
    let in_dir = normalize_xy(p1x - prx, p1y - pry);
    let out_dir = normalize_xy(nx - p2x, ny - p2y);
    let span = ((p2x - p1x).powi(2) + (p2y - p1y).powi(2)).sqrt().max(1.0);
    let h = (span * 0.5).min(18.0);
    let c1 = (p1x + in_dir.0 * h, p1y + in_dir.1 * h);
    let c2 = (p2x - out_dir.0 * h, p2y - out_dir.1 * h);
    let samples = cubic_bezier_samples((p1x, p1y), c1, c2, (p2x, p2y), 12);
    let mut points = Vec::with_capacity(samples.len());
    let mut length_m = 0.0f32;
    let mut prev_xy: Option<(f64, f64)> = None;
    for (x, y) in samples {
        let (lat, lng) = m_xy_to_geo(x, y);
        points.push([lat, lng]);
        if let Some((px, py)) = prev_xy {
            let seg = ((x - px).powi(2) + (y - py).powi(2)).sqrt() as f32;
            length_m += seg;
        }
        prev_xy = Some((x, y));
    }
    Some(Lane {
        id: lane_id,
        path: BezierPath { points, length_m: length_m.max(0.5) },
        width: from.width.min(to.width),
        connections: Vec::new(),
        conflict_areas: Vec::new(),
        from_node_osm_id: from.from_node_osm_id,
        to_node_osm_id: to.to_node_osm_id,
        edge_id: u64::MAX,
        lane_index: 0,
    })
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

fn cubic_bezier_samples(
    p0: (f64, f64),
    p1: (f64, f64),
    p2: (f64, f64),
    p3: (f64, f64),
    segments: usize,
) -> Vec<(f64, f64)> {
    let n = segments.max(4);
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let u = 1.0 - t;
        let x = u * u * u * p0.0
            + 3.0 * u * u * t * p1.0
            + 3.0 * u * t * t * p2.0
            + t * t * t * p3.0;
        let y = u * u * u * p0.1
            + 3.0 * u * u * t * p1.1
            + 3.0 * u * t * t * p2.1
            + t * t * t * p3.1;
        out.push((x, y));
    }
    out
}
