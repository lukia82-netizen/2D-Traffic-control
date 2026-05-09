use std::collections::HashMap;
use kurbo::{
    flatten, BezPath, CubicBez, ParamCurve, ParamCurveArclen, PathEl, Point, Shape, Vec2,
};
use petgraph::graph::{DiGraph, NodeIndex};
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};

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
    /// Cubic connector control points in local metres (x = east, y = north). `None` for road lanes.
    #[serde(default)]
    pub connector_cubic_m: Option<[[f64; 2]; 4]>,
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
    /// Nodes in an outer band of the map bbox (map “edge”) — for UI and catalog; not interior junctions.
    pub spawn_points: Vec<NodeIndex>,
    /// Narrower edge band than `spawn_points` — used by `SpawnSystem` for transit / random O-D endpoints.
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
    /// Physical lane width used to offset lanes from the road centerline.
    pub lane_width_m: f32,
    pub turn_connectors: Vec<TurnConnector>,
    /// Full lane graph used by lane-based movement and conflict reservation.
    pub lanes: HashMap<LaneId, Lane>,
    pub conflict_areas: HashMap<ConflictAreaId, ConflictArea>,
    /// Index: (edge_index, lane_index) → LaneId for fast connector lookup.
    pub lane_by_edge_lane: HashMap<(usize, u8), LaneId>,
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
        lane_width_m: 3.5,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
        lane_by_edge_lane: HashMap::new(),
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
    // oneway: false because we add two directed edges (one per direction),
    // so the lane offset logic can place them on opposite sides of the centerline.
    let make_edge = |len: f32, reversed: bool| RoadEdge {
        osm_id: 0,
        lanes:  1,
        max_speed: 13.89, // 50 km/h
        oneway: false,
        infra_type: InfraType::Normal,
        layer: 0,
        length_m: len,
        lane_directions: if reversed {
            vec![LaneDirection::Straight]
        } else {
            vec![LaneDirection::Straight]
        },
        decision_points: [len * 0.25, len * 0.5, len * 0.75],
        road_type: "secondary".to_string(),
        has_tram_track: false,
    };
    graph.add_edge(start_node, mid_node, make_edge(half_m, false));
    graph.add_edge(mid_node, start_node, make_edge(half_m, true));
    graph.add_edge(mid_node,   end_node, make_edge(half_m, false));
    graph.add_edge(end_node,   mid_node, make_edge(half_m, true));

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
        lane_width_m: 3.5,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
        lane_by_edge_lane: HashMap::new(),
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

    // oneway: false because we add two directed edges (one per direction).
    // This allows lane_center_offset_m to separate them onto opposite carriageway sides.
    let make_edge = |len: f32| RoadEdge {
        osm_id: 0,
        lanes: 1,
        max_speed: 11.11, // 40 km/h
        oneway: false,
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
        lane_width_m: 3.5,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
        lane_by_edge_lane: HashMap::new(),
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
        lane_width_m: 3.5,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
        lane_by_edge_lane: HashMap::new(),
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

/// Nodes whose coordinates fall in an outer band of fraction `margin` (0..1) along the bbox edges.
pub(crate) fn node_indices_in_bbox_margin(
    graph: &RoadGraph,
    bbox: &[f64; 4],
    margin: f64,
) -> Vec<NodeIndex> {
    let [min_lat, min_lng, max_lat, max_lng] = *bbox;
    let lat_range = (max_lat - min_lat).max(1e-12);
    let lng_range = (max_lng - min_lng).max(1e-12);
    graph
        .node_indices()
        .filter(|&idx| {
            let n = &graph[idx];
            n.lat < min_lat + lat_range * margin
                || n.lat > max_lat - lat_range * margin
                || n.lng < min_lng + lng_range * margin
                || n.lng > max_lng - lng_range * margin
        })
        .collect()
}

fn road_graph_node_degree(graph: &RoadGraph, idx: NodeIndex) -> usize {
    graph.edges(idx).count()
        + graph
            .edges_directed(idx, petgraph::Direction::Incoming)
            .count()
}

/// Spawn / catalog points: **only** the geographic edge of the map, never interior junctions.
pub(crate) fn find_spawn_points(graph: &RoadGraph, bbox: &[f64; 4]) -> Vec<NodeIndex> {
    for margin in [0.05_f64, 0.08, 0.12, 0.18, 0.26] {
        let pts = node_indices_in_bbox_margin(graph, bbox, margin);
        if pts.len() >= 4 {
            return pts;
        }
    }
    let mut pts = node_indices_in_bbox_margin(graph, bbox, 0.35);
    if pts.len() < 4 {
        let leaves: Vec<NodeIndex> = graph
            .node_indices()
            .filter(|&idx| road_graph_node_degree(graph, idx) == 1)
            .collect();
        if leaves.len() >= 4 {
            return leaves;
        }
        pts = graph.node_indices().collect();
    }
    pts
}

/// Transit / fallback random O-D: edge band (tighter than spawn), then degree-1 stubs, then any nodes.
pub(crate) fn find_boundary_nodes(graph: &RoadGraph, bbox: &[f64; 4]) -> Vec<NodeIndex> {
    for margin in [0.03_f64, 0.06, 0.10, 0.15, 0.22] {
        let v = node_indices_in_bbox_margin(graph, bbox, margin);
        if v.len() >= 2 {
            return v;
        }
    }
    let leaves: Vec<NodeIndex> = graph
        .node_indices()
        .filter(|&idx| road_graph_node_degree(graph, idx) == 1)
        .collect();
    if leaves.len() >= 2 {
        return leaves;
    }
    graph.node_indices().take(4).collect()
}

/// Turn class used for lane-connectivity rules.
#[derive(Clone, Copy, PartialEq, Eq)]
enum TurnClass { Right, Straight, Left, UTurn }

/// Classify the turn made by traversing `in_edge` → `out_edge` through `node`.
fn classify_turn_at_node(
    graph: &RoadGraph,
    in_edge: petgraph::graph::EdgeIndex,
    out_edge: petgraph::graph::EdgeIndex,
    node: petgraph::graph::NodeIndex,
) -> TurnClass {
    let Some((in_src, in_tgt)) = graph.edge_endpoints(in_edge) else { return TurnClass::Straight; };
    let Some((out_src, out_tgt)) = graph.edge_endpoints(out_edge) else { return TurnClass::Straight; };
    if in_tgt != node || out_src != node { return TurnClass::Straight; }
    // U-turn: the destination of out_edge is the source of in_edge.
    if out_tgt == in_src { return TurnClass::UTurn; }
    let n = &graph[node];
    let s = &graph[in_src];
    let t = &graph[out_tgt];
    let in_x  = (n.lng - s.lng) as f32;
    let in_y  = (n.lat - s.lat) as f32;
    let out_x = (t.lng - n.lng) as f32;
    let out_y = (t.lat - n.lat) as f32;
    let in_len  = (in_x*in_x   + in_y*in_y)  .sqrt().max(1e-6);
    let out_len = (out_x*out_x + out_y*out_y).sqrt().max(1e-6);
    let dot = ((in_x/in_len)*(out_x/out_len) + (in_y/in_len)*(out_y/out_len)).clamp(-1.0, 1.0);
    // Very small deviation → straight continuation (no connector needed).
    if dot.acos() < 0.25 { return TurnClass::Straight; }
    // Cross product > 0 → counter-clockwise → left turn (right-hand traffic convention).
    let cross = in_x * out_y - in_y * out_x;
    if cross > 0.0 { TurnClass::Left } else { TurnClass::Right }
}

/// Return the (in_lane, out_lane) pairs that are valid for a given turn class.
#[allow(dead_code)]
fn valid_connector_pairs(in_lanes: u8, out_lanes: u8, turn: TurnClass) -> Vec<(u8, u8)> {
    match turn {
        TurnClass::Right    => vec![(in_lanes - 1, out_lanes - 1)],
        TurnClass::Straight => (0..in_lanes).map(|i| (i, i.min(out_lanes - 1))).collect(),
        TurnClass::Left     => vec![(0, 0)],
        TurnClass::UTurn    => vec![],
    }
}

/// Map \(N\) incoming lanes to \(M\) outgoing lanes: lane `i→i` for `i < min(N,M)`;
/// remaining outgoing lanes attach to the **rightmost** incoming lane `N-1`;
/// extra incoming lanes attach to outgoing `M-1`.
#[inline]
fn proportional_lane_pairs(n: u8, m: u8) -> Vec<(u8, u8)> {
    if n == 0 || m == 0 {
        return Vec::new();
    }
    let mut pairs = Vec::new();
    let k = n.min(m);
    for i in 0..k {
        pairs.push((i, i));
    }
    if m > n {
        let ri = n.saturating_sub(1);
        for j in n..m {
            pairs.push((ri, j));
        }
    } else if n > m {
        let ro = m.saturating_sub(1);
        for i in m..n {
            pairs.push((i, ro));
        }
    }
    pairs
}

#[inline]
fn lane_already_reaches_to(
    connection_ids: &[LaneId],
    lanes: &HashMap<LaneId, Lane>,
    to_lane_id: LaneId,
) -> bool {
    for &cid in connection_ids {
        if cid == to_lane_id {
            return true;
        }
        if lanes
            .get(&cid)
            .is_some_and(|cl| cl.edge_id == u64::MAX && cl.connections.contains(&to_lane_id))
        {
            return true;
        }
    }
    false
}

/// Insert a lane→lane link via cubic connector (preferred) unless one already appears in `conns`.
fn append_lane_turn_connection(
    lanes: &mut HashMap<LaneId, Lane>,
    from_lane_id: LaneId,
    to_lane_id: LaneId,
    next_lane_id: &mut LaneId,
    conns: &mut Vec<LaneId>,
) {
    if lane_already_reaches_to(conns, lanes, to_lane_id) {
        return;
    }
    if let Some(connector) =
        build_lane_connector_with_fallback(lanes, from_lane_id, to_lane_id, *next_lane_id)
    {
        *next_lane_id += 1;
        let cid = connector.id;
        lanes.insert(cid, connector);
        if let Some(c) = lanes.get_mut(&cid) {
            if !c.connections.contains(&to_lane_id) {
                c.connections.push(to_lane_id);
            }
        }
        if !conns.contains(&cid) {
            conns.push(cid);
        }
    }
}

pub fn ensure_lane_connector_between(map: &mut MapData, from_lane_id: LaneId, to_lane_id: LaneId) -> Option<LaneId> {
    if from_lane_id == to_lane_id {
        return None;
    }
    {
        let from = map.lanes.get(&from_lane_id)?;
        if from.connections.contains(&to_lane_id) {
            return None;
        }
        for &c in &from.connections {
            if map
                .lanes
                .get(&c)
                .is_some_and(|cl| cl.edge_id == u64::MAX && cl.connections.contains(&to_lane_id))
            {
                return Some(c);
            }
        }
    }
    let next_lane_id = map.lanes.keys().max().copied().unwrap_or(0).saturating_add(1);
    let connector = build_lane_connector_with_fallback(&map.lanes, from_lane_id, to_lane_id, next_lane_id)?;
    let cid = connector.id;
    map.lanes.insert(cid, connector);
    if let Some(c) = map.lanes.get_mut(&cid) {
        if !c.connections.contains(&to_lane_id) {
            c.connections.push(to_lane_id);
        }
    }
    if let Some(from_lane) = map.lanes.get_mut(&from_lane_id) {
        if !from_lane.connections.contains(&cid) {
            from_lane.connections.push(cid);
        }
    }
    Some(cid)
}

pub fn populate_lane_graph(map: &mut MapData) {
    let lane_width_m = map.lane_width_m.max(2.5);
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
        let center_path = edge_centerline_bezpath_m(sx, sy, tx, ty);
        let center_samples = sample_bezpath_points_m(&center_path, 12);

        for lane_idx in 0..edge.lanes.max(1) {
            let offset_m = lane_center_offset_m(
                lane_idx,
                edge.lanes.max(1),
                edge.oneway,
                lane_width_m as f64,
            );
            log::debug!(
                "populate_lane_graph: edge {}→{} oneway={} lane={}/{} offset={:+.2}m",
                from.osm_id, to.osm_id, edge.oneway,
                lane_idx, edge.lanes.max(1), offset_m
            );
            let lane_samples = offset_polyline_m(&center_samples, offset_m);
            let mut points: Vec<[f64; 2]> = Vec::with_capacity(lane_samples.len());
            let mut length_m = 0.0f32;
            let mut prev_xy: Option<(f64, f64)> = None;
            for (x, y) in lane_samples {
                let (lat, lng) = m_xy_to_geo(x, y);
                points.push([lat, lng]);
                if let Some((px, py)) = prev_xy {
                    length_m += ((x - px).powi(2) + (y - py).powi(2)).sqrt() as f32;
                }
                prev_xy = Some((x, y));
            }

            let lane_id = next_lane_id;
            next_lane_id += 1;
            by_edge_lane.insert((edge_id.index(), lane_idx), lane_id);
            lanes.insert(lane_id, Lane {
                id: lane_id,
                path: BezierPath {
                    points,
                    length_m: length_m.max(0.5),
                },
                connector_cubic_m: None,
                width: lane_width_m,
                connections: Vec::new(),
                conflict_areas: Vec::new(),
                from_node_osm_id: from.osm_id,
                to_node_osm_id: to.osm_id,
                edge_id: edge_id.index() as u64,
                lane_index: lane_idx,
            });
        }
    }

    // Build lane connections at each node using Cities:Skylines-style rules:
    //   Right turn  → rightmost in-lane  → rightmost out-lane
    //   Straight    → lane i             → lane i (clamped)
    //   Left turn   → leftmost  in-lane  → leftmost  out-lane
    //   U-turn      → skipped
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
        // For each incoming lane build outgoing connections at this node using **proportional**
        // lane mapping \(N→M\) for every incoming/outgoing directed edge pair.
        // U-turns intentionally get no connectors (matches prior behaviour).
        let mut in_lane_conns: std::collections::HashMap<LaneId, Vec<LaneId>> = std::collections::HashMap::new();
        for (in_edge, in_lanes) in &incoming {
            for in_lane in 0..*in_lanes {
                let Some(&in_lane_id) = by_edge_lane.get(&(in_edge.index(), in_lane)) else { continue; };
                let mut conns: Vec<LaneId> = Vec::new();
                for (out_edge, out_lanes) in &outgoing {
                    let turn = classify_turn_at_node(&map.graph, *in_edge, *out_edge, node);
                    if matches!(turn, TurnClass::UTurn) {
                        continue;
                    }
                    let pairs = proportional_lane_pairs(*in_lanes, *out_lanes);
                    for (vin, vout) in pairs {
                        if vin != in_lane {
                            continue;
                        }
                        let Some(&out_lane_id) = by_edge_lane.get(&(out_edge.index(), vout)) else {
                            continue;
                        };
                        append_lane_turn_connection(
                            &mut lanes,
                            in_lane_id,
                            out_lane_id,
                            &mut next_lane_id,
                            &mut conns,
                        );
                    }
                }
                in_lane_conns.insert(in_lane_id, conns);
            }
        }
        for (lid, conns) in in_lane_conns {
            if let Some(l) = lanes.get_mut(&lid) {
                l.connections = conns;
            }
        }
    }

    // Conflict areas: only intersections between two connector cubics (kurbo-flattened polylines).
    for lane in lanes.values_mut() {
        lane.conflict_areas.clear();
    }
    let mut areas: HashMap<ConflictAreaId, ConflictArea> = HashMap::new();
    let mut next_conflict_id: ConflictAreaId = 1;
    const FLATTEN_TOL: f64 = 0.22;
    let connector_ids: Vec<LaneId> = lanes
        .iter()
        .filter(|(_, l)| l.edge_id == u64::MAX && l.connector_cubic_m.is_some())
        .map(|(id, _)| *id)
        .collect();
    let mut polylines: HashMap<LaneId, Vec<(Point, Point)>> = HashMap::new();
    for id in &connector_ids {
        if let Some(l) = lanes.get(id) {
            if let Some(c) = lane_connector_cubic(l) {
                polylines.insert(*id, cubic_to_flat_segments(&c, FLATTEN_TOL));
            }
        }
    }
    for i in 0..connector_ids.len() {
        for j in (i + 1)..connector_ids.len() {
            let a_id = connector_ids[i];
            let b_id = connector_ids[j];
            let Some(sega) = polylines.get(&a_id) else { continue; };
            let Some(segb) = polylines.get(&b_id) else { continue; };
            let mut hits: Vec<(f64, f64)> = Vec::new();
            for &(p0, p1) in sega {
                for &(q0, q1) in segb {
                    if let Some((x, y)) = segment_intersection_xy(
                        (p0.x, p0.y, p1.x, p1.y),
                        (q0.x, q0.y, q1.x, q1.y),
                    ) {
                        hits.push((x, y));
                    }
                }
            }
            merge_close_points(&mut hits, 2.5);
            for (x, y) in hits {
                let (clat, clng) = m_xy_to_geo(x, y);
                let cid = next_conflict_id;
                next_conflict_id += 1;
                areas.insert(cid, ConflictArea {
                    id: cid,
                    center_lat: clat,
                    center_lng: clng,
                    radius_m: 2.0,
                    lane_ids: vec![a_id, b_id],
                });
                if let Some(la) = lanes.get_mut(&a_id) {
                    la.conflict_areas.push(cid);
                }
                if let Some(lb) = lanes.get_mut(&b_id) {
                    lb.conflict_areas.push(cid);
                }
            }
        }
    }

    map.lanes = lanes;
    map.conflict_areas = areas;
    map.lane_by_edge_lane = by_edge_lane;
}

#[inline]
fn lane_center_offset_m(
    lane: u8,
    lanes_total: u8,
    oneway: bool,
    lane_width_m: f64,
) -> f64 {
    // Symmetric lane placement around a ONE carriageway.
    // For a one-way road the carriageway IS the centerline, so lanes fan out symmetrically.
    // For a two-way road each directed edge already carries traffic in one direction only;
    // we offset its lane(s) to the RIGHT of travel (positive = right of the tangent normal).
    // The normal direction automatically flips for the opposite edge, so using the same
    // positive offset puts each carriageway on the geometrically correct side without any
    // direction_sign — adding direction_sign would cancel the normal flip and cause overlap.
    let centered = ((lane as f64) - ((lanes_total as f64 - 1.0) * 0.5)) * lane_width_m;
    if oneway {
        centered
    } else {
        // Shift this direction's lanes to the right side of the centerline.
        centered + (lanes_total as f64 * lane_width_m * 0.5)
    }
}

fn edge_centerline_bezpath_m(sx: f64, sy: f64, tx: f64, ty: f64) -> BezPath {
    let mut p = BezPath::new();
    p.move_to(Point::new(sx, sy));
    p.line_to(Point::new(tx, ty));
    p
}

fn sample_bezpath_points_m(path: &BezPath, segments_per_curve: usize) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    let n = segments_per_curve.max(4);
    let mut current: Option<Point> = None;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                current = Some(p);
                out.push((p.x, p.y));
            }
            PathEl::LineTo(p1) => {
                if let Some(p0) = current {
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        out.push((p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t));
                    }
                }
                current = Some(p1);
            }
            PathEl::QuadTo(p1, p2) => {
                if let Some(p0) = current {
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let u = 1.0 - t;
                        let x = u * u * p0.x + 2.0 * u * t * p1.x + t * t * p2.x;
                        let y = u * u * p0.y + 2.0 * u * t * p1.y + t * t * p2.y;
                        out.push((x, y));
                    }
                }
                current = Some(p2);
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if let Some(p0) = current {
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let u = 1.0 - t;
                        let x = u * u * u * p0.x
                            + 3.0 * u * u * t * p1.x
                            + 3.0 * u * t * t * p2.x
                            + t * t * t * p3.x;
                        let y = u * u * u * p0.y
                            + 3.0 * u * u * t * p1.y
                            + 3.0 * u * t * t * p2.y
                            + t * t * t * p3.y;
                        out.push((x, y));
                    }
                }
                current = Some(p3);
            }
            PathEl::ClosePath => {}
        }
    }
    out
}

fn offset_polyline_m(points: &[(f64, f64)], offset_m: f64) -> Vec<(f64, f64)> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let (px, py) = points[i];
        let (tx, ty) = if i == 0 {
            (points[1].0 - px, points[1].1 - py)
        } else if i + 1 == points.len() {
            (px - points[i - 1].0, py - points[i - 1].1)
        } else {
            (points[i + 1].0 - points[i - 1].0, points[i + 1].1 - points[i - 1].1)
        };
        let tangent = Vec2::new(tx, ty);
        let len = (tangent.x * tangent.x + tangent.y * tangent.y).sqrt();
        if len <= 1e-9 {
            out.push((px, py));
            continue;
        }
        let nx = tangent.y / len;
        let ny = -tangent.x / len;
        out.push((px + nx * offset_m, py + ny * offset_m));
    }
    out
}

#[inline]
fn geo_to_m_xy(lat: f64, lng: f64) -> (f64, f64) {
    (lng * 71_700.0, lat * 111_320.0)
}

#[inline]
fn m_xy_to_geo(x: f64, y: f64) -> (f64, f64) {
    (y / 111_320.0, x / 71_700.0)
}

/// Handle length (m) along incoming / outgoing tangents for connector cubics (matches sim / spec).
pub const CONNECTOR_CUBIC_HANDLE_M: f64 = 5.0;
/// Arc-length quadrature accuracy for connector cubics (sim + length_m).
pub const CONNECTOR_ARCLEN_ACC: f64 = 0.08;
/// Pull connector endpoints away from the junction node along each arm (metres).
pub const CONNECTOR_ENDPOINT_INSET_M: f64 = 7.0;

#[inline]
pub fn lane_connector_cubic(lane: &Lane) -> Option<CubicBez> {
    let c = lane.connector_cubic_m.as_ref()?;
    Some(CubicBez::new(
        Point::new(c[0][0], c[0][1]),
        Point::new(c[1][0], c[1][1]),
        Point::new(c[2][0], c[2][1]),
        Point::new(c[3][0], c[3][1]),
    ))
}

fn cubic_to_flat_segments(c: &CubicBez, tol: f64) -> Vec<(Point, Point)> {
    let mut bez = BezPath::new();
    bez.move_to(c.p0);
    bez.curve_to(c.p1, c.p2, c.p3);
    let mut out = Vec::new();
    let mut cur: Option<Point> = None;
    flatten(bez.path_elements(tol), tol, |el| match el {
        PathEl::MoveTo(p) => {
            cur = Some(p);
        }
        PathEl::LineTo(p) => {
            if let Some(c0) = cur {
                out.push((c0, p));
                cur = Some(p);
            }
        }
        PathEl::ClosePath | PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => {}
    });
    out
}

/// Line–line intersection in metre space; `t`/`u` must lie in \[0,1\].
fn segment_intersection_xy(
    a: (f64, f64, f64, f64),
    b: (f64, f64, f64, f64),
) -> Option<(f64, f64)> {
    let (x1, y1, x2, y2) = a;
    let (x3, y3, x4, y4) = b;
    let rx = x2 - x1;
    let ry = y2 - y1;
    let sx = x4 - x3;
    let sy = y4 - y3;
    let den = rx * sy - ry * sx;
    if den.abs() < 1e-12 {
        return None;
    }
    let t = ((x3 - x1) * sy - (y3 - y1) * sx) / den;
    let u = ((x3 - x1) * ry - (y3 - y1) * rx) / den;
    if t < -1e-5 || t > 1.0 + 1e-5 || u < -1e-5 || u > 1.0 + 1e-5 {
        return None;
    }
    Some((x1 + t * rx, y1 + t * ry))
}

fn merge_close_points(pts: &mut Vec<(f64, f64)>, min_dist: f64) {
    if pts.is_empty() {
        return;
    }
    let mut out: Vec<(f64, f64)> = Vec::new();
    for &(x, y) in pts.iter() {
        let mut merged = false;
        for m in out.iter_mut() {
            let dx = m.0 - x;
            let dy = m.1 - y;
            if (dx * dx + dy * dy).sqrt() < min_dist {
                m.0 = (m.0 + x) * 0.5;
                m.1 = (m.1 + y) * 0.5;
                merged = true;
                break;
            }
        }
        if !merged {
            out.push((x, y));
        }
    }
    *pts = out;
}

fn build_lane_connector(
    lanes: &HashMap<LaneId, Lane>,
    from_lane_id: LaneId,
    to_lane_id: LaneId,
    lane_id: LaneId,
) -> Option<Lane> {
    let from = lanes.get(&from_lane_id)?;
    let to = lanes.get(&to_lane_id)?;
    let p0_geo = *from.path.points.last()?;
    let p3_geo = *to.path.points.first()?;
    if p0_geo == p3_geo {
        return None;
    }
    let prev = if from.path.points.len() >= 2 {
        from.path.points[from.path.points.len() - 2]
    } else {
        p0_geo
    };
    let next = if to.path.points.len() >= 2 {
        to.path.points[1]
    } else {
        p3_geo
    };
    let (mut p0x, mut p0y) = geo_to_m_xy(p0_geo[0], p0_geo[1]);
    let (mut p3x, mut p3y) = geo_to_m_xy(p3_geo[0], p3_geo[1]);
    let (prx, pry) = geo_to_m_xy(prev[0], prev[1]);
    let (nx, ny) = geo_to_m_xy(next[0], next[1]);
    let in_dir = normalize_xy(p0x - prx, p0y - pry);
    let out_dir = normalize_xy(nx - p3x, ny - p3y);
    // Move endpoints off the junction: back along the incoming arm, forward on the outgoing arm.
    let back_len = ((p0x - prx).powi(2) + (p0y - pry).powi(2)).sqrt();
    let fwd_len = ((nx - p3x).powi(2) + (ny - p3y).powi(2)).sqrt();
    let inset_back = CONNECTOR_ENDPOINT_INSET_M.min(back_len * 0.88).max(0.0);
    let inset_fwd = CONNECTOR_ENDPOINT_INSET_M.min(fwd_len * 0.88).max(0.0);
    p0x -= in_dir.0 * inset_back;
    p0y -= in_dir.1 * inset_back;
    p3x += out_dir.0 * inset_fwd;
    p3y += out_dir.1 * inset_fwd;
    let h = CONNECTOR_CUBIC_HANDLE_M;
    let p0 = Point::new(p0x, p0y);
    let p1 = Point::new(p0x + in_dir.0 * h, p0y + in_dir.1 * h);
    let p2 = Point::new(p3x - out_dir.0 * h, p3y - out_dir.1 * h);
    let p3 = Point::new(p3x, p3y);
    let cubic = CubicBez::new(p0, p1, p2, p3);
    let length_m = cubic.arclen(CONNECTOR_ARCLEN_ACC) as f32;

    const SAMPLES: usize = 40;
    let mut points = Vec::with_capacity(SAMPLES + 1);
    for i in 0..=SAMPLES {
        let t = i as f64 / SAMPLES as f64;
        let p = cubic.eval(t);
        let (lat, lng) = m_xy_to_geo(p.x, p.y);
        points.push([lat, lng]);
    }
    let connector_cubic_m = Some([
        [p0.x, p0.y],
        [p1.x, p1.y],
        [p2.x, p2.y],
        [p3.x, p3.y],
    ]);
    Some(Lane {
        id: lane_id,
        path: BezierPath {
            points,
            length_m: length_m.max(0.5),
        },
        connector_cubic_m,
        width: from.width.min(to.width),
        connections: Vec::new(),
        conflict_areas: Vec::new(),
        from_node_osm_id: from.from_node_osm_id,
        to_node_osm_id: to.to_node_osm_id,
        edge_id: u64::MAX,
        lane_index: 0,
    })
}

/// Always produces a connector lane with cubic + sampled `BezierPath` when endpoints exist.
/// Used when the strict geometry check in [`build_lane_connector`] fails (merged junction points).
fn build_lane_connector_with_fallback(
    lanes: &HashMap<LaneId, Lane>,
    from_lane_id: LaneId,
    to_lane_id: LaneId,
    lane_id: LaneId,
) -> Option<Lane> {
    if let Some(c) = build_lane_connector(lanes, from_lane_id, to_lane_id, lane_id) {
        return Some(c);
    }
    let from = lanes.get(&from_lane_id)?;
    let to = lanes.get(&to_lane_id)?;
    let p0_geo = *from.path.points.last()?;
    let p3_geo = *to.path.points.first()?;
    let prev = if from.path.points.len() >= 2 {
        from.path.points[from.path.points.len() - 2]
    } else {
        p0_geo
    };
    let next = if to.path.points.len() >= 2 {
        to.path.points[1]
    } else {
        p3_geo
    };

    let (mut p0x, mut p0y) = geo_to_m_xy(p0_geo[0], p0_geo[1]);
    let (mut p3x, mut p3y) = geo_to_m_xy(p3_geo[0], p3_geo[1]);
    let (prx, pry) = geo_to_m_xy(prev[0], prev[1]);
    let (nx, ny) = geo_to_m_xy(next[0], next[1]);
    let mut in_dir = normalize_xy(p0x - prx, p0y - pry);
    let mut out_dir = normalize_xy(nx - p3x, ny - p3y);

    let back_len = ((p0x - prx).powi(2) + (p0y - pry).powi(2)).sqrt();
    let fwd_len = ((nx - p3x).powi(2) + (ny - p3y).powi(2)).sqrt();
    if in_dir == (0.0, 0.0) {
        in_dir = normalize_xy(p3x - p0x, p3y - p0y);
        if in_dir == (0.0, 0.0) {
            in_dir = (1.0, 0.0);
        }
    }
    if out_dir == (0.0, 0.0) {
        out_dir = in_dir;
    }

    let inset_back = CONNECTOR_ENDPOINT_INSET_M.min(back_len * 0.88).max(0.0);
    let inset_fwd = CONNECTOR_ENDPOINT_INSET_M.min(fwd_len * 0.88).max(0.0);
    p0x -= in_dir.0 * inset_back;
    p0y -= in_dir.1 * inset_back;
    p3x += out_dir.0 * inset_fwd;
    p3y += out_dir.1 * inset_fwd;

    let chord = ((p3x - p0x).powi(2) + (p3y - p0y).powi(2)).sqrt();
    if chord < 1.5 {
        // Pull endpoints apart so the cubic is well-conditioned.
        const NUDGE: f64 = 3.0;
        p0x -= in_dir.0 * NUDGE;
        p0y -= in_dir.1 * NUDGE;
        p3x += out_dir.0 * NUDGE;
        p3y += out_dir.1 * NUDGE;
    }

    let h = CONNECTOR_CUBIC_HANDLE_M;
    let p0 = Point::new(p0x, p0y);
    let p1 = Point::new(p0x + in_dir.0 * h, p0y + in_dir.1 * h);
    let p2 = Point::new(p3x - out_dir.0 * h, p3y - out_dir.1 * h);
    let p3 = Point::new(p3x, p3y);
    let cubic = CubicBez::new(p0, p1, p2, p3);
    let length_m = cubic.arclen(CONNECTOR_ARCLEN_ACC) as f32;

    const SAMPLES: usize = 40;
    let mut points = Vec::with_capacity(SAMPLES + 1);
    for i in 0..=SAMPLES {
        let t = i as f64 / SAMPLES as f64;
        let p = cubic.eval(t);
        let (lat, lng) = m_xy_to_geo(p.x, p.y);
        points.push([lat, lng]);
    }
    let connector_cubic_m = Some([
        [p0.x, p0.y],
        [p1.x, p1.y],
        [p2.x, p2.y],
        [p3.x, p3.y],
    ]);
    Some(Lane {
        id: lane_id,
        path: BezierPath {
            points,
            length_m: length_m.max(0.5),
        },
        connector_cubic_m,
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

