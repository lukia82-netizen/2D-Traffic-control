use std::collections::{HashMap, HashSet};
use petgraph::graph::{DiGraph, NodeIndex};

use crate::map::osm_loader::OsmData;
use crate::map::road_network::{RoadGraph, haversine_distance_m};

// ── Data structures ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TramTrackType {
    /// Tram track runs on its own dedicated right-of-way.
    Dedicated,
    /// Tram track shares the carriageway with road traffic.
    SharedWithRoad,
}

#[derive(Debug, Clone)]
pub struct TramNode {
    pub id: u64,
    pub lat: f64,
    pub lng: f64,
    /// Whether this node corresponds to a passenger stop.
    pub is_stop: bool,
    /// Dwell time at this stop in game seconds (default 30).
    pub stop_dwell_s: f32,
}

#[derive(Debug, Clone)]
pub struct TramEdge {
    pub osm_id: u64,
    pub length_m: f32,
    /// Maximum speed for this segment [m/s]; default 40 km/h ≈ 11.1 m/s
    pub max_speed: f32,
    pub track_type: TramTrackType,
}

pub type TramGraph = DiGraph<TramNode, TramEdge>;

/// A single tram service line defined by an ordered stop sequence.
#[derive(Debug, Clone)]
pub struct TramLine {
    /// OSM `ref` / `name` tag for the line (e.g. "1", "6", "8").
    pub line_ref: String,
    pub stop_sequence: Vec<NodeIndex>,
}

/// All tram data for the loaded area.
#[derive(Debug)]
pub struct TramData {
    pub graph: TramGraph,
    pub node_index_map: HashMap<u64, NodeIndex>,
    /// NodeIndex list of nodes where `is_stop = true`.
    pub stops: Vec<NodeIndex>,
    pub lines: Vec<TramLine>,
}

impl TramData {
    pub fn is_empty(&self) -> bool {
        self.graph.node_count() == 0
    }
}

// ── Builder ──────────────────────────────────────────────────────────────────

/// Build a `TramData` from the loaded OSM data and the road graph.
pub fn build_tram_network(osm_data: &OsmData, road_graph: &RoadGraph) -> TramData {
    let mut graph = TramGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();
    let mut stops: Vec<NodeIndex> = Vec::new();

    // Collect tram-stop node ids
    let stop_ids: HashSet<u64> = osm_data
        .nodes
        .values()
        .filter(|n| n.tags.get("railway").map(String::as_str) == Some("tram_stop"))
        .map(|n| n.id)
        .collect();

    // Road-node osm-ids for shared-track detection
    let road_node_ids: HashSet<u64> = road_graph
        .node_indices()
        .map(|idx| road_graph[idx].osm_id)
        .collect();

    // Collect tram ways (railway=tram) – stored in the dedicated tram_ways field
    let tram_ways: Vec<&crate::map::osm_loader::OsmWay> = osm_data
        .tram_ways
        .iter()
        .collect();

    // Gather all node ids referenced by tram ways + explicit stops
    let mut tram_node_ids: HashSet<u64> = HashSet::new();
    for way in &tram_ways {
        for &nid in &way.node_refs {
            tram_node_ids.insert(nid);
        }
    }
    for &stop_id in &stop_ids {
        tram_node_ids.insert(stop_id);
    }

    // Add nodes to tram graph
    for &nid in &tram_node_ids {
        if let Some(osm_node) = osm_data.nodes.get(&nid) {
            let is_stop = stop_ids.contains(&nid);
            let idx = graph.add_node(TramNode {
                id: nid,
                lat: osm_node.lat,
                lng: osm_node.lng,
                is_stop,
                stop_dwell_s: 30.0,
            });
            node_index_map.insert(nid, idx);
            if is_stop {
                stops.push(idx);
            }
        }
    }

    // Add edges
    for way in &tram_ways {
        // Detect shared-track: ≥2 way nodes exist in the road graph
        let shared = way
            .node_refs
            .iter()
            .filter(|&&nid| road_node_ids.contains(&nid))
            .count()
            >= 2;
        let track_type = if shared {
            TramTrackType::SharedWithRoad
        } else {
            TramTrackType::Dedicated
        };

        for window in way.node_refs.windows(2) {
            let (from_id, to_id) = (window[0], window[1]);
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

            graph.add_edge(
                from_idx,
                to_idx,
                TramEdge {
                    osm_id: way.id,
                    length_m,
                    max_speed: 40.0 / 3.6,
                    track_type: track_type.clone(),
                },
            );
        }
    }

    // Build tram lines from OSM route relations (route=tram)
    let mut lines: Vec<TramLine> = Vec::new();
    for rel in &osm_data.relations {
        if rel.tags.get("route").map(String::as_str) != Some("tram") {
            continue;
        }
        let line_ref = rel
            .tags
            .get("ref")
            .or_else(|| rel.tags.get("name"))
            .cloned()
            .unwrap_or_else(|| rel.id.to_string());

        // Collect stops in the order they appear in the relation members
        let stop_sequence: Vec<NodeIndex> = rel
            .members
            .iter()
            .filter(|m| {
                m.member_type == "node"
                    && (m.role == "stop"
                        || m.role == "stop_entry_only"
                        || m.role == "stop_exit_only")
            })
            .filter_map(|m| node_index_map.get(&m.ref_id).copied())
            .collect();

        if stop_sequence.len() >= 2 {
            lines.push(TramLine { line_ref, stop_sequence });
        }
    }

    log::info!(
        "Built tram network: {} nodes, {} edges, {} stops, {} lines",
        graph.node_count(),
        graph.edge_count(),
        stops.len(),
        lines.len()
    );

    TramData { graph, node_index_map, stops, lines }
}
