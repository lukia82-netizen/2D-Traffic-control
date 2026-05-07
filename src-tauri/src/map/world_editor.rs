use std::collections::{HashMap, HashSet};

use petgraph::visit::EdgeRef;

use crate::map::road_network::{haversine_distance_m, LaneDirection, MapData, RoadEdge, TurnConnector};

const TURN_CONNECTOR_ENTRY_M: f64 = 30.0;
const TURN_CONNECTOR_EXIT_M: f64 = 30.0;
const TURN_CONNECTOR_MIN_ANGLE_RAD: f64 = 0.35;
const TURN_CONNECTOR_LUT_SAMPLES: usize = 24;

impl MapData {
    pub fn rebuild_geometry(&mut self, node_id: u64) {
        let mut affected: HashSet<u64> = HashSet::from([node_id]);
        if let Some(&idx) = self.node_index_map.get(&node_id) {
            for n in self.graph.neighbors_directed(idx, petgraph::Direction::Incoming) {
                affected.insert(self.graph[n].osm_id);
            }
            for n in self.graph.neighbors_directed(idx, petgraph::Direction::Outgoing) {
                affected.insert(self.graph[n].osm_id);
            }
        }
        self.turn_connectors
            .retain(|c| !affected.contains(&c.via_node_id));
        for id in affected {
            self.turn_connectors.extend(self.compute_connectors_for_node(id));
        }
    }

    pub fn rebuild_all_geometry(&mut self) {
        self.turn_connectors.clear();
        let nodes: Vec<u64> = self.graph.node_indices().map(|n| self.graph[n].osm_id).collect();
        for node_id in nodes {
            self.turn_connectors.extend(self.compute_connectors_for_node(node_id));
        }
    }

    pub fn update_node_position(&mut self, node_id: u64, lat: f64, lng: f64) -> Result<(), String> {
        let Some(&idx) = self.node_index_map.get(&node_id) else {
            return Err(format!("Unknown node id {}", node_id));
        };
        self.graph[idx].lat = lat;
        self.graph[idx].lng = lng;

        let edge_ids: Vec<_> = self.graph.edges(idx).map(|e| e.id()).collect();
        for edge_id in edge_ids {
            if let Some((a, b)) = self.graph.edge_endpoints(edge_id) {
                let na = &self.graph[a];
                let nb = &self.graph[b];
                if let Some(edge) = self.graph.edge_weight_mut(edge_id) {
                    let len = haversine_distance_m(na.lat, na.lng, nb.lat, nb.lng);
                    edge.length_m = len;
                    edge.decision_points = [len * 0.25, len * 0.5, len * 0.75];
                }
            }
        }
        self.rebuild_geometry(node_id);
        Ok(())
    }

    pub fn add_edge_default(&mut self, from_node_id: u64, to_node_id: u64) -> Result<(), String> {
        let Some(&from) = self.node_index_map.get(&from_node_id) else {
            return Err(format!("Unknown from node {}", from_node_id));
        };
        let Some(&to) = self.node_index_map.get(&to_node_id) else {
            return Err(format!("Unknown to node {}", to_node_id));
        };
        let a = &self.graph[from];
        let b = &self.graph[to];
        let length_m = haversine_distance_m(a.lat, a.lng, b.lat, b.lng);
        let lanes = 2;
        let edge = RoadEdge {
            osm_id: 0,
            lanes,
            max_speed: 50.0 / 3.6,
            oneway: false,
            infra_type: crate::map::road_network::InfraType::Normal,
            layer: 0,
            length_m,
            lane_directions: vec![LaneDirection::Left, LaneDirection::Straight],
            decision_points: [length_m * 0.25, length_m * 0.5, length_m * 0.75],
            road_type: "secondary".to_string(),
            has_tram_track: false,
        };
        self.graph.add_edge(from, to, edge);
        self.rebuild_geometry(from_node_id);
        self.rebuild_geometry(to_node_id);
        Ok(())
    }

    fn compute_connectors_for_node(&self, via_node_id: u64) -> Vec<TurnConnector> {
        let Some(&via_idx) = self.node_index_map.get(&via_node_id) else {
            return Vec::new();
        };
        let mut incoming = Vec::new();
        let mut outgoing = Vec::new();
        for edge in self.graph.edges_directed(via_idx, petgraph::Direction::Incoming) {
            incoming.push((self.graph[edge.source()].osm_id, edge.weight().length_m as f64));
        }
        for edge in self.graph.edges_directed(via_idx, petgraph::Direction::Outgoing) {
            outgoing.push((self.graph[edge.target()].osm_id, edge.weight().length_m as f64));
        }

        let mut connectors = Vec::new();
        for (from_id, in_len_m) in &incoming {
            let Some(&from_idx) = self.node_index_map.get(from_id) else {
                continue;
            };
            for (to_id, out_len_m) in &outgoing {
                if from_id == to_id {
                    continue;
                }
                let Some(&to_idx) = self.node_index_map.get(to_id) else {
                    continue;
                };
                let from = &self.graph[from_idx];
                let via = &self.graph[via_idx];
                let to = &self.graph[to_idx];
                let angle = turn_angle_rad(from.lng, from.lat, via.lng, via.lat, to.lng, to.lat);
                if angle < TURN_CONNECTOR_MIN_ANGLE_RAD {
                    continue;
                }
                let (p1, ctrl, p2) = build_bezier_points(
                    [from.lng, from.lat],
                    [via.lng, via.lat],
                    [to.lng, to.lat],
                    *in_len_m,
                    *out_len_m,
                );
                let mut bezier_lut = Vec::with_capacity(TURN_CONNECTOR_LUT_SAMPLES + 1);
                for i in 0..=TURN_CONNECTOR_LUT_SAMPLES {
                    let t = i as f64 / TURN_CONNECTOR_LUT_SAMPLES as f64;
                    let u = 1.0 - t;
                    let lng = u * u * p1[0] + 2.0 * u * t * ctrl[0] + t * t * p2[0];
                    let lat = u * u * p1[1] + 2.0 * u * t * ctrl[1] + t * t * p2[1];
                    bezier_lut.push([lng, lat]);
                }
                connectors.push(TurnConnector {
                    from_node_id: *from_id,
                    via_node_id,
                    to_node_id: *to_id,
                    bezier_lut,
                });
            }
        }
        connectors
    }
}

fn turn_angle_rad(in_lng: f64, in_lat: f64, j_lng: f64, j_lat: f64, out_lng: f64, out_lat: f64) -> f64 {
    let ax = j_lng - in_lng;
    let ay = j_lat - in_lat;
    let bx = out_lng - j_lng;
    let by = out_lat - j_lat;
    let al = (ax * ax + ay * ay).sqrt().max(1e-9);
    let bl = (bx * bx + by * by).sqrt().max(1e-9);
    let dot = ((ax / al) * (bx / bl) + (ay / al) * (by / bl)).clamp(-1.0, 1.0);
    dot.acos()
}

fn build_bezier_points(
    in_src: [f64; 2],
    via: [f64; 2],
    out_tgt: [f64; 2],
    in_len_m: f64,
    out_len_m: f64,
) -> ([f64; 2], [f64; 2], [f64; 2]) {
    let entry_t = (1.0 - TURN_CONNECTOR_ENTRY_M / in_len_m.max(1.0)).clamp(0.0, 1.0);
    let exit_t = (TURN_CONNECTOR_EXIT_M / out_len_m.max(1.0)).clamp(0.0, 1.0);
    let p1 = [
        in_src[0] + (via[0] - in_src[0]) * entry_t,
        in_src[1] + (via[1] - in_src[1]) * entry_t,
    ];
    let p2 = [
        via[0] + (out_tgt[0] - via[0]) * exit_t,
        via[1] + (out_tgt[1] - via[1]) * exit_t,
    ];

    let lng_m = 71_700.0;
    let lat_m = 111_320.0;
    let in_fx_raw = (via[0] - in_src[0]) * lng_m;
    let in_fy_raw = (via[1] - in_src[1]) * lat_m;
    let out_fx_raw = (out_tgt[0] - via[0]) * lng_m;
    let out_fy_raw = (out_tgt[1] - via[1]) * lat_m;
    let in_len = (in_fx_raw * in_fx_raw + in_fy_raw * in_fy_raw).sqrt().max(1e-9);
    let out_len = (out_fx_raw * out_fx_raw + out_fy_raw * out_fy_raw).sqrt().max(1e-9);
    let in_fx = in_fx_raw / in_len;
    let in_fy = in_fy_raw / in_len;
    let out_fx = out_fx_raw / out_len;
    let out_fy = out_fy_raw / out_len;

    let p1x = p1[0] * lng_m;
    let p1y = p1[1] * lat_m;
    let p2x = p2[0] * lng_m;
    let p2y = p2[1] * lat_m;
    let det = in_fx * (-out_fy) - in_fy * (-out_fx);
    if det.abs() < 1e-9 {
        return (p1, via, p2);
    }
    let dx = p2x - p1x;
    let dy = p2y - p1y;
    let t = (dx * (-out_fy) - dy * (-out_fx)) / det;
    let cx = p1x + t * in_fx;
    let cy = p1y + t * in_fy;
    (p1, [cx / lng_m, cy / lat_m], p2)
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EditorTool {
    None,
    MoveNode,
    AddRoad,
    Delete,
    Select,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GraphChange {
    NodePosition { node_id: u64, before_lat: f64, before_lng: f64, after_lat: f64, after_lng: f64 },
    EdgeAdded { from_node_id: u64, to_node_id: u64 },
    EdgeDeleted { from_node_id: u64, to_node_id: u64 },
}

#[derive(Default, Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EditorHistory {
    pub undo: Vec<GraphChange>,
    pub redo: Vec<GraphChange>,
}

#[derive(Default, Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MapOverrides {
    pub node_positions: HashMap<u64, [f64; 2]>,
    pub added_edges: Vec<[u64; 2]>,
    pub deleted_edges: Vec<[u64; 2]>,
}

