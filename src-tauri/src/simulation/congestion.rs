use serde::{Deserialize, Serialize};
use crate::map::road_network::RoadGraph;
use crate::vehicles::vehicle::Vehicle;

const AVG_CAR_LENGTH_M: f32 = 5.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CongestionData {
    pub edge_id: u64,
    pub level: f32,
    pub lat: f64,
    pub lng: f64,
}

/// Compute congestion level per edge.
///
/// `level = vehicle_count / (lanes × edge_length_m / avg_car_length_m)`
///
/// Clamped to [0.0, 1.0].
pub fn compute_congestion(graph: &RoadGraph, vehicles: &[Vehicle]) -> Vec<CongestionData> {
    use petgraph::visit::EdgeRef;
    use std::collections::HashMap;

    // Count vehicles per edge
    let mut counts: HashMap<petgraph::graph::EdgeIndex, u32> = HashMap::new();
    for vehicle in vehicles {
        if vehicle.route_pos < vehicle.route.len() {
            let edge_idx = vehicle.route[vehicle.route_pos];
            *counts.entry(edge_idx).or_insert(0) += 1;
        }
    }

    let mut result = Vec::new();

    for edge_ref in graph.edge_references() {
        let edge = edge_ref.weight();
        let count = *counts.get(&edge_ref.id()).unwrap_or(&0) as f32;

        let capacity = (edge.lanes as f32 * edge.length_m / AVG_CAR_LENGTH_M).max(1.0);
        let level = (count / capacity).min(1.0);

        // Only include edges with at least some congestion
        if level < 0.01 && count == 0.0 {
            continue;
        }

        // Position: midpoint of the edge
        let src = &graph[edge_ref.source()];
        let tgt = &graph[edge_ref.target()];
        let mid_lat = (src.lat + tgt.lat) / 2.0;
        let mid_lng = (src.lng + tgt.lng) / 2.0;

        result.push(CongestionData {
            edge_id: edge.osm_id,
            level,
            lat: mid_lat,
            lng: mid_lng,
        });
    }

    result
}
