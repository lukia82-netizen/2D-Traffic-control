use petgraph::graph::{NodeIndex, EdgeIndex};
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use crate::map::road_network::{RoadGraph, haversine_distance_m};

/// Find the shortest (by travel time) path from `from` to `to` using A*.
/// Returns the sequence of `EdgeIndex` values to follow, or `None` if unreachable.
pub fn find_path(
    graph: &RoadGraph,
    from: NodeIndex,
    to: NodeIndex,
) -> Option<Vec<EdgeIndex>> {
    let to_lat = graph[to].lat;
    let to_lng = graph[to].lng;

    let result = astar(
        graph,
        from,
        |n| n == to,
        |edge_ref| {
            let edge = edge_ref.weight();
            // Cost = travel time in seconds
            if edge.max_speed > 0.0 {
                edge.length_m / edge.max_speed
            } else {
                f32::INFINITY
            }
        },
        |node_idx| {
            // Heuristic: minimum travel time assuming max speed of 130 km/h (~36 m/s)
            let n = &graph[node_idx];
            let dist = haversine_distance_m(n.lat, n.lng, to_lat, to_lng);
            dist / 36.0
        },
    );

    result.map(|(_, node_path)| {
        // Convert the node path into the corresponding edge sequence
        node_path
            .windows(2)
            .filter_map(|pair| {
                let (a, b) = (pair[0], pair[1]);
                graph
                    .edges_connecting(a, b)
                    .next()
                    .map(|e| e.id())
            })
            .collect()
    })
}

/// Pick a random destination node that is reachable from `from`.
/// Falls back to `from` if no reachable nodes are found.
pub fn random_destination(
    graph: &RoadGraph,
    from: NodeIndex,
    rng: &mut impl rand::Rng,
) -> NodeIndex {
    use rand::seq::IteratorRandom;

    let count = graph.node_count();
    if count < 2 {
        return from;
    }

    // Try a few random nodes and find one that is different from `from`
    for _ in 0..20 {
        if let Some(idx) = graph.node_indices().choose(rng) {
            if idx != from {
                return idx;
            }
        }
    }

    from
}
