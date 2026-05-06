use petgraph::graph::{NodeIndex, EdgeIndex};
use petgraph::algo::astar;
use petgraph::visit::EdgeRef;
use crate::map::road_network::{RoadGraph, haversine_distance_m};

/// Reference speed constant for `route_alpha = 0.5` blending [m/s] (= 50 km/h).
pub const REF_SPEED_MS: f32 = 13.89;

/// Find a path from `from` to `to` using A*.
///
/// The edge cost blends shortest-distance and fastest-time based on `route_alpha`:
/// - `route_alpha = 0.0` → minimises distance (ignores speed limits)
/// - `route_alpha = 1.0` → minimises travel time (fastest route)
/// - intermediate values → weighted blend
///
/// `ref_speed` is the reference constant used for the alpha interpolation.
/// Returns the sequence of `EdgeIndex` values to follow, or `None` if unreachable.
pub fn find_path(
    graph: &RoadGraph,
    from: NodeIndex,
    to: NodeIndex,
    route_alpha: f32,
    ref_speed: f32,
) -> Option<Vec<EdgeIndex>> {
    let to_lat = graph[to].lat;
    let to_lng = graph[to].lng;

    let result = astar(
        graph,
        from,
        |n| n == to,
        |edge_ref| {
            let edge = edge_ref.weight();
            // Effective speed: lerp between ref_speed and edge.max_speed
            let eff_speed = ref_speed + route_alpha * (edge.max_speed - ref_speed);
            edge.length_m / eff_speed.max(0.1)
        },
        |node_idx| {
            // Admissible heuristic: minimum travel time at 130 km/h (~36 m/s)
            let n = &graph[node_idx];
            let dist = haversine_distance_m(n.lat, n.lng, to_lat, to_lng);
            dist / 36.0
        },
    );

    result.map(|(_, node_path)| {
        node_path
            .windows(2)
            .filter_map(|pair| {
                let (a, b) = (pair[0], pair[1]);
                graph.edges_connecting(a, b).next().map(|e| e.id())
            })
            .collect()
    })
}

/// Pick a random destination node different from `from`.
/// Falls back to `from` if no alternative node can be found.
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

    for _ in 0..20 {
        if let Some(idx) = graph.node_indices().choose(rng) {
            if idx != from {
                return idx;
            }
        }
    }

    from
}
