use petgraph::graph::NodeIndex;
use rstar::{RTree, RTreeObject, AABB, PointDistance};

use crate::map::building_loader::OdBuilding;
use crate::map::road_network::RoadGraph;

// ── R-tree entry ─────────────────────────────────────────────────────────────

/// An entry in the road-node R-tree.
/// Stores a 2-D position [lng, lat] alongside the graph `NodeIndex`.
struct RoadNodePoint {
    position: [f64; 2],
    node_idx: NodeIndex,
}

impl RTreeObject for RoadNodePoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.position)
    }
}

impl PointDistance for RoadNodePoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        // Squared Euclidean distance in (lng, lat) space – fine for nearest-node lookup
        // (we only need relative ordering, not true geodesic distances)
        let dlng = self.position[0] - point[0];
        let dlat = self.position[1] - point[1];
        dlng * dlng + dlat * dlat
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn build_rtree(graph: &RoadGraph) -> RTree<RoadNodePoint> {
    let entries: Vec<RoadNodePoint> = graph
        .node_indices()
        .map(|idx| {
            let node = &graph[idx];
            RoadNodePoint {
                position: [node.lng, node.lat],
                node_idx: idx,
            }
        })
        .collect();
    RTree::bulk_load(entries)
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Fill `access_node` for every building by finding the nearest road-graph node.
///
/// Complexity: O(n × log m) where n = buildings, m = road nodes.
/// For Kraków Śródmieście (~2 000 buildings, ~5 000 nodes) this takes < 50 ms.
pub fn link_to_road_nodes(buildings: &mut Vec<OdBuilding>, graph: &RoadGraph) {
    if graph.node_count() == 0 {
        return;
    }

    let rtree = build_rtree(graph);

    for building in buildings.iter_mut() {
        if let Some(nearest) = rtree.nearest_neighbor(&building.centroid) {
            building.access_node = Some(nearest.node_idx);
        }
    }
}
