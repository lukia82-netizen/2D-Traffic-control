use std::collections::HashMap;
use petgraph::graph::NodeIndex;
use serde::{Deserialize, Serialize};

use crate::map::osm_loader::OsmData;

/// Semantic type of a building used by the OD model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BuildingType {
    Residential,
    Commercial,
    Office,
    Other,
}

impl BuildingType {
    pub fn as_str(self) -> &'static str {
        match self {
            BuildingType::Residential => "residential",
            BuildingType::Commercial  => "commercial",
            BuildingType::Office      => "office",
            BuildingType::Other       => "other",
        }
    }
}

/// A building enriched with OD-simulation data.
/// `access_node` is `None` until `building_network::link_to_road_nodes` is called.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OdBuilding {
    pub id: u64,
    /// Polygon vertices as \[lng, lat\] pairs.
    pub polygon: Vec<[f64; 2]>,
    /// Centroid \[lng, lat\].
    pub centroid: [f64; 2],
    pub building_type: BuildingType,
    /// Nearest road-graph node; filled by `building_network::link_to_road_nodes`.
    #[serde(skip)]
    pub access_node: Option<NodeIndex>,
}

// ── Classification helpers ──────────────────────────────────────────────────

fn classify_building(tags: &HashMap<String, String>) -> BuildingType {
    // Specific amenity / shop tags take priority
    if let Some(amenity) = tags.get("amenity") {
        match amenity.as_str() {
            "supermarket" | "mall" | "marketplace" | "food_court"
            | "fast_food" | "restaurant" | "cafe" | "bar" | "pub" => {
                return BuildingType::Commercial;
            }
            "bank" | "townhall" | "courthouse" | "government"
            | "embassy" | "post_office" | "police" | "fire_station" => {
                return BuildingType::Office;
            }
            _ => {}
        }
    }
    if tags.contains_key("shop") {
        return BuildingType::Commercial;
    }
    if tags.contains_key("office") {
        return BuildingType::Office;
    }

    // Fall back to building= value
    if let Some(building) = tags.get("building") {
        match building.as_str() {
            "residential" | "apartments" | "house" | "detached"
            | "semidetached_house" | "terrace" | "dormitory" | "bungalow" => {
                return BuildingType::Residential;
            }
            "commercial" | "retail" | "supermarket" | "warehouse" | "kiosk" => {
                return BuildingType::Commercial;
            }
            "office" | "government" | "civic" | "public"
            | "school" | "university" | "hospital" | "church"
            | "cathedral" | "chapel" | "synagogue" | "mosque" => {
                return BuildingType::Office;
            }
            _ => {}
        }
    }

    BuildingType::Other
}

fn compute_centroid(polygon: &[[f64; 2]]) -> [f64; 2] {
    let n = polygon.len() as f64;
    let lng = polygon.iter().map(|p| p[0]).sum::<f64>() / n;
    let lat = polygon.iter().map(|p| p[1]).sum::<f64>() / n;
    [lng, lat]
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Extract OD buildings from OSM data.
/// `access_node` is `None` for every building; call `building_network::link_to_road_nodes`
/// after the road graph is available to fill them in.
pub fn extract_od_buildings(osm_data: &OsmData) -> Vec<OdBuilding> {
    let mut buildings: Vec<OdBuilding> = Vec::new();

    for way in &osm_data.ways {
        if !way.tags.contains_key("building") {
            continue;
        }
        // Collect polygon as [lng, lat] vertices
        let polygon: Vec<[f64; 2]> = way
            .node_refs
            .iter()
            .filter_map(|&nid| osm_data.nodes.get(&nid))
            .map(|n| [n.lng, n.lat])
            .collect();

        if polygon.len() < 3 {
            continue;
        }

        let centroid = compute_centroid(&polygon);
        let building_type = classify_building(&way.tags);

        buildings.push(OdBuilding {
            id: way.id,
            polygon,
            centroid,
            building_type,
            access_node: None,
        });
    }

    buildings
}
