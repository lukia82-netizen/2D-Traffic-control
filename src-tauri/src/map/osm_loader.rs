use serde::Deserialize;
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
pub struct OsmNode {
    pub id: u64,
    pub lat: f64,
    pub lng: f64,
    pub tags: HashMap<String, String>,
}

#[derive(Debug, Clone)]
pub struct OsmWay {
    pub id: u64,
    pub node_refs: Vec<u64>,
    pub tags: HashMap<String, String>,
}

/// A single member of an OSM relation (role = "from", "via", or "to").
#[derive(Debug, Clone)]
pub struct OsmMember {
    pub member_type: String, // "node" | "way"
    pub ref_id: u64,
    pub role: String,
}

/// A parsed OSM relation — used primarily for turn restrictions
/// (`type=restriction`).
#[derive(Debug, Clone)]
pub struct OsmRelation {
    pub id: u64,
    pub tags: HashMap<String, String>,
    pub members: Vec<OsmMember>,
}

#[derive(Debug, Default)]
pub struct OsmData {
    pub nodes: HashMap<u64, OsmNode>,
    pub ways: Vec<OsmWay>,
    pub relations: Vec<OsmRelation>,
    /// Ways with `railway=tram` tag — used by tram_network.rs
    pub tram_ways: Vec<OsmWay>,
    /// Nodes with `railway=tram_stop` tag
    pub tram_stops: Vec<OsmNode>,
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Returns `true` when the OSM `oneway` tag marks forward-only travel.
pub fn is_oneway(way: &OsmWay) -> bool {
    matches!(
        way.tags.get("oneway").map(String::as_str),
        Some("yes" | "true" | "1")
    )
}

/// Returns `true` when the `oneway` tag marks reverse-only travel (rare but
/// valid in OSM: `oneway=-1`).
pub fn is_reverse_oneway(way: &OsmWay) -> bool {
    matches!(
        way.tags.get("oneway").map(String::as_str),
        Some("-1" | "reverse")
    )
}

// --- Overpass JSON deserialization ---

#[derive(Deserialize)]
struct OverpassResponse {
    elements: Vec<OverpassElement>,
}

#[derive(Deserialize)]
struct OverpassMember {
    #[serde(rename = "type")]
    member_type: String,
    #[serde(rename = "ref")]
    ref_id: u64,
    #[serde(default)]
    role: String,
}

#[derive(Deserialize)]
struct OverpassElement {
    #[serde(rename = "type")]
    element_type: String,
    id: u64,
    #[serde(default)]
    lat: f64,
    #[serde(default)]
    lon: f64,
    #[serde(default)]
    nodes: Vec<u64>,
    #[serde(default)]
    tags: HashMap<String, String>,
    #[serde(default)]
    members: Vec<OverpassMember>,
}

/// Fetch road data from the Overpass API for the given bounding box.
/// bbox = [west, south, east, north] (GeoJSON convention).
/// Uses tokio::task::spawn_blocking to safely wrap the blocking HTTP call
/// without blocking the async Tauri command executor.
pub async fn fetch_osm_data(bbox: [f64; 4]) -> Result<OsmData, String> {
    // bbox is [west, south, east, north] (GeoJSON / frontend convention)
    let [west, south, east, north] = bbox;

    // Overpass QL:
    //  1. Collect highway ways, building ways, and turn-restriction relations
    //  2. (._;>;) — re-union with itself + recurse-down to get ALL member nodes
    //     and ways referenced by the collected elements
    //  3. Output everything in one response
    let query = format!(
        "[out:json][timeout:90];\
        (\
          way[highway~\"motorway|trunk|primary|secondary|tertiary|residential|service|unclassified|living_street\"]\
          ({south},{west},{north},{east});\
          way[building]({south},{west},{north},{east});\
          way[railway=tram]({south},{west},{north},{east});\
          node[railway=tram_stop]({south},{west},{north},{east});\
          relation[type=restriction]({south},{west},{north},{east});\
          relation[route=tram]({south},{west},{north},{east});\
        );\
        (._;>;);\
        out body qt;",
    );

    let url = "https://overpass-api.de/api/interpreter".to_string();

    log::info!("Fetching OSM data from Overpass API, bbox: {:?}", bbox);

    let text = tokio::task::spawn_blocking(move || -> Result<String, String> {
        let client = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(90))
            .user_agent("TrafficControl2D/0.1 (tauri desktop app)")
            .build()
            .map_err(|e| format!("Failed to build HTTP client: {}", e))?;

        // Use .form() for correct application/x-www-form-urlencoded encoding
        let response = client
            .post(&url)
            .form(&[("data", &query)])
            .send()
            .map_err(|e| format!("HTTP request failed: {}", e))?;

        if !response.status().is_success() {
            return Err(format!("Overpass API returned status: {}", response.status()));
        }

        response
            .text()
            .map_err(|e| format!("Failed to read response body: {}", e))
    })
    .await
    .map_err(|e| format!("spawn_blocking failed: {}", e))??;

    parse_overpass_json(&text)
}


fn parse_overpass_json(json_text: &str) -> Result<OsmData, String> {
    let overpass: OverpassResponse = serde_json::from_str(json_text)
        .map_err(|e| format!("Failed to parse Overpass JSON: {}", e))?;

    let mut osm_data = OsmData::default();

    for element in overpass.elements {
        match element.element_type.as_str() {
            "node" => {
                let node = OsmNode {
                    id: element.id,
                    lat: element.lat,
                    lng: element.lon,
                    tags: element.tags,
                };
                // Tram stops go into both nodes (for coordinate lookup) and tram_stops
                if node.tags.get("railway").map(|s| s.as_str()) == Some("tram_stop") {
                    osm_data.tram_stops.push(node.clone());
                }
                osm_data.nodes.insert(element.id, node);
            }
            "way" => {
                if !element.nodes.is_empty() {
                    let way = OsmWay {
                        id: element.id,
                        node_refs: element.nodes,
                        tags: element.tags,
                    };
                    // Route to appropriate collection based on primary tag
                    if way.tags.get("railway").map(|s| s.as_str()) == Some("tram") {
                        osm_data.tram_ways.push(way);
                    } else {
                        // highway + building ways
                        osm_data.ways.push(way);
                    }
                }
            }
            "relation" => {
                let members = element
                    .members
                    .into_iter()
                    .map(|m| OsmMember {
                        member_type: m.member_type,
                        ref_id: m.ref_id,
                        role: m.role,
                    })
                    .collect();
                osm_data.relations.push(OsmRelation {
                    id: element.id,
                    tags: element.tags,
                    members,
                });
            }
            _ => {}
        }
    }

    log::info!(
        "Parsed OSM data: {} nodes, {} ways, {} tram ways, {} tram stops, {} relations",
        osm_data.nodes.len(),
        osm_data.ways.len(),
        osm_data.tram_ways.len(),
        osm_data.tram_stops.len(),
        osm_data.relations.len(),
    );

    Ok(osm_data)
}

pub fn load_osm_pbf(path: &std::path::Path) -> Result<OsmData, String> {
    use osmpbf::{ElementReader, Element};

    let reader = ElementReader::from_path(path)
        .map_err(|e| format!("Failed to open PBF file: {}", e))?;

    let mut osm_data = OsmData::default();

    reader
        .for_each(|element| match element {
            Element::Node(n) => {
                let mut tags = HashMap::new();
                for (k, v) in n.tags() {
                    tags.insert(k.to_string(), v.to_string());
                }
                osm_data.nodes.insert(
                    n.id() as u64,
                    OsmNode {
                        id: n.id() as u64,
                        lat: n.lat(),
                        lng: n.lon(),
                        tags,
                    },
                );
            }
            Element::DenseNode(n) => {
                let mut tags = HashMap::new();
                for (k, v) in n.tags() {
                    tags.insert(k.to_string(), v.to_string());
                }
                osm_data.nodes.insert(
                    n.id() as u64,
                    OsmNode {
                        id: n.id() as u64,
                        lat: n.lat(),
                        lng: n.lon(),
                        tags,
                    },
                );
            }
            Element::Way(w) => {
                let highway_tag = w.tags().find(|(k, _)| *k == "highway");
                if highway_tag.is_some() {
                    let mut tags = HashMap::new();
                    for (k, v) in w.tags() {
                        tags.insert(k.to_string(), v.to_string());
                    }
                    let node_refs: Vec<u64> = w.refs().map(|r| r as u64).collect();
                    if !node_refs.is_empty() {
                        osm_data.ways.push(OsmWay {
                            id: w.id() as u64,
                            node_refs,
                            tags,
                        });
                    }
                }
            }
            Element::Relation(_) => {}
        })
        .map_err(|e| format!("Failed to read PBF elements: {}", e))?;

    log::info!(
        "Loaded PBF: {} nodes, {} ways",
        osm_data.nodes.len(),
        osm_data.ways.len()
    );

    Ok(osm_data)
}
