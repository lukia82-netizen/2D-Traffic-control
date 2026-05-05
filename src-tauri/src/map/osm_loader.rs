use serde::Deserialize;
use std::collections::HashMap;

#[derive(Debug, Clone)]
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

#[derive(Debug, Default)]
pub struct OsmData {
    pub nodes: HashMap<u64, OsmNode>,
    pub ways: Vec<OsmWay>,
}

// --- Overpass JSON deserialization ---

#[derive(Deserialize)]
struct OverpassResponse {
    elements: Vec<OverpassElement>,
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
}

/// Fetch road data from the Overpass API for the given bounding box.
/// bbox = [west, south, east, north] (GeoJSON convention).
/// Uses tokio::task::spawn_blocking to safely wrap the blocking HTTP call
/// without blocking the async Tauri command executor.
pub async fn fetch_osm_data(bbox: [f64; 4]) -> Result<OsmData, String> {
    // bbox is [west, south, east, north] (GeoJSON / frontend convention)
    let [west, south, east, north] = bbox;

    let query = format!(
        "[out:json][timeout:90];(way[highway]({south},{west},{north},{east});node(w););out body;>;out skel qt;",
    );

    let url = "https://overpass-api.de/api/interpreter".to_string();
    let body = format!("data={}", urlencoding_simple(&query));

    log::info!("Fetching OSM data from Overpass API, bbox: {:?}", bbox);

    let text = tokio::task::spawn_blocking(move || -> Result<String, String> {
        let client = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(90))
            .build()
            .map_err(|e| format!("Failed to build HTTP client: {}", e))?;

        let response = client
            .post(&url)
            .header("Content-Type", "application/x-www-form-urlencoded")
            .body(body)
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

fn urlencoding_simple(s: &str) -> String {
    let mut out = String::with_capacity(s.len() * 2);
    for byte in s.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9'
            | b'-' | b'_' | b'.' | b'~' => out.push(byte as char),
            b' ' => out.push('+'),
            b => out.push_str(&format!("%{:02X}", b)),
        }
    }
    out
}

fn parse_overpass_json(json_text: &str) -> Result<OsmData, String> {
    let overpass: OverpassResponse = serde_json::from_str(json_text)
        .map_err(|e| format!("Failed to parse Overpass JSON: {}", e))?;

    let mut osm_data = OsmData::default();

    for element in overpass.elements {
        match element.element_type.as_str() {
            "node" => {
                osm_data.nodes.insert(
                    element.id,
                    OsmNode {
                        id: element.id,
                        lat: element.lat,
                        lng: element.lon,
                        tags: element.tags,
                    },
                );
            }
            "way" => {
                if !element.nodes.is_empty() {
                    osm_data.ways.push(OsmWay {
                        id: element.id,
                        node_refs: element.nodes,
                        tags: element.tags,
                    });
                }
            }
            _ => {}
        }
    }

    log::info!(
        "Parsed OSM data: {} nodes, {} ways",
        osm_data.nodes.len(),
        osm_data.ways.len()
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
