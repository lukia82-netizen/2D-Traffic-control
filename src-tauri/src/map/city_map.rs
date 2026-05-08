use std::collections::HashMap;

use abstutil::Timer;
use glam::DVec2;
use osm2streets::{Direction, IntersectionControl, LaneType, MapConfig, StreetNetwork};
use petgraph::graph::NodeIndex;
use streets_reader::osm_to_street_network;

use crate::map::road_network::{
    find_boundary_nodes, find_spawn_points, InfraType, IntersectionType, MapData, RoadEdge,
    RoadGraph, RoadNode,
};
use crate::map::tram_network::TramData;
use crate::simulation::bezier_smooth::BezierPath;

pub struct CityLanePath {
    pub lane_id: String,
    pub bezier_path: BezierPath,
}

pub struct CityIntersection {
    pub intersection_id: String,
    pub lane_ids: Vec<String>,
    pub conflict_pairs: Vec<(String, String)>,
}

pub struct CityMap {
    pub lanes: Vec<CityLanePath>,
    pub intersections: Vec<CityIntersection>,
}

pub async fn build_city_map_from_bbox(bbox: [f64; 4]) -> Result<(CityMap, MapData), String> {
    let xml = download_overpass_xml(bbox).await?;
    let center_lat = (bbox[1] + bbox[3]) * 0.5;
    let center_lon = (bbox[0] + bbox[2]) * 0.5;

    let mut cfg = MapConfig::default();
    cfg.country_code = "PL".to_string();
    cfg.date_time = None;
    let _local_origin_hint = (center_lat, center_lon);

    let mut timer = Timer::throwaway();
    let (mut network, _) = osm_to_street_network(xml.as_bytes(), None, cfg, &mut timer)
        .map_err(|e| format!("osm_to_street_network failed: {e}"))?;
    network.apply_transformations(vec![], &mut timer);

    let city_map = build_city_map(&network);
    let map_data = build_map_data_from_network(&network);
    Ok((city_map, map_data))
}

async fn download_overpass_xml(bbox: [f64; 4]) -> Result<String, String> {
    let [west, south, east, north] = bbox;
    let query = format!(
        "[out:xml][timeout:120];(way[\"highway\"~\"motorway|trunk|primary|secondary|tertiary\"]({south},{west},{north},{east});>;);out body qt;"
    );

    tokio::task::spawn_blocking(move || -> Result<String, String> {
        let client = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(120))
            .user_agent("TrafficControl2D/0.2 (osm2streets)")
            .build()
            .map_err(|e| format!("HTTP client build failed: {e}"))?;

        let response = client
            .post("https://overpass-api.de/api/interpreter")
            .form(&[("data", query)])
            .send()
            .map_err(|e| format!("Overpass request failed: {e}"))?;

        if !response.status().is_success() {
            return Err(format!("Overpass API returned {}", response.status()));
        }

        response
            .text()
            .map_err(|e| format!("Failed to read Overpass body: {e}"))
    })
    .await
    .map_err(|e| format!("spawn_blocking failure: {e}"))?
}

fn build_city_map(network: &StreetNetwork) -> CityMap {
    let mut lanes = Vec::new();
    let mut lanes_by_road: HashMap<usize, Vec<String>> = HashMap::new();

    for road in network.roads.values() {
        let lane_lines = driving_lane_center_lines(road);
        for (lane_idx, polyline) in lane_lines {
            let simplified = simplify_polyline(&polyline.points(), 1.25);
            if simplified.len() < 2 {
                continue;
            }

            let bezier = polyline_to_bezier(&simplified);
            let lane_id = format!("r{}_l{}", road.id.0, lane_idx);
            lanes_by_road
                .entry(road.id.0)
                .or_default()
                .push(lane_id.clone());
            lanes.push(CityLanePath {
                lane_id,
                bezier_path: bezier,
            });
        }
    }

    let intersections = network
        .intersections
        .values()
        .map(|intersection| {
            let mut lane_ids = Vec::new();
            for road_id in &intersection.roads {
                if let Some(ids) = lanes_by_road.get(&road_id.0) {
                    lane_ids.extend(ids.iter().cloned());
                }
            }
            lane_ids.sort();
            lane_ids.dedup();

            let mut conflict_pairs = Vec::new();
            for i in 0..lane_ids.len() {
                for j in (i + 1)..lane_ids.len() {
                    conflict_pairs.push((lane_ids[i].clone(), lane_ids[j].clone()));
                }
            }

            CityIntersection {
                intersection_id: format!("i{}", intersection.id.0),
                lane_ids,
                conflict_pairs,
            }
        })
        .collect();

    CityMap {
        lanes,
        intersections,
    }
}

fn build_map_data_from_network(network: &StreetNetwork) -> MapData {
    let mut graph = RoadGraph::new();
    let mut node_index_map: HashMap<u64, NodeIndex> = HashMap::new();

    for intersection in network.intersections.values() {
        let center = intersection.polygon.center();
        let gps = center.to_gps(&network.gps_bounds);
        let osm_id = intersection
            .osm_ids
            .first()
            .map(|n| n.0)
            .unwrap_or(intersection.id.0 as i64);
        let node_idx = graph.add_node(RoadNode {
            osm_id: osm_id as u64,
            lat: gps.y(),
            lng: gps.x(),
            intersection_type: match intersection.control {
                IntersectionControl::Signalled => IntersectionType::TrafficLight,
                _ => IntersectionType::Plain,
            },
        });
        node_index_map.insert(intersection.id.0 as u64, node_idx);
    }

    for road in network.roads.values() {
        let Some(&from) = node_index_map.get(&(road.src_i.0 as u64)) else {
            continue;
        };
        let Some(&to) = node_index_map.get(&(road.dst_i.0 as u64)) else {
            continue;
        };

        let driving_lanes = road
            .lane_specs_ltr
            .iter()
            .filter(|lane| lane.lt == LaneType::Driving)
            .count()
            .max(1) as u8;
        let max_speed = road
            .speed_limit
            .map(|s| s.inner_meters_per_second() as f32)
            .unwrap_or(13.9);
        let length_m = road.center_line.length().inner_meters() as f32;

        let edge = RoadEdge {
            osm_id: road
                .osm_ids
                .first()
                .map(|x| x.0 as u64)
                .unwrap_or(road.id.0 as u64),
            lanes: driving_lanes,
            max_speed,
            oneway: road.oneway_for_driving().is_some(),
            infra_type: InfraType::Normal,
            layer: road.layer as i8,
            length_m,
            lane_directions: build_lane_directions(driving_lanes),
            decision_points: [length_m * 0.25, length_m * 0.5, length_m * 0.75],
            road_type: road.highway_type.clone(),
            has_tram_track: false,
        };
        graph.add_edge(from, to, edge.clone());

        if road.oneway_for_driving().is_none() {
            graph.add_edge(to, from, edge);
        }
    }

    let bbox = compute_bbox(&graph);
    let spawn_points = find_spawn_points(&graph, &bbox);
    let boundary_nodes = find_boundary_nodes(&graph, &bbox);
    let tram_data = TramData {
        graph: crate::map::tram_network::TramGraph::new(),
        node_index_map: HashMap::new(),
        stops: Vec::new(),
        lines: Vec::new(),
    };

    MapData {
        graph,
        node_index_map,
        bbox,
        spawn_points,
        boundary_nodes,
        od_buildings: Vec::new(),
        restrictions: Vec::new(),
        tram_data,
        is_sandbox: false,
        sandbox_simple_cross_tl: false,
        lane_width_m: 3.5,
        turn_connectors: Vec::new(),
        lanes: HashMap::new(),
        conflict_areas: HashMap::new(),
        lane_by_edge_lane: HashMap::new(),
    }
}

fn compute_bbox(graph: &RoadGraph) -> [f64; 4] {
    let mut min_lat = f64::MAX;
    let mut max_lat = f64::MIN;
    let mut min_lng = f64::MAX;
    let mut max_lng = f64::MIN;
    for idx in graph.node_indices() {
        let n = &graph[idx];
        min_lat = min_lat.min(n.lat);
        max_lat = max_lat.max(n.lat);
        min_lng = min_lng.min(n.lng);
        max_lng = max_lng.max(n.lng);
    }
    if min_lat == f64::MAX {
        [0.0, 0.0, 0.0, 0.0]
    } else {
        [min_lat, min_lng, max_lat, max_lng]
    }
}

fn build_lane_directions(lanes: u8) -> Vec<crate::map::road_network::LaneDirection> {
    use crate::map::road_network::LaneDirection;
    match lanes {
        0 | 1 => vec![LaneDirection::Straight],
        2 => vec![LaneDirection::Left, LaneDirection::Straight],
        3 => vec![
            LaneDirection::Left,
            LaneDirection::Straight,
            LaneDirection::Right,
        ],
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

fn driving_lane_center_lines(road: &osm2streets::Road) -> Vec<(usize, geom::PolyLine)> {
    let mut result = Vec::new();
    let total_width = road.total_width();
    let mut width_so_far = geom::Distance::ZERO;

    for (idx, lane) in road.lane_specs_ltr.iter().enumerate() {
        width_so_far += lane.width / 2.0;
        if lane.lt == LaneType::Driving {
            let shifted = road
                .center_line
                .shift_from_center(total_width, width_so_far)
                .unwrap_or_else(|_| road.center_line.clone());
            let oriented = if lane.dir == Direction::Forward {
                shifted
            } else {
                shifted.reversed()
            };
            result.push((idx, oriented));
        }
        width_so_far += lane.width / 2.0;
    }
    result
}

fn simplify_polyline(points: &[geom::Pt2D], max_deviation_m: f64) -> Vec<geom::Pt2D> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let mut simplified = Vec::with_capacity(points.len());
    simplified.push(points[0]);
    for w in points.windows(3) {
        let a = w[0];
        let b = w[1];
        let c = w[2];
        let ab = DVec2::new(b.x() - a.x(), b.y() - a.y());
        let bc = DVec2::new(c.x() - b.x(), c.y() - b.y());
        if ab.length_squared() < 1e-6 || bc.length_squared() < 1e-6 {
            continue;
        }
        let bend = (ab.normalize().dot(bc.normalize())).clamp(-1.0, 1.0);
        let angle = bend.acos();
        let keep = angle > 0.12 || point_line_distance(b, a, c) > max_deviation_m;
        if keep {
            simplified.push(b);
        }
    }
    simplified.push(*points.last().unwrap());
    simplified
}

fn point_line_distance(p: geom::Pt2D, a: geom::Pt2D, b: geom::Pt2D) -> f64 {
    let ap = DVec2::new(p.x() - a.x(), p.y() - a.y());
    let ab = DVec2::new(b.x() - a.x(), b.y() - a.y());
    let denom = ab.length_squared();
    if denom < 1e-9 {
        return ap.length();
    }
    let t = (ap.dot(ab) / denom).clamp(0.0, 1.0);
    let proj = DVec2::new(a.x(), a.y()) + t * ab;
    (DVec2::new(p.x(), p.y()) - proj).length()
}

fn polyline_to_bezier(points: &[geom::Pt2D]) -> BezierPath {
    let start = points[0];
    let end = points[points.len() - 1];
    let mid = points[points.len() / 2];
    let ctrl = DVec2::new(mid.x(), mid.y());
    BezierPath::new(
        DVec2::new(start.x(), start.y()),
        ctrl,
        DVec2::new(end.x(), end.y()),
    )
}
