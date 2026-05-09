use petgraph::graph::NodeIndex;
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

use crate::map::road_network::LaneId;
use crate::map::road_network::MapData;
use crate::simulation::lane_change::compute_vehicle_target_lane;
use crate::simulation::od_model::{OdModel, TripKind};
use crate::simulation::pathfinding::{find_path, REF_SPEED_MS};
use crate::simulation::speed_config::SpeedConfig;
use crate::vehicles::driver::DriverProfile;
use crate::vehicles::types::VehicleType;
use crate::vehicles::vehicle::Vehicle;

/// Default hard cap on total non-tram vehicles in the simulation.
/// A conservative start value; the frontend sends SetMaxVehicles to override.
const DEFAULT_MAX_VEHICLES: usize = 30;

/// Fraction of spawns that are pure-transit (boundary → boundary).
const TRANSIT_FRACTION: f32 = 0.25;
/// Fraction of spawns that are external in/out (boundary ↔ building).
const EXTERNAL_FRACTION: f32 = 0.15;
// Remaining 0.60 are local OD between buildings.

pub struct SpawnSystem {
    pub spawn_points: Vec<NodeIndex>,
    pub boundary_nodes: Vec<NodeIndex>,
    /// Vehicles per real second at spawn multiplier = 1.0
    pub base_rate: f32,
    /// Fractional vehicle accumulator
    pub accumulator: f32,
    pub rng: StdRng,
    pub next_id: u32,
    pub speed_config: SpeedConfig,
    /// Hard cap on total non-tram vehicles (configurable at runtime)
    pub max_vehicles: usize,
    /// When true, always spawn `Car` (one size for sandbox/demo maps).
    pub sandbox_mode: bool,
}

impl SpawnSystem {
    pub fn new(
        spawn_points: Vec<NodeIndex>,
        boundary_nodes: Vec<NodeIndex>,
        speed_config: SpeedConfig,
        sandbox_mode: bool,
    ) -> Self {
        SpawnSystem {
            spawn_points,
            boundary_nodes,
            base_rate: 1.0,
            accumulator: 0.0,
            rng: StdRng::from_entropy(),
            next_id: 1,
            speed_config,
            max_vehicles: DEFAULT_MAX_VEHICLES,
            sandbox_mode,
        }
    }

    /// Update the speed config (called when `SimCommand::SetSpeedConfig` arrives).
    pub fn set_speed_config(&mut self, cfg: SpeedConfig) {
        self.speed_config = cfg;
    }

    /// Tick the spawn system and return any new vehicles for this frame.
    pub fn tick(
        &mut self,
        dt_real_s: f32,
        multiplier: f32,
        map: &MapData,
        od_model: &OdModel,
        current_vehicle_count: usize,
    ) -> Vec<Vehicle> {
        if current_vehicle_count >= self.max_vehicles {
            return Vec::new();
        }

        self.accumulator += self.base_rate * multiplier * dt_real_s;
        let to_spawn = self.accumulator.floor() as u32;
        self.accumulator -= to_spawn as f32;

        let cap = (self.max_vehicles - current_vehicle_count) as u32;
        let to_spawn = to_spawn.min(cap);

        let mut new_vehicles = Vec::with_capacity(to_spawn as usize);

        for _ in 0..to_spawn {
            if let Some(v) = self.spawn_one(map, od_model) {
                new_vehicles.push(v);
            }
        }

        new_vehicles
    }

    fn spawn_one(&mut self, map: &MapData, od_model: &OdModel) -> Option<Vehicle> {
        let driver_profile = self.random_driver_profile();
        let vehicle_type = self.random_vehicle_type();

        // Sample personal_compliance and route_alpha from speed_config
        let personal_compliance =
            sample_compliance(driver_profile, &self.speed_config, &mut self.rng);
        let route_alpha = sample_route_alpha(driver_profile, &self.speed_config, &mut self.rng);

        // Decide spawn strategy
        let roll: f32 = self.rng.gen();
        let (from, to, trip_kind) = if roll < TRANSIT_FRACTION {
            // Pure transit: boundary → boundary
            self.transit_od(map)?
        } else if roll < TRANSIT_FRACTION + EXTERNAL_FRACTION {
            // External: boundary ↔ building
            self.external_od(map, od_model)?
        } else {
            // Local OD from OD model
            if od_model.is_empty() {
                // Fall back to random if no buildings
                self.random_od(map)?
            } else {
                let game_hour = 8.0f32; // Will be overridden by caller; default to morning rush
                od_model
                    .generate_od_pair(game_hour, &mut self.rng)
                    .or_else(|| self.random_od(map))?
            }
        };

        if from == to {
            return None;
        }

        let route = find_path(&map.graph, from, to, route_alpha, REF_SPEED_MS)?;
        if route.is_empty() {
            return None;
        }

        let node = &map.graph[from];
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);

        let mut vehicle = Vehicle::new(
            id,
            node.lat,
            node.lng,
            vehicle_type,
            driver_profile,
            route,
            personal_compliance,
            route_alpha,
            trip_kind as u8,
        );
        // Set the initial target lane based on the first planned turn.
        let initial_target = compute_vehicle_target_lane(&vehicle, map);
        vehicle.target_lane = initial_target;
        vehicle.current_lane = initial_target;
        vehicle.current_lateral_offset = initial_target as f32;
        vehicle.target_lateral_offset = initial_target as f32;
        let lane_route = build_lane_route_from_edge_route(map, &vehicle.route, initial_target);
        vehicle.current_lane_id = lane_route.first().copied();
        vehicle.lane_route = lane_route;
        vehicle.lane_route_pos = 0;
        vehicle.lane_progress_m = 0.0;

        // Give new vehicles a tiny initial speed so they are not all stuck
        // at edge_progress=0.0 simultaneously.  This ensures a non-zero gap
        // in the IDM leader bucket from the very first frame.
        vehicle.speed = 2.0; // 2 m/s ≈ 7 km/h — slow start

        Some(vehicle)
    }

    /// Tick with explicit game hour for OD model trip-type selection.
    pub fn tick_with_hour(
        &mut self,
        dt_real_s: f32,
        multiplier: f32,
        game_hour: f32,
        map: &MapData,
        od_model: &OdModel,
        current_vehicle_count: usize,
    ) -> Vec<Vehicle> {
        if current_vehicle_count >= self.max_vehicles {
            return Vec::new();
        }

        self.accumulator += self.base_rate * multiplier * dt_real_s;
        let to_spawn = self.accumulator.floor() as u32;
        self.accumulator -= to_spawn as f32;

        let cap = (self.max_vehicles - current_vehicle_count) as u32;
        let to_spawn = to_spawn.min(cap);

        let mut new_vehicles = Vec::with_capacity(to_spawn as usize);

        for _ in 0..to_spawn {
            if let Some(v) = self.spawn_one_with_hour(map, od_model, game_hour) {
                new_vehicles.push(v);
            }
        }

        new_vehicles
    }

    fn spawn_one_with_hour(
        &mut self,
        map: &MapData,
        od_model: &OdModel,
        game_hour: f32,
    ) -> Option<Vehicle> {
        let driver_profile = self.random_driver_profile();
        let vehicle_type = self.random_vehicle_type();
        let personal_compliance =
            sample_compliance(driver_profile, &self.speed_config, &mut self.rng);
        let route_alpha = sample_route_alpha(driver_profile, &self.speed_config, &mut self.rng);

        let roll: f32 = self.rng.gen();
        let (from, to, trip_kind) = if roll < TRANSIT_FRACTION {
            self.transit_od(map)?
        } else if roll < TRANSIT_FRACTION + EXTERNAL_FRACTION {
            self.external_od(map, od_model)?
        } else if !od_model.is_empty() {
            od_model
                .generate_od_pair(game_hour, &mut self.rng)
                .or_else(|| self.random_od(map))?
        } else {
            self.random_od(map)?
        };

        if from == to {
            return None;
        }

        let route = find_path(&map.graph, from, to, route_alpha, REF_SPEED_MS)?;
        if route.is_empty() {
            return None;
        }

        let node = &map.graph[from];
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);

        let mut vehicle = Vehicle::new(
            id,
            node.lat,
            node.lng,
            vehicle_type,
            driver_profile,
            route,
            personal_compliance,
            route_alpha,
            trip_kind as u8,
        );
        let initial_target = compute_vehicle_target_lane(&vehicle, map);
        vehicle.target_lane = initial_target;
        vehicle.current_lane = initial_target;
        vehicle.current_lateral_offset = initial_target as f32;
        vehicle.target_lateral_offset = initial_target as f32;
        let lane_route = build_lane_route_from_edge_route(map, &vehicle.route, initial_target);
        vehicle.current_lane_id = lane_route.first().copied();
        vehicle.lane_route = lane_route;
        vehicle.lane_route_pos = 0;
        vehicle.lane_progress_m = 0.0;
        vehicle.speed = 2.0; // slow start — prevents all spawned cars sharing edge_progress=0

        Some(vehicle)
    }

    // ── OD strategies ─────────────────────────────────────────────────────────

    fn transit_od(&mut self, map: &MapData) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.boundary_nodes.len() < 2 {
            return self
                .random_od(map)
                .map(|(f, t, _)| (f, t, TripKind::Transit));
        }
        let n = self.boundary_nodes.len();
        let fi = self.rng.gen_range(0..n);
        let mut ti = self.rng.gen_range(0..n);
        if ti == fi {
            ti = (ti + 1) % n;
        }
        Some((
            self.boundary_nodes[fi],
            self.boundary_nodes[ti],
            TripKind::Transit,
        ))
    }

    fn external_od(
        &mut self,
        map: &MapData,
        od_model: &OdModel,
    ) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.boundary_nodes.is_empty() {
            return self
                .random_od(map)
                .map(|(f, t, _)| (f, t, TripKind::ExternalInbound));
        }
        let boundary_idx = self.boundary_nodes[self.rng.gen_range(0..self.boundary_nodes.len())];

        if !od_model.is_empty() && self.rng.gen_bool(0.5) {
            // Inbound: boundary → building
            let building_node = od_model
                .buildings
                .iter()
                .filter(|b| b.access_node.is_some())
                .nth(self.rng.gen_range(0..od_model.buildings.len().max(1)))
                .and_then(|b| b.access_node)?;
            Some((boundary_idx, building_node, TripKind::ExternalInbound))
        } else {
            // Outbound: building → boundary  (or random → boundary when no OD model)
            let origin = if !od_model.is_empty() {
                od_model
                    .buildings
                    .iter()
                    .filter(|b| b.access_node.is_some())
                    .nth(self.rng.gen_range(0..od_model.buildings.len().max(1)))
                    .and_then(|b| b.access_node)
                    .unwrap_or(boundary_idx)
            } else {
                let n = self.boundary_nodes.len();
                if n == 0 {
                    return None;
                }
                self.boundary_nodes[self.rng.gen_range(0..n)]
            };
            Some((origin, boundary_idx, TripKind::ExternalOutbound))
        }
    }

    fn random_od(&mut self, map: &MapData) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.boundary_nodes.len() < 2 {
            return None;
        }
        let n = self.boundary_nodes.len();
        let fi = self.rng.gen_range(0..n);
        let mut ti = self.rng.gen_range(0..n);
        if ti == fi {
            ti = (ti + 1) % n;
        }
        let from = self.boundary_nodes[fi];
        let to = self.boundary_nodes[ti];
        if map.graph.node_weight(from).is_none() || map.graph.node_weight(to).is_none() {
            return None;
        }
        Some((from, to, TripKind::LocalOD))
    }

    // ── Randomisers ───────────────────────────────────────────────────────────

    fn random_vehicle_type(&mut self) -> VehicleType {
        if self.sandbox_mode {
            return VehicleType::Car;
        }
        let roll: f32 = self.rng.gen();
        if roll < 0.70 {
            VehicleType::Car
        } else if roll < 0.85 {
            VehicleType::Van
        } else if roll < 0.95 {
            VehicleType::Bus
        } else {
            VehicleType::Truck
        }
    }

    fn random_driver_profile(&mut self) -> DriverProfile {
        let roll: f32 = self.rng.gen();
        if roll < 0.70 {
            DriverProfile::Normal
        } else if roll < 0.85 {
            DriverProfile::Sunday
        } else if roll < 0.95 {
            DriverProfile::Pirat
        } else {
            DriverProfile::Cautious
        }
    }
}

fn build_lane_route_from_edge_route(
    map: &MapData,
    route: &[petgraph::graph::EdgeIndex],
    lane_idx: u8,
) -> Vec<LaneId> {
    let mut out = Vec::new();
    for edge in route {
        let mut candidates: Vec<&crate::map::road_network::Lane> = map
            .lanes
            .values()
            .filter(|l| l.edge_id == edge.index() as u64 && l.lane_index == lane_idx)
            .collect();
        if candidates.is_empty() {
            candidates = map
                .lanes
                .values()
                .filter(|l| l.edge_id == edge.index() as u64)
                .collect();
        }
        // Choose a lane that is actually reachable from previous lane via:
        //   prev -> chosen (direct) OR prev -> connector -> chosen.
        let chosen_opt = if let Some(prev) = out.last().copied() {
            let reachability_rank = |lane: &crate::map::road_network::Lane| -> i32 {
                let Some(prev_lane) = map.lanes.get(&prev) else {
                    return 3;
                };
                // Best case: direct continuation.
                if prev_lane.connections.contains(&lane.id) {
                    return 0;
                }
                // Otherwise look for a connector continuation.
                let via_connector = prev_lane.connections.iter().copied().any(|c| {
                    map.lanes
                        .get(&c)
                        .map(|cl| cl.connections.contains(&lane.id))
                        .unwrap_or(false)
                });
                if via_connector { 1 } else { 3 }
            };
            candidates
                .into_iter()
                .min_by_key(|l| (reachability_rank(l), l.lane_index))
        } else {
            candidates.into_iter().min_by_key(|l| l.lane_index)
        };

        if let Some(chosen) = chosen_opt {
            if let Some(prev) = out.last().copied() {
                // Direct prev -> chosen continuation exists.
                let direct = map
                    .lanes
                    .get(&prev)
                    .is_some_and(|l| l.connections.contains(&chosen.id));
                if direct {
                    out.push(chosen.id);
                    continue;
                }
                // If a connector exists from previous lane to this lane, include it.
                let connector = map.lanes.get(&prev).and_then(|l| {
                    l.connections.iter().copied().find(|c| {
                        map.lanes
                            .get(c)
                            .map(|cl| cl.connections.contains(&chosen.id))
                            .unwrap_or(false)
                    })
                });
                if let Some(conn) = connector {
                    out.push(conn);
                } else if map
                    .lanes
                    .get(&prev)
                    .is_some_and(|l| l.edge_id != chosen.edge_id)
                {
                    log::warn!(
                        "Lane route gap: missing connector lane {} -> {} (edge {} -> {})",
                        prev,
                        chosen.id,
                        map.lanes.get(&prev).map(|l| l.edge_id).unwrap_or(u64::MAX),
                        chosen.edge_id
                    );
                }
            }
            out.push(chosen.id);
        }
    }
    out
}

// ── Sampling helpers ─────────────────────────────────────────────────────────

/// Sample a personal compliance multiplier for `profile` from `SpeedConfig`.
pub fn sample_compliance(profile: DriverProfile, config: &SpeedConfig, rng: &mut impl Rng) -> f32 {
    let range = config.compliance_for(profile);
    let normal =
        Normal::new(0.0f32, config.noise_sigma).unwrap_or(Normal::new(0.0, 0.001).unwrap());
    let noise: f32 = normal.sample(rng);
    (range.base + noise).clamp(range.min, range.max)
}

/// Sample a route-alpha value for `profile` from `SpeedConfig`.
pub fn sample_route_alpha(profile: DriverProfile, config: &SpeedConfig, rng: &mut impl Rng) -> f32 {
    let (lo, hi) = config.route_alpha_range(profile);
    let base: f32 = rng.gen_range(lo..=hi);
    let normal =
        Normal::new(0.0f32, config.route.noise_sigma).unwrap_or(Normal::new(0.0, 0.001).unwrap());
    let noise: f32 = normal.sample(rng);
    (base + noise).clamp(0.0, 1.0)
}
