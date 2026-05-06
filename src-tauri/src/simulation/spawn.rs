use rand::Rng;
use rand::rngs::StdRng;
use rand::{SeedableRng};
use rand_distr::{Normal, Distribution};
use petgraph::graph::NodeIndex;

use crate::map::road_network::MapData;
use crate::simulation::od_model::{OdModel, TripKind};
use crate::simulation::pathfinding::{find_path, random_destination, REF_SPEED_MS};
use crate::simulation::speed_config::SpeedConfig;
use crate::simulation::lane_change::compute_vehicle_target_lane;
use crate::vehicles::vehicle::Vehicle;
use crate::vehicles::types::VehicleType;
use crate::vehicles::driver::DriverProfile;

/// Hard cap on total non-tram vehicles in the simulation.
const MAX_VEHICLES: usize = 500;

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
}

impl SpawnSystem {
    pub fn new(
        spawn_points: Vec<NodeIndex>,
        boundary_nodes: Vec<NodeIndex>,
        speed_config: SpeedConfig,
    ) -> Self {
        SpawnSystem {
            spawn_points,
            boundary_nodes,
            base_rate: 1.0,
            accumulator: 0.0,
            rng: StdRng::from_entropy(),
            next_id: 1,
            speed_config,
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
        if current_vehicle_count >= MAX_VEHICLES {
            return Vec::new();
        }

        self.accumulator += self.base_rate * multiplier * dt_real_s;
        let to_spawn = self.accumulator.floor() as u32;
        self.accumulator -= to_spawn as f32;

        let cap = (MAX_VEHICLES - current_vehicle_count) as u32;
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
        let vehicle_type   = self.random_vehicle_type();

        // Sample personal_compliance and route_alpha from speed_config
        let personal_compliance = sample_compliance(driver_profile, &self.speed_config, &mut self.rng);
        let route_alpha         = sample_route_alpha(driver_profile, &self.speed_config, &mut self.rng);

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
                od_model.generate_od_pair(game_hour, &mut self.rng)
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

        let node  = &map.graph[from];
        let id    = self.next_id;
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
        vehicle.target_lane  = initial_target;
        vehicle.current_lane = initial_target;

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
        if current_vehicle_count >= MAX_VEHICLES {
            return Vec::new();
        }

        self.accumulator += self.base_rate * multiplier * dt_real_s;
        let to_spawn = self.accumulator.floor() as u32;
        self.accumulator -= to_spawn as f32;

        let cap = (MAX_VEHICLES - current_vehicle_count) as u32;
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
        let driver_profile     = self.random_driver_profile();
        let vehicle_type       = self.random_vehicle_type();
        let personal_compliance = sample_compliance(driver_profile, &self.speed_config, &mut self.rng);
        let route_alpha         = sample_route_alpha(driver_profile, &self.speed_config, &mut self.rng);

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
        let id   = self.next_id;
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
        vehicle.target_lane  = initial_target;
        vehicle.current_lane = initial_target;

        Some(vehicle)
    }

    // ── OD strategies ─────────────────────────────────────────────────────────

    fn transit_od(&mut self, map: &MapData) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.boundary_nodes.len() < 2 {
            return self.random_od(map).map(|(f, t, _)| (f, t, TripKind::Transit));
        }
        let n = self.boundary_nodes.len();
        let fi = self.rng.gen_range(0..n);
        let mut ti = self.rng.gen_range(0..n);
        if ti == fi { ti = (ti + 1) % n; }
        Some((self.boundary_nodes[fi], self.boundary_nodes[ti], TripKind::Transit))
    }

    fn external_od(
        &mut self,
        map: &MapData,
        od_model: &OdModel,
    ) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.boundary_nodes.is_empty() {
            return self.random_od(map).map(|(f, t, _)| (f, t, TripKind::ExternalInbound));
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
                let n = self.spawn_points.len();
                if n == 0 { return None; }
                self.spawn_points[self.rng.gen_range(0..n)]
            };
            Some((origin, boundary_idx, TripKind::ExternalOutbound))
        }
    }

    fn random_od(&mut self, map: &MapData) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.spawn_points.is_empty() {
            return None;
        }
        let n = self.spawn_points.len();
        let from = self.spawn_points[self.rng.gen_range(0..n)];
        let to   = random_destination(&map.graph, from, &mut self.rng);
        if from == to { return None; }
        Some((from, to, TripKind::LocalOD))
    }

    // ── Randomisers ───────────────────────────────────────────────────────────

    fn random_vehicle_type(&mut self) -> VehicleType {
        let roll: f32 = self.rng.gen();
        if roll < 0.70      { VehicleType::Car   }
        else if roll < 0.85 { VehicleType::Van   }
        else if roll < 0.95 { VehicleType::Bus   }
        else                { VehicleType::Truck }
    }

    fn random_driver_profile(&mut self) -> DriverProfile {
        let roll: f32 = self.rng.gen();
        if roll < 0.70      { DriverProfile::Normal   }
        else if roll < 0.85 { DriverProfile::Sunday   }
        else if roll < 0.95 { DriverProfile::Pirat    }
        else                { DriverProfile::Cautious }
    }
}

// ── Sampling helpers ─────────────────────────────────────────────────────────

/// Sample a personal compliance multiplier for `profile` from `SpeedConfig`.
pub fn sample_compliance(
    profile: DriverProfile,
    config: &SpeedConfig,
    rng: &mut impl Rng,
) -> f32 {
    let range  = config.compliance_for(profile);
    let normal = Normal::new(0.0f32, config.noise_sigma).unwrap_or(Normal::new(0.0, 0.001).unwrap());
    let noise: f32 = normal.sample(rng);
    (range.base + noise).clamp(range.min, range.max)
}

/// Sample a route-alpha value for `profile` from `SpeedConfig`.
pub fn sample_route_alpha(
    profile: DriverProfile,
    config: &SpeedConfig,
    rng: &mut impl Rng,
) -> f32 {
    let (lo, hi) = config.route_alpha_range(profile);
    let base: f32 = rng.gen_range(lo..=hi);
    let normal = Normal::new(0.0f32, config.route.noise_sigma).unwrap_or(Normal::new(0.0, 0.001).unwrap());
    let noise: f32 = normal.sample(rng);
    (base + noise).clamp(0.0, 1.0)
}
