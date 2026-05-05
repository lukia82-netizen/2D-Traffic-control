use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use petgraph::graph::NodeIndex;

use crate::map::road_network::MapData;
use crate::vehicles::vehicle::Vehicle;
use crate::vehicles::types::VehicleType;
use crate::vehicles::driver::DriverProfile;
use crate::simulation::pathfinding::{find_path, random_destination};

/// Hard cap on total vehicles in the simulation.
/// Keeps the demo area (2 km²) playable at 60 fps.
const MAX_VEHICLES: usize = 150;

pub struct SpawnSystem {
    pub spawn_points: Vec<NodeIndex>,
    /// Vehicles per real second at spawn multiplier = 1.0
    pub base_rate: f32,
    /// Fractional vehicle accumulator
    pub accumulator: f32,
    pub rng: StdRng,
    pub next_id: u32,
}

impl SpawnSystem {
    pub fn new(spawn_points: Vec<NodeIndex>) -> Self {
        SpawnSystem {
            spawn_points,
            base_rate: 0.5, // vehicles/real_second at multiplier=1.0
            accumulator: 0.0,
            rng: StdRng::from_entropy(),
            next_id: 1,
        }
    }

    /// Tick the spawn system and return any new vehicles to add this frame.
    /// `current_vehicle_count` is used to enforce the global vehicle cap.
    pub fn tick(
        &mut self,
        dt_real_s: f32,
        multiplier: f32,
        map: &MapData,
        current_vehicle_count: usize,
    ) -> Vec<Vehicle> {
        if self.spawn_points.is_empty() || current_vehicle_count >= MAX_VEHICLES {
            return Vec::new();
        }

        self.accumulator += self.base_rate * multiplier * dt_real_s;
        let to_spawn = self.accumulator.floor() as u32;
        self.accumulator -= to_spawn as f32;

        // Never exceed the cap in a single tick
        let to_spawn = to_spawn.min((MAX_VEHICLES - current_vehicle_count) as u32);

        let mut new_vehicles = Vec::with_capacity(to_spawn as usize);

        for _ in 0..to_spawn {
            if let Some(vehicle) = self.spawn_one(map) {
                new_vehicles.push(vehicle);
            }
        }

        new_vehicles
    }

    fn spawn_one(&mut self, map: &MapData) -> Option<Vehicle> {
        let spawn_count = self.spawn_points.len();
        let spawn_idx = self.rng.gen_range(0..spawn_count);
        let from = self.spawn_points[spawn_idx];

        let to = random_destination(&map.graph, from, &mut self.rng);
        if to == from {
            return None;
        }

        let route = find_path(&map.graph, from, to)?;
        if route.is_empty() {
            return None;
        }

        let node = &map.graph[from];
        let vehicle_type = self.random_vehicle_type();
        let driver_profile = self.random_driver_profile();

        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);

        Some(Vehicle::new(
            id,
            node.lat,
            node.lng,
            vehicle_type,
            driver_profile,
            route,
        ))
    }

    fn random_vehicle_type(&mut self) -> VehicleType {
        // 70% Car, 15% Van, 10% Bus, 5% Truck
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
        // 70% Normal, 15% Sunday, 10% Pirat, 5% Cautious
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
