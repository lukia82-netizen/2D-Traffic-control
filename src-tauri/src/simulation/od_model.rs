use std::collections::HashMap;

use rand::Rng;
use petgraph::graph::NodeIndex;

use crate::map::building_loader::{BuildingType, OdBuilding};
use crate::map::road_network::haversine_distance_m;

/// Trips shorter than this are assumed to be made on foot → no vehicle spawned.
pub const WALK_THRESHOLD_M: f32 = 400.0;

// ── Trip taxonomy ────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripType {
    HomeToWork,
    WorkToHome,
    HomeToShop,
    ShopToHome,
    LunchTrip,
}

/// Trip kind value packed into the binary vehicle frame (1 byte at offset 22).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TripKind {
    LocalOD          = 0,
    Transit          = 1,
    ExternalInbound  = 2,
    ExternalOutbound = 3,
}

// ── OD assignment ────────────────────────────────────────────────────────────

/// Precomputed stable pairings: each residential building has a fixed "work"
/// and "shop" target generated once at simulation start.
pub struct OdAssignment {
    /// residential building id → index in buildings slice for work destination
    pub work: HashMap<u64, usize>,
    /// residential building id → index in buildings slice for shop destination
    pub shop: HashMap<u64, usize>,
}

impl OdAssignment {
    pub fn new(buildings: &[OdBuilding], rng: &mut impl Rng) -> Self {
        let residential: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Residential)
            .map(|(i, _)| i)
            .collect();

        let offices: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Office)
            .map(|(i, _)| i)
            .collect();

        let commercial: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Commercial)
            .map(|(i, _)| i)
            .collect();

        // Fallback when specific types are absent
        let non_residential: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type != BuildingType::Residential)
            .map(|(i, _)| i)
            .collect();

        let work_pool = if !offices.is_empty() { &offices } else { &non_residential };
        let shop_pool = if !commercial.is_empty() { &commercial } else { &non_residential };

        let mut work = HashMap::new();
        let mut shop = HashMap::new();

        if !work_pool.is_empty() && !shop_pool.is_empty() {
            for &res_idx in &residential {
                work.insert(buildings[res_idx].id, work_pool[rng.gen_range(0..work_pool.len())]);
                shop.insert(buildings[res_idx].id, shop_pool[rng.gen_range(0..shop_pool.len())]);
            }
        }

        OdAssignment { work, shop }
    }
}

// ── OD model ─────────────────────────────────────────────────────────────────

pub struct OdModel {
    pub buildings: Vec<OdBuilding>,
    pub assignments: OdAssignment,
    residential: Vec<usize>,
    offices: Vec<usize>,
    commercial: Vec<usize>,
    any_accessible: Vec<usize>,
}

impl OdModel {
    pub fn new(buildings: Vec<OdBuilding>, rng: &mut impl Rng) -> Self {
        let residential: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Residential && b.access_node.is_some())
            .map(|(i, _)| i)
            .collect();
        let offices: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Office && b.access_node.is_some())
            .map(|(i, _)| i)
            .collect();
        let commercial: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.building_type == BuildingType::Commercial && b.access_node.is_some())
            .map(|(i, _)| i)
            .collect();
        let any_accessible: Vec<usize> = buildings
            .iter()
            .enumerate()
            .filter(|(_, b)| b.access_node.is_some())
            .map(|(i, _)| i)
            .collect();

        let assignments = OdAssignment::new(&buildings, rng);
        OdModel { buildings, assignments, residential, offices, commercial, any_accessible }
    }

    pub fn is_empty(&self) -> bool {
        self.any_accessible.is_empty()
    }

    /// Choose an appropriate `TripType` for the current game hour.
    pub fn select_trip_type(game_hour: f32, rng: &mut impl Rng) -> TripType {
        if game_hour >= 6.0 && game_hour < 9.5 {
            TripType::HomeToWork
        } else if game_hour >= 16.0 && game_hour < 18.5 {
            TripType::WorkToHome
        } else if game_hour >= 12.0 && game_hour < 13.5 {
            if rng.gen_bool(0.4) {
                TripType::LunchTrip
            } else {
                TripType::HomeToShop
            }
        } else if (game_hour >= 10.0 && game_hour < 13.0)
            || (game_hour >= 17.0 && game_hour < 19.0)
        {
            if rng.gen_bool(0.5) { TripType::HomeToShop } else { TripType::ShopToHome }
        } else {
            // Off-peak: simple commute mix
            if rng.gen_bool(0.5) { TripType::HomeToWork } else { TripType::WorkToHome }
        }
    }

    /// Generate an OD pair for the given game hour.
    /// Returns `(origin_node, dest_node, TripKind)` or `None` if no valid pair.
    ///
    /// Endpoints are each building's road `access_node` (nearest graph vertex to the footprint).
    /// Those nodes are not restricted to the map bbox edge. For edge-only O/D, the spawn layer
    /// uses `MapData::boundary_nodes` (transit, random fallback, and one side of external trips).
    pub fn generate_od_pair(
        &self,
        game_hour: f32,
        rng: &mut impl Rng,
    ) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        if self.any_accessible.is_empty() {
            return None;
        }
        let trip_type = Self::select_trip_type(game_hour, rng);
        self.od_pair_for(trip_type, rng)
    }

    fn od_pair_for(
        &self,
        trip_type: TripType,
        rng: &mut impl Rng,
    ) -> Option<(NodeIndex, NodeIndex, TripKind)> {
        let (origin_pool, dest_pool) = match trip_type {
            TripType::HomeToWork => (&self.residential, &self.offices),
            TripType::WorkToHome => (&self.offices, &self.residential),
            TripType::HomeToShop | TripType::LunchTrip => (&self.residential, &self.commercial),
            TripType::ShopToHome => (&self.commercial, &self.residential),
        };

        // Fall back to any-accessible pool when specific types are scarce
        let origin_pool_eff = if origin_pool.is_empty() { &self.any_accessible } else { origin_pool };
        let dest_pool_eff   = if dest_pool.is_empty()   { &self.any_accessible } else { dest_pool };

        if origin_pool_eff.is_empty() || dest_pool_eff.is_empty() {
            return None;
        }

        let oi = origin_pool_eff[rng.gen_range(0..origin_pool_eff.len())];
        let di = dest_pool_eff  [rng.gen_range(0..dest_pool_eff.len())];
        if oi == di {
            return None;
        }

        let origin_b = &self.buildings[oi];
        let dest_b   = &self.buildings[di];

        // Pedestrian threshold: skip if origin and destination are close enough to walk
        let dist = haversine_distance_m(
            origin_b.centroid[1], origin_b.centroid[0],
            dest_b.centroid[1],   dest_b.centroid[0],
        );
        if dist < WALK_THRESHOLD_M {
            return None;
        }

        let origin = origin_b.access_node?;
        let dest   = dest_b.access_node?;
        if origin == dest {
            return None;
        }

        Some((origin, dest, TripKind::LocalOD))
    }
}
