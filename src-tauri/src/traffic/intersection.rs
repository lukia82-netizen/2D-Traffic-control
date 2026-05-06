use std::collections::{HashMap, HashSet};
use crate::state::LightControlMode;
use crate::traffic::traffic_light::{TrafficLight, LightPhase, LightStateUpdate};
use crate::map::road_network::{RoadGraph, IntersectionType};

pub struct IntersectionManager {
    pub traffic_lights: HashMap<u64, TrafficLight>,
    /// OSM node IDs that carry a stop sign.
    pub stop_nodes: HashSet<u64>,
    /// OSM node IDs that carry a yield/give-way sign.
    pub yield_nodes: HashSet<u64>,
}

impl IntersectionManager {
    /// Build the manager by scanning all nodes with intersection-type tags.
    pub fn from_graph(graph: &RoadGraph) -> Self {
        let mut traffic_lights = HashMap::new();
        let mut stop_nodes     = HashSet::new();
        let mut yield_nodes    = HashSet::new();

        for node_idx in graph.node_indices() {
            let node = &graph[node_idx];
            match node.intersection_type {
                IntersectionType::TrafficLight => {
                    traffic_lights.insert(node.osm_id, TrafficLight::new(node.osm_id));
                }
                IntersectionType::Stop => {
                    stop_nodes.insert(node.osm_id);
                }
                IntersectionType::Yield => {
                    yield_nodes.insert(node.osm_id);
                }
                IntersectionType::PedestrianCrossing => {
                    traffic_lights.insert(
                        node.osm_id,
                        TrafficLight::new_pedestrian(node.osm_id),
                    );
                }
                IntersectionType::Plain | IntersectionType::Roundabout => {}
            }
        }

        IntersectionManager { traffic_lights, stop_nodes, yield_nodes }
    }

    /// Advance all traffic lights and collect phase-change events.
    pub fn update(&mut self, dt_real_s: f32) -> Vec<LightStateUpdate> {
        let mut updates = Vec::new();

        for tl in self.traffic_lights.values_mut() {
            let phase_before = tl.current_phase;
            tl.update(dt_real_s);
            if tl.current_phase != phase_before {
                updates.push(tl.to_state_update());
            }
        }

        updates
    }

    /// Returns `true` if there is no controlling signal at `intersection_id`
    /// (plain node or roundabout) or the traffic light is currently green.
    ///
    /// **Stop signs and yield signs require per-vehicle state – use
    /// `can_vehicle_proceed` instead.**
    pub fn can_proceed(&self, intersection_id: u64) -> bool {
        if let Some(tl) = self.traffic_lights.get(&intersection_id) {
            return tl.is_green();
        }
        true
    }

    /// Vehicle-aware right-of-way check.
    ///
    /// * **Traffic light** – blocked unless the light is green.
    /// * **Stop sign** – blocked until the vehicle has come to a full stop at the
    ///   sign (tracked via `vehicle_has_stopped_at_sign`).
    /// * **Yield sign** – never hard-blocked; IDM naturally slows vehicles when
    ///   crossing traffic is nearby (handled in `apply_intersection_effect`).
    /// * **Plain / Roundabout** – always `true`.
    pub fn can_vehicle_proceed(
        &self,
        osm_id: u64,
        vehicle_has_stopped_at_sign: bool,
    ) -> bool {
        if let Some(tl) = self.traffic_lights.get(&osm_id) {
            return tl.is_green();
        }
        if self.stop_nodes.contains(&osm_id) {
            return vehicle_has_stopped_at_sign;
        }
        true
    }

    /// Returns `true` when `osm_id` is a stop-sign node.
    pub fn is_stop_node(&self, osm_id: u64) -> bool {
        self.stop_nodes.contains(&osm_id)
    }

    /// Returns `true` when `osm_id` is a yield-sign node.
    pub fn is_yield_node(&self, osm_id: u64) -> bool {
        self.yield_nodes.contains(&osm_id)
    }

    pub fn set_mode(&mut self, intersection_id: u64, mode: LightControlMode) {
        if let Some(tl) = self.traffic_lights.get_mut(&intersection_id) {
            tl.set_mode(mode);
        }
    }

    pub fn set_phase(&mut self, intersection_id: u64, phase_byte: u8) {
        if let Some(tl) = self.traffic_lights.get_mut(&intersection_id) {
            tl.force_phase(LightPhase::from_u8(phase_byte));
        }
    }

    pub fn set_durations(&mut self, intersection_id: u64, green_s: f32, red_s: f32) {
        if let Some(tl) = self.traffic_lights.get_mut(&intersection_id) {
            tl.set_durations(green_s, red_s);
        }
    }

    /// Update queue count for adaptive mode (called from congestion monitor).
    pub fn update_queue(&mut self, intersection_id: u64, count: u32) {
        if let Some(tl) = self.traffic_lights.get_mut(&intersection_id) {
            tl.queue_count = count;
        }
    }

    pub fn all_state_updates(&self) -> Vec<LightStateUpdate> {
        self.traffic_lights
            .values()
            .map(|tl| tl.to_state_update())
            .collect()
    }
}
