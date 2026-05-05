use std::collections::HashMap;
use crate::state::LightControlMode;
use crate::traffic::traffic_light::{TrafficLight, LightPhase, LightStateUpdate};
use crate::map::road_network::{RoadGraph, IntersectionType};

pub struct IntersectionManager {
    pub traffic_lights: HashMap<u64, TrafficLight>,
}

impl IntersectionManager {
    /// Build the manager by scanning all nodes with TrafficLight intersection type.
    pub fn from_graph(graph: &RoadGraph) -> Self {
        let mut traffic_lights = HashMap::new();

        for node_idx in graph.node_indices() {
            let node = &graph[node_idx];
            if matches!(node.intersection_type, IntersectionType::TrafficLight) {
                let tl = TrafficLight::new(node.osm_id);
                traffic_lights.insert(node.osm_id, tl);
            }
        }

        IntersectionManager { traffic_lights }
    }

    /// Advance all traffic lights and collect any state changes.
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

    /// Check whether a vehicle approaching `intersection_id` is allowed to proceed.
    /// Returns `true` if the light is green or if there is no light at that node.
    pub fn can_proceed(&self, intersection_id: u64) -> bool {
        match self.traffic_lights.get(&intersection_id) {
            Some(tl) => tl.is_green(),
            None => true, // no signal → right-of-way by default
        }
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
