use std::collections::{HashMap, HashSet};

use crate::map::road_network::{IntersectionType, MapData, RoadGraph};
use crate::state::LightControlMode;
use crate::traffic::phased_traffic_light::JunctionLayout;
use crate::traffic::traffic_light::{LightStateUpdate, TrafficLight};
use crate::vehicles::vehicle::Vehicle;

pub struct IntersectionManager {
    pub traffic_lights: HashMap<u64, TrafficLight>,
    /// OSM node IDs that carry a stop sign.
    pub stop_nodes: HashSet<u64>,
    /// OSM node IDs that carry a yield/give-way sign.
    pub yield_nodes: HashSet<u64>,
    last_light_broadcast_sig: HashMap<u64, u64>,
}

impl IntersectionManager {
    /// Build the manager by scanning all nodes with intersection-type tags.
    ///
    /// `sandbox_simple_cross_tl`: single-intersection sandbox — one TL, manual 2-phase (N–S / E–W, no lefts).
    pub fn from_graph(graph: &RoadGraph, sandbox_simple_cross_tl: bool) -> Self {
        let mut traffic_lights = HashMap::new();
        let mut stop_nodes     = HashSet::new();
        let mut yield_nodes    = HashSet::new();

        for node_idx in graph.node_indices() {
            let node = &graph[node_idx];
            match node.intersection_type {
                IntersectionType::TrafficLight => {
                    if let Some(layout) = JunctionLayout::build(graph, node_idx) {
                        if !layout.arms.is_empty() {
                            traffic_lights.insert(
                                node.osm_id,
                                TrafficLight::new_vehicle_multiphase(
                                    node.osm_id,
                                    layout,
                                    graph,
                                    node_idx,
                                    sandbox_simple_cross_tl,
                                ),
                            );
                        }
                    }
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

        IntersectionManager {
            traffic_lights,
            stop_nodes,
            yield_nodes,
            last_light_broadcast_sig: HashMap::new(),
        }
    }

    /// Advance all traffic lights and collect state broadcasts when timers or bulbs change.
    pub fn update(&mut self, dt_real_s: f32) -> Vec<LightStateUpdate> {
        for tl in self.traffic_lights.values_mut() {
            tl.update(dt_real_s);
        }

        let mut updates = Vec::new();
        for (id, tl) in self.traffic_lights.iter() {
            let sig = tl.broadcast_signature();
            if self.last_light_broadcast_sig.get(id).copied() != Some(sig) {
                self.last_light_broadcast_sig.insert(*id, sig);
                updates.push(tl.to_state_update());
            }
        }

        updates
    }

    /// Vehicle-aware right-of-way check.
    ///
    /// * **Traffic light** – lane / movement-aware (vehicle junctions share opposing greens across arms).
    /// * **Stop sign** – blocked until fully stopped (`vehicle_has_stopped_at_stop_sign`).
    /// * **Yield sign** – always `true` (yield handled in simulation).
    /// * **Plain / Roundabout** – always `true`.
    pub fn can_vehicle_proceed(
        &self,
        osm_id: u64,
        vehicle_has_stopped_at_stop_sign: bool,
        vehicle: &Vehicle,
        map: &MapData,
    ) -> bool {
        if let Some(tl) = self.traffic_lights.get(&osm_id) {
            return tl.allows_vehicle(vehicle, map);
        }
        if self.stop_nodes.contains(&osm_id) {
            return vehicle_has_stopped_at_stop_sign;
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
            tl.force_phase_cmd(phase_byte);
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
            if let Some(q) = tl.queue_count_mut() {
                *q = count;
            }
        }
    }

    pub fn all_state_updates(&self) -> Vec<LightStateUpdate> {
        self.traffic_lights.values().map(TrafficLight::to_state_update).collect()
    }
}
