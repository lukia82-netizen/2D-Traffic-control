use serde::{Deserialize, Serialize};

use petgraph::graph::EdgeIndex;

use crate::map::road_network::{LaneDirection, MapData};
use crate::state::LightControlMode;
use crate::traffic::phased_traffic_light::{JunctionLayout, PhasedVehicleLight};
use crate::vehicles::vehicle::Vehicle;

/// Force next program step for multi-phase vehicle junctions (manual control).
pub const TL_CMD_ADVANCE_STEP: u8 = 250;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum LightPhase {
    Red = 0,
    Yellow = 1,
    Green = 2,
}

impl LightPhase {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => LightPhase::Yellow,
            2 => LightPhase::Green,
            _ => LightPhase::Red,
        }
    }

    pub fn to_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LightStateUpdate {
    pub intersection_id: u64,
    pub phase: u8,
    pub time_remaining: f32,
    /// Number of vehicles queued (relevant for Adaptive mode display).
    pub queue_count: u32,
    /// Current control mode as a string: "manual" | "semi_auto" | "auto" | "adaptive"
    pub mode: String,
    /// Configured straight / main green duration (seconds)
    pub green_duration: f32,
    /// Configured protected-left / secondary green duration (seconds)
    pub red_duration: f32,
    /// Per inbound-arm signal (0=R, 1=Y, 2=G); order matches clockwise arm sort (vehicle TL only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub junction_arm_phases: Option<Vec<u8>>,
}

/// Simple three-state lamp for pedestrian crossings (cars vs peds alternating).
#[derive(Debug, Clone)]
pub struct PedestrianCrossingLight {
    pub mode: LightControlMode,
    pub current_phase: LightPhase,
    /// Real seconds spent in current phase
    pub phase_timer: f32,
    pub green_duration: f32,
    pub yellow_duration: f32,
    pub red_duration: f32,
    pub queue_count: u32,
}

impl PedestrianCrossingLight {
    pub fn new(intersection_id: u64, green_s: f32, yellow_s: f32, red_s: f32) -> Self {
        let mut slf = Self {
            mode: LightControlMode::Auto,
            current_phase: LightPhase::Red,
            phase_timer: 0.0,
            green_duration: green_s.max(5.0),
            yellow_duration: yellow_s.max(2.0),
            red_duration: red_s.max(5.0),
            queue_count: 0,
        };
        Self::seed_start_offset(intersection_id, &mut slf);
        slf
    }

    fn seed_start_offset(intersection_id: u64, slf: &mut Self) {
        let cycle = slf.green_duration + slf.yellow_duration + slf.red_duration;
        let seed = intersection_id
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let offset = (seed % 1_000_000) as f32 / 1_000_000.0 * cycle;
        if offset < slf.green_duration {
            slf.current_phase = LightPhase::Green;
            slf.phase_timer = offset;
        } else if offset < slf.green_duration + slf.yellow_duration {
            slf.current_phase = LightPhase::Yellow;
            slf.phase_timer = offset - slf.green_duration;
        } else {
            slf.current_phase = LightPhase::Red;
            slf.phase_timer = offset - slf.green_duration - slf.yellow_duration;
        }
    }

    pub fn update(&mut self, dt_real_s: f32) {
        match self.mode {
            LightControlMode::Manual => {}
            LightControlMode::SemiAuto | LightControlMode::Auto => {
                self.phase_timer += dt_real_s;
                self.advance_phase_if_due_standard();
            }
            LightControlMode::Adaptive => {
                self.phase_timer += dt_real_s;
                match self.current_phase {
                    LightPhase::Green => {
                        if self.phase_timer >= self.adaptive_green_duration() {
                            self.transition_to(LightPhase::Yellow);
                        }
                    }
                    LightPhase::Yellow => {
                        if self.phase_timer >= self.yellow_duration {
                            self.transition_to(LightPhase::Red);
                        }
                    }
                    LightPhase::Red => {
                        if self.phase_timer >= self.red_duration {
                            self.transition_to(LightPhase::Green);
                        }
                    }
                }
            }
        }
    }

    fn adaptive_green_duration(&self) -> f32 {
        let extra = (self.queue_count as f32 / 20.0).min(1.0) * 40.0;
        (20.0 + extra).min(60.0)
    }

    fn advance_phase_if_due_standard(&mut self) {
        match self.current_phase {
            LightPhase::Green => {
                if self.phase_timer >= self.green_duration {
                    self.transition_to(LightPhase::Yellow);
                }
            }
            LightPhase::Yellow => {
                if self.phase_timer >= self.yellow_duration {
                    self.transition_to(LightPhase::Red);
                }
            }
            LightPhase::Red => {
                if self.phase_timer >= self.red_duration {
                    self.transition_to(LightPhase::Green);
                }
            }
        }
    }

    fn transition_to(&mut self, phase: LightPhase) {
        self.current_phase = phase;
        self.phase_timer = 0.0;
    }

    #[inline]
    pub fn force_phase(&mut self, phase: LightPhase) {
        self.current_phase = phase;
        self.phase_timer = 0.0;
    }

    #[inline]
    pub fn time_remaining(&self) -> f32 {
        match self.current_phase {
            LightPhase::Green => (self.green_duration - self.phase_timer).max(0.0),
            LightPhase::Yellow => (self.yellow_duration - self.phase_timer).max(0.0),
            LightPhase::Red => (self.red_duration - self.phase_timer).max(0.0),
        }
    }

    #[inline]
    pub fn broadcast_signature(&self) -> u64 {
        let tr = self.time_remaining();
        (self.current_phase as u64) << 40 | (((tr * 5.0) as u64) & 0xFFFF)
    }

    pub fn set_durations(&mut self, green_s: f32, red_s: f32) {
        self.green_duration = green_s.max(5.0);
        self.red_duration = red_s.max(5.0);
    }

    pub fn set_mode(&mut self, mode: LightControlMode) {
        self.mode = mode;
    }

    pub fn to_state_update(&self, intersection_id: u64) -> LightStateUpdate {
        let mode_str = match self.mode {
            LightControlMode::Manual   => "manual",
            LightControlMode::SemiAuto => "semi_auto",
            LightControlMode::Auto     => "auto",
            LightControlMode::Adaptive => "adaptive",
        };
        LightStateUpdate {
            intersection_id,
            phase: self.current_phase.to_u8(),
            time_remaining: self.time_remaining(),
            queue_count: self.queue_count,
            mode: mode_str.to_string(),
            green_duration: self.green_duration,
            red_duration: self.red_duration,
            junction_arm_phases: None,
        }
    }
}

#[derive(Debug)]
pub enum TrafficLightKind {
    Pedestrian(PedestrianCrossingLight),
    VehiclePhased(PhasedVehicleLight),
}

pub struct TrafficLight {
    pub intersection_id: u64,
    pub kind: TrafficLightKind,
}

impl TrafficLight {
    /// Pedestrian crossing with short driver cycle vs pedestrian time.
    pub fn new_pedestrian(intersection_id: u64) -> Self {
        TrafficLight {
            intersection_id,
            kind: TrafficLightKind::Pedestrian(PedestrianCrossingLight::new(
                intersection_id,
                25.0,
                3.0,
                15.0,
            )),
        }
    }

    pub fn new_vehicle_multiphase(intersection_id: u64, layout: JunctionLayout) -> Self {
        TrafficLight {
            intersection_id,
            kind: TrafficLightKind::VehiclePhased(PhasedVehicleLight::new(intersection_id, layout)),
        }
    }

    pub fn update(&mut self, dt_real_s: f32) {
        match &mut self.kind {
            TrafficLightKind::Pedestrian(p) => p.update(dt_real_s),
            TrafficLightKind::VehiclePhased(ph) => ph.update(dt_real_s),
        }
    }

    /// Vehicle junction: Green or Yellow for the active movement. Pedestrian crossing: **green only**
    /// for cars (matches legacy behaviour; yellow blocks the stop line).
    pub fn allows_vehicle(&self, vehicle: &Vehicle, map: &MapData) -> bool {
        match &self.kind {
            TrafficLightKind::Pedestrian(p) => matches!(p.current_phase, LightPhase::Green),
            TrafficLightKind::VehiclePhased(ph) => {
                if vehicle.route_pos >= vehicle.route.len() {
                    return true;
                }
                let e: EdgeIndex = vehicle.route[vehicle.route_pos];
                let Some(edge) = map.graph.edge_weight(e) else {
                    return true;
                };
                let lane = vehicle.current_lane as usize;
                let dir = edge.lane_directions.get(lane).copied().unwrap_or(LaneDirection::Straight);
                let sig = ph.signal_for(e, dir);
                matches!(sig, LightPhase::Green | LightPhase::Yellow)
            }
        }
    }

    #[inline]
    pub fn broadcast_signature(&self) -> u64 {
        match &self.kind {
            TrafficLightKind::Pedestrian(p) => p.broadcast_signature(),
            TrafficLightKind::VehiclePhased(ph) => ph.broadcast_signature(),
        }
    }

    pub fn to_state_update(&self) -> LightStateUpdate {
        match &self.kind {
            TrafficLightKind::Pedestrian(p) => p.to_state_update(self.intersection_id),
            TrafficLightKind::VehiclePhased(ph) => ph.to_state_update(),
        }
    }

    #[inline]
    pub fn set_mode(&mut self, mode: LightControlMode) {
        match &mut self.kind {
            TrafficLightKind::Pedestrian(p) => p.set_mode(mode),
            TrafficLightKind::VehiclePhased(ph) => ph.set_mode(mode),
        }
    }

    pub fn set_durations(&mut self, green_s: f32, red_s: f32) {
        match &mut self.kind {
            TrafficLightKind::Pedestrian(p) => p.set_durations(green_s, red_s),
            TrafficLightKind::VehiclePhased(ph) => ph.set_durations(green_s, red_s),
        }
    }

    pub fn force_phase_cmd(&mut self, phase_byte: u8) {
        match &mut self.kind {
            TrafficLightKind::Pedestrian(p) => {
                p.force_phase(LightPhase::from_u8(phase_byte));
            }
            TrafficLightKind::VehiclePhased(ph) => {
                if phase_byte >= TL_CMD_ADVANCE_STEP {
                    ph.force_advance_step();
                }
            }
        }
    }

    pub fn queue_count_mut(&mut self) -> Option<&mut u32> {
        match &mut self.kind {
            TrafficLightKind::Pedestrian(p) => Some(&mut p.queue_count),
            TrafficLightKind::VehiclePhased(ph) => Some(&mut ph.queue_count),
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub fn is_pedestrian(&self) -> bool {
        matches!(self.kind, TrafficLightKind::Pedestrian(..))
    }
}
