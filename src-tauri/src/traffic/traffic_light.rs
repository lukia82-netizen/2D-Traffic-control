use serde::{Deserialize, Serialize};
use crate::state::LightControlMode;

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
pub struct LightStateUpdate {
    pub intersection_id: u64,
    pub phase: u8,
    pub time_remaining: f32,
    /// Number of vehicles queued (relevant for Adaptive mode display).
    pub queue_count: u32,
    /// Current control mode as a string: "manual" | "semi_auto" | "auto" | "adaptive"
    pub mode: String,
    /// Configured green duration in seconds
    pub green_duration: f32,
    /// Configured red duration in seconds
    pub red_duration: f32,
}

#[derive(Debug, Clone)]
pub struct TrafficLight {
    pub intersection_id: u64,
    pub mode: LightControlMode,
    pub current_phase: LightPhase,
    /// Real seconds spent in current phase
    pub phase_timer: f32,
    pub green_duration: f32,
    /// Yellow is always ~3 seconds
    pub yellow_duration: f32,
    pub red_duration: f32,
    /// Number of vehicles queued (used for adaptive mode)
    pub queue_count: u32,
}

impl TrafficLight {
    pub fn new(intersection_id: u64) -> Self {
        TrafficLight {
            intersection_id,
            mode: LightControlMode::Auto,
            current_phase: LightPhase::Red,
            phase_timer: 0.0,
            green_duration: 30.0,
            yellow_duration: 3.0,
            red_duration: 30.0,
            queue_count: 0,
        }
    }

    /// Advance the traffic light FSM by `dt_real_s` real seconds.
    pub fn update(&mut self, dt_real_s: f32) {
        match self.mode {
            LightControlMode::Manual => {
                // Manual: do not auto-advance; player controls phase
            }
            LightControlMode::SemiAuto | LightControlMode::Auto => {
                self.phase_timer += dt_real_s;
                self.advance_phase_if_due();
            }
            LightControlMode::Adaptive => {
                self.phase_timer += dt_real_s;
                let effective_green = self.adaptive_green_duration();
                match self.current_phase {
                    LightPhase::Green => {
                        if self.phase_timer >= effective_green {
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

    fn advance_phase_if_due(&mut self) {
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

    fn adaptive_green_duration(&self) -> f32 {
        // Base 20s + up to 40s additional based on queue (saturates at ~20 vehicles)
        let extra = (self.queue_count as f32 / 20.0).min(1.0) * 40.0;
        (20.0 + extra).min(60.0)
    }

    /// Force a specific phase (for manual control).
    pub fn force_phase(&mut self, phase: LightPhase) {
        self.current_phase = phase;
        self.phase_timer = 0.0;
    }

    pub fn set_mode(&mut self, mode: LightControlMode) {
        self.mode = mode;
    }

    pub fn is_green(&self) -> bool {
        matches!(self.current_phase, LightPhase::Green)
    }

    pub fn is_red(&self) -> bool {
        matches!(self.current_phase, LightPhase::Red)
    }

    pub fn time_remaining(&self) -> f32 {
        match self.current_phase {
            LightPhase::Green => (self.green_duration - self.phase_timer).max(0.0),
            LightPhase::Yellow => (self.yellow_duration - self.phase_timer).max(0.0),
            LightPhase::Red => (self.red_duration - self.phase_timer).max(0.0),
        }
    }

    pub fn to_state_update(&self) -> LightStateUpdate {
        let mode_str = match self.mode {
            LightControlMode::Manual   => "manual",
            LightControlMode::SemiAuto => "semi_auto",
            LightControlMode::Auto     => "auto",
            LightControlMode::Adaptive => "adaptive",
        };
        LightStateUpdate {
            intersection_id: self.intersection_id,
            phase: self.current_phase.to_u8(),
            time_remaining: self.time_remaining(),
            queue_count: self.queue_count,
            mode: mode_str.to_string(),
            green_duration: self.green_duration,
            red_duration: self.red_duration,
        }
    }

    /// Set fixed phase durations (used by SemiAuto mode).
    pub fn set_durations(&mut self, green_s: f32, red_s: f32) {
        self.green_duration = green_s.max(5.0);
        self.red_duration = red_s.max(5.0);
    }
}
