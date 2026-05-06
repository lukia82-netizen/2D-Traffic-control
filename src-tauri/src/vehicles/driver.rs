use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum DriverProfile {
    Normal = 0,
    Sunday = 1,
    Pirat = 2,
    Cautious = 3,
}

impl DriverProfile {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => DriverProfile::Sunday,
            2 => DriverProfile::Pirat,
            3 => DriverProfile::Cautious,
            _ => DriverProfile::Normal,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DriverParams {
    /// Multiplier on road max_speed for desired speed
    pub desired_speed_factor: f32,
    /// Time headway T in IDM (seconds)
    pub time_headway: f32,
    /// Minimum gap s0 in IDM (meters)
    pub min_gap: f32,
    /// Comfortable acceleration a in IDM (m/s²)
    pub comfort_accel: f32,
    /// Comfortable deceleration b in IDM (m/s²)
    pub comfort_decel: f32,
    /// Real seconds before frustration starts accumulating
    pub wait_threshold_real_s: f32,
    /// Satisfaction points lost per real second when frustrated (stopped too long)
    pub frustration_decay_rate: f32,
    /// Satisfaction points recovered per real second when moving
    pub recovery_rate: f32,
}

impl DriverProfile {
    pub fn params(&self) -> DriverParams {
        match self {
            DriverProfile::Normal => DriverParams {
                desired_speed_factor: 1.0,
                time_headway: 1.5,
                min_gap: 10.0,
                comfort_accel: 1.5,
                comfort_decel: 2.0,
                wait_threshold_real_s: 45.0,
                frustration_decay_rate: 1.0,
                recovery_rate: 0.5,
            },
            DriverProfile::Sunday => DriverParams {
                desired_speed_factor: 0.8,
                time_headway: 2.5,
                min_gap: 10.0,
                comfort_accel: 0.8,
                comfort_decel: 1.5,
                wait_threshold_real_s: 90.0,
                frustration_decay_rate: 0.4,
                recovery_rate: 0.3,
            },
            DriverProfile::Pirat => DriverParams {
                desired_speed_factor: 1.4,
                time_headway: 0.8,
                min_gap: 8.0,
                comfort_accel: 3.0,
                comfort_decel: 5.0,
                wait_threshold_real_s: 15.0,
                frustration_decay_rate: 3.0,
                recovery_rate: 1.5,
            },
            DriverProfile::Cautious => DriverParams {
                desired_speed_factor: 0.9,
                time_headway: 2.0,
                min_gap: 12.0,
                comfort_accel: 1.0,
                comfort_decel: 2.5,
                wait_threshold_real_s: 60.0,
                frustration_decay_rate: 0.7,
                recovery_rate: 0.4,
            },
        }
    }
}
