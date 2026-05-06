use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum VehicleType {
    Car   = 0,
    Van   = 1,
    Bus   = 2,
    Truck = 3,
    Tram  = 4,
}

impl VehicleType {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => VehicleType::Van,
            2 => VehicleType::Bus,
            3 => VehicleType::Truck,
            4 => VehicleType::Tram,
            _ => VehicleType::Car,
        }
    }
}

#[derive(Debug, Clone)]
pub struct VehicleTypeParams {
    pub width_m: f32,
    pub length_m: f32,
    /// Maximum speed in m/s (city default)
    pub max_speed: f32,
    pub max_accel: f32,
    pub max_decel: f32,
    pub color_rgb: [u8; 3],
}

impl VehicleType {
    pub fn params(&self) -> VehicleTypeParams {
        match self {
            VehicleType::Car => VehicleTypeParams {
                width_m: 1.8,
                length_m: 4.5,
                max_speed: 50.0 / 3.6,   // ~13.89 m/s
                max_accel: 3.0,
                max_decel: 5.0,
                color_rgb: [70, 130, 180],   // steel blue
            },
            VehicleType::Van => VehicleTypeParams {
                width_m: 2.0,
                length_m: 6.0,
                max_speed: 40.0 / 3.6,   // ~11.11 m/s
                max_accel: 2.0,
                max_decel: 4.0,
                color_rgb: [218, 165, 32],   // goldenrod
            },
            VehicleType::Bus => VehicleTypeParams {
                width_m: 2.5,
                length_m: 12.0,
                max_speed: 30.0 / 3.6,   // ~8.33 m/s
                max_accel: 1.2,
                max_decel: 3.0,
                color_rgb: [255, 140, 0],    // dark orange
            },
            VehicleType::Truck => VehicleTypeParams {
                width_m: 2.6,
                length_m: 16.0,
                max_speed: 25.0 / 3.6,   // ~6.94 m/s
                max_accel: 0.8,
                max_decel: 2.0,
                color_rgb: [139, 0, 0],      // dark red
            },
            VehicleType::Tram => VehicleTypeParams {
                width_m: 2.4,
                length_m: 20.0,
                max_speed: 40.0 / 3.6,   // ~11.1 m/s
                max_accel: 1.2,
                max_decel: 2.0,
                color_rgb: [255, 215, 0],    // gold / yellow
            },
        }
    }
}
