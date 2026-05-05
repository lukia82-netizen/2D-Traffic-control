use petgraph::graph::EdgeIndex;
use super::types::VehicleType;
use super::driver::DriverProfile;

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u32,
    pub lat: f64,
    pub lng: f64,
    /// Heading in radians (0 = east, PI/2 = north, etc.)
    pub angle: f32,
    /// Current speed m/s
    pub speed: f32,
    /// Target/desired speed m/s (road limit × driver factor)
    pub target_speed: f32,
    /// Current acceleration m/s²
    pub accel: f32,
    pub vehicle_type: VehicleType,
    pub driver_profile: DriverProfile,
    /// Driver satisfaction 0..=100
    pub satisfaction: f32,
    /// Accumulated real time stopped (for frustration threshold)
    pub wait_time_real_s: f32,
    /// Planned route as a sequence of edge indices
    pub route: Vec<EdgeIndex>,
    /// Index into `route` for the current edge
    pub route_pos: usize,
    /// Progress along current edge, 0.0 = start, 1.0 = end
    pub edge_progress: f32,
    pub current_lane: u8,
    pub target_lane: u8,
    /// Cooldown in real seconds before another lane change is allowed
    pub lane_change_cooldown: f32,
    /// Marks vehicle for removal (reached destination)
    pub despawned: bool,
}

impl Vehicle {
    pub fn new(
        id: u32,
        lat: f64,
        lng: f64,
        vehicle_type: VehicleType,
        driver_profile: DriverProfile,
        route: Vec<EdgeIndex>,
    ) -> Self {
        let target_speed = vehicle_type.params().max_speed
            * driver_profile.params().desired_speed_factor;

        Vehicle {
            id,
            lat,
            lng,
            angle: 0.0,
            speed: 0.0,
            target_speed,
            accel: 0.0,
            vehicle_type,
            driver_profile,
            satisfaction: 100.0,
            wait_time_real_s: 0.0,
            route,
            route_pos: 0,
            edge_progress: 0.0,
            current_lane: 0,
            target_lane: 0,
            lane_change_cooldown: 0.0,
            despawned: false,
        }
    }

    /// Returns true if the vehicle is stopped (very low speed)
    pub fn is_stopped(&self) -> bool {
        self.speed < 0.5
    }
}
