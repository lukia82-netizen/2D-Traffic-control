use petgraph::graph::EdgeIndex;
use super::types::VehicleType;
use super::driver::DriverProfile;

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u32,
    pub lat: f64,
    pub lng: f64,
    /// Heading in radians (east = 0, north = π/2 etc.)
    pub angle: f32,
    /// Current speed m/s
    pub speed: f32,
    /// Current acceleration m/s²
    pub accel: f32,
    pub vehicle_type: VehicleType,
    pub driver_profile: DriverProfile,

    // ── Per-vehicle compliance & routing (sampled once at spawn) ────────────

    /// Speed compliance multiplier: personal_compliance × edge.max_speed = v₀.
    /// Sampled from SpeedConfig.compliance_for(profile) + noise at spawn.
    pub personal_compliance: f32,

    /// Route-planning preference 0 = shortest, 1 = fastest.
    /// Sampled once at spawn; only used by A* to choose the route.
    pub route_alpha: f32,

    /// Trip classification packed into the binary frame (offset 22).
    /// 0=local_od, 1=transit, 2=ext_inbound, 3=ext_outbound
    pub trip_kind: u8,

    // ── Frustration (replaces satisfaction; higher = worse) ─────────────────

    /// Driver frustration 0 (calm) … 100 (rage quit). Sent in binary frame.
    pub frustration: f32,
    /// Accumulated real seconds standing still (speed < 0.5 m/s).
    pub standstill_time_real_s: f32,
    /// Accumulated real seconds crawling (0.5..crawl_threshold m/s).
    pub crawl_time_real_s: f32,
    /// Last intersection OSM id where the vehicle stopped.
    pub last_intersection_id: Option<u64>,
    /// Number of consecutive red-light cycles at `last_intersection_id`.
    pub same_intersection_stops: u8,

    // ── Route ────────────────────────────────────────────────────────────────

    /// Planned route as a sequence of edge indices.
    pub route: Vec<EdgeIndex>,
    /// Index into `route` for the edge currently being traversed.
    pub route_pos: usize,
    /// Progress along the current edge: 0.0 = start, 1.0 = end.
    pub edge_progress: f32,

    // ── Lane management ──────────────────────────────────────────────────────

    pub current_lane: u8,
    pub target_lane: u8,
    /// Cooldown in real seconds before the next lane change is allowed.
    pub lane_change_cooldown: f32,

    /// Set to `true` when the vehicle has reached its destination.
    pub despawned: bool,
}

impl Vehicle {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: u32,
        lat: f64,
        lng: f64,
        vehicle_type: VehicleType,
        driver_profile: DriverProfile,
        route: Vec<EdgeIndex>,
        personal_compliance: f32,
        route_alpha: f32,
        trip_kind: u8,
    ) -> Self {
        Vehicle {
            id,
            lat,
            lng,
            angle: 0.0,
            speed: 0.0,
            accel: 0.0,
            vehicle_type,
            driver_profile,
            personal_compliance,
            route_alpha,
            trip_kind,
            frustration: 0.0,
            standstill_time_real_s: 0.0,
            crawl_time_real_s: 0.0,
            last_intersection_id: None,
            same_intersection_stops: 0,
            route,
            route_pos: 0,
            edge_progress: 0.0,
            current_lane: 0,
            target_lane: 0,
            lane_change_cooldown: 0.0,
            despawned: false,
        }
    }

    /// Returns true when the vehicle is effectively stopped.
    pub fn is_stopped(&self) -> bool {
        self.speed < 0.5
    }
}
