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
    /// Smooth lateral position in lane-index units (0.0 = centre of lane 0,
    /// 1.0 = centre of lane 1, …).  Interpolates toward `target_lateral_offset`
    /// every tick to produce GTA-style glide instead of an instant jump.
    pub current_lateral_offset: f32,
    /// Desired lateral position (== target_lane as f32 once the lane is confirmed).
    pub target_lateral_offset: f32,

    // ── Traffic law compliance ───────────────────────────────────────────────

    /// Set to `true` once the vehicle has come to a full stop at a stop-sign
    /// node at the end of the current edge.  Reset when the vehicle advances
    /// to the next edge.
    pub has_stopped_at_stop_sign: bool,

    /// Set to `true` when the vehicle has reached its destination.
    pub despawned: bool,

    // ── Junction turn connector (Bezier) ────────────────────────────────────
    /// True while the vehicle follows an invisible connector curve through a turn.
    pub on_turn_connector: bool,
    /// Arc-length distance travelled along the connector curve (metres).
    pub turn_dist_m: f64,
    /// Entry progress on the current edge where the connector starts.
    pub turn_entry_progress: f32,
    /// Exit progress on the next edge where the connector ends.
    pub turn_exit_progress: f32,
    /// Approximate connector length in meters (for speed->t conversion).
    pub turn_length_m: f32,
    /// Quadratic Bezier start point (lat/lng).
    pub turn_p1_lat: f64,
    pub turn_p1_lng: f64,
    /// Quadratic Bezier control point (lat/lng).
    pub turn_ctrl_lat: f64,
    pub turn_ctrl_lng: f64,
    /// Quadratic Bezier end point (lat/lng).
    pub turn_p2_lat: f64,
    pub turn_p2_lng: f64,
    /// Edge index of incoming approach that started current connector traversal.
    pub turn_from_edge: usize,
    /// Edge index of outgoing edge targeted by current connector traversal.
    pub turn_to_edge: usize,
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
            current_lateral_offset: 0.0,
            target_lateral_offset: 0.0,
            has_stopped_at_stop_sign: false,
            despawned: false,
            on_turn_connector: false,
            turn_dist_m: 0.0,
            turn_entry_progress: 0.0,
            turn_exit_progress: 0.0,
            turn_length_m: 1.0,
            turn_p1_lat: lat,
            turn_p1_lng: lng,
            turn_ctrl_lat: lat,
            turn_ctrl_lng: lng,
            turn_p2_lat: lat,
            turn_p2_lng: lng,
            turn_from_edge: 0,
            turn_to_edge: 0,
        }
    }

    /// Returns true when the vehicle is effectively stopped.
    pub fn is_stopped(&self) -> bool {
        self.speed < 0.5
    }
}
