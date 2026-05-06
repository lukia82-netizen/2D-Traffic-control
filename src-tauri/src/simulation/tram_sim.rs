use petgraph::graph::NodeIndex;

use crate::map::road_network::haversine_distance_m;
use crate::map::tram_network::TramData;

// ── Tram physics constants ────────────────────────────────────────────────────

const TRAM_V0: f32       = 11.1;   // 40 km/h
const TRAM_A_MAX: f32    = 1.2;    // m/s²
const TRAM_B: f32        = 2.0;    // comfortable braking m/s²
/// Distance from next stop at which the tram begins braking [m]
const TRAM_BRAKE_START: f32 = 60.0;

// ── Tram vehicle ──────────────────────────────────────────────────────────────

/// A tram following a fixed stop sequence with IDM-simplified physics.
#[derive(Debug, Clone)]
pub struct TramVehicle {
    pub id: u32,
    pub lat: f64,
    pub lng: f64,
    pub angle: f32,
    pub speed: f32,
    pub line_ref: String,
    pub stop_sequence: Vec<NodeIndex>,
    /// Index of the current "from" stop in `stop_sequence`.
    pub current_stop_idx: usize,
    /// Progress 0..1 along the current segment (from → next stop).
    pub edge_progress: f32,
    pub edge_length_m: f32,
    /// Remaining dwell time at a stop in **game** seconds.
    pub dwell_timer: f32,
    pub is_dwelling: bool,
    /// Always `TripKind::Transit = 1`.
    pub trip_kind: u8,
    pub frustration: f32,
}

impl TramVehicle {
    fn new(
        id: u32,
        line_ref: String,
        stop_sequence: Vec<NodeIndex>,
        tram_data: &TramData,
    ) -> Self {
        let (lat, lng) = stop_sequence
            .first()
            .map(|&idx| {
                let n = &tram_data.graph[idx];
                (n.lat, n.lng)
            })
            .unwrap_or((0.0, 0.0));

        let edge_length_m = segment_length(&stop_sequence, 0, tram_data);

        TramVehicle {
            id,
            lat,
            lng,
            angle: 0.0,
            speed: 0.0,
            line_ref,
            stop_sequence,
            current_stop_idx: 0,
            edge_progress: 0.0,
            edge_length_m,
            dwell_timer: 30.0,
            is_dwelling: true,
            trip_kind: 1, // TripKind::Transit
            frustration: 0.0,
        }
    }

    /// Advance the tram by one physics step.
    pub fn advance(&mut self, real_dt_s: f32, game_dt_s: f32, tram_data: &TramData) {
        // Dwell at stop
        if self.is_dwelling {
            self.dwell_timer -= game_dt_s;
            if self.dwell_timer <= 0.0 {
                self.is_dwelling = false;
            }
            return;
        }

        let n_stops = self.stop_sequence.len();
        if n_stops < 2 {
            return;
        }

        let next_idx = (self.current_stop_idx + 1) % n_stops;
        let cur_node = &tram_data.graph[self.stop_sequence[self.current_stop_idx]];
        let nxt_node = &tram_data.graph[self.stop_sequence[next_idx]];

        // Remaining distance to the next stop
        let dist_to_stop = self.edge_length_m * (1.0 - self.edge_progress);

        // Desired speed: brake smoothly near the stop
        let v_desired = if dist_to_stop < TRAM_BRAKE_START {
            TRAM_V0 * (dist_to_stop / TRAM_BRAKE_START).sqrt().max(0.0)
        } else {
            TRAM_V0
        };

        // Proportional controller (simple, no full IDM needed for trams)
        let accel = (v_desired - self.speed) * 2.5;
        self.speed = (self.speed + accel.clamp(-TRAM_B, TRAM_A_MAX) * real_dt_s)
            .clamp(0.0, TRAM_V0);

        // Advance progress
        if self.edge_length_m > 0.0 {
            self.edge_progress += self.speed * real_dt_s / self.edge_length_m;
        }

        // Arrived at next stop
        if self.edge_progress >= 1.0 {
            self.edge_progress = 0.0;
            self.current_stop_idx = next_idx;
            self.is_dwelling = true;
            let dwell = tram_data.graph[self.stop_sequence[self.current_stop_idx]].stop_dwell_s;
            self.dwell_timer = dwell;
            self.edge_length_m = segment_length(&self.stop_sequence, self.current_stop_idx, tram_data);
            self.speed = 0.0;
        }

        // Interpolate geo position
        let t = self.edge_progress as f64;
        self.lat = cur_node.lat + (nxt_node.lat - cur_node.lat) * t;
        self.lng = cur_node.lng + (nxt_node.lng - cur_node.lng) * t;

        // Heading
        let dlng = nxt_node.lng - cur_node.lng;
        let dlat = nxt_node.lat - cur_node.lat;
        self.angle = (dlng as f32).atan2(dlat as f32);
    }
}

fn segment_length(stops: &[NodeIndex], from_idx: usize, tram_data: &TramData) -> f32 {
    if stops.len() < 2 {
        return 1.0;
    }
    let next_idx = (from_idx + 1) % stops.len();
    let a = &tram_data.graph[stops[from_idx]];
    let b = &tram_data.graph[stops[next_idx]];
    haversine_distance_m(a.lat, a.lng, b.lat, b.lng).max(1.0)
}

// ── Tram simulation manager ───────────────────────────────────────────────────

pub struct TramSim {
    pub trams: Vec<TramVehicle>,
    /// Next tram entity id if trams are spawned dynamically later.
    _next_id: u32,
}

impl TramSim {
    /// Spawn one tram per line starting from the first stop.
    pub fn new(tram_data: &TramData, id_offset: u32) -> Self {
        let mut trams = Vec::new();
        let mut next_id = id_offset;

        for line in &tram_data.lines {
            if line.stop_sequence.len() < 2 {
                continue;
            }
            trams.push(TramVehicle::new(
                next_id,
                line.line_ref.clone(),
                line.stop_sequence.clone(),
                tram_data,
            ));
            next_id = next_id.wrapping_add(1);
        }

        log::info!("TramSim: spawned {} trams", trams.len());
        TramSim { trams, _next_id: next_id }
    }

    pub fn is_empty(&self) -> bool {
        self.trams.is_empty()
    }

    /// Advance all trams by one tick.
    pub fn tick(&mut self, real_dt_s: f32, game_dt_s: f32, tram_data: &TramData) {
        for tram in &mut self.trams {
            tram.advance(real_dt_s, game_dt_s, tram_data);
        }
    }
}
