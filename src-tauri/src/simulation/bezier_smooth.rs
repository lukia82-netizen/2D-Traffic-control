//! Smooth car movement along a quadratic Bézier curve using:
//!
//! * **Arc-length parameterization** via a Look-Up Table (LUT) so the car
//!   always moves at a constant linear speed regardless of control-point geometry.
//! * **Fixed-timestep physics** (dt = 1/60 s) with a time accumulator to
//!   decouple physics from frame rate.
//! * **Linear interpolation** (lerp) of the visual state between the last two
//!   physics snapshots, eliminating jitter on high-refresh-rate displays.
//!
//! # Typical usage
//!
//! ```rust,no_run
//! use glam::DVec2;
//! use traffic_control_lib::simulation::bezier_smooth::{BezierPath, Simulation};
//! use std::time::Instant;
//!
//! let path = BezierPath::new(
//!     DVec2::new(0.0, 0.0),
//!     DVec2::new(50.0, 100.0),
//!     DVec2::new(100.0, 0.0),
//! );
//! let mut sim = Simulation::new(path, 15.0); // 15 world-units/s
//!
//! let mut accumulator = 0.0f64;
//! let mut last = Instant::now();
//!
//! loop {
//!     let elapsed = last.elapsed().as_secs_f64().min(0.25);
//!     last = Instant::now();
//!     accumulator = sim.tick(elapsed, accumulator);
//!     let rs = sim.render_state(accumulator);
//!     // pass rs.position / rs.rotation to your renderer
//!     break; // remove in real usage
//! }
//! ```

use std::f64::consts::{PI, TAU};
use glam::DVec2;
use lyon_geom::{QuadraticBezierSegment, Point, Vector};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Number of samples used to build the arc-length LUT.
/// 512 gives sub-millimetre accuracy for curves up to ~500 m long.
const LUT_SAMPLES: usize = 512;

/// Fixed physics time-step in seconds (1/60 s ≈ 16.667 ms).
pub const PHYSICS_DT: f64 = 1.0 / 60.0;

// ---------------------------------------------------------------------------
// BezierPath
// ---------------------------------------------------------------------------

/// A quadratic Bézier curve with pre-computed arc-length parameterization.
///
/// The LUT maps a uniform parameter index to cumulative arc length, allowing
/// `get_state` to invert the mapping and retrieve the curve point that lies at
/// any requested distance from the start with O(log N) cost.
pub struct BezierPath {
    /// Underlying curve stored as a `lyon_geom` segment for precise geometry.
    segment: QuadraticBezierSegment<f64>,
    /// `lut[i]` = cumulative arc length at `t = i / (LUT_SAMPLES - 1)`.
    lut: Vec<f64>,
    /// Total arc length of the curve (metres / world units).
    pub total_length: f64,
}

impl BezierPath {
    /// Build a new path from three control points and pre-compute the LUT.
    pub fn new(p0: DVec2, ctrl: DVec2, p2: DVec2) -> Self {
        let segment = QuadraticBezierSegment {
            from: Point::new(p0.x, p0.y),
            ctrl: Point::new(ctrl.x, ctrl.y),
            to:   Point::new(p2.x, p2.y),
        };
        let mut path = BezierPath {
            segment,
            lut: Vec::with_capacity(LUT_SAMPLES),
            total_length: 0.0,
        };
        path.build_lut();
        path
    }

    // ── Internal helpers ────────────────────────────────────────────────────

    /// Evaluate the curve at parameter `t ∈ [0, 1]` and return a `DVec2`.
    #[inline]
    fn eval(&self, t: f64) -> DVec2 {
        let p: Point<f64> = self.segment.sample(t);
        DVec2::new(p.x, p.y)
    }

    /// Return the tangent vector (first derivative) at `t`. Not normalised.
    #[inline]
    fn tangent(&self, t: f64) -> DVec2 {
        let v: Vector<f64> = self.segment.derivative(t);
        DVec2::new(v.x, v.y)
    }

    /// Build the arc-length LUT using `LUT_SAMPLES` uniform `t` samples.
    fn build_lut(&mut self) {
        self.lut.clear();
        let mut total = 0.0f64;
        let mut prev  = self.eval(0.0);
        self.lut.push(0.0); // t=0 → distance 0

        for i in 1..LUT_SAMPLES {
            let t    = i as f64 / (LUT_SAMPLES - 1) as f64;
            let curr = self.eval(t);
            total   += curr.distance(prev);
            self.lut.push(total);
            prev = curr;
        }
        self.total_length = total;
    }

    /// Convert an arc-length `distance` to the corresponding Bézier parameter
    /// `t ∈ [0, 1]` using binary search + linear interpolation.
    fn distance_to_t(&self, distance: f64) -> f64 {
        let d = distance.clamp(0.0, self.total_length);

        // `partition_point` returns the first index whose LUT value is ≥ d.
        let hi = self.lut.partition_point(|&v| v < d);

        if hi == 0            { return 0.0; }
        if hi >= LUT_SAMPLES  { return 1.0; }

        let lo   = hi - 1;
        let d_lo = self.lut[lo];
        let d_hi = self.lut[hi];
        let frac = if (d_hi - d_lo).abs() < 1e-14 {
            0.0
        } else {
            (d - d_lo) / (d_hi - d_lo)
        };

        let t_lo = lo as f64 / (LUT_SAMPLES - 1) as f64;
        let t_hi = hi as f64 / (LUT_SAMPLES - 1) as f64;
        t_lo + frac * (t_hi - t_lo)
    }

    // ── Public API ──────────────────────────────────────────────────────────

    /// Return the car state (position + heading angle) at the given
    /// arc-length `distance` from the curve start.
    ///
    /// Uses the tangent vector (analytical first derivative) for the angle so
    /// there is no finite-difference error in the heading.
    pub fn get_state(&self, distance: f64) -> CarState {
        let t        = self.distance_to_t(distance);
        let position = self.eval(t);
        let tan      = self.tangent(t);

        // atan2(y, x) — standard math convention; adapt to your coordinate system.
        let rotation = if tan.length_squared() > 1e-20 {
            tan.y.atan2(tan.x)
        } else {
            // Fallback: use the chord direction when the tangent is degenerate.
            let p_end = self.eval((t + 1e-4).min(1.0));
            (p_end - position).y.atan2((p_end - position).x)
        };

        CarState { position, rotation }
    }
}

// ---------------------------------------------------------------------------
// CarState
// ---------------------------------------------------------------------------

/// A snapshot of the car's visual state (position + heading).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarState {
    /// World-space position.
    pub position: DVec2,
    /// Heading angle in radians (positive X-axis = 0, positive Y-axis = π/2).
    pub rotation: f64,
}

impl CarState {
    /// Linearly interpolate between `a` (previous) and `b` (current) by `alpha ∈ [0, 1]`.
    ///
    /// Angle interpolation always takes the shortest arc to prevent the 360°
    /// flip artefact that causes visual "jitter" on tight turns.
    #[inline]
    pub fn lerp(a: &CarState, b: &CarState, alpha: f64) -> CarState {
        // Position: straight vector lerp.
        let position = a.position.lerp(b.position, alpha);

        // Angle: shortest-arc lerp.
        let mut delta = b.rotation - a.rotation;
        // Wrap delta into (-π, +π].
        if delta >  PI { delta -= TAU; }
        if delta < -PI { delta += TAU; }
        let rotation = a.rotation + delta * alpha;

        CarState { position, rotation }
    }
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

/// Physics simulation for a single car following a `BezierPath`.
///
/// Holds both the *previous* and *current* physics snapshots so that the
/// render step can interpolate between them with the residual accumulator time.
pub struct Simulation {
    /// The curve the car follows.
    pub path: BezierPath,
    /// Arc-length distance travelled so far (wraps at `path.total_length`).
    pub distance: f64,
    /// Constant linear speed in world-units per second.
    pub speed: f64,
    /// Car state at the start of the last completed physics step.
    pub previous_state: CarState,
    /// Car state after the last completed physics step.
    pub current_state: CarState,
}

impl Simulation {
    /// Create a new simulation with the given path and speed.
    pub fn new(path: BezierPath, speed: f64) -> Self {
        let initial = path.get_state(0.0);
        Simulation {
            path,
            distance: 0.0,
            speed,
            previous_state: initial,
            current_state: initial,
        }
    }

    /// Advance the simulation by `real_elapsed` real seconds.
    ///
    /// Internally performs as many fixed-`PHYSICS_DT` steps as the accumulator
    /// allows, then returns the remaining (residual) accumulator time so the
    /// caller can pass it to `render_state`.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use traffic_control_lib::simulation::bezier_smooth::{BezierPath, Simulation};
    /// # use glam::DVec2;
    /// # let path = BezierPath::new(DVec2::ZERO, DVec2::new(50.0, 100.0), DVec2::new(100.0, 0.0));
    /// # let mut sim = Simulation::new(path, 15.0);
    /// let mut accumulator = 0.0f64;
    /// // called every outer frame:
    /// accumulator = sim.tick(1.0 / 144.0, accumulator);
    /// let rs = sim.render_state(accumulator);
    /// ```
    pub fn tick(&mut self, real_elapsed: f64, accumulator: f64) -> f64 {
        // Cap to avoid "spiral of death" on very slow machines.
        let capped = real_elapsed.min(0.25);
        let mut acc = accumulator + capped;

        while acc >= PHYSICS_DT {
            acc -= PHYSICS_DT;
            self.step();
        }

        acc
    }

    /// Return the interpolated visual state for the current display frame.
    ///
    /// `residual` is the value returned by the last call to `tick` — the
    /// leftover time that has not yet been consumed by a physics step.
    /// `alpha = residual / PHYSICS_DT` lies in [0, 1) and represents how far
    /// between the two latest physics states the renderer should draw.
    #[inline]
    pub fn render_state(&self, residual: f64) -> CarState {
        let alpha = (residual / PHYSICS_DT).clamp(0.0, 1.0);
        CarState::lerp(&self.previous_state, &self.current_state, alpha)
    }

    // ── Internal ────────────────────────────────────────────────────────────

    /// Advance physics by exactly one `PHYSICS_DT` step.
    fn step(&mut self) {
        self.previous_state = self.current_state;

        // Advance distance and wrap so the car loops indefinitely.
        self.distance = (self.distance + self.speed * PHYSICS_DT) % self.path.total_length;
        self.current_state = self.path.get_state(self.distance);
    }
}

// ---------------------------------------------------------------------------
// Standalone demo loop
// ---------------------------------------------------------------------------

/// Self-contained demonstration of the fixed-timestep + interpolation pattern.
///
/// Runs the simulation for `laps` complete traversals of the curve, logging a
/// sample render state every 60 frames.  In production, replace the logging
/// and sleep with your actual render calls.
pub fn run_demo_loop(path: BezierPath, speed: f64, laps: u32) {
    use std::time::Instant;

    let total_travel = path.total_length * laps as f64;
    let mut sim = Simulation::new(path, speed);

    let mut accumulator      = 0.0f64;
    let mut total_distance   = 0.0f64;
    let mut last_tick        = Instant::now();
    let mut frame_count      = 0u64;

    log::info!(
        "bezier_smooth demo: speed={:.2} u/s  curve_len={:.2} u  laps={}",
        speed,
        sim.path.total_length,
        laps,
    );

    loop {
        // ── Wall-clock elapsed time ────────────────────────────────────────
        let now          = Instant::now();
        let real_elapsed = now.duration_since(last_tick).as_secs_f64().min(0.25);
        last_tick        = now;

        // ── Fixed physics steps via accumulator ───────────────────────────
        // Tracks distance before and after to determine when to exit.
        let steps_before = (accumulator / PHYSICS_DT).floor() as u32;
        accumulator = sim.tick(real_elapsed, accumulator);
        let steps_after  = (accumulator / PHYSICS_DT).floor() as u32;
        let steps_run    = steps_before.saturating_sub(steps_after);
        total_distance  += sim.speed * PHYSICS_DT * steps_run as f64;

        // ── Render: interpolate between previous_state and current_state ──
        // `alpha` encodes how far in [0, 1) between the two physics ticks
        // the current display moment falls.  Applying lerp here guarantees
        // sub-frame-accurate positions with zero jitter on 144 Hz displays.
        let render = sim.render_state(accumulator);

        if frame_count % 60 == 0 {
            log::debug!(
                "[frame {:>6}] alpha={:.4}  pos=({:>9.4}, {:>9.4})  rot={:.4} rad",
                frame_count,
                accumulator / PHYSICS_DT,
                render.position.x,
                render.position.y,
                render.rotation,
            );
        }

        frame_count += 1;

        if total_distance >= total_travel {
            log::info!("bezier_smooth demo: finished {} laps in {} frames", laps, frame_count);
            break;
        }

        // Sleep to target ~60 fps outer loop when running as a standalone demo.
        let frame_elapsed = last_tick.elapsed().as_secs_f64();
        let remaining     = (1.0 / 60.0) - frame_elapsed;
        if remaining > 0.001 {
            std::thread::sleep(std::time::Duration::from_secs_f64(remaining));
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn straight_path() -> BezierPath {
        // Straight line from (0,0) to (100,0) with control point on the midpoint.
        BezierPath::new(
            DVec2::new(0.0, 0.0),
            DVec2::new(50.0, 0.0),
            DVec2::new(100.0, 0.0),
        )
    }

    fn arc_path() -> BezierPath {
        // Quarter-circle approximation: P0=(1,0), ctrl=(1,1), P2=(0,1).
        BezierPath::new(
            DVec2::new(1.0, 0.0),
            DVec2::new(1.0, 1.0),
            DVec2::new(0.0, 1.0),
        )
    }

    // ── LUT / arc-length tests ───────────────────────────────────────────────

    #[test]
    fn lut_starts_at_zero() {
        let p = straight_path();
        assert_eq!(p.lut[0], 0.0);
    }

    #[test]
    fn lut_ends_at_total_length() {
        let p = straight_path();
        assert!((p.lut[LUT_SAMPLES - 1] - p.total_length).abs() < 1e-10);
    }

    #[test]
    fn lut_is_monotonically_non_decreasing() {
        let p = arc_path();
        for w in p.lut.windows(2) {
            assert!(w[1] >= w[0], "LUT is not monotone: {} < {}", w[1], w[0]);
        }
    }

    #[test]
    fn straight_path_total_length_approx_100() {
        let p = straight_path();
        assert!((p.total_length - 100.0).abs() < 0.01, "length={}", p.total_length);
    }

    // ── get_state accuracy tests ─────────────────────────────────────────────

    #[test]
    fn get_state_at_zero_is_start() {
        let p = straight_path();
        let s = p.get_state(0.0);
        assert!(s.position.distance(DVec2::new(0.0, 0.0)) < 1e-6);
    }

    #[test]
    fn get_state_at_total_length_is_end() {
        let p = straight_path();
        let s = p.get_state(p.total_length);
        assert!(s.position.distance(DVec2::new(100.0, 0.0)) < 0.01);
    }

    #[test]
    fn get_state_at_midpoint_of_straight_is_midpoint() {
        let p = straight_path();
        let s = p.get_state(p.total_length / 2.0);
        assert!((s.position.x - 50.0).abs() < 0.2, "x={}", s.position.x);
        assert!(s.position.y.abs() < 1e-6);
    }

    #[test]
    fn straight_path_heading_is_zero_radians() {
        let p = straight_path();
        // The tangent along a horizontal straight line must point in +X → angle = 0.
        for frac in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let s = p.get_state(p.total_length * frac);
            assert!(s.rotation.abs() < 1e-6, "rotation={} at frac={}", s.rotation, frac);
        }
    }

    // ── CarState lerp tests ──────────────────────────────────────────────────

    #[test]
    fn lerp_alpha_zero_returns_a() {
        let a = CarState { position: DVec2::new(0.0, 0.0), rotation: 0.0 };
        let b = CarState { position: DVec2::new(10.0, 0.0), rotation: 1.0 };
        let r = CarState::lerp(&a, &b, 0.0);
        assert_eq!(r.position, a.position);
        assert_eq!(r.rotation, a.rotation);
    }

    #[test]
    fn lerp_alpha_one_returns_b() {
        let a = CarState { position: DVec2::new(0.0, 0.0), rotation: 0.0 };
        let b = CarState { position: DVec2::new(10.0, 0.0), rotation: 1.0 };
        let r = CarState::lerp(&a, &b, 1.0);
        assert!((r.position.x - 10.0).abs() < 1e-10);
        assert!((r.rotation - 1.0).abs() < 1e-10);
    }

    #[test]
    fn lerp_midpoint() {
        let a = CarState { position: DVec2::new(0.0, 0.0), rotation: 0.0 };
        let b = CarState { position: DVec2::new(10.0, 0.0), rotation: 2.0 };
        let r = CarState::lerp(&a, &b, 0.5);
        assert!((r.position.x - 5.0).abs() < 1e-10);
        assert!((r.rotation - 1.0).abs() < 1e-10);
    }

    #[test]
    fn lerp_angle_shortest_arc_across_pi() {
        // a.rotation = π - 0.1, b.rotation = -(π - 0.1): shortest arc is 0.2 rad, not 2π-0.2.
        let a = CarState { position: DVec2::ZERO, rotation:  PI - 0.1 };
        let b = CarState { position: DVec2::ZERO, rotation: -(PI - 0.1) };
        let r = CarState::lerp(&a, &b, 0.5);
        // Mid angle should be near ±π (not near 0).
        assert!(r.rotation.abs() > PI - 0.2, "rotation={}", r.rotation);
    }

    #[test]
    fn lerp_angle_no_wrap_needed() {
        let a = CarState { position: DVec2::ZERO, rotation: 0.0 };
        let b = CarState { position: DVec2::ZERO, rotation: FRAC_PI_2 };
        let r = CarState::lerp(&a, &b, 0.5);
        assert!((r.rotation - FRAC_PI_2 / 2.0).abs() < 1e-10);
    }

    // ── Simulation tick tests ────────────────────────────────────────────────

    #[test]
    fn simulation_advances_distance_per_step() {
        let p = straight_path();
        let speed = 10.0;
        let mut sim = Simulation::new(p, speed);
        let d_before = sim.distance;
        sim.step();
        let expected = speed * PHYSICS_DT;
        assert!((sim.distance - d_before - expected).abs() < 1e-10);
    }

    #[test]
    fn simulation_wraps_at_total_length() {
        let p = straight_path();
        let len = p.total_length;
        // Speed that advances the car just past the end in one step.
        let speed = len / PHYSICS_DT + 1.0;
        let mut sim = Simulation::new(p, speed);
        sim.step();
        assert!(sim.distance < len, "distance={} should be < {}", sim.distance, len);
    }

    #[test]
    fn simulation_previous_equals_current_before_first_step() {
        let p = straight_path();
        let sim = Simulation::new(p, 10.0);
        assert_eq!(sim.previous_state.position, sim.current_state.position);
    }

    #[test]
    fn simulation_previous_is_old_current_after_step() {
        let p = straight_path();
        let mut sim = Simulation::new(p, 10.0);
        let pos_before = sim.current_state.position;
        sim.step();
        assert_eq!(sim.previous_state.position, pos_before);
    }

    #[test]
    fn tick_returns_residual_less_than_physics_dt() {
        let p = straight_path();
        let mut sim = Simulation::new(p, 10.0);
        let residual = sim.tick(1.0 / 30.0, 0.0); // half a 60 Hz frame
        assert!(residual >= 0.0);
        assert!(residual < PHYSICS_DT);
    }

    #[test]
    fn render_state_alpha_zero_equals_previous() {
        let p = straight_path();
        let mut sim = Simulation::new(p, 10.0);
        sim.step();
        let rendered = sim.render_state(0.0);
        assert!((rendered.position - sim.previous_state.position).length() < 1e-10);
    }

    #[test]
    fn render_state_alpha_one_equals_current() {
        let p = straight_path();
        let mut sim = Simulation::new(p, 10.0);
        sim.step();
        let rendered = sim.render_state(PHYSICS_DT);
        assert!((rendered.position - sim.current_state.position).length() < 1e-10);
    }

    #[test]
    fn render_state_is_between_previous_and_current() {
        let p = straight_path();
        let mut sim = Simulation::new(p, 10.0);
        // Run several steps to ensure states differ.
        for _ in 0..5 {
            sim.step();
        }
        let alpha    = 0.5;
        let residual = PHYSICS_DT * alpha;
        let rendered = sim.render_state(residual);

        let min_x = sim.previous_state.position.x.min(sim.current_state.position.x);
        let max_x = sim.previous_state.position.x.max(sim.current_state.position.x);
        assert!(
            rendered.position.x >= min_x - 1e-10 && rendered.position.x <= max_x + 1e-10,
            "rendered x={} not in [{}, {}]",
            rendered.position.x,
            min_x,
            max_x,
        );
    }
}
