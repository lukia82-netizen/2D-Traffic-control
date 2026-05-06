//! Multi-phase vehicle traffic-light programs (opposing straight greens, clearance yellow, all-red).

use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Direction;

use std::collections::HashMap;

use crate::map::road_network::{LaneDirection, RoadGraph};
use crate::state::LightControlMode;

use super::traffic_light::{LightPhase, LightStateUpdate};

pub type MovementMask = u128;

#[derive(Debug, Clone)]
pub struct JunctionLayout {
    /// Incoming edges to the junction vertex, sorted clockwise by approach bearing (atan2 dlng/dlat).
    pub arms: Vec<EdgeIndex>,
    pub edge_to_arm: HashMap<EdgeIndex, u8>,
}

impl JunctionLayout {
    pub fn build(graph: &RoadGraph, junction: NodeIndex) -> Option<Self> {
        let mut inbound: Vec<EdgeIndex> = graph
            .edges_directed(junction, Direction::Incoming)
            .map(|e| e.id())
            .collect();
        if inbound.is_empty() {
            return None;
        }

        inbound.sort_by(|&a, &b| {
            bearing_incoming(graph, a, junction)
                .partial_cmp(&bearing_incoming(graph, b, junction))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut edge_to_arm = HashMap::with_capacity(inbound.len());
        for (i, &eid) in inbound.iter().enumerate() {
            edge_to_arm.insert(eid, i as u8);
        }

        Some(Self {
            arms: inbound,
            edge_to_arm,
        })
    }

    #[inline]
    pub fn arm_for_inbound_edge(&self, edge: EdgeIndex) -> Option<u8> {
        self.edge_to_arm.get(&edge).copied()
    }

    #[inline]
    pub fn arm_count(&self) -> usize {
        self.arms.len()
    }
}

fn bearing_incoming(graph: &RoadGraph, edge: EdgeIndex, junction: NodeIndex) -> f32 {
    let (src, tgt) = graph.edge_endpoints(edge).expect("edge endpoints");
    debug_assert_eq!(tgt, junction);
    let s = &graph[src];
    let t = &graph[junction];
    let dlat = (t.lat - s.lat) as f32;
    let dlng = (t.lng - s.lng) as f32;
    dlng.atan2(dlat)
}

#[inline]
fn bit_for(arm: u8, mv: LaneDirection) -> Option<MovementMask> {
    let group = match mv {
        LaneDirection::Straight => 0u8,
        LaneDirection::Left => 4,
        LaneDirection::Right => 8,
        LaneDirection::UTurn => 12,
    };
    let sh = (group + arm) as u32;
    if sh >= 128 {
        return None;
    }
    Some(1u128 << sh)
}

pub fn combine_movements(arm: u8, kinds: &[LaneDirection]) -> MovementMask {
    let mut m: MovementMask = 0;
    for &k in kinds {
        if let Some(b) = bit_for(arm, k) {
            m |= b;
        }
    }
    m
}

#[inline]
pub fn mask_allows(mask: MovementMask, arm: u8, mv: LaneDirection) -> bool {
    bit_for(arm, mv).map_or(false, |b| mask & b != 0)
}

#[derive(Debug, Clone)]
pub enum ProgramStepKind {
    Go { allowed: MovementMask },
    Caution { allowed: MovementMask },
    AllRed,
}

#[derive(Debug, Clone)]
pub struct TimedStep {
    pub kind: ProgramStepKind,
    pub duration: f32,
}

#[derive(Debug, Clone)]
pub struct PhasedVehicleLight {
    pub intersection_id: u64,
    pub layout: JunctionLayout,
    pub mode: LightControlMode,
    pub queue_count: u32,
    pub steps: Vec<TimedStep>,
    pub step_index: usize,
    pub timer: f32,
    pub yellow_duration: f32,
    pub all_red_duration: f32,
    pub green_straight: f32,
    pub green_left: f32,
    /// Sandbox + test cross: two phases only — ((N,S),(E,W)) arm indices, straight+right only.
    simple_cross_arm_pairs: Option<((u8, u8), (u8, u8))>,
}

/// Map each compass approach to its `JunctionLayout` arm index (0…3) for a symmetric + junction.
fn classify_plus_cross_arm_pairs(
    graph: &RoadGraph,
    layout: &JunctionLayout,
    junction: NodeIndex,
) -> Option<((u8, u8), (u8, u8))> {
    #[derive(Clone, Copy, PartialEq, Eq, Hash)]
    enum Compass {
        N,
        S,
        E,
        W,
    }
    if layout.arm_count() != 4 {
        return None;
    }
    let j = &graph[junction];
    let mut m: HashMap<Compass, u8> = HashMap::new();
    for (i, &eid) in layout.arms.iter().enumerate() {
        let arm_i = i as u8;
        let (src, tgt) = graph.edge_endpoints(eid)?;
        if tgt != junction {
            return None;
        }
        let s = &graph[src];
        let dlat = s.lat - j.lat;
        let dlng = s.lng - j.lng;
        let c = if dlat.abs() >= dlng.abs() {
            if dlat >= 0.0 {
                Compass::N
            } else {
                Compass::S
            }
        } else if dlng >= 0.0 {
            Compass::E
        } else {
            Compass::W
        };
        m.insert(c, arm_i);
    }
    if m.len() != 4 {
        return None;
    }
    Some((
        (*m.get(&Compass::N)?, *m.get(&Compass::S)?),
        (*m.get(&Compass::E)?, *m.get(&Compass::W)?),
    ))
}

impl PhasedVehicleLight {
    pub fn new(
        intersection_id: u64,
        layout: JunctionLayout,
        graph: &RoadGraph,
        junction: NodeIndex,
        sandbox_simple_cross_tl: bool,
    ) -> Self {
        let green_straight = 10.0_f32;
        let green_left = 8.0_f32;
        let yellow_duration = 3.0_f32;
        let all_red_duration = 2.5_f32;

        let simple_cross_arm_pairs = if sandbox_simple_cross_tl {
            classify_plus_cross_arm_pairs(graph, &layout, junction)
        } else {
            None
        };

        let (steps, mode, step_index, timer) = if let Some(((n, s), (e, w))) = simple_cross_arm_pairs {
            let dur = green_straight.max(5.0);
            let steps = vec![
                go(pair_through(n, s), dur),
                go(pair_through(e, w), dur),
            ];
            (steps, LightControlMode::Manual, 0, 0.0)
        } else {
            let steps = build_steps_for_layout(
                layout.arm_count(),
                green_straight,
                green_left,
                yellow_duration,
                all_red_duration,
            );
            let (step_index, timer) = stagger_start(&steps, intersection_id);
            (steps, LightControlMode::Auto, step_index, timer)
        };

        Self {
            intersection_id,
            layout,
            mode,
            queue_count: 0,
            steps,
            step_index,
            timer,
            yellow_duration,
            all_red_duration,
            green_straight,
            green_left,
            simple_cross_arm_pairs,
        }
    }

    pub fn rebuild_steps(&mut self) {
        if let Some(((n, s), (e, w))) = self.simple_cross_arm_pairs {
            let dur = self.green_straight.max(5.0);
            self.steps = vec![
                go(pair_through(n, s), dur),
                go(pair_through(e, w), dur),
            ];
            if self.steps.is_empty() {
                self.step_index = 0;
                self.timer = 0.0;
            } else {
                self.step_index = self.step_index.min(self.steps.len() - 1);
            }
            return;
        }

        let n = self.layout.arm_count();
        self.steps = build_steps_for_layout(
            n,
            self.green_straight,
            self.green_left,
            self.yellow_duration,
            self.all_red_duration,
        );
        if self.steps.is_empty() {
            self.step_index = 0;
            self.timer = 0.0;
        } else {
            self.step_index = self.step_index.min(self.steps.len() - 1);
        }
    }

    pub fn set_durations(&mut self, green_main_s: f32, secondary_s: f32) {
        // Primary = straight-phase green; secondary = protected left-phase green & fallback tuning.
        self.green_straight = green_main_s.max(5.0);
        self.green_left = secondary_s.max(5.0);
        self.rebuild_steps();
    }

    pub fn set_mode(&mut self, mode: LightControlMode) {
        self.mode = mode;
    }

    #[inline]
    pub fn adaptive_effective_green_base(&self) -> f32 {
        let extra = (self.queue_count as f32 / 20.0).min(1.0) * 40.0;
        (20.0 + extra).min(60.0)
    }

    fn step_duration_for_adaptive(&self, step_idx: usize) -> f32 {
        if step_idx >= self.steps.len() {
            return 0.0;
        }
        let base = self.steps[step_idx].duration;
        if self.mode != LightControlMode::Adaptive {
            return base;
        }
        match &self.steps[step_idx].kind {
            ProgramStepKind::Go { .. } => {
                let b = self.adaptive_effective_green_base();
                base * (b / 20.0).clamp(1.0, 3.0)
            }
            _ => base,
        }
    }

    pub fn update(&mut self, dt_real_s: f32) {
        match self.mode {
            LightControlMode::Manual => {}
            LightControlMode::SemiAuto | LightControlMode::Auto | LightControlMode::Adaptive => {
                self.advance_steps(dt_real_s);
            }
        }
    }

    fn advance_steps(&mut self, mut dt: f32) {
        if self.steps.is_empty() || dt <= 0.0 {
            return;
        }
        loop {
            let dur = self.step_duration_for_adaptive(self.step_index);
            let remaining_in_step = (dur - self.timer).max(0.0);
            if remaining_in_step > dt + 1e-6 {
                self.timer += dt;
                break;
            }
            dt -= remaining_in_step;
            self.timer = 0.0;
            self.step_index = (self.step_index + 1) % self.steps.len();
            if dt <= 1e-6 {
                break;
            }
        }
    }

    pub fn force_advance_step(&mut self) {
        if self.steps.is_empty() {
            return;
        }
        self.step_index = (self.step_index + 1) % self.steps.len();
        self.timer = 0.0;
    }

    pub fn signal_for(&self, inbound_edge: EdgeIndex, movement: LaneDirection) -> LightPhase {
        if self.steps.is_empty() {
            return LightPhase::Red;
        }
        let Some(arm) = self.layout.arm_for_inbound_edge(inbound_edge) else {
            return LightPhase::Red;
        };
        let step = &self.steps[self.step_index];
        match &step.kind {
            ProgramStepKind::AllRed => LightPhase::Red,
            ProgramStepKind::Go { allowed } => {
                if mask_allows(*allowed, arm, movement) {
                    LightPhase::Green
                } else {
                    LightPhase::Red
                }
            }
            ProgramStepKind::Caution { allowed } => {
                if mask_allows(*allowed, arm, movement) {
                    LightPhase::Yellow
                } else {
                    LightPhase::Red
                }
            }
        }
    }

    /// Best / most permissive signal shown on a single bulb for this inbound arm (all lane movements).
    pub fn signal_summarize_arm(&self, arm_idx: usize) -> LightPhase {
        if arm_idx >= self.layout.arms.len() {
            return LightPhase::Red;
        }
        let edge = self.layout.arms[arm_idx];
        let mut best = LightPhase::Red;
        for mv in [
            LaneDirection::Straight,
            LaneDirection::Left,
            LaneDirection::Right,
            LaneDirection::UTurn,
        ] {
            best = phase_max(best, self.signal_for(edge, mv));
        }
        best
    }

    pub fn arm_phase_bytes(&self) -> Vec<u8> {
        (0..self.layout.arm_count())
            .map(|i| self.signal_summarize_arm(i).to_u8())
            .collect()
    }

    #[inline]
    pub fn aggregate_phase_byte(&self) -> u8 {
        if self.steps.is_empty() {
            return LightPhase::Red.to_u8();
        }
        match &self.steps[self.step_index].kind {
            ProgramStepKind::AllRed => LightPhase::Red.to_u8(),
            ProgramStepKind::Caution { .. } => LightPhase::Yellow.to_u8(),
            ProgramStepKind::Go { .. } => LightPhase::Green.to_u8(),
        }
    }

    pub fn current_time_remaining(&self) -> f32 {
        if self.steps.is_empty() {
            return 0.0;
        }
        let dur = self.step_duration_for_adaptive(self.step_index);
        (dur - self.timer).max(0.0)
    }

    pub fn broadcast_signature(&self) -> u64 {
        let mut sig: u128 = self.step_index as u128;
        sig = sig.wrapping_shl(48);
        sig |= ((self.current_time_remaining() * 5.0) as u64 as u128) & 0xFFFF;
        sig = sig.wrapping_shl(32);
        for &p in self.arm_phase_bytes().iter().take(8) {
            sig = sig.wrapping_mul(131).wrapping_add(p as u128);
        }
        sig as u64 ^ ((sig >> 64) as u64)
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
            phase: self.aggregate_phase_byte(),
            time_remaining: self.current_time_remaining(),
            queue_count: self.queue_count,
            mode: mode_str.to_string(),
            green_duration: self.green_straight,
            red_duration: self.green_left,
            junction_arm_phases: Some(self.arm_phase_bytes()),
        }
    }
}

fn stagger_start(steps: &[TimedStep], intersection_id: u64) -> (usize, f32) {
    if steps.is_empty() {
        return (0, 0.0);
    }
    let cycle_len: f32 = steps.iter().map(|s| s.duration).sum();
    if cycle_len < 0.01 {
        return (0, 0.0);
    }
    let seed = intersection_id
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    let mut t = ((seed % 1_000_000) as f32 / 1_000_000.0) * cycle_len;
    for (i, s) in steps.iter().enumerate() {
        if t < s.duration {
            return (i, t);
        }
        t -= s.duration;
    }
    (0, 0.0)
}

#[inline]
fn phase_max(a: LightPhase, b: LightPhase) -> LightPhase {
    use LightPhase::*;
    match (a, b) {
        (Green, _) | (_, Green) => Green,
        (Yellow, _) | (_, Yellow) => Yellow,
        _ => Red,
    }
}

fn go(mask: MovementMask, d: f32) -> TimedStep {
    TimedStep {
        kind: ProgramStepKind::Go { allowed: mask },
        duration: d,
    }
}

fn caution(mask: MovementMask, d: f32) -> TimedStep {
    TimedStep {
        kind: ProgramStepKind::Caution { allowed: mask },
        duration: d,
    }
}

fn all_red(d: f32) -> TimedStep {
    TimedStep {
        kind: ProgramStepKind::AllRed,
        duration: d,
    }
}

fn pair_through(a: u8, b: u8) -> MovementMask {
    combine_movements(a, &[LaneDirection::Straight, LaneDirection::Right])
        | combine_movements(b, &[LaneDirection::Straight, LaneDirection::Right])
}

fn pair_left(a: u8, b: u8) -> MovementMask {
    combine_movements(a, &[LaneDirection::Left]) | combine_movements(b, &[LaneDirection::Left])
}

fn build_four_arm_two_axis(g_str: f32, g_left: f32, y: f32, ar: f32) -> Vec<TimedStep> {
    let mut v = Vec::with_capacity(12);
    // Axis arms 0 & 2
    v.push(go(pair_through(0, 2), g_str));
    v.push(caution(pair_through(0, 2), y));
    v.push(all_red(ar));
    v.push(go(pair_left(0, 2), g_left));
    v.push(caution(pair_left(0, 2), y));
    v.push(all_red(ar));
    // Axis arms 1 & 3
    v.push(go(pair_through(1, 3), g_str));
    v.push(caution(pair_through(1, 3), y));
    v.push(all_red(ar));
    v.push(go(pair_left(1, 3), g_left));
    v.push(caution(pair_left(1, 3), y));
    v.push(all_red(ar));
    v
}

fn build_sequential_arms(n: usize, g: f32, y: f32, ar: f32) -> Vec<TimedStep> {
    let mut v = Vec::with_capacity(n * 3);
    for arm in 0..n {
        let a = arm as u8;
        let m = combine_movements(
            a,
            &[
                LaneDirection::Straight,
                LaneDirection::Left,
                LaneDirection::Right,
                LaneDirection::UTurn,
            ],
        );
        v.push(go(m, g));
        v.push(caution(m, y));
        v.push(all_red(ar));
    }
    v
}

pub fn build_steps_for_layout(
    arm_count: usize,
    g_str: f32,
    g_left: f32,
    y: f32,
    ar: f32,
) -> Vec<TimedStep> {
    match arm_count {
        4 => build_four_arm_two_axis(g_str, g_left, y, ar),
        0 | 1 => vec![],
        n if n == 2 || n == 3 => build_sequential_arms(n, g_str, y, ar),
        n => {
            // 5+ arms: safe round-robin (no geometry pairing).
            build_sequential_arms(n, g_str, y, ar)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::road_network::LaneDirection;
    use crate::state::LightControlMode;
    use crate::traffic::traffic_light::LightPhase;
    use petgraph::graph::EdgeIndex;

    #[test]
    fn combine_movements_sets_expected_bits() {
        let m = combine_movements(0, &[LaneDirection::Straight, LaneDirection::Left]);
        assert!(mask_allows(m, 0, LaneDirection::Straight));
        assert!(mask_allows(m, 0, LaneDirection::Left));
        assert!(!mask_allows(m, 0, LaneDirection::Right));
        assert!(!mask_allows(m, 1, LaneDirection::Straight));
    }

    #[test]
    fn build_steps_four_arms_has_axis_phases() {
        let steps = build_steps_for_layout(4, 10.0, 8.0, 3.0, 2.5);
        assert_eq!(steps.len(), 12);
        assert!(matches!(
            steps[0].kind,
            ProgramStepKind::Go { .. }
        ));
        assert!(matches!(steps[1].kind, ProgramStepKind::Caution { .. }));
        assert!(matches!(steps[2].kind, ProgramStepKind::AllRed));
    }

    #[test]
    fn build_steps_two_arms_is_sequential() {
        let steps = build_steps_for_layout(2, 10.0, 8.0, 3.0, 2.5);
        assert_eq!(steps.len(), 6);
    }

    fn four_arm_layout() -> JunctionLayout {
        let arms: Vec<EdgeIndex> = (0..4).map(EdgeIndex::new).collect();
        let mut edge_to_arm = HashMap::new();
        for (i, e) in arms.iter().enumerate() {
            edge_to_arm.insert(*e, i as u8);
        }
        JunctionLayout {
            arms,
            edge_to_arm,
        }
    }

    #[test]
    fn first_green_allows_opposing_straight_pair_only() {
        let layout = four_arm_layout();
        let mut ph = PhasedVehicleLight::new(42, layout);
        ph.step_index = 0;
        ph.timer = 0.0;
        let e0 = EdgeIndex::new(0);
        let e1 = EdgeIndex::new(1);
        assert_eq!(ph.signal_for(e0, LaneDirection::Straight), LightPhase::Green);
        assert_eq!(ph.signal_for(e1, LaneDirection::Straight), LightPhase::Red);
    }

    #[test]
    fn manual_mode_does_not_advance_timer() {
        let layout = four_arm_layout();
        let mut ph = PhasedVehicleLight::new(1, layout);
        ph.set_mode(LightControlMode::Manual);
        let idx_before = ph.step_index;
        let t_before = ph.timer;
        ph.update(5.0);
        assert_eq!(ph.step_index, idx_before);
        assert!((ph.timer - t_before).abs() < 1e-5);
    }

    #[test]
    fn adaptive_mode_increases_time_remaining_for_go_step() {
        let layout = four_arm_layout();
        let mut ph_auto = PhasedVehicleLight::new(3, layout.clone());
        ph_auto.set_mode(LightControlMode::Auto);
        ph_auto.step_index = 0;
        ph_auto.timer = 0.0;

        let mut ph_adapt = PhasedVehicleLight::new(3, layout);
        ph_adapt.set_mode(LightControlMode::Adaptive);
        ph_adapt.queue_count = 80;
        ph_adapt.step_index = 0;
        ph_adapt.timer = 0.0;

        assert!(
            ph_adapt.current_time_remaining() > ph_auto.current_time_remaining(),
            "adaptive go phase should last longer with high queue"
        );
    }
}
