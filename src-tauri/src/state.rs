use std::sync::{Arc, Mutex};
use parking_lot::RwLock;
use crate::map::road_network::MapData;
use crate::map::world_editor::{EditorHistory, EditorTool};
use crate::simulation::speed_config::SpeedConfig;

pub struct AppState {
    pub road_graph: Arc<RwLock<Option<MapData>>>,
    pub sim_control: Arc<Mutex<Option<SimControl>>>,
    pub editor_tool: Arc<RwLock<EditorTool>>,
    pub editor_history: Arc<RwLock<EditorHistory>>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            road_graph:  Arc::new(RwLock::new(None)),
            sim_control: Arc::new(Mutex::new(None)),
            editor_tool: Arc::new(RwLock::new(EditorTool::None)),
            editor_history: Arc::new(RwLock::new(EditorHistory::default())),
        }
    }
}

pub struct SimControl {
    pub command_tx: std::sync::mpsc::Sender<SimCommand>,
}

#[derive(Debug, Clone)]
pub enum SimCommand {
    Pause,
    Resume,
    SetTimeScale(f32),
    SetSpeedConfig(SpeedConfig),
    SetMaxVehicles(usize),
    SetLightMode {
        intersection_id: u64,
        mode: LightControlMode,
    },
    SetLightPhase {
        intersection_id: u64,
        phase: u8,
    },
    /// Set fixed phase durations for SemiAuto / Auto modes.
    SetLightDurations {
        intersection_id: u64,
        green_s: f32,
        red_s: f32,
    },
    SetDebugVehicle(Option<u32>),
    Stop,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LightControlMode {
    Manual,
    SemiAuto,
    Auto,
    Adaptive,
}
