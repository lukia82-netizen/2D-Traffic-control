use std::sync::{Arc, Mutex};
use parking_lot::RwLock;
use crate::map::road_network::MapData;

pub struct AppState {
    pub road_graph: Arc<RwLock<Option<MapData>>>,
    pub sim_control: Arc<Mutex<Option<SimControl>>>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            road_graph: Arc::new(RwLock::new(None)),
            sim_control: Arc::new(Mutex::new(None)),
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
    SetLightMode {
        intersection_id: u64,
        mode: LightControlMode,
    },
    SetLightPhase {
        intersection_id: u64,
        phase: u8,
    },
    Stop,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LightControlMode {
    Manual,
    SemiAuto,
    Auto,
    Adaptive,
}
