pub mod commands;
pub mod map;
pub mod simulation;
pub mod state;
pub mod time;
pub mod traffic;
pub mod vehicles;

use commands::{
    auto_fix_map, load_map, pause_simulation, resume_simulation, set_debug_vehicle,
    set_light_durations, set_max_vehicles, set_speed_config, set_time_scale,
    set_traffic_light_mode, set_traffic_light_phase, start_simulation,
};
use state::AppState;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    env_logger::init();

    tauri::Builder::default()
        .manage(AppState::new())
        .invoke_handler(tauri::generate_handler![
            load_map,
            start_simulation,
            pause_simulation,
            resume_simulation,
            set_time_scale,
            set_traffic_light_mode,
            set_traffic_light_phase,
            set_speed_config,
            set_light_durations,
            set_max_vehicles,
            set_debug_vehicle,
            auto_fix_map,
        ])
        .setup(|app| {
            log::info!("Traffic Control app started");
            let _ = app;
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
