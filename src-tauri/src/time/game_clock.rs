#[derive(Debug, Clone)]
pub struct GameClock {
    /// Current in-game time in seconds since midnight (0 = 00:00:00)
    pub game_time_s: f64,
    /// Real-to-game time multiplier (e.g. 60 means 1 real second = 1 game minute)
    pub time_scale: f32,
    pub paused: bool,
}

impl GameClock {
    /// Start the day at 06:00 with a default time scale of 60 (1 real min = 1 game hour)
    pub fn new() -> Self {
        GameClock {
            game_time_s: 6.0 * 3600.0,
            time_scale: 60.0,
            paused: false,
        }
    }

    /// Advance the clock by `real_dt_s` real seconds.
    /// Returns the game delta in seconds, or 0 if paused.
    pub fn tick(&mut self, real_dt_s: f32) -> f32 {
        if self.paused {
            return 0.0;
        }
        let game_dt = real_dt_s * self.time_scale;
        self.game_time_s += game_dt as f64;
        // Wrap at 24h
        if self.game_time_s >= 86_400.0 {
            self.game_time_s -= 86_400.0;
        }
        game_dt
    }

    /// Returns the current game hour as a float in [0, 24).
    pub fn game_hour(&self) -> f32 {
        (self.game_time_s / 3600.0) as f32
    }

    pub fn pause(&mut self) {
        self.paused = true;
    }

    pub fn resume(&mut self) {
        self.paused = false;
    }

    pub fn set_time_scale(&mut self, scale: f32) {
        self.time_scale = scale.max(0.1).min(3600.0);
    }
}

impl Default for GameClock {
    fn default() -> Self {
        Self::new()
    }
}
