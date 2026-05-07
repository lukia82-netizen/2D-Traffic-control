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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_starts_at_six_am() {
        let c = GameClock::new();
        assert!((c.game_time_s - 6.0 * 3600.0).abs() < 1e-6);
        assert!((c.game_hour() - 6.0).abs() < 1e-3);
    }

    #[test]
    fn tick_advances_by_time_scale() {
        let mut c = GameClock::new();
        c.time_scale = 2.0;
        let dt = c.tick(1.0);
        assert!((dt - 2.0).abs() < 1e-5);
        assert!((c.game_time_s - (6.0 * 3600.0 + 2.0)).abs() < 1e-4);
    }

    #[test]
    fn paused_tick_returns_zero_and_does_not_advance() {
        let mut c = GameClock::new();
        let t0 = c.game_time_s;
        c.pause();
        assert_eq!(c.tick(1.0), 0.0);
        assert_eq!(c.game_time_s, t0);
    }

    #[test]
    fn wraps_past_midnight() {
        let mut c = GameClock::new();
        c.time_scale = 1.0;
        c.game_time_s = 86_400.0 - 0.5;
        c.tick(1.0);
        assert!(c.game_time_s < 1.0, "expected wrap, got {}", c.game_time_s);
    }

    #[test]
    fn set_time_scale_clamps() {
        let mut c = GameClock::new();
        c.set_time_scale(0.01);
        assert!((c.time_scale - 0.1).abs() < 1e-5);
        c.set_time_scale(10_000.0);
        assert!((c.time_scale - 3600.0).abs() < 1e-3);
    }
}
