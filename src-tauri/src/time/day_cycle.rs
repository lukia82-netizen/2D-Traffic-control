/// Keyframe for the day-cycle spawn-rate curve.
struct Keyframe {
    hour: f32,
    multiplier: f32,
}

const KEYFRAMES: &[Keyframe] = &[
    Keyframe { hour: 0.0,  multiplier: 0.05 },
    Keyframe { hour: 6.0,  multiplier: 0.3  },
    Keyframe { hour: 7.0,  multiplier: 1.5  },
    Keyframe { hour: 8.0,  multiplier: 3.0  },
    Keyframe { hour: 9.0,  multiplier: 2.0  },
    Keyframe { hour: 12.0, multiplier: 1.5  },
    Keyframe { hour: 13.0, multiplier: 1.8  },
    Keyframe { hour: 14.0, multiplier: 1.3  },
    Keyframe { hour: 16.0, multiplier: 1.5  },
    Keyframe { hour: 17.0, multiplier: 3.0  },
    Keyframe { hour: 18.0, multiplier: 2.5  },
    Keyframe { hour: 20.0, multiplier: 1.2  },
    Keyframe { hour: 22.0, multiplier: 0.4  },
    Keyframe { hour: 24.0, multiplier: 0.05 },
];

pub struct DayCycle;

impl DayCycle {
    /// Returns the spawn-rate multiplier (0.05 – 3.0) for a given game hour.
    pub fn spawn_multiplier(game_hour: f32) -> f32 {
        let hour = game_hour.rem_euclid(24.0);

        // Find the surrounding keyframes
        let mut lo = &KEYFRAMES[0];
        let mut hi = &KEYFRAMES[KEYFRAMES.len() - 1];

        for window in KEYFRAMES.windows(2) {
            if hour >= window[0].hour && hour <= window[1].hour {
                lo = &window[0];
                hi = &window[1];
                break;
            }
        }

        let span = hi.hour - lo.hour;
        if span <= 0.0 {
            return lo.multiplier;
        }

        let t = (hour - lo.hour) / span;
        smoothstep_lerp(lo.multiplier, hi.multiplier, t)
    }
}

/// Smoothstep interpolation: t mapped through 3t²-2t³, then linearly interpolated.
fn smoothstep_lerp(a: f32, b: f32, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    let s = t * t * (3.0 - 2.0 * t);
    a + (b - a) * s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn midnight_is_low() {
        let m = DayCycle::spawn_multiplier(0.0);
        assert!(m < 0.1, "midnight multiplier should be low, got {}", m);
    }

    #[test]
    fn morning_rush_is_high() {
        let m = DayCycle::spawn_multiplier(8.0);
        assert!(m > 2.5, "morning rush multiplier should be high, got {}", m);
    }
}
