use serde::{Deserialize, Serialize};
use crate::vehicles::driver::DriverProfile;

/// Per-profile compliance range: base multiplier + noise bounds.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceRange {
    /// Base multiplier on road max_speed (>1.0 = speeding)
    pub base: f32,
    /// Minimum after noise
    pub min: f32,
    /// Maximum after noise
    pub max: f32,
}

/// Route planning configuration (alpha = 0 → shortest, 1 → fastest).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteConfig {
    /// Reference speed for alpha blending [m/s]; default 13.89 = 50 km/h
    pub ref_speed_ms: f32,
    /// Gaussian noise sigma applied to sampled alpha; default 0.05
    pub noise_sigma: f32,
    /// Uniform range for Normal drivers (min, max)
    pub normal_alpha: (f32, f32),
    /// Uniform range for Sunday drivers
    pub sunday_alpha: (f32, f32),
    /// Uniform range for Pirat drivers
    pub pirat_alpha: (f32, f32),
    /// Uniform range for Cautious drivers
    pub cautious_alpha: (f32, f32),
}

/// Frustration / rage system configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RageConfig {
    /// Real seconds standing still before frustration starts – per profile [Normal,Sunday,Pirat,Cautious]
    pub standstill_threshold_s: [f32; 4],
    /// Frustration points/s while standing still (linear phase) – per profile
    pub decay_rate_linear: [f32; 4],
    /// Frustration points/s recovered while moving – per profile
    pub recovery_rate: [f32; 4],
    /// Fraction of road max_speed below which the vehicle is considered "crawling"
    pub crawl_fraction: f32,
    /// Real seconds crawling before crawl frustration kicks in
    pub crawl_threshold_s: f32,
    /// Frustration points/s while crawling – per profile
    pub crawl_rate: [f32; 4],
    /// Bonus frustration per repeated red-light stop at the same intersection – per profile
    pub repeat_stop_bonus: [f32; 4],
    /// Average frustration level that triggers game-over condition (when held ≥ duration)
    pub global_loss_threshold: f32,
    /// Real seconds the average must exceed threshold before game over
    pub global_loss_duration_s: f32,
    /// Fraction of vehicles simultaneously at frustration=100 that triggers game over
    pub mass_rage_fraction: f32,
}

/// Top-level speed and compliance configuration.
/// Stored in `AppState`, sent to sim thread via `SimCommand::SetSpeedConfig`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedConfig {
    // ── Urban speed limits [km/h] ──────────────────────────────────────────
    pub urban_1lane: f32,
    pub urban_2lane: f32,
    pub urban_3lane_plus: f32,
    pub urban_motorway: f32,
    pub urban_residential: f32,
    pub urban_living: f32,

    // ── Rural speed limits [km/h] ──────────────────────────────────────────
    pub rural_1lane: f32,
    pub rural_2lane_plus: f32,
    pub rural_motorway: f32,

    // ── Per-profile compliance ranges ──────────────────────────────────────
    pub compliance_normal: ComplianceRange,
    pub compliance_sunday: ComplianceRange,
    pub compliance_pirat: ComplianceRange,
    pub compliance_cautious: ComplianceRange,

    /// Gaussian noise sigma for compliance sampling (default 0.04 = ±4 %)
    pub noise_sigma: f32,

    pub route: RouteConfig,
    pub rage: RageConfig,
}

impl Default for SpeedConfig {
    fn default() -> Self {
        SpeedConfig {
            urban_1lane: 50.0,
            urban_2lane: 70.0,
            urban_3lane_plus: 70.0,
            urban_motorway: 120.0,
            urban_residential: 30.0,
            urban_living: 20.0,

            rural_1lane: 90.0,
            rural_2lane_plus: 90.0,
            rural_motorway: 140.0,

            compliance_normal:   ComplianceRange { base: 1.10, min: 0.95, max: 1.20 },
            compliance_sunday:   ComplianceRange { base: 0.95, min: 0.85, max: 1.05 },
            compliance_pirat:    ComplianceRange { base: 1.35, min: 1.15, max: 1.60 },
            compliance_cautious: ComplianceRange { base: 0.92, min: 0.85, max: 1.00 },

            noise_sigma: 0.04,
            route: RouteConfig::default(),
            rage: RageConfig::default(),
        }
    }
}

impl Default for RouteConfig {
    fn default() -> Self {
        RouteConfig {
            ref_speed_ms: 13.89,
            noise_sigma: 0.05,
            normal_alpha:   (0.30, 0.80),
            sunday_alpha:   (0.00, 0.40),
            pirat_alpha:    (0.70, 1.00),
            cautious_alpha: (0.10, 0.60),
        }
    }
}

impl Default for RageConfig {
    fn default() -> Self {
        RageConfig {
            // [Normal, Sunday, Pirat, Cautious]
            standstill_threshold_s: [45.0, 90.0, 15.0, 60.0],
            decay_rate_linear:      [0.50, 0.20, 2.00, 0.35],
            recovery_rate:          [0.30, 0.20, 0.80, 0.25],
            crawl_fraction:         0.20,
            crawl_threshold_s:      10.0,
            crawl_rate:             [0.20, 0.10, 0.60, 0.15],
            repeat_stop_bonus:      [2.0, 0.5, 8.0, 1.0],
            global_loss_threshold:  80.0,
            global_loss_duration_s: 30.0,
            mass_rage_fraction:     0.05,
        }
    }
}

impl SpeedConfig {
    /// Return the compliance range for `profile`.
    pub fn compliance_for(&self, profile: DriverProfile) -> &ComplianceRange {
        match profile {
            DriverProfile::Normal   => &self.compliance_normal,
            DriverProfile::Sunday   => &self.compliance_sunday,
            DriverProfile::Pirat    => &self.compliance_pirat,
            DriverProfile::Cautious => &self.compliance_cautious,
        }
    }

    /// Return the (min, max) route-alpha range for `profile`.
    pub fn route_alpha_range(&self, profile: DriverProfile) -> (f32, f32) {
        match profile {
            DriverProfile::Normal   => self.route.normal_alpha,
            DriverProfile::Sunday   => self.route.sunday_alpha,
            DriverProfile::Pirat    => self.route.pirat_alpha,
            DriverProfile::Cautious => self.route.cautious_alpha,
        }
    }

    /// Profile index (0=Normal, 1=Sunday, 2=Pirat, 3=Cautious) for array lookups.
    pub fn profile_idx(profile: DriverProfile) -> usize {
        match profile {
            DriverProfile::Normal   => 0,
            DriverProfile::Sunday   => 1,
            DriverProfile::Pirat    => 2,
            DriverProfile::Cautious => 3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compliance_for_matches_profile() {
        let cfg = SpeedConfig::default();
        assert!((cfg.compliance_for(DriverProfile::Pirat).base - 1.35).abs() < 1e-5);
        assert!((cfg.compliance_for(DriverProfile::Sunday).base - 0.95).abs() < 1e-5);
    }

    #[test]
    fn route_alpha_range_ordering() {
        let cfg = SpeedConfig::default();
        let (n_lo, n_hi) = cfg.route_alpha_range(DriverProfile::Normal);
        assert!(n_lo <= n_hi);
        let (p_lo, p_hi) = cfg.route_alpha_range(DriverProfile::Pirat);
        assert!(p_lo >= n_lo, "pirat should prefer faster routing on average");
        assert!(p_hi > n_hi);
    }

    #[test]
    fn profile_idx_is_stable() {
        assert_eq!(SpeedConfig::profile_idx(DriverProfile::Normal), 0);
        assert_eq!(SpeedConfig::profile_idx(DriverProfile::Sunday), 1);
        assert_eq!(SpeedConfig::profile_idx(DriverProfile::Pirat), 2);
        assert_eq!(SpeedConfig::profile_idx(DriverProfile::Cautious), 3);
    }
}
