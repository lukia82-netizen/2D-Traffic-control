use crate::vehicles::driver::DriverParams;
use crate::vehicles::types::VehicleTypeParams;

/// Compute IDM acceleration for a vehicle.
///
/// # Arguments
/// * `v`        – current speed (m/s)
/// * `v0`       – desired speed (m/s)
/// * `s`        – gap to the leader (m); use a large value (e.g. 1000) if no leader
/// * `delta_v`  – speed difference v - v_leader (m/s); 0 if no leader
/// * `params`   – driver profile parameters
/// * `vtype`    – vehicle type parameters (for accel/decel limits)
///
/// Returns clamped acceleration in m/s².
pub fn idm_acceleration(
    v: f32,
    v0: f32,
    s: f32,
    delta_v: f32,
    params: &DriverParams,
    vtype: &VehicleTypeParams,
) -> f32 {
    let a = params.comfort_accel;
    let b = params.comfort_decel;
    let s0 = params.min_gap;
    let t_head = params.time_headway;

    // Desired gap: s*(v, Δv) = s0 + v·T + v·Δv / (2√(a·b))
    let s_star = s0 + v * t_head + (v * delta_v) / (2.0 * (a * b).sqrt());

    // IDM formula: a_idm = a · [1 − (v/v0)^4 − (s*/s)^2]
    let v_ratio = if v0 > 0.0 { v / v0 } else { 0.0 };
    // IDM expects free space between bumpers; never allow non-positive gap.
    let s_clamped = s.max(0.1);
    let s_ratio = s_star / s_clamped;

    let accel = a * (1.0 - v_ratio.powi(4) - s_ratio.powi(2));

    // Clamp to vehicle limits
    accel
        .max(-vtype.max_decel)
        .min(vtype.max_accel)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicles::driver::DriverProfile;
    use crate::vehicles::types::VehicleType;

    #[test]
    fn free_road_accelerates() {
        let params = DriverProfile::Normal.params();
        let vtype = VehicleType::Car.params();
        // No leader (large gap), below desired speed → should get positive accel
        let a = idm_acceleration(5.0, 14.0, 1000.0, 0.0, &params, &vtype);
        assert!(a > 0.0, "expected positive accel, got {}", a);
    }

    #[test]
    fn too_close_brakes() {
        let params = DriverProfile::Normal.params();
        let vtype = VehicleType::Car.params();
        // Very close to leader at high speed → should brake
        let a = idm_acceleration(10.0, 14.0, 1.0, 5.0, &params, &vtype);
        assert!(a < 0.0, "expected negative accel, got {}", a);
    }
}
