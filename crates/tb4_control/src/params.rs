#[derive(Debug, Clone, PartialEq)]
pub struct MppiParams {
    pub horizon: usize,
    pub dt: f64,
    pub samples: usize,
    pub iterations: usize,
    pub temperature: f64,
    pub noise_v: f64,
    pub noise_w: f64,
    pub vx_min: f64,
    pub vx_max: f64,
    pub wz_max: f64,
    pub ax_max: f64,
    pub aw_max: f64,
    /// 0 = available parallelism
    pub threads: usize,
    pub seed: u64,
    pub robot_radius: f64,
    pub path_resolution: f64,

    pub path_distance_weight: f64,
    pub path_follow_weight: f64,
    pub follow_lookahead: f64,
    pub path_angle_weight: f64,
    pub heading_lookahead: f64,
    pub goal_weight: f64,
    pub goal_activate_dist: f64,
    pub goal_heading_weight: f64,
    /// distance at which goal heading is scored before xy tolerance is reached; 0 = only after
    pub goal_heading_activate: f64,
    pub obstacle_weight: f64,
    /// soft obstacle cost band beyond robot_radius
    pub inflation: f64,
    pub collision_cost: f64,
    pub smoothness_weight: f64,
    pub twirl_weight: f64,
    pub prefer_forward_weight: f64,

    /// heading error (rad) to the lookahead point that switches to rotate-in-place
    pub rotate_in_place_angle: f64,
    pub rotate_exit_angle: f64,
    pub smoothing: bool,
    pub v_deadband: f64,
    pub w_deadband: f64,
    pub goal_tolerance_xy: f64,
    pub goal_tolerance_yaw: f64,
}

impl Default for MppiParams {
    fn default() -> Self {
        Self {
            horizon: 40,
            dt: 0.05,
            samples: 1000,
            iterations: 1,
            temperature: 0.15,
            noise_v: 0.1,
            noise_w: 0.5,
            vx_min: 0.0,
            vx_max: 0.30,
            wz_max: 1.9,
            ax_max: 1.0,
            aw_max: 3.2,
            threads: 4,
            seed: 42,
            robot_radius: 0.18,
            path_resolution: 0.05,

            path_distance_weight: 4.0,
            path_follow_weight: 6.0,
            follow_lookahead: 0.7,
            path_angle_weight: 1.5,
            heading_lookahead: 0.4,
            goal_weight: 6.0,
            goal_activate_dist: 0.6,
            goal_heading_weight: 4.0,
            goal_heading_activate: 0.0,
            obstacle_weight: 6.0,
            inflation: 0.30,
            collision_cost: 1e6,
            smoothness_weight: 0.3,
            twirl_weight: 0.05,
            prefer_forward_weight: 5.0,

            rotate_in_place_angle: 1.2,
            rotate_exit_angle: 0.35,
            smoothing: true,
            v_deadband: 0.005,
            w_deadband: 0.01,
            goal_tolerance_xy: 0.10,
            goal_tolerance_yaw: 0.15,
        }
    }
}

impl MppiParams {
    /// set one field by name (counts and flags from the number); for sweeps
    pub fn set(&mut self, name: &str, x: f64) -> Result<(), String> {
        let f = match name {
            "horizon" => {
                self.horizon = x as usize;
                return Ok(());
            }
            "samples" => {
                self.samples = x as usize;
                return Ok(());
            }
            "iterations" => {
                self.iterations = x as usize;
                return Ok(());
            }
            "threads" => {
                self.threads = x as usize;
                return Ok(());
            }
            "smoothing" => {
                self.smoothing = x != 0.0;
                return Ok(());
            }
            "dt" => &mut self.dt,
            "temperature" => &mut self.temperature,
            "noise_v" => &mut self.noise_v,
            "noise_w" => &mut self.noise_w,
            "ax_max" => &mut self.ax_max,
            "aw_max" => &mut self.aw_max,
            "robot_radius" => &mut self.robot_radius,
            "path_distance_weight" => &mut self.path_distance_weight,
            "path_follow_weight" => &mut self.path_follow_weight,
            "follow_lookahead" => &mut self.follow_lookahead,
            "path_angle_weight" => &mut self.path_angle_weight,
            "heading_lookahead" => &mut self.heading_lookahead,
            "goal_weight" => &mut self.goal_weight,
            "goal_activate_dist" => &mut self.goal_activate_dist,
            "goal_heading_weight" => &mut self.goal_heading_weight,
            "goal_heading_activate" => &mut self.goal_heading_activate,
            "obstacle_weight" => &mut self.obstacle_weight,
            "inflation" => &mut self.inflation,
            "collision_cost" => &mut self.collision_cost,
            "smoothness_weight" => &mut self.smoothness_weight,
            "twirl_weight" => &mut self.twirl_weight,
            "prefer_forward_weight" => &mut self.prefer_forward_weight,
            "rotate_in_place_angle" => &mut self.rotate_in_place_angle,
            "rotate_exit_angle" => &mut self.rotate_exit_angle,
            "goal_tolerance_xy" => &mut self.goal_tolerance_xy,
            "goal_tolerance_yaw" => &mut self.goal_tolerance_yaw,
            _ => return Err(format!("unknown MPPI parameter {name}")),
        };
        *f = x;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_by_name() {
        let mut p = MppiParams::default();
        p.set("temperature", 0.4).unwrap();
        p.set("samples", 500.0).unwrap();
        p.set("smoothing", 0.0).unwrap();
        assert_eq!((p.temperature, p.samples, p.smoothing), (0.4, 500, false));
        assert!(p.set("nope", 1.0).is_err());
    }
}
