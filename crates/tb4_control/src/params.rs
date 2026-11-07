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
