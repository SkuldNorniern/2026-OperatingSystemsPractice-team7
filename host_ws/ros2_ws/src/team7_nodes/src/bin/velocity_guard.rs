use std::{cell::RefCell, rc::Rc, time::{Duration, Instant}};
use futures::{StreamExt, executor::LocalPool, task::LocalSpawnExt};
use r2r::{QosProfile, geometry_msgs::msg::{Twist, TwistStamped}};
use team7_nodes::Params;

fn forward_only(mut twist: Twist, stale: bool, max_forward: f64, max_turn: f64) -> Twist {
    if stale || !twist.linear.x.is_finite() || !twist.angular.z.is_finite() {
        return Twist::default();
    }
    twist.linear.x = twist.linear.x.clamp(0.0, max_forward);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = twist.angular.z.clamp(-max_turn, max_turn);
    twist
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut node = r2r::Node::create(r2r::Context::create()?, "velocity_guard", "")?;
    let params = Params(&node);
    let input = params.string("input_topic", "/cmd_vel_filtered");
    let output = params.string("output_topic", "/cmd_vel");
    if input == output { return Err("velocity guard input and output must differ".into()); }
    let max_forward = params.f64("max_forward", 0.18);
    let max_turn = params.f64("max_turn", 0.5);
    let timeout = params.f64("command_timeout", 0.5);
    if !max_forward.is_finite() || max_forward < 0.0 || !max_turn.is_finite()
        || max_turn < 0.0 || !timeout.is_finite() || timeout <= 0.0 {
        return Err("invalid velocity guard limits".into());
    }
    let timeout = Duration::from_secs_f64(timeout);
    let clock = node.get_ros_clock();
    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let latest: Rc<RefCell<Option<(Instant, TwistStamped)>>> = Rc::default();
    let qos = QosProfile::default().keep_last(1);
    let sub = node.subscribe::<TwistStamped>(&input, qos.clone())?;
    let state = latest.clone();
    spawner.spawn_local(sub.for_each(move |msg| {
        *state.borrow_mut() = Some((Instant::now(), msg));
        futures::future::ready(())
    }))?;
    let publisher = node.create_publisher::<TwistStamped>(&output, qos)?;
    let mut timer = node.create_wall_timer(Duration::from_millis(50))?;
    r2r::log_info!(node.logger(), "forward-only guard: {input} -> {output}; max {max_forward} m/s, {max_turn} rad/s, timeout {timeout:?}");
    spawner.spawn_local(async move {
        while timer.tick().await.is_ok() {
            let mut command = latest.borrow().as_ref().map(|(at, msg)| {
                let mut msg = msg.clone();
                msg.twist = forward_only(msg.twist, at.elapsed() >= timeout, max_forward, max_turn);
                msg
            }).unwrap_or_default();
            if let Ok(now) = clock.lock().unwrap().get_now() {
                command.header.stamp = r2r::Clock::to_builtin_time(&now);
            }
            let _ = publisher.publish(&command);
        }
    })?;
    team7_nodes::run(node, pool, || false);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reverse_is_blocked_but_turning_is_allowed() {
        let mut t = Twist::default(); t.linear.x = -0.2; t.angular.z = -0.8;
        let t = forward_only(t, false, 0.18, 0.5);
        assert_eq!(t.linear.x, 0.0); assert_eq!(t.angular.z, -0.5);
    }
    #[test]
    fn stale_and_invalid_commands_stop() {
        let mut t = Twist::default(); t.linear.x = 0.1; t.angular.z = 0.4;
        let stopped = forward_only(t.clone(), true, 0.18, 0.5);
        assert_eq!(stopped.linear.x, 0.0); assert_eq!(stopped.angular.z, 0.0);
        t.angular.z = f64::NAN;
        assert_eq!(forward_only(t, false, 0.18, 0.5).linear.x, 0.0);
    }
    #[test]
    fn forward_speed_is_limited() {
        let mut t = Twist::default(); t.linear.x = 1.0;
        assert_eq!(forward_only(t, false, 0.18, 0.5).linear.x, 0.18);
    }
}
