use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use futures::StreamExt;
use futures::executor::{LocalPool, LocalSpawner};
use futures::task::LocalSpawnExt;
use r2r::{ParameterValue, QosProfile};
use tb4_core::nalgebra::{Quaternion, Translation3, UnitQuaternion};
use tb4_core::{Grid, GridMeta, Isometry3, Stamp, TfBuffer};

pub use r2r;

pub fn stamp(t: &r2r::builtin_interfaces::msg::Time) -> Stamp {
    Stamp::from_ros(t.sec, t.nanosec)
}

pub fn iso_from_pose(p: &r2r::geometry_msgs::msg::Pose) -> Isometry3<f64> {
    let q = &p.orientation;
    Isometry3::from_parts(
        Translation3::new(p.position.x, p.position.y, p.position.z),
        UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

pub fn iso_from_transform(t: &r2r::geometry_msgs::msg::Transform) -> Isometry3<f64> {
    let q = &t.rotation;
    Isometry3::from_parts(
        Translation3::new(t.translation.x, t.translation.y, t.translation.z),
        UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z)),
    )
}

pub fn grid_from_msg(m: &r2r::nav_msgs::msg::OccupancyGrid) -> Option<Grid> {
    let i = &m.info;
    if i.width == 0 || i.height == 0 || m.data.len() != i.width as usize * i.height as usize {
        return None;
    }
    let meta = GridMeta {
        width: i.width,
        height: i.height,
        resolution: i.resolution as f64,
        origin_x: i.origin.position.x,
        origin_y: i.origin.position.y,
    };
    Some(Grid::new(meta, m.data.clone()))
}

pub fn pose_msg(x: f64, y: f64, yaw: f64) -> r2r::geometry_msgs::msg::Pose {
    let [qx, qy, qz, qw] = tb4_core::pose::yaw_to_quaternion(yaw);
    r2r::geometry_msgs::msg::Pose {
        position: r2r::geometry_msgs::msg::Point { x, y, z: 0.0 },
        orientation: r2r::geometry_msgs::msg::Quaternion { x: qx, y: qy, z: qz, w: qw },
    }
}

pub struct Params<'a>(pub &'a r2r::Node);

impl Params<'_> {
    fn raw(&self, name: &str) -> ParameterValue {
        self.0
            .params
            .lock()
            .unwrap()
            .get(name)
            .map(|p| p.value.clone())
            .unwrap_or(ParameterValue::NotSet)
    }

    pub fn f64(&self, name: &str, default: f64) -> f64 {
        match self.raw(name) {
            ParameterValue::Double(v) => v,
            ParameterValue::Integer(v) => v as f64,
            _ => default,
        }
    }

    pub fn i64(&self, name: &str, default: i64) -> i64 {
        match self.raw(name) {
            ParameterValue::Integer(v) => v,
            _ => default,
        }
    }

    pub fn bool(&self, name: &str, default: bool) -> bool {
        match self.raw(name) {
            ParameterValue::Bool(v) => v,
            _ => default,
        }
    }

    pub fn string(&self, name: &str, default: &str) -> String {
        match self.raw(name) {
            ParameterValue::String(v) => v,
            _ => default.to_owned(),
        }
    }

    pub fn i64_list(&self, name: &str) -> Vec<i64> {
        match self.raw(name) {
            ParameterValue::IntegerArray(v) => v,
            _ => Vec::new(),
        }
    }
}

/// ros2 param services for this node, also turns on use_sim_time (/clock) when set
pub fn spawn_param_services(node: &mut r2r::Node, spawner: &LocalSpawner) -> r2r::Result<()> {
    let (services, events) = node.make_parameter_handler()?;
    spawner.spawn_local(services).expect("spawn param services");
    spawner.spawn_local(events.for_each(|_| futures::future::ready(()))).expect("spawn param events");
    Ok(())
}

pub type SharedTf = Rc<RefCell<TfBuffer>>;

pub const TF_STATIC_QOS: QosProfile = QosProfile::default().keep_last(100).reliable().transient_local();

pub fn spawn_tf_listener(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    buffer: SharedTf,
    tf_topic: &str,
    tf_static_topic: &str,
) -> r2r::Result<()> {
    type TfMsg = r2r::tf2_msgs::msg::TFMessage;
    let dynamic = node.subscribe::<TfMsg>(tf_topic, QosProfile::default().keep_last(100))?;
    let statics = node.subscribe::<TfMsg>(tf_static_topic, TF_STATIC_QOS)?;
    for (stream, is_static) in [(dynamic, false), (statics, true)] {
        let buf = buffer.clone();
        spawner
            .spawn_local(stream.for_each(move |msg| {
                let mut b = buf.borrow_mut();
                for t in &msg.transforms {
                    b.insert(
                        &t.header.frame_id,
                        &t.child_frame_id,
                        stamp(&t.header.stamp),
                        iso_from_transform(&t.transform),
                        is_static,
                    );
                }
                futures::future::ready(())
            }))
            .expect("spawn tf listener");
    }
    Ok(())
}

pub fn run(mut node: r2r::Node, mut pool: LocalPool, stop: impl Fn() -> bool) {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    let interrupted = Arc::new(AtomicBool::new(false));
    let flag = interrupted.clone();
    if let Err(e) = ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed)) {
        r2r::log_warn!(node.logger(), "cannot install signal handler: {e}");
    }
    while !stop() && !interrupted.load(Ordering::Relaxed) {
        node.spin_once(Duration::from_millis(10));
        pool.run_until_stalled();
    }
}

pub fn write_atomic(path: &std::path::Path, contents: &str) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, contents)?;
    std::fs::rename(tmp, path)
}

pub fn expand_home(p: &str) -> std::path::PathBuf {
    match (p.strip_prefix("~/"), std::env::var_os("HOME")) {
        (Some(rest), Some(home)) => std::path::Path::new(&home).join(rest),
        _ => p.into(),
    }
}

pub mod viz {
    use r2r::visualization_msgs::msg::Marker;

    pub const ADD: i32 = 0;
    pub const DELETEALL: i32 = 3;
    pub const SPHERE: i32 = 2;
    pub const POINTS: i32 = 8;
    pub const TEXT_VIEW_FACING: i32 = 9;

    pub fn color(r: f32, g: f32, b: f32, a: f32) -> r2r::std_msgs::msg::ColorRGBA {
        r2r::std_msgs::msg::ColorRGBA { r, g, b, a }
    }

    pub fn base(frame: &str, ns: &str, id: i32, kind: i32) -> Marker {
        let mut m = Marker::default();
        m.header.frame_id = frame.to_owned();
        m.ns = ns.to_owned();
        m.id = id;
        m.type_ = kind;
        m.action = ADD;
        m.pose.orientation.w = 1.0;
        m
    }

    pub fn delete_all(frame: &str) -> Marker {
        let mut m = Marker::default();
        m.header.frame_id = frame.to_owned();
        m.action = DELETEALL;
        m
    }
}
