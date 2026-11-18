//! Small C ABI around the unchanged rust_base detector. No ROS or OpenCV dependencies.
use std::panic::{AssertUnwindSafe, catch_unwind};
use tb4_aruco::{Detector, DetectorParams, Dictionary, GrayImage};

struct Context {
    detector: Detector,
    dictionary: Dictionary,
    image: GrayImage,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Detection {
    pub id: u32,
    /// Printed marker order: top-left, top-right, bottom-right, bottom-left.
    pub corners: [f32; 8],
    pub hamming: u32,
}

#[unsafe(no_mangle)]
pub extern "C" fn tb4_detector_create() -> *mut std::ffi::c_void {
    catch_unwind(|| {
        Box::into_raw(Box::new(Context {
            detector: Detector::new(DetectorParams::default()),
            dictionary: Dictionary::by_name("5X5_50").expect("built-in dictionary"),
            image: GrayImage::default(),
        }))
        .cast()
    })
    .unwrap_or(std::ptr::null_mut())
}

/// # Safety
/// `context` must be a live pointer returned by create, used by one thread at a time.
/// `data` addresses `len` bytes, and `output` addresses `capacity` Detection structs.
/// Returns written detections, -1 for invalid input, -2 for an internal panic.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tb4_detector_detect(
    context: *mut std::ffi::c_void,
    data: *const u8,
    len: usize,
    width: usize,
    height: usize,
    output: *mut Detection,
    capacity: usize,
) -> i32 {
    if context.is_null()
        || data.is_null()
        || output.is_null()
        || width == 0
        || height == 0
        || width > 8192
        || height > 8192
        || width.checked_mul(height).is_none_or(|n| n > len)
        || capacity == 0
        || capacity > 4096
    {
        return -1;
    }
    catch_unwind(AssertUnwindSafe(|| {
        let ctx = unsafe { &mut *context.cast::<Context>() };
        let data = unsafe { std::slice::from_raw_parts(data, len) };
        if !ctx.image.set_mono8(width, height, width, data) {
            return -1;
        }
        let markers = ctx.detector.detect(&ctx.image, &ctx.dictionary);
        let count = markers.len().min(capacity);
        for (i, marker) in markers.iter().take(count).enumerate() {
            let result = Detection {
                id: marker.id,
                corners: std::array::from_fn(|j| marker.corners[j / 2][j % 2]),
                hamming: marker.hamming,
            };
            unsafe {
                output.add(i).write(result);
            }
        }
        count as i32
    }))
    .unwrap_or(-2)
}

/// # Safety
/// `context` must be null or a live create pointer which is never used again.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tb4_detector_destroy(context: *mut std::ffi::c_void) {
    if !context.is_null() {
        drop(unsafe { Box::from_raw(context.cast::<Context>()) });
    }
}

// V2: reuse rust_base's frontier scoring and camera wall-coverage algorithms.
use tb4_core::{Grid, GridMeta};
use tb4_explore::{
    Coverage, CoverageParams, ExploreParams, find_frontiers, select_goal_with_heading,
};

struct ExplorerContext {
    coverage: Coverage,
    explore: ExploreParams,
    views: CoverageParams,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ExploreGoal {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    /// 1 = frontier, 2 = camera coverage viewpoint.
    pub kind: u32,
}

#[unsafe(no_mangle)]
pub extern "C" fn tb4_explorer_create() -> *mut std::ffi::c_void {
    Box::into_raw(Box::new(ExplorerContext {
        coverage: Coverage::new(),
        explore: ExploreParams {
            distance_weight: 1.5,
            goal_standoff: 0.3,
            min_goal_distance: 0.65,
            frontier_segment_length: 1.2,
            blacklist_radius: 0.7,
            ..ExploreParams::default()
        },
        views: CoverageParams {
            half_fov: 0.6,
            cam_offset: 0.13,
            standoff: 1.6,
            max_cluster_cells: 32,
            ..CoverageParams::default()
        },
    }))
    .cast()
}

/// # Safety
/// Pointers must address their declared lengths; context must be exclusively used.
/// map/costmap are row-major occupancy (-1 unknown, 0 free, 100 occupied).
/// blacklist contains n_blacklist consecutive x,y pairs.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tb4_explorer_goals(
    context: *mut std::ffi::c_void,
    map: *const i8,
    costmap: *const i8,
    width: u32,
    height: u32,
    resolution: f64,
    ox: f64,
    oy: f64,
    x: f64,
    y: f64,
    yaw: f64,
    blacklist: *const f64,
    n_blacklist: usize,
    output: *mut ExploreGoal,
    capacity: usize,
) -> i32 {
    if context.is_null()
        || map.is_null()
        || costmap.is_null()
        || output.is_null()
        || width == 0
        || height == 0
        || width > 4096
        || height > 4096
        || capacity == 0
        || capacity > 128
        || n_blacklist > 4096
        || (n_blacklist > 0 && blacklist.is_null())
        || ![resolution, ox, oy, x, y, yaw]
            .iter()
            .all(|v| v.is_finite())
        || resolution <= 0.0
    {
        return -1;
    }
    catch_unwind(AssertUnwindSafe(|| {
        let ctx = unsafe { &mut *context.cast::<ExplorerContext>() };
        let len = width as usize * height as usize;
        let meta = GridMeta {
            width,
            height,
            resolution,
            origin_x: ox,
            origin_y: oy,
        };
        let grid = Grid::new(
            meta,
            unsafe { std::slice::from_raw_parts(map, len) }.to_vec(),
        );
        let costs = Grid::new(
            meta,
            unsafe { std::slice::from_raw_parts(costmap, len) }.to_vec(),
        );
        let blocked: Vec<(f64, f64)> = if n_blacklist == 0 {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(blacklist, n_blacklist * 2) }
                .chunks_exact(2)
                .map(|p| (p[0], p[1]))
                .collect()
        };
        ctx.coverage.update(&grid, (x, y, yaw), &ctx.views);
        let mut frontiers = find_frontiers(&costs, (x, y), &ctx.explore);
        let mut result = Vec::new();
        while result.len() < capacity {
            let Some(f) =
                select_goal_with_heading(&frontiers, &blocked, &ctx.explore, (x, y), yaw, 0.5)
            else {
                break;
            };
            let goal = f.goal;
            result.push(ExploreGoal {
                x: goal.0,
                y: goal.1,
                yaw: (f.centroid.1 - goal.1).atan2(f.centroid.0 - goal.0),
                kind: 1,
            });
            frontiers.retain(|f| f.goal != goal);
        }
        if result.is_empty() {
            for v in ctx.coverage.views(&grid, (x, y), &ctx.views) {
                if blocked.iter().any(|b| (b.0 - v.x).hypot(b.1 - v.y) < 0.7) {
                    continue;
                }
                result.push(ExploreGoal {
                    x: v.x,
                    y: v.y,
                    yaw: v.yaw,
                    kind: 2,
                });
                if result.len() >= capacity {
                    break;
                }
            }
        }
        for (i, g) in result.iter().enumerate() {
            unsafe {
                output.add(i).write(*g);
            }
        }
        result.len() as i32
    }))
    .unwrap_or(-2)
}

/// # Safety
/// context must be null or a live explorer pointer which is never used again.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tb4_explorer_destroy(context: *mut std::ffi::c_void) {
    if !context.is_null() {
        drop(unsafe { Box::from_raw(context.cast::<ExplorerContext>()) });
    }
}
