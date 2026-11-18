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

/// # Safety
/// name must be null or a valid null-terminated UTF-8 C string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tb4_detector_create_named(
    name: *const std::ffi::c_char,
) -> *mut std::ffi::c_void {
    if name.is_null() {
        return std::ptr::null_mut();
    }
    catch_unwind(|| {
        let Ok(name) = (unsafe { std::ffi::CStr::from_ptr(name) }).to_str() else {
            return std::ptr::null_mut();
        };
        let Some(dictionary) = Dictionary::by_name(name) else {
            return std::ptr::null_mut();
        };
        Box::into_raw(Box::new(Context {
            detector: Detector::new(DetectorParams::default()),
            dictionary,
            image: GrayImage::default(),
        }))
        .cast()
    })
    .unwrap_or(std::ptr::null_mut())
}
