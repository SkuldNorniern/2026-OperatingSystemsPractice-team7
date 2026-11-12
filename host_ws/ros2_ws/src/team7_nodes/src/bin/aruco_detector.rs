use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

use futures::StreamExt;
use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r::aruco_opencv_msgs::msg::{ArucoDetection, MarkerPose};
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::{QosProfile, log_info, log_warn};
use tb4_aruco::{CameraModel, Detector, DetectorParams, Dictionary, GrayImage, Marker};
use team7_nodes::Params;
use zune_jpeg::JpegDecoder;
use zune_jpeg::zune_core::colorspace::ColorSpace;
use zune_jpeg::zune_core::options::DecoderOptions;

struct State {
    cam: Option<CameraModel>,
    img: GrayImage,
    det: Detector,
    dict: Dictionary,
    markers: Vec<Marker>,
    frames: u64,
    total_ms: f64,
    warned: bool,
    rgb: Option<(usize, usize, Vec<u8>)>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "aruco_detector", "")?;
    let log = node.logger().to_owned();
    let p = Params(&node);

    let image_topic = p.string("image_topic", "/oakd/rgb/preview/image_raw");
    let info_topic = p.string("camera_info_topic", "");
    let info_topic = if info_topic.is_empty() {
        match image_topic.rsplit_once('/') {
            Some((dir, _)) => format!("{dir}/camera_info"),
            None => "camera_info".to_owned(),
        }
    } else {
        info_topic
    };
    // compressed: subscribe <image_topic>/compressed (jpeg), less wifi load than raw
    let compressed = p.string("image_transport", "compressed") == "compressed";
    let dict_name = p.string("marker_dict", "6X6_250");
    let Some(dict) = Dictionary::by_name(&dict_name) else {
        return Err(format!("unknown marker_dict {dict_name}, one of {}", Dictionary::names().join(" ")).into());
    };
    let marker_size = p.f64("marker_size", 0.10);
    let debug_every = p.i64("debug_every", 3).max(0) as u64;
    let d = DetectorParams::default();
    let dp = DetectorParams {
        max_hamming: p.i64("max_hamming", d.max_hamming as i64) as u32,
        min_side_px: p.f64("min_side_px", d.min_side_px),
        refine_corners: p.bool("refine_corners", d.refine_corners),
        ..d
    };
    log_info!(&log, "{image_topic}{} (+ {info_topic}), {dict_name} ({} ids, corrects {} bits), marker {marker_size} m",
        if compressed { "/compressed" } else { "" }, dict.len(), dp.max_hamming.min(dict.max_correction));

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    team7_nodes::spawn_param_services(&mut node, &spawner)?;
    let state = Rc::new(RefCell::new(State {
        cam: None,
        rgb: None,
        img: GrayImage::new(1, 1),
        det: Detector::new(dp),
        dict,
        markers: Vec::new(),
        frames: 0,
        total_ms: 0.0,
        warned: false,
    }));

    let infos = node.subscribe::<CameraInfo>(&info_topic, QosProfile::sensor_data())?;
    let st = state.clone();
    spawner.spawn_local(infos.for_each(move |m| {
        if m.k.len() == 9 && m.k[0] > 0.0 {
            let k: [f64; 9] = m.k[..9].try_into().unwrap();
            st.borrow_mut().cam = Some(CameraModel::from_k_d(&k, &m.d));
        }
        futures::future::ready(())
    }))?;

    let det_pub = node.create_publisher::<ArucoDetection>("/aruco_detections", QosProfile::default())?;
    let dbg_pub = node.create_publisher::<Image>("/team7/aruco/debug", QosProfile::sensor_data())?;
    let log_in = log.clone();
    let handle = move |s: &mut State, header: &r2r::std_msgs::msg::Header| {
        let Some(cam) = s.cam else {
            if !s.warned {
                log_warn!(&log, "no camera_info yet on {info_topic}");
                s.warned = true;
            }
            return;
        };
        let t0 = Instant::now();
        s.det.detect_into(&s.img, &s.dict, &mut s.markers);
        s.total_ms += t0.elapsed().as_secs_f64() * 1e3;
        s.frames += 1;
        if s.frames % 300 == 0 {
            log_info!(&log, "{} frames, mean detect {:.2} ms", s.frames, s.total_ms / s.frames as f64);
        }

        let mut out = ArucoDetection { header: header.clone(), ..Default::default() };
        for mk in &s.markers {
            let ray = tb4_aruco::marker_bearing(mk, &cam);
            let mut range = tb4_aruco::estimate_range(mk, &cam, marker_size);
            if !range.is_finite() || range <= 0.0 {
                range = 1.0;
            }
            let mut mp = MarkerPose { marker_id: mk.id as u16, ..Default::default() };
            mp.pose.position.x = ray[0] * range;
            mp.pose.position.y = ray[1] * range;
            mp.pose.position.z = ray[2] * range;
            mp.pose.orientation.w = 1.0;
            out.markers.push(mp);
        }
        let _ = det_pub.publish(&out);

        if debug_every > 0 && s.frames % debug_every == 0 {
            let _ = dbg_pub.publish(&debug_image_color(s.rgb.as_ref(), &s.img, &s.markers, header));
        }
    };

    let st = state.clone();
    if compressed {
        let images = node.subscribe::<CompressedImage>(&format!("{image_topic}/compressed"), QosProfile::sensor_data())?;
        let mut bad = false;
        spawner.spawn_local(images.for_each(move |m| {
            let mut guard = st.borrow_mut();
            let s = &mut *guard;
            match decode_jpeg_gray(&m.data) {
                Some((w, h, gray)) if s.img.set_mono8(w, h, w, &gray) => {
                    s.rgb = decode_jpeg_rgb(&m.data);
                    handle(s, &m.header)
                }
                _ if !bad => {
                    bad = true;
                    log_warn!(&log_in, "cannot decode compressed image format '{}'", m.format);
                }
                _ => {}
            }
            futures::future::ready(())
        }))?;
    } else {
        let images = node.subscribe::<Image>(&image_topic, QosProfile::sensor_data())?;
        let mut bad = false;
        spawner.spawn_local(images.for_each(move |m| {
            let mut guard = st.borrow_mut();
            let s = &mut *guard;
            let (w, h, step) = (m.width as usize, m.height as usize, m.step as usize);
            let ok = match m.encoding.as_str() {
                "mono8" => s.img.set_mono8(w, h, step, &m.data),
                "bgr8" => s.img.set_color(w, h, step, &m.data, true),
                "rgb8" => s.img.set_color(w, h, step, &m.data, false),
                _ => false,
            };
            if ok {
                handle(s, &m.header);
            } else if !bad {
                bad = true;
                log_warn!(&log_in, "unsupported image {} {}x{}", m.encoding, w, h);
            }
            futures::future::ready(())
        }))?;
    }

    team7_nodes::run(node, pool, || false);
    Ok(())
}

/// jpeg straight to luma, no colour conversion needed for detection
fn decode_jpeg_gray(data: &[u8]) -> Option<(usize, usize, Vec<u8>)> {
    let opts = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma);
    let mut dec = JpegDecoder::new_with_options(std::io::Cursor::new(data), opts);
    let gray = dec.decode().ok()?;
    let info = dec.info()?;
    let (w, h) = (info.width as usize, info.height as usize);
    (gray.len() == w * h).then_some((w, h, gray))
}

/// colour decode of the same JPEG, for the debug view only (detection stays on gray)
fn decode_jpeg_rgb(data: &[u8]) -> Option<(usize, usize, Vec<u8>)> {
    let opts = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::RGB);
    let mut dec = JpegDecoder::new_with_options(std::io::Cursor::new(data), opts);
    let px = dec.decode().ok()?;
    let info = dec.info()?;
    let (w, h) = (info.width as usize, info.height as usize);
    (px.len() == w * h * 3).then_some((w, h, px))
}

/// colour debug view: marker outlines in green, first corner in red. falls back to gray.
fn debug_image_color(rgb: Option<&(usize, usize, Vec<u8>)>, img: &GrayImage, markers: &[Marker], header: &r2r::std_msgs::msg::Header) -> Image {
    let Some((w, h, px)) = rgb else {
        return debug_image(img, markers, header);
    };
    let (w, h) = (*w, *h);
    if w != img.width || h != img.height || px.len() != w * h * 3 {
        return debug_image(img, markers, header);
    }
    let mut data = px.clone();
    let mut put = |x: f64, y: f64, c: [u8; 3]| {
        let (xi, yi) = (x.round() as i64, y.round() as i64);
        if xi >= 0 && yi >= 0 && (xi as usize) < w && (yi as usize) < h {
            let i = (yi as usize * w + xi as usize) * 3;
            data[i..i + 3].copy_from_slice(&c);
        }
    };
    for mk in markers {
        let c = mk.corners_f64();
        for k in 0..4 {
            let (a, b) = (c[k], c[(k + 1) % 4]);
            let n = ((b[0] - a[0]).hypot(b[1] - a[1]).ceil() as usize).max(1);
            for i in 0..=n {
                let t = i as f64 / n as f64;
                put(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, [0, 255, 0]);
            }
        }
        for dy in -1..=1 {
            for dx in -1..=1 {
                put(c[0][0] + dx as f64, c[0][1] + dy as f64, [255, 0, 0]);
            }
        }
    }
    Image {
        header: header.clone(),
        height: h as u32,
        width: w as u32,
        encoding: "rgb8".into(),
        is_bigendian: 0,
        step: (w * 3) as u32,
        data,
    }
}

/// gray copy with marker outlines in white and the first corner in black
fn debug_image(img: &GrayImage, markers: &[Marker], header: &r2r::std_msgs::msg::Header) -> Image {
    let (w, h) = (img.width, img.height);
    let mut data = img.data.clone();
    let mut put = |x: f64, y: f64, v: u8| {
        let (xi, yi) = (x.round() as i64, y.round() as i64);
        if xi >= 0 && yi >= 0 && (xi as usize) < w && (yi as usize) < h {
            data[yi as usize * w + xi as usize] = v;
        }
    };
    for mk in markers {
        let c = mk.corners_f64();
        for k in 0..4 {
            let (a, b) = (c[k], c[(k + 1) % 4]);
            let n = ((b[0] - a[0]).hypot(b[1] - a[1]).ceil() as usize).max(1);
            for i in 0..=n {
                let t = i as f64 / n as f64;
                put(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, 255);
            }
        }
        for dy in -1..=1 {
            for dx in -1..=1 {
                put(c[0][0] + dx as f64, c[0][1] + dy as f64, 0);
            }
        }
    }
    Image {
        header: header.clone(),
        height: h as u32,
        width: w as u32,
        encoding: "mono8".into(),
        is_bigendian: 0,
        step: w as u32,
        data,
    }
}
