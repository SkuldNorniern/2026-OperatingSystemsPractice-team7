// cargo run --release -p tb4_aruco --example frames -- <dir of .pgm> [dict]
// one line per frame: file id:cx,cy ... for comparing with other detectors
use tb4_aruco::{Detector, DetectorParams, Dictionary, GrayImage};

fn load_pgm(bytes: &[u8]) -> Option<GrayImage> {
    let mut fields = Vec::new();
    let mut i = 0;
    while fields.len() < 4 {
        while bytes.get(i)?.is_ascii_whitespace() {
            i += 1;
        }
        let s = i;
        while !bytes.get(i)?.is_ascii_whitespace() {
            i += 1;
        }
        fields.push(std::str::from_utf8(&bytes[s..i]).ok()?.to_string());
    }
    let (w, h): (usize, usize) = (fields[1].parse().ok()?, fields[2].parse().ok()?);
    GrayImage::from_mono8(w, h, bytes.get(i + 1..i + 1 + w * h)?)
}

fn main() {
    let mut args = std::env::args().skip(1);
    let dir = args.next().expect("dir");
    let dict = Dictionary::by_name(&args.next().unwrap_or("6X6_250".into())).expect("dict");
    let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "pgm")).collect();
    files.sort();
    let mut det = Detector::new(DetectorParams::default());
    let t0 = std::time::Instant::now();
    for f in &files {
        let img = load_pgm(&std::fs::read(f).unwrap()).unwrap();
        let ms = det.detect(&img, &dict);
        let s: Vec<String> = ms.iter().map(|m| { let c = m.center(); format!("{}:{:.2},{:.2}", m.id, c[0], c[1]) }).collect();
        println!("{} {}", f.file_name().unwrap().to_string_lossy(), s.join(" "));
    }
    eprintln!("{} frames, {:.2} ms each", files.len(), t0.elapsed().as_secs_f64() * 1e3 / files.len().max(1) as f64);
}
