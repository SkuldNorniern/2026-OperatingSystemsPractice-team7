//! The run's output folder: where the mission writes targets.csv and map.pgm, and whether they exist.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

/// files the mission must leave behind
pub const FILES: [&str; 2] = ["targets.csv", "map.pgm"];

/// the newest folder in `root` by modification time (run_demo.sh makes one per run)
pub fn newest_run_dir(root: &Path) -> Option<PathBuf> {
    std::fs::read_dir(root)
        .ok()?
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .filter_map(|e| Some((e.metadata().ok()?.modified().ok()?, e.path())))
        .max_by_key(|(t, _)| *t)
        .map(|(_, p)| p)
}

/// `~/team7_runs`, where run_demo.sh puts each run
pub fn runs_root() -> PathBuf {
    std::env::var_os("HOME").map_or_else(|| PathBuf::from("."), PathBuf::from).join("team7_runs")
}

#[derive(Debug, Clone, PartialEq)]
pub struct FileStatus {
    pub name: &'static str,
    pub modified: Option<SystemTime>,
}

pub fn status(dir: &Path) -> Vec<FileStatus> {
    FILES.iter().map(|&name| FileStatus { name, modified: std::fs::metadata(dir.join(name)).and_then(|m| m.modified()).ok() }).collect()
}

/// "missing", "written 12 s ago", "written 3 min ago", "written 2 h ago"
pub fn describe(f: &FileStatus, now: SystemTime) -> String {
    let Some(t) = f.modified else { return format!("{}: missing", f.name) };
    let s = now.duration_since(t).map_or(0, |d| d.as_secs());
    let ago = match s {
        0..60 => format!("{s} s"),
        60..3600 => format!("{} min", s / 60),
        _ => format!("{} h", s / 3600),
    };
    format!("{}: written {ago} ago", f.name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn finds_the_newest_run_and_its_files() {
        let root = std::env::temp_dir().join(format!("team7_viz_test_{}", std::process::id()));
        let (old, new) = (root.join("20260101_old"), root.join("20260102_new"));
        std::fs::create_dir_all(&old).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        std::fs::create_dir_all(&new).unwrap();
        std::fs::write(root.join("not_a_dir.txt"), "x").unwrap();
        assert_eq!(newest_run_dir(&root), Some(new.clone()));

        std::fs::write(new.join("targets.csv"), "id,x,y\n").unwrap();
        let s = status(&new);
        assert!(s[0].modified.is_some() && s[1].modified.is_none());
        let now = s[0].modified.unwrap() + Duration::from_secs(125);
        assert_eq!(describe(&s[0], now), "targets.csv: written 2 min ago");
        assert_eq!(describe(&s[1], now), "map.pgm: missing");
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(newest_run_dir(&root), None);
    }
}
