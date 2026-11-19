//! The run's output folder: where the mission writes targets.csv and map.pgm, and whether they exist.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

/// files the mission must leave behind
pub const FILES: [&str; 2] = ["targets.csv", "map.pgm"];

fn mtime(p: &Path) -> Option<SystemTime> {
    std::fs::metadata(p).and_then(|m| m.modified()).ok()
}

/// when a run folder was last active: the folder itself, or its launch log being written
pub fn activity(dir: &Path) -> Option<SystemTime> {
    [mtime(dir), mtime(&dir.join("ros_logs")), mtime(&dir.join("ros_logs").join("console.log"))].into_iter().flatten().max()
}

/// the most recently active folder in `root` (run_demo.sh makes one per run, live or replay);
/// `screens` (our screenshots) is not a run
pub fn newest_run_dir(root: &Path) -> Option<PathBuf> {
    std::fs::read_dir(root)
        .ok()?
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()) && e.file_name() != "screens")
        .filter_map(|e| Some((activity(&e.path())?, e.path())))
        .max_by_key(|(t, _)| *t)
        .map(|(_, p)| p)
}

/// the folder name, cut at the start when long: ".../replay_bag_..._081727"
pub fn short_name(dir: &Path, max: usize) -> String {
    let name = dir.file_name().map_or_else(|| dir.display().to_string(), |n| n.to_string_lossy().into_owned());
    let n = name.chars().count();
    if n <= max { name } else { format!("...{}", name.chars().skip(n + 3 - max).collect::<String>()) }
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
        // an older folder whose launch log is being written is the active run
        std::thread::sleep(Duration::from_millis(20));
        std::fs::create_dir_all(old.join("ros_logs")).unwrap();
        std::fs::write(old.join("ros_logs").join("console.log"), "x").unwrap();
        assert_eq!(newest_run_dir(&root), Some(old.clone()));
        std::fs::create_dir_all(root.join("screens")).unwrap();
        assert_eq!(newest_run_dir(&root), Some(old));
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(newest_run_dir(&root), None);
    }

    #[test]
    fn long_names_are_cut_at_the_start() {
        let p = Path::new("/home/x/team7_runs/replay_rosbag_20261118_183647_714393900_20261119_081727_288998639");
        let s = short_name(p, 30);
        assert_eq!(s.chars().count(), 30);
        assert!(s.starts_with("...") && s.ends_with("081727_288998639"), "{s}");
        assert_eq!(short_name(Path::new("/a/run1"), 30), "run1");
    }
}
