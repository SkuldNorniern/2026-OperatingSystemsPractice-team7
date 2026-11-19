//! Per-mode profiles: what the window looks like for team7 or stock runs, live or replayed. Each
//! profile keeps the layers (on, opacity, topic, added layers), the dock (open, width, sections,
//! section heights), the view (zoom, rotation, follow) and the active tool, in
//! `~/.config/team7_viz/<profile>.toml`. It is saved shortly after a change and loaded when the
//! mode is detected; a mode change mid-session swaps profiles live.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::app::{App, Tool};
use crate::chrome::Chrome;
use crate::scene::{self, Binding, LAYERS, LayerSpec, Layers};

/// a change is saved once nothing else changed for this long
pub const DEBOUNCE: Duration = Duration::from_millis(800);

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LayerEntry {
    pub kind: String,
    pub on: bool,
    pub opacity: f32,
    /// "auto" or a topic name
    pub topic: String,
    pub extra: bool,
}

impl Default for LayerEntry {
    fn default() -> Self {
        Self { kind: String::new(), on: true, opacity: 1.0, topic: "auto".into(), extra: false }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DockEntry {
    pub open: bool,
    pub width: f32,
    pub sections: [bool; 4],
    pub heights: [f32; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ViewEntry {
    pub zoom: f64,
    pub rotation: f64,
    pub follow: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Profile {
    pub layers: Vec<LayerEntry>,
    /// "auto" or a topic name
    pub camera_topic: String,
    pub dock: DockEntry,
    pub view: ViewEntry,
    /// "interact", "measure" or "goal"
    pub tool: String,
}

impl Default for DockEntry {
    fn default() -> Self {
        defaults().dock
    }
}

impl Default for ViewEntry {
    fn default() -> Self {
        defaults().view
    }
}

impl Default for Profile {
    fn default() -> Self {
        defaults()
    }
}

/// the built-in profile: a fresh window's state
pub fn defaults() -> Profile {
    let app = App::new(false);
    let chrome = Chrome::new(String::new(), None);
    capture(&app, &chrome)
}

fn binding_str(b: &Binding) -> String {
    match b {
        Binding::Auto => "auto".into(),
        Binding::Pinned(t) => t.clone(),
    }
}

fn binding_of(s: &str) -> Binding {
    if s.is_empty() || s == "auto" { Binding::Auto } else { Binding::Pinned(s.to_owned()) }
}

fn tool_str(t: Tool) -> &'static str {
    match t {
        Tool::Interact => "interact",
        Tool::Measure => "measure",
        Tool::Goal => "goal",
    }
}

/// the state of the window as a profile
pub fn capture(app: &App, chrome: &Chrome) -> Profile {
    Profile {
        layers: app
            .layers
            .list
            .iter()
            .map(|s| LayerEntry { kind: s.info().key.into(), on: s.on, opacity: s.opacity, topic: binding_str(&s.binding), extra: s.extra })
            .collect(),
        camera_topic: binding_str(&app.layers.camera),
        dock: DockEntry { open: chrome.dock_open, width: chrome.dock_w, sections: chrome.sections, heights: chrome.layout.heights },
        view: ViewEntry { zoom: app.camera.zoom, rotation: app.camera.rotation, follow: app.camera.follow },
        tool: tool_str(app.tool).into(),
    }
}

/// put a profile into the window; unknown layer kinds are skipped
pub fn apply(p: &Profile, app: &mut App, chrome: &mut Chrome) {
    let mut layers = Layers { camera: binding_of(&p.camera_topic), ..Layers::default() };
    for e in &p.layers {
        let Some(info) = LAYERS.iter().find(|i| i.key == e.kind) else { continue };
        let spec = LayerSpec { kind: info.layer, on: e.on, opacity: e.opacity.clamp(0.0, 1.0), binding: binding_of(&e.topic), extra: e.extra };
        if e.extra {
            if scene::EXTRA_KINDS.contains(&info.layer) {
                layers.list.push(spec);
            }
        } else if let Some(s) = layers.list.iter_mut().find(|s| s.kind == info.layer && !s.extra) {
            *s = spec;
        }
    }
    app.layers = layers;
    app.camera.zoom = if p.view.zoom.is_finite() && p.view.zoom > 0.0 { p.view.zoom } else { 1.0 };
    app.camera.rotation = if p.view.rotation.is_finite() { p.view.rotation } else { 0.0 };
    app.camera.follow = p.view.follow;
    app.camera.centre = None;
    app.tool = match p.tool.as_str() {
        "measure" => Tool::Measure,
        "goal" => Tool::Goal,
        _ => Tool::Interact,
    };
    chrome.dock_open = p.dock.open;
    if p.dock.width.is_finite() && p.dock.width > 0.0 {
        chrome.dock_w = p.dock.width;
    }
    chrome.sections = p.dock.sections;
    if p.dock.heights.iter().all(|h| h.is_finite() && *h > 0.0) {
        chrome.layout.heights = p.dock.heights;
    }
}

/// the profile name of a mode: stack (team7 or stock; a mixed graph counts as team7) and source
/// (replay or live); None while the stack is unknown
pub fn mode_key(stack: &str, replay: bool) -> Option<String> {
    let s = match stack {
        "team7" | "mixed" => "team7",
        "stock" => "stock",
        _ => return None,
    };
    Some(format!("{s}-{}", if replay { "replay" } else { "live" }))
}

/// a run folder counts as the current run while its log was written this recently
const RUN_ACTIVE: Duration = Duration::from_secs(120);

/// replay or live: ROS_DOMAIN_ID 79 is the replay domain, and run_demo.sh names replay run
/// folders `replay_*` (only a folder still being written counts: an old replay folder must not
/// turn a live session into a replay)
pub fn is_replay(domain: &str, run_dir: Option<&Path>) -> bool {
    domain == "79"
        || run_dir.is_some_and(|d| {
            let replay = d.file_name().is_some_and(|n| n.to_string_lossy().starts_with("replay_"));
            let active = crate::outputs::activity(d).and_then(|t| t.elapsed().ok()).is_some_and(|a| a < RUN_ACTIVE);
            replay && active
        })
}

/// `$XDG_CONFIG_HOME/team7_viz`, else `~/.config/team7_viz`
pub fn config_dir() -> PathBuf {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("team7_viz")
}

/// read a profile file; Err with one line saying why
pub fn read(path: &Path) -> Result<Profile, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    toml::from_str(&text).map_err(|e| format!("{}: {}", path.display(), e.message()))
}

pub fn write(path: &Path, p: &Profile) -> Result<(), String> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d).map_err(|e| e.to_string())?;
    }
    let text = toml::to_string_pretty(p).map_err(|e| e.to_string())?;
    // write then rename, so a crash never leaves half a file
    let tmp = path.with_extension("toml.tmp");
    std::fs::write(&tmp, text).map_err(|e| e.to_string())?;
    std::fs::rename(&tmp, path).map_err(|e| e.to_string())
}

/// the active profile, its saving and its switching
pub struct Profiles {
    dir: PathBuf,
    forced: Option<String>,
    pub active: Option<String>,
    saved: Option<Profile>,
    changed_at: Option<Instant>,
    /// lines for the user (a missing or bad file, a failed save), taken by the window
    pub messages: Vec<String>,
}

impl Profiles {
    pub fn new(dir: PathBuf, forced: Option<String>) -> Self {
        Self { dir, forced, active: None, saved: None, changed_at: None, messages: Vec::new() }
    }

    pub fn path(&self, name: &str) -> PathBuf {
        self.dir.join(format!("{name}.toml"))
    }

    pub fn default_path(&self, name: &str) -> PathBuf {
        self.dir.join(format!("{name}.default.toml"))
    }

    /// the mode's own default: the saved "default for this mode", else the built-in one
    fn mode_default(&mut self, name: &str) -> Profile {
        let path = self.default_path(name);
        if path.exists() {
            match read(&path) {
                Ok(p) => return p,
                Err(e) => self.messages.push(format!("profile default {e}; using built-in defaults")),
            }
        }
        defaults()
    }

    /// the profile file of `name`; a missing or bad file gives the mode's default and one line
    pub fn load(&mut self, name: &str) -> Profile {
        let path = self.path(name);
        match read(&path) {
            Ok(p) => p,
            Err(e) => {
                let why = if path.exists() { format!("profile {e}; using defaults") } else { format!("no saved profile {name}; using defaults") };
                self.messages.push(why);
                self.mode_default(name)
            }
        }
    }

    fn save(&mut self, name: &str, p: &Profile) {
        if let Err(e) = write(&self.path(name), p) {
            self.messages.push(format!("cannot save profile {name}: {e}"));
        }
    }

    /// call after each frame: switches to the profile of `mode` (or the forced one), and saves
    /// the active one shortly after it changed. True when it switched.
    pub fn update(&mut self, mode: Option<String>, app: &mut App, chrome: &mut Chrome, now: Instant) -> bool {
        let want = self.forced.clone().or(mode);
        if let Some(name) = want
            && Some(&name) != self.active.as_ref()
        {
            self.flush(app, chrome);
            let p = self.load(&name);
            apply(&p, app, chrome);
            self.saved = Some(capture(app, chrome));
            self.active = Some(name);
            self.changed_at = None;
            return true;
        }
        let Some(name) = self.active.clone() else { return false };
        let snap = capture(app, chrome);
        if Some(&snap) == self.saved.as_ref() {
            self.changed_at = None;
            return false;
        }
        let since = *self.changed_at.get_or_insert(now);
        if now.saturating_duration_since(since) >= DEBOUNCE {
            self.save(&name, &snap);
            self.saved = Some(snap);
            self.changed_at = None;
        }
        false
    }

    /// save an unsaved change now (mode switch, exit)
    pub fn flush(&mut self, app: &App, chrome: &Chrome) {
        let Some(name) = self.active.clone() else { return };
        let snap = capture(app, chrome);
        if Some(&snap) != self.saved.as_ref() {
            self.save(&name, &snap);
            self.saved = Some(snap);
        }
        self.changed_at = None;
    }

    /// back to the mode's default (its saved default, else built-in)
    pub fn reset(&mut self, app: &mut App, chrome: &mut Chrome) {
        let Some(name) = self.active.clone() else { return };
        let p = self.mode_default(&name);
        apply(&p, app, chrome);
    }

    /// the current state becomes this mode's default
    pub fn save_as_default(&mut self, app: &App, chrome: &Chrome) {
        let Some(name) = self.active.clone() else { return };
        if let Err(e) = write(&self.default_path(&name), &capture(app, chrome)) {
            self.messages.push(format!("cannot save default for {name}: {e}"));
        } else {
            self.messages.push(format!("saved as the default for {name}"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::Layer;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("team7_viz_profiles_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn changed(app: &mut App, chrome: &mut Chrome) {
        app.layers.toggle(Layer::Scan);
        app.layers.set_opacity(Layer::GlobalCost, 0.7);
        app.layers.list[scene::index(Layer::Path)].binding = Binding::Pinned("/plan_b".into());
        app.layers.add(Layer::Scan, Some("/scan_filtered".into()));
        app.layers.camera = Binding::Pinned("/aruco_tracker/debug".into());
        app.camera.zoom = 2.5;
        app.camera.rotation = 0.4;
        app.camera.follow = false;
        app.tool = Tool::Measure;
        chrome.dock_open = false;
        chrome.dock_w = 410.0;
        chrome.sections = [true, false, true, true];
        chrome.layout.heights = [300.0, 120.0, 200.0, 90.0];
    }

    #[test]
    fn a_profile_round_trips_through_its_file() {
        let (mut app, mut chrome) = (App::new(false), Chrome::new(String::new(), None));
        changed(&mut app, &mut chrome);
        let p = capture(&app, &chrome);
        let dir = tmp("round");
        let path = dir.join("team7-live.toml");
        write(&path, &p).unwrap();
        let back = read(&path).unwrap();
        assert_eq!(back, p);
        let (mut app2, mut chrome2) = (App::new(false), Chrome::new(String::new(), None));
        apply(&back, &mut app2, &mut chrome2);
        assert_eq!(app2.layers, app.layers);
        assert_eq!((app2.camera.zoom, app2.camera.rotation, app2.camera.follow), (2.5, 0.4, false));
        assert_eq!(app2.tool, Tool::Measure);
        assert_eq!((chrome2.dock_open, chrome2.dock_w, chrome2.sections, chrome2.layout.heights), (false, 410.0, [true, false, true, true], [300.0, 120.0, 200.0, 90.0]));
        assert_eq!(capture(&app2, &chrome2), p);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_bad_or_missing_file_falls_back_to_defaults_with_one_line() {
        let dir = tmp("bad");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("stock-live.toml"), "layers = 7\n[[[ not toml").unwrap();
        let mut ps = Profiles::new(dir.clone(), None);
        let p = ps.load("stock-live");
        assert_eq!(p, defaults());
        assert_eq!(ps.messages.len(), 1, "{:?}", ps.messages);
        assert!(ps.messages[0].contains("stock-live.toml"), "{}", ps.messages[0]);
        let p = ps.load("team7-replay");
        assert_eq!(p, defaults());
        assert_eq!(ps.messages.len(), 2);
        assert!(ps.messages[1].contains("no saved profile team7-replay"));
        // a partial file keeps the defaults for what it does not say
        std::fs::write(dir.join("team7-live.toml"), "tool = \"goal\"\n[view]\nzoom = 3.0\n").unwrap();
        let p = ps.load("team7-live");
        assert_eq!((p.tool.as_str(), p.view.zoom, p.view.follow), ("goal", 3.0, defaults().view.follow));
        assert_eq!(p.layers, defaults().layers);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn modes_switch_profiles_live_and_changes_are_saved() {
        let dir = tmp("switch");
        let (mut app, mut chrome) = (App::new(false), Chrome::new(String::new(), None));
        let mut ps = Profiles::new(dir.clone(), None);
        let t0 = Instant::now();
        // no stack yet: nothing loaded
        assert!(!ps.update(mode_key("-", false), &mut app, &mut chrome, t0));
        assert_eq!(ps.active, None);
        // team7 live detected: its (missing) profile gives defaults
        assert!(ps.update(mode_key("team7", false), &mut app, &mut chrome, t0));
        assert_eq!(ps.active.as_deref(), Some("team7-live"));
        changed(&mut app, &mut chrome);
        // saved only after the debounce
        ps.update(mode_key("team7", false), &mut app, &mut chrome, t0);
        assert!(!ps.path("team7-live").exists());
        ps.update(mode_key("team7", false), &mut app, &mut chrome, t0 + DEBOUNCE + Duration::from_millis(10));
        assert!(ps.path("team7-live").exists());
        let team7 = capture(&app, &chrome);

        // the graph turns stock mid-session: stock-replay is loaded (defaults), team7 kept on disk
        assert!(ps.update(mode_key("stock", true), &mut app, &mut chrome, t0 + Duration::from_secs(5)));
        assert_eq!(ps.active.as_deref(), Some("stock-replay"));
        assert_eq!(capture(&app, &chrome), defaults());
        app.camera.zoom = 4.0;
        ps.flush(&app, &chrome);
        // and back: team7's state returns
        assert!(ps.update(mode_key("mixed", false), &mut app, &mut chrome, t0 + Duration::from_secs(9)));
        assert_eq!(capture(&app, &chrome), team7);
        assert_eq!(read(&ps.path("stock-replay")).unwrap().view.zoom, 4.0);

        // save as default, change, reset: back to the saved default
        ps.save_as_default(&app, &chrome);
        app.camera.zoom = 9.0;
        ps.reset(&mut app, &mut chrome);
        assert_eq!(capture(&app, &chrome), team7);
        // a forced profile ignores the mode
        let mut forced = Profiles::new(dir.clone(), Some("demo".into()));
        forced.update(mode_key("stock", false), &mut app, &mut chrome, t0);
        assert_eq!(forced.active.as_deref(), Some("demo"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn replay_or_live() {
        assert!(is_replay("79", None));
        let dir = tmp("mode");
        let (replay, live) = (dir.join("replay_bag_1"), dir.join("20261119_1"));
        std::fs::create_dir_all(&replay).unwrap();
        std::fs::create_dir_all(&live).unwrap();
        // a replay folder being written: replay; a live folder: live; a folder that does not exist
        // (an old one cleaned up) never counts
        assert!(is_replay("0", Some(&replay)));
        assert!(!is_replay("0", Some(&live)));
        assert!(!is_replay("0", Some(Path::new("/nonexistent/replay_old"))));
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(mode_key("stock", false).as_deref(), Some("stock-live"));
        assert_eq!(mode_key("-", true), None);
    }
}
