# 2026-OperatingSystemsPractice-team7

TurtleBot 4 final project: **explore an unknown room, build an occupancy grid map, and find 3 ArUco targets within 10 minutes.** The output is the map (`.pgm`/`.yaml`), a target list (`id,x,y` in the map frame) and an RViz view of the map with the marker IDs on it.

The design keeps ROS at the edges. Algorithms live in plain Rust crates with no ROS dependency (`cargo test` runs them in under a second). Thin r2r nodes connect them to ROS 2 Jazzy. Standard ROS software (slam_toolbox, Nav2, aruco_opencv, explore_lite) provides the baseline.

## Layout

```
Cargo.toml                 Rust workspace for crates/ (ROS-free)
crates/
  tb4_core/                Pose2, occupancy grid + raycast, TF buffer
  tb4_explore/             frontier detection and goal scoring
  tb4_markers/             marker-size-independent target localisation + multi-view fusion
host_ws/                   LAPTOP (Ubuntu 24.04, ROS 2 Jazzy): runs everything heavy
  ros2_ws/src/team7_nodes/   r2r nodes: marker_mapper, explorer, mission
  ros2_ws/src/team7_bringup/ launch, params (SLAM/Nav2/ArUco/explore_lite), RViz
  ros2_ws/team7.repos        third-party sources (explore_lite)
  scripts/run_demo.sh        reset odom + launch
rpi4_ws/                   ROBOT (TurtleBot 4 Raspberry Pi): factory software only
  scripts/oakd_preview.sh    optional: bigger camera preview for longer ArUco range
Reference/                 course material (C-space demos, time sync scripts)
```

## Pipeline

```
Pi (factory TB4 bringup)        Create 3 driver, RPLIDAR /scan, OAK-D /oakd/rgb/preview/*, /tf
        │ Ethernet (Pi eth0 = 192.168.185.3)
Laptop  slam_toolbox ───────────▶ /map, map→odom
        Nav2 ◀── NavigateToPose ── explorer (team7, Rust)  |or|  explore_lite (baseline)
        aruco_opencv ───────────▶ /aruco_detections
        marker_mapper (Rust) ───▶ /team7/targets (RViz), targets.csv
        mission (Rust) ─────────▶ stop at 3 targets / time limit, cancel Nav2, save map
```

**Target positions don't depend on marker size.** The competition can change marker sizes, which breaks single-marker PnP range estimates. PnP still gives the correct *direction*, so `marker_mapper` casts that bearing through the SLAM map and takes the first occupied cell. The PnP range is only a fallback and a sanity check. Per-ID median fusion over several views rejects one-frame false positives.

## Quick start

```bash
cargo test                                   # ROS-free crates, from the repo root
```

Laptop setup, build and run: [host_ws/README.md](host_ws/README.md).
Robot setup and networking: [rpi4_ws/README.md](rpi4_ws/README.md).
