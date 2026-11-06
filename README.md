# 2026-OperatingSystemsPractice-team7

TurtleBot 4 final project. robot explores unknown room, makes occupancy grid map and finds 3 ArUco targets in 10 min.
at the end we need map (.pgm/.yaml), target list (id,x,y in map frame) and RViz showing marker ids on map.

this is rust_base branch, rust version. c_base has same thing in C (rclc) with same launch files and params.

algorithms are plain rust crates with no ROS in them, `cargo test` runs them.
ROS part is small r2r nodes. slam_toolbox, Nav2, aruco_opencv and explore_lite are used as baseline.

## layout

```
Cargo.toml                 rust workspace for crates/
crates/
  tb4_core/                Pose2, occupancy grid + raycast, TF buffer
  tb4_explore/             frontier detection, goal scoring
  tb4_markers/             target position from bearing + map, multi view fusion
host_ws/                   laptop (Ubuntu 24.04, Jazzy), runs everything heavy
  ros2_ws/src/team7_nodes/   r2r nodes: marker_mapper, explorer, mission
  ros2_ws/src/team7_bringup/ launch, params, RViz config
  ros2_ws/team7.repos        explore_lite source
  scripts/run_demo.sh        undock, reset odom, launch
rpi4_ws/                   robot pi, factory software only
  scripts/oakd_preview.sh    bigger camera preview if aruco range is too short
Reference/                 class material (C-space demos, time sync)
```

## how it runs

```
Pi      create3_republisher, RPLIDAR /scan, OAK-D /oakd/rgb/preview/*, /tf
          | ethernet (pi eth0 = 192.168.185.3)
Laptop  slam_toolbox  -> /map, map->odom
        explorer (ours) or explore_lite -> NavigateToPose -> Nav2
        aruco_opencv  -> /aruco_detections
        marker_mapper -> /team7/targets (RViz), targets.csv
        mission       -> stops at 3 targets or time limit, cancels Nav2, saves map
```

marker size can change on demo day so PnP range from one marker is not reliable. direction from camera is still right.
so marker_mapper casts that direction through the SLAM map and takes first occupied cell. PnP range is only fallback.
each id needs a few views that agree before it counts.

## start

```bash
cargo test
```

laptop setup, build and run is in [host_ws/README.md](host_ws/README.md).
robot setup and network is in [rpi4_ws/README.md](rpi4_ws/README.md).
