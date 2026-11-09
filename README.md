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
  tb4_scan/                scan range/median/outlier filter, motion deskew
  tb4_slam/                own 2D SLAM: correlative matching + point-to-line ICP, keyframes, loop closure, pose graph
  tb4_planning/            C-space inflation, Lazy Theta*, A*
  tb4_control/             own MPPI, multithreaded rollouts, create3 limits
  tb4_aruco/               own ArUco detector (all OpenCV dictionaries), camera model with rational_polynomial
host_ws/                   laptop (Ubuntu 24.04, Jazzy), runs everything heavy
  ros2_ws/src/team7_nodes/   r2r nodes: marker_mapper, explorer, mission, scan_filter, slam, navigator, aruco_detector
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
first occupied cell is up to one cell short, so it fits a line to wall cells around the hit and uses where ray meets that line.
if rays a bit left and right of the marker land at different depth, hit is at an edge. if something nearer is beside it, ray clips an obstacle and is not used.
if one side runs past (marker near end of small box face), hit is used only when PnP range agrees with it, so a ray clipping past the end of a wall in front is still dropped.
each id needs a few views that agree before it counts. views are fused by weighted least squares, each view trusted along ray by how good its range is (head on wall hit good, grazing or PnP bad) and across ray by bearing error.
confirmed when fused position is tight and views agree, and stays confirmed unless views clearly fall apart.

every part has stock ROS version and ours, switch with launch args (host_ws/README.md):
aruco_opencv or our detector, slam_toolbox or our SLAM, Nav2 NavFn+RPP or our Lazy Theta* with Nav2 RPP or with our MPPI.

## start

```bash
cargo test
```

laptop setup, build and run is in [host_ws/README.md](host_ws/README.md).
robot setup and network is in [rpi4_ws/README.md](rpi4_ws/README.md).
