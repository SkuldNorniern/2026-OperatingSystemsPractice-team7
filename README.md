# 2026-OperatingSystemsPractice-team7

TurtleBot 4 final project. robot explores unknown room, makes occupancy grid map and finds 3 ArUco targets in 10 min.
at the end we need map (.pgm/.yaml), target list (id,x,y in map frame) and RViz showing marker ids on map.

this is c_base branch, C version. rust_base has same thing in rust (r2r) with same launch files and params.

algorithms are in libtb4, plain C11 with no ROS in it, tested with ctest.
ROS part is small rclc nodes. slam_toolbox, Nav2, aruco_opencv and explore_lite are used as baseline.

## layout

```
libtb4/                    C11 library + tests
  include/tb4/grid.h         occupancy grid + raycast
  include/tb4/tf.h           TF buffer with interpolation
  include/tb4/explore.h      frontier detection, goal scoring
  include/tb4/markers.h      target position from bearing + map, multi view fusion
host_ws/                   laptop (Ubuntu 24.04, Jazzy), runs everything heavy
  ros2_ws/src/team7_nodes/   rclc nodes: marker_mapper, explorer, mission
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

OS side of the C version: one thread with rclc executor, buffers reused so main loop doesnt allocate, sigaction for SIGINT/SIGTERM,
map_saver_cli started with posix_spawnp and reaped with waitpid, output files written to tmp and renamed.

## start

```bash
cmake -S libtb4 -B build/libtb4 -DTB4_SANITIZE=ON && cmake --build build/libtb4 && ctest --test-dir build/libtb4
```

laptop setup, build and run is in [host_ws/README.md](host_ws/README.md).
robot setup and network is in [rpi4_ws/README.md](rpi4_ws/README.md).
