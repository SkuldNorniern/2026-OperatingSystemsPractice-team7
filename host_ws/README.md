# host_ws

laptop workspace. laptop sits on the robot (Ubuntu 24.04, Jazzy) and runs SLAM, Nav2, aruco, exploration, mission and RViz.
pi only runs factory TurtleBot 4 drivers.

```
ros2_ws/
  team7.repos            vcs import list
  src/team7_nodes/       r2r nodes: marker_mapper, explorer, mission, scan_filter, slam, navigator, aruco_detector. uses ../../../../crates
  src/team7_bringup/     launch/team7.launch.py, config/*.yaml, rviz/team7.rviz
  src/external/          m-explore-ros2 from team7.repos (git ignored)
scripts/run_demo.sh      undock if docked, reset odom, launch
```

## setup (once)

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep libclang-dev \
  ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-aruco-opencv ros-jazzy-irobot-create-msgs

# rust from rustup, apt one is too old for edition 2024 (needs 1.85+)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

cd host_ws/ros2_ws
vcs import src < team7.repos
rosdep install --from-paths src --ignore-src -y -r
```

## build

```bash
source /opt/ros/jazzy/setup.bash
cd host_ws/ros2_ws
colcon build --symlink-install --packages-skip multirobot_map_merge
source install/setup.bash
```

In zsh, source `/opt/ros/jazzy/setup.zsh` and `install/setup.zsh` instead.
Only source the workspace install after a successful build. If dependencies were
installed after an earlier build, regenerate the cached r2r bindings before rebuilding:

```bash
cd src/team7_nodes
CARGO_TARGET_DIR=../../build/team7_nodes/cargo cargo clean --profile colcon -p r2r -p r2r_msg_gen -p r2r_rcl
cd ../..
```

team7_nodes runs `cargo build --profile colcon`. first build makes r2r msg bindings and takes ~30s, after that its incremental.
r2r only makes bindings for packages in `src/team7_nodes/.cargo/config.toml`. if a node uses new msg package add it there and to package.xml.
multirobot_map_merge comes with m-explore-ros2 but we dont use it.

## network

pi eth0 is static 192.168.185.3/24, no DHCP. set laptop wired port on same subnet:

```bash
nmcli con add type ethernet ifname <eth-if> con-name tb4 ipv4.method manual ipv4.addresses 192.168.185.10/24
nmcli con up tb4
ping 192.168.185.3
```

no route to create3 192.168.186.0/24 needed on Jazzy, pi republishes create3 with create3_republisher.
check `/etc/turtlebot4/setup.bash` on pi and match it on laptop:

| pi mode (turtlebot4-setup, ROS Setup) | laptop |
|---|---|
| simple discovery (factory) | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, no `ROS_DISCOVERY_SERVER` |
| discovery server | turtlebot4_setup `configure_discovery.sh`, or `ROS_DISCOVERY_SERVER="192.168.185.3:11811;"` and `[ -t 0 ] && export ROS_SUPER_CLIENT=True \|\| export ROS_SUPER_CLIENT=False` |

after changing run `ros2 daemon stop; ros2 daemon start`. if `/scan` is missing in `ros2 topic list` its almost always this.

This host now has the `team7-tb4` wired profile (`enp2s0`, `192.168.185.10/24`).
`run_demo.sh` detects that address and selects `config/fastdds_wired.xml`, binding
DDS to Ethernet and shared memory so Wi-Fi can remain available for internet.
On another host, configure the same wired address or adjust the XML whitelist.
Run only one demo at a time; the wrapper holds a process lock to prevent duplicate
SLAM, navigation, or command publishers. For manual RViz goals, launch with
`explorer:=none` so autonomous frontier goals do not compete with manual goals.

clock sync is needed or TF lookups at image stamps fail. use `Reference/tb4_timesync/`, laptop serves NTP to pi and default subnet already matches the cable.
`chronyc tracking` on both, offset should be under ~10 ms.

## run

put robot on start pose then:

```bash
host_ws/scripts/run_demo.sh                          # our explorer
host_ws/scripts/run_demo.sh explorer:=explore_lite   # baseline
host_ws/scripts/run_demo.sh explorer:=none           # mapping + detection only, drive with teleop
```

switch stock parts and ours:

```bash
run_demo.sh nav:=nav2 slam_backend:=toolbox aruco:=opencv     # all stock (default)
run_demo.sh nav:=lazy_nav2                                    # our Lazy Theta*, Nav2 RPP follows it
run_demo.sh nav:=team7 slam_backend:=team7 aruco:=team7       # all ours, Nav2 not started
```

| arg | values | |
|---|---|---|
| `nav` | `nav2`, `lazy_nav2`, `team7` | team7 runs our MPPI, no Nav2 planner/controller |
| `slam_backend` | `toolbox`, `team7` | both publish /map and map->odom. slam_toolbox grid is half cell off, marker_mapper and saved map.yaml get -0.025 m |
| `aruco` | `opencv`, `team7` | both publish /aruco_detections |
| `scan_filter` | `true` | /scan -> /scan_filtered (range, median, outlier, deskew) for SLAM |

aruco_opencv needs reliable camera_info, our detector takes both reliable and best effort.
our detector reads `<camera_topic>/compressed` (jpeg) by default like aruco_opencv, `marker_dict` in team7.yaml must match aruco.yaml (6X6_250 for test markers 10, 11, ARUCO_ORIGINAL in task sheet). all OpenCV predefined dictionaries work, names as in OpenCV without `DICT_`. bit correction is capped per dictionary so big ones like 4X4_1000 dont match random squares.
marker_mapper writes every detection to `observations.csv` in output dir. `cargo run --release -p tb4_markers --example replay -- <run dir> map` reruns tracker on it offline.

in nav:=team7 our MPPI still goes through nav2 velocity_smoother, collision_monitor and velocity_guard (cmd_vel_nav -> cmd_vel_filtered -> cmd_vel), same as nav2 controller. navigator vx_max/wz_max match guard (0.18 m/s, 0.5 rad/s).
collision_monitor uses config/collision_monitor_team7.yaml there because there is no local costmap footprint.
all our nodes have ros2 param services and follow use_sim_time.

after frontiers are done explorer goes to look at walls the camera has not seen yet (lidar maps walls much further than camera reads markers).
orange points in RViz are those camera views. turn off with `camera_coverage: false` in team7.yaml.
when an unconfirmed marker is in view, marker_mapper publishes /speed_limit (nav2_msgs/SpeedLimit, 40%) so less motion blur. nav2 controller and our navigator both follow it.

run_demo.sh undocks if robot is on dock (power_saver keeps lidar and camera off there), calls create3 `/reset_pose` so map origin is start pose, then launches `team7.launch.py`.

Each invocation now writes a separate `~/team7_runs/<timestamp>/` directory.
Recording starts **before undocking and odometry reset**. It contains an MCAP
rosbag, `recorder.log`, `console.log`, `ros_logs/`, a configuration snapshot,
`run_info.txt` (commit and launch arguments), and `source_changes.patch`.
Map and target outputs go into the same directory. Stop with Ctrl+C and allow
the recorder to finish writing `metadata.yaml` before closing the terminal.

The bag records lidar, odometry, IMU, TF (including latched static TF), maps and
costmaps, compressed camera images and calibration, detections, frontier and
selected-goal topics, navigation feedback/status, velocity commands, hazards,
diagnostics and ROS log messages. Raw camera images are deliberately omitted
to avoid adding a high-bandwidth Wi-Fi subscriber. The detector also uses
compressed camera transport; its dictionary is `6X6_250` for the observed IDs
10 and 11.

For a stationary check that records without undocking, resetting odometry, or
sending exploration goals:

```bash
bash host_ws/scripts/run_demo.sh --no-prepare explorer:=none mission:=false
```

Use `output_dir:=/path/to/run` for a specific directory, or `record:=false` to
disable recording. Direct `ros2 launch team7_bringup team7.launch.py` also
records by default, but the wrapper additionally captures setup and console logs.

After stopping a run (source ROS first; in zsh use `setup.zsh`):

```bash
python3 host_ws/scripts/summarize_run.py ~/team7_runs/<timestamp>
ros2 bag info ~/team7_runs/<timestamp>/rosbag_<timestamp>
# Replay in a separate ROS domain, away from the live robot.
ROS_DOMAIN_ID=77 ros2 bag play ~/team7_runs/<timestamp>/rosbag_<timestamp> --clock
```

The wrapper generates `summary.json` automatically when recording stops. It reports
message counts, sensor median/p95 age, detected IDs, and
reverse command counts. Compare it with goal decisions in `console.log` and
recorder transport-loss warnings when tuning.

Navigation uses reachable frontiers on the inflated global costmap, free-space
path distance, a penalty for turning, and a 0.3 m stand-off from unknown space.
Goals remain active until a terminal result; cancellation cannot overlap the
next goal, and failed regions expire from the blacklist after 45 s. SLAM uses
the asynchronous node to avoid processing a backlog of old scans.
The launch starts only the seven required Nav2 servers, excluding the unused
route, waypoint and docking servers. Large frontier boundaries are segmented
so a boundary wrapping around the robot cannot hide valid goals at startup.

The controller is [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/),
limited to 0.18 m/s forward and 0.5 rad/s rotation. Reverse navigation is blocked
by the controller, the smoother, and a final `velocity_guard` between collision
monitor output (`/cmd_vel_filtered`) and `/cmd_vel`. The guard clamps negative
forward speed to zero and stops commands after 0.5 s without fresh input. Recovery
clears costmaps and waits; it does not back up or spin automatically. These limits
apply to this navigation pipeline; a separate publisher directly on `/cmd_vel`
would bypass the guard.

run stops when 3 ids are confirmed or `time_limit` (590 s) is over. mission cancels Nav2 goals, stops explore_lite and saves:

```
~/team7_runs/<timestamp>/map.pgm, map.yaml    final map, map_autosave.* every 60 s
~/team7_runs/<timestamp>/targets.csv          id,x,y in map frame (m), confirmed only, header written at start
~/team7_runs/<timestamp>/targets_tentative.csv  not confirmed at end (>= 2 views), best guess only
```

map frame origin = robot pose at `/reset_pose`, so coordinates are relative to wherever robot starts.
run_demo.sh warns before start if `/odom` or camera topics are missing.

in RViz confirmed targets are green with `ID n (x, y)`, not confirmed yet are yellow `ID n? kv`.
our parts show up too: Lazy Theta* path (cyan), MPPI predicted trajectory (magenta), SLAM keyframes and loop closures (orange/pink), aruco debug image, filtered scan (off by default).

| launch arg | default | |
|---|---|---|
| `explorer` | `team7` | `team7`, `explore_lite` or `none` |
| `output_dir` | `~/team7_run` | |
| `camera_topic` | `/oakd/rgb/preview/image_raw` | frame `oakd_rgb_camera_optical_frame` |
| `marker_size` | `0.10` | only for PnP fallback |
| `time_limit` | `590.0` | seconds |
| `expected_targets` | `3` | |
| `use_sim_time` | `false` | `true` for gazebo |
| `record` | `true` | record an MCAP rosbag |
| `slam_sync` | `false` | process latest scans asynchronously |
| `rviz`, `slam`, `mission` | `true` | turn parts off for testing (Nav2 follows `nav`) |

node params are in `src/team7_bringup/config/team7.yaml`. nav2.yaml and slam.yaml are turtlebot4_navigation 2.1.1 copies, our changes have `team7:` on them.

## simulation

```bash
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=warehouse     # press play in gazebo
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"           # sim robot starts docked
ros2 launch team7_bringup team7.launch.py use_sim_time:=true
```

sim camera is on same `/oakd/rgb/preview/*` topics but 320x240, not 250x250. need aruco textures in the world to test detection.

## things about the robot

- `/cmd_vel` is TwistStamped on Jazzy, plain Twist goes to `/cmd_vel_unstamped`. nav2.yaml already has `enable_stamped_cmd_vel: true`
- max speed is 0.306 m/s unless create3 `safety_override` is `full`. backup limit blocks reversing until robot drove forward, MPPI `vx_min: -0.35` can get ignored
- bump and cliff reflexes override commands for a moment. `hazard_detection` is not republished, edit `turtlebot4_bringup/config/republisher.yaml` on pi to get it
- `/e_stop` latches the wheels, check `/wheel_status` if it doesnt move
- lidar motor: `/stop_motor`, `/start_motor`
- OAK-D starts ~30 s after boot, wait for `/oakd/rgb/preview/image_raw`

## crates only

`cargo test` at repo root tests crates without ROS. for nodes, `cargo build` in `src/team7_nodes` with ROS sourced.
