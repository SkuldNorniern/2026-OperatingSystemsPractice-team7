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

turn laptop wifi off during run, or DDS can send camera data over wifi.

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
run_demo.sh nav:=lazy_nav2                                    # our Lazy Theta*, Nav2 MPPI follows it
run_demo.sh nav:=team7 slam_backend:=team7 aruco:=team7       # all ours, Nav2 not started
```

| arg | values | |
|---|---|---|
| `nav` | `nav2`, `lazy_nav2`, `team7` | team7 publishes /cmd_vel itself, no Nav2 running |
| `slam_backend` | `toolbox`, `team7` | both publish /map and map->odom. slam_toolbox grid is half cell off, marker_mapper and saved map.yaml get -0.025 m |
| `aruco` | `opencv`, `team7` | both publish /aruco_detections |
| `scan_filter` | `true` | /scan -> /scan_filtered (range, median, outlier, deskew) for SLAM |

nav2.yaml MPPI is tuned for create3 too: vx_max 0.30, vx_min 0.0 (no reversing), accel 1.0.
aruco_opencv needs reliable camera_info, our detector takes both reliable and best effort.

in nav:=team7 our MPPI still goes through nav2 velocity_smoother and collision_monitor (cmd_vel_nav -> cmd_vel), same as nav2 controller.
collision_monitor uses config/collision_monitor_team7.yaml there because there is no local costmap footprint.
all our nodes have ros2 param services and follow use_sim_time.

after frontiers are done explorer goes to look at walls the camera has not seen yet (lidar maps walls much further than camera reads markers).
orange points in RViz are those camera views. turn off with `camera_coverage: false` in team7.yaml.
when an unconfirmed marker is in view, marker_mapper publishes /speed_limit (nav2_msgs/SpeedLimit, 40%) so less motion blur. nav2 controller and our navigator both follow it.

run_demo.sh undocks if robot is on dock (power_saver keeps lidar and camera off there), calls create3 `/reset_pose` so map origin is start pose, then launches `team7.launch.py`.

run stops when 3 ids are confirmed or `time_limit` (590 s) is over. mission cancels Nav2 goals, stops explore_lite and saves:

```
~/team7_run/map.pgm, map.yaml    final map, map_autosave.* every 60 s
~/team7_run/targets.csv          id,x,y in map frame (m)
```

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
