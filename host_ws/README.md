# host_ws: laptop workspace

Runs on the laptop that rides on the robot (Ubuntu 24.04, ROS 2 Jazzy). Everything compute-heavy runs here: SLAM, Nav2, ArUco detection, exploration, the mission logic and RViz. The Pi only runs the factory TurtleBot 4 drivers.

```
ros2_ws/
├── team7.repos              third-party sources (vcs import)
└── src/
    ├── team7_nodes/         ament_cmake wrapper around cargo (r2r): marker_mapper, explorer, mission
    ├── team7_bringup/       launch/team7.launch.py, config/*.yaml, rviz/team7.rviz
    └── external/            (git-ignored) m-explore-ros2, from team7.repos
scripts/run_demo.sh          reset odometry, then launch
```

## One-time setup

```bash
# ROS 2 Jazzy desktop is assumed to be installed already.
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep libclang-dev \
  ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-aruco-opencv ros-jazzy-irobot-create-msgs

# Rust: use rustup, not apt (edition 2024 needs >= 1.85)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

cd host_ws/ros2_ws
vcs import src < team7.repos           # explore_lite (baseline explorer)
rosdep install --from-paths src --ignore-src -y -r
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd host_ws/ros2_ws
colcon build --symlink-install --packages-skip multirobot_map_merge
source install/setup.bash
```

- `team7_nodes` runs `cargo build --profile colcon`. The first build generates r2r message bindings (about 30 s). After that, rebuilds are incremental.
- r2r only generates bindings for the packages listed in `src/team7_nodes/.cargo/config.toml`. When a node starts using a new message package, add it there and to `package.xml`.
- `multirobot_map_merge` ships in the m-explore-ros2 repo, but we don't use it.

## Network (laptop ↔ Pi over LAN cable)

The Pi's `eth0` defaults to the static address `192.168.185.3/24`. Give the laptop's wired port an address on the same subnet:

```bash
nmcli con add type ethernet ifname <eth-if> con-name tb4 ipv4.method manual ipv4.addresses 192.168.185.10/24
nmcli con up tb4
ping 192.168.185.3
```

Both machines must use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`, and the same discovery mode (simple discovery or discovery server). Read the robot's settings with `turtlebot4-setup` on the Pi and copy them into the laptop's `~/.bashrc`. If `ros2 topic list` on the laptop doesn't show `/scan`, this is almost always the cause.

Clock sync matters: TF lookups at image timestamps fail when the clocks drift apart. Use `Reference/tb4_timesync/`. Its default subnet `192.168.185.0/24` already matches the wired link.

## Run

Put the robot at the start pose, then run:

```bash
host_ws/scripts/run_demo.sh                          # team7 Rust explorer
host_ws/scripts/run_demo.sh explorer:=explore_lite   # baseline explorer
host_ws/scripts/run_demo.sh explorer:=none           # mapping + detection only (drive with teleop)
```

`run_demo.sh` calls the Create 3 `/reset_pose` service first, so that odom (and therefore the SLAM `map` frame) starts at the start pose, as the rules require. Then it runs `ros2 launch team7_bringup team7.launch.py`.

The run ends when 3 IDs are confirmed or `time_limit` (default 590 s) runs out. The mission node then cancels Nav2 goals, stops explore_lite, and saves:

```
~/team7_run/map.pgm, map.yaml    final map (also map_autosave.* every 60 s)
~/team7_run/targets.csv          id,x,y (map frame, m), rewritten whenever a target is confirmed
```

RViz shows confirmed targets as green spheres labelled `ID n (x, y)`, and candidates as yellow `ID n? kv`.

| Launch argument | Default | |
|---|---|---|
| `explorer` | `team7` | `team7`, `explore_lite`, or `none` |
| `output_dir` | `~/team7_run` | map + CSV destination |
| `camera_topic` | `/oakd/rgb/preview/image_raw` | check with `ros2 topic list` on the robot |
| `marker_size` | `0.10` | only used for the PnP fallback range |
| `time_limit` | `590.0` | seconds |
| `expected_targets` | `3` | |
| `rviz`, `slam`, `nav2`, `mission` | `true` | turn parts off for testing |

Node parameters (thresholds, timeouts, scoring weights) are in `src/team7_bringup/config/team7.yaml`. `nav2.yaml` and `slam.yaml` are copies of the turtlebot4_navigation 2.1.1 defaults; our changes are marked `team7:`.

## Quick ROS-free iteration

The algorithms live in `/crates`, so `cargo test` at the repo root tests them without ROS. To iterate on a node, run `cargo build` in `src/team7_nodes` with ROS sourced.
