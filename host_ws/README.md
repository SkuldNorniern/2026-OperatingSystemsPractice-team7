# host_ws

laptop workspace. laptop sits on the robot (Ubuntu 24.04, Jazzy) and runs SLAM, Nav2, aruco, exploration, mission and RViz.
pi only runs factory TurtleBot 4 drivers.

```
ros2_ws/
  team7.repos            vcs import list
  src/team7_nodes/       rclc nodes: marker_mapper, explorer, mission. links ../../../../libtb4
  src/team7_bringup/     launch/team7.launch.py, config/*.yaml, rviz/team7.rviz
  src/external/          m-explore-ros2 from team7.repos (git ignored)
scripts/run_demo.sh      undock if docked, reset odom, launch
```

## setup (once)

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep \
  ros-jazzy-rclc ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-aruco-opencv ros-jazzy-irobot-create-msgs

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

team7_nodes builds libtb4 itself with add_subdirectory.
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

run_demo.sh undocks if robot is on dock (power_saver keeps lidar and camera off there), calls create3 `/reset_pose` so map origin is start pose, then launches `team7.launch.py`.

run stops when 3 ids are confirmed or `time_limit` (590 s) is over. mission cancels Nav2 goals, stops explore_lite and saves:

```
~/team7_run/map.pgm, map.yaml    final map, map_autosave.* every 60 s
~/team7_run/targets.csv          id,x,y in map frame (m)
```

in RViz confirmed targets are green with `ID n (x, y)`, not confirmed yet are yellow `ID n? kv`.

| launch arg | default | |
|---|---|---|
| `explorer` | `team7` | `team7`, `explore_lite` or `none` |
| `output_dir` | `~/team7_run` | |
| `camera_topic` | `/oakd/rgb/preview/image_raw` | frame `oakd_rgb_camera_optical_frame` |
| `marker_size` | `0.10` | only for PnP fallback |
| `time_limit` | `590.0` | seconds |
| `expected_targets` | `3` | |
| `use_sim_time` | `false` | `true` for gazebo |
| `rviz`, `slam`, `nav2`, `mission` | `true` | turn parts off for testing |

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

## libtb4 only

```bash
cmake -S libtb4 -B build/libtb4 -DTB4_SANITIZE=ON && cmake --build build/libtb4 && ctest --test-dir build/libtb4
```
