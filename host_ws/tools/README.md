# tools

rosbag2 recording for replay. runs on laptop, bags go to `~/bags` (change with `BAG_DIR`).

```bash
host_ws/tools/record_start.sh full            # sensors + our outputs, use during demo runs
host_ws/tools/record_start.sh raw slam_test1  # sensors only, record with SLAM/Nav2 off and drive with teleop
host_ws/tools/record_start.sh all             # every topic
host_ws/tools/record_stop.sh                  # stops and prints bag info
```

only one recording at a time. stop sends SIGINT so mcap gets closed, dont kill -9 it.
camera is most of the size, ~5-6 MB/s raw before zstd.

replay, start nodes first so they get latched /tf_static and /robot_description:

```bash
ros2 launch team7_bringup team7.launch.py use_sim_time:=true
ros2 bag play ~/bags/<name> --clock
```

raw bags are for SLAM tuning, full bags already have map->odom in /tf so dont run SLAM again on them.
MPPI cant be tuned from bags, use gazebo sim for that.

offline SLAM replay without ROS running (timing, loop closures, map jitter per run). source ROS in bash
for rosbag2_py, pass the .mcap files of a run in order:

```bash
python3 host_ws/tools/bag_scans.py ~/team7_runs/<run>/rosbag_* > /tmp/scans.bin
cargo run --release -p tb4_slam --example replay -- map=/tmp/map.pgm < /tmp/scans.bin
cargo run --release -p tb4_slam --example replay -- time_offset=-0.15 redraw_min_turn=0.005 < /tmp/scans.bin
```
