# rpi4_ws: TurtleBot 4 Raspberry Pi

The Pi keeps its **factory TurtleBot 4 software** (Ubuntu 24.04, ROS 2 Jazzy, `turtlebot4_bringup` started by the `turtlebot4` systemd service). It publishes the sensors and drives the Create 3. All team code runs on the laptop (see [../host_ws](../host_ws/README.md)).

Don't reinstall or overwrite the factory packages. If code ever needs to run on the Pi (for example a lightweight scan filter or a watchdog), it goes in `ros2_ws/src/` as an overlay. That overlay is currently empty.

## What the laptop expects from the robot

| Topic | Type | From |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR |
| `/tf`, `/tf_static` | TF (`odom → base_link → … → oakd_rgb_camera_optical_frame`) | Create 3 + robot_state_publisher |
| `/oakd/rgb/preview/image_raw`, `/oakd/rgb/preview/camera_info` | `sensor_msgs/Image`, `CameraInfo` | OAK-D |
| `/reset_pose` | `irobot_create_msgs/srv/ResetPose` | Create 3 |

On the real robot, check the topic names with `ros2 topic list`. A robot namespace adds a prefix, for example `/robot1/scan`. The launch files assume no namespace.

## Setup checklist

1. **Ethernet:** `eth0` has the static address `192.168.185.3`. Plug the laptop in directly and set its address as described in the host README.
2. **ROS settings:** run `turtlebot4-setup` → ROS Setup. Note `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and the discovery mode, and set the same values on the laptop. Simple discovery is the easiest option over a direct cable.
3. **Clock sync:** `Reference/tb4_timesync/tb4_rpi_timesync.sh setup <LAPTOP_IP>` (one time). Without it, TF lookups at camera timestamps fail.
4. **Camera resolution (optional, test first):** the factory preview image is 250×250 px, which limits ArUco range to about 1.5 m for 10 cm markers. To trade Pi CPU for range:

   ```bash
   sudo ./scripts/oakd_preview.sh set 480 15     # preview 480x480 at 15 fps
   turtlebot4-service-restart
   ./scripts/oakd_preview.sh status
   sudo ./scripts/oakd_preview.sh restore        # undo
   ```

   The script edits `/opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_pro.yaml` and keeps a backup. For a TurtleBot 4 Lite, use `TB4_CAMERA=oakd_lite`. An apt upgrade of the bringup package overwrites the change.
