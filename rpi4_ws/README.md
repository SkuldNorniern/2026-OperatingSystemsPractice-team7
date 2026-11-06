# rpi4_ws

pi keeps factory TurtleBot 4 software (Ubuntu 24.04, Jazzy). `turtlebot4.service` runs turtlebot4_bringup: RPLIDAR, OAK-D and create3_republisher, which relays create3 (create3 itself is under `/_do_not_use`).
our code runs on laptop, see [../host_ws](../host_ws/README.md).

dont reinstall or overwrite factory packages. if something has to run on pi later (scan filter, watchdog) it goes in `ros2_ws/src/` as overlay. empty for now.

## what laptop needs from robot

| topic | type | |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | frame `rplidar_link`, reliable |
| `/odom`, `/tf`, `/tf_static`, `/imu` | | reliable, transient local, depth 1 |
| `/oakd/rgb/preview/image_raw`, `/oakd/rgb/preview/camera_info` | Image, CameraInfo | 250x250, frame `oakd_rgb_camera_optical_frame` |
| `/reset_pose` | `irobot_create_msgs/srv/ResetPose` | map origin = start pose |
| `/undock`, `/dock` | `irobot_create_msgs/action/Undock`, `Dock` | undock fails if already undocked |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | |

namespace comes from `ROBOT_NAMESPACE`, empty by default. launch files assume no namespace.

## setup

1. create3 webserver (http://192.168.185.3:8080): firmware should be `I.*` FastDDS build, namespace `/_do_not_use`, create3 not on wifi
2. `cat /etc/turtlebot4/setup.bash`, note `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, `ROS_DISCOVERY_SERVER` and set same on laptop (host README, network)
3. `systemctl status turtlebot4 discovery`. restart with `turtlebot4-service-restart`
4. clock: `Reference/tb4_timesync/tb4_rpi_timesync.sh setup <LAPTOP_IP>` once. pi has no RTC and factory chrony only serves create3 network. `turtlebot4-ntpd-sync` resyncs create3
5. robot usually sits on dock and power_saver turns lidar and camera off there. run_demo.sh undocks
6. camera (optional, test first): factory preview is 250x250, aruco range is around 1.5 m for 10 cm markers. bigger preview costs pi CPU:

   ```bash
   sudo ./scripts/oakd_preview.sh set 480 15
   turtlebot4-service-restart
   ./scripts/oakd_preview.sh status
   sudo ./scripts/oakd_preview.sh restore
   ```

   standard.launch.py doesnt pass camera `params_file`, so there is no official override. script edits `/opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_pro.yaml` with backup, apt upgrade overwrites it.
   cleaner way is building turtlebot4_robot from source in this overlay. Lite: `TB4_CAMERA=oakd_lite`.

## check on robot

```bash
ros2 topic list --include-hidden-topics | grep _do_not_use      # pi, create3 visible
ros2 topic info -v /cmd_vel                                     # laptop, TwistStamped
ros2 topic hz /scan; ros2 topic hz /oakd/rgb/preview/image_raw  # laptop, data over cable
ros2 service list | grep -E "reset_pose|e_stop|start_motor"
chronyc tracking                                                # both, offset < ~10 ms
```
