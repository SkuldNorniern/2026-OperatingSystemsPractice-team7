# 실제 TurtleBot4 실행 (V1)

## 기존 rust_base 데모에서 V1 실행

`host_ws/scripts/run_rust_base_v1.sh`는 기존 빌드의 `run_demo.sh`를 실행합니다. 기존 SLAM, Nav2, 충돌 감시, 속도 제한, RViz, rosbag을 사용합니다. 기존 탐색과 미션 종료 노드를 비활성화하고, V1이 `/navigate_to_pose`와 `/spin`으로 네 모서리 순회, 5 m마다 회전, 마커 정면 방문을 수행합니다. V1은 카메라 영상에서 Rust ArUco로 검출한 마커를 자체 결과에 기록합니다.

이미 빌드한 `rust_base` 작업공간이 있으면 `--rust-base /경로/기존-체크아웃`으로 지정하세요. 없으면 Ubuntu에서 저장소 루트 기준으로 아래처럼 별도 작업 트리를 만든 후, 그 작업 트리의 `host_ws/README.md`에 따라 ROS 작업공간을 한 번 빌드합니다.

```bash
git fetch origin rust_base:refs/remotes/origin/rust_base
git worktree add --detach ../2026-OperatingSystemsPractice-team7-rust_base origin/rust_base
```

`mapping_algo_sj` 저장소 루트에서 다음 명령을 실행합니다. 방 실측 크기와 마커 한 변 길이는 실제 값으로 바꾸세요.

```bash
bash host_ws/scripts/build_robot.sh  # 이 Ubuntu 호스트에서 최초 한 번
bash host_ws/scripts/run_rust_base_v1.sh --check
bash host_ws/scripts/run_rust_base_v1.sh --execute \
  --room-width 19.85 --room-height 11.05 --marker-size 0.10
```

`--check`는 데모 스택을 시작한 뒤 센서·지도·TF·Nav2 연결을 확인하고 종료합니다. `--execute`는 원래 데모처럼 필요시 언도킹하고 odom을 초기화합니다. 이미 시작 위치에 놓은 경우에는 `--no-prepare`를 붙여 이 단계를 건너뜁니다. 화면 없는 Ubuntu에서는 `--no-rviz`를 붙이세요. `Ctrl+C`를 누르면 V1 목표 취소를 요청한 뒤 데모를 종료합니다. 결과는 `~/team7_robot_runs/*-rust-base-v1/`의 `rust_base/`(rosbag과 데모 로그)와 `mission/`(V1 지도·마커·이벤트)에 저장됩니다. ROS 2 Jazzy와 실물 로봇을 통한 실행 검증은 아직 하지 않았습니다.

## 독립 실행

`host_ws/scripts/run_robot.sh`는 실제 로봇의 ROS 2 입력으로 기존 V1 미션을 실행합니다. Gazebo를 실행하지 않으며 Gazebo Python 모듈도 필요하지 않습니다. 실행 장소는 **로봇의 ROS 토픽에 연결된 Ubuntu 24.04 / ROS 2 Jazzy 호스트**, 또는 같은 패키지가 설치된 로봇의 Pi입니다. ROS 2가 없는 Mac에서는 이 실물 실행기를 직접 실행할 수 없습니다. Mac 시뮬레이터는 기존 명령을 그대로 사용하세요.

## 준비와 빌드 (Ubuntu에서 최초 한 번)

로봇은 기본 TurtleBot4 bringup으로 라이다·카메라·odom·TF를 제공해야 합니다. 호스트의 ROS_DOMAIN_ID와 RMW 설정은 로봇과 맞추고, 이미지 시각의 TF 조회를 위해 시계도 동기화하세요. 관련 기존 자료는 `Reference/tb4_timesync/`에 있습니다. 로봇이 도킹 상태라 센서가 꺼져 있다면 별도로 언도킹해야 합니다. 이 스크립트는 자동 언도킹/odom 초기화/펌웨어 안전 설정 변경을 하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-numpy python3-opencv ros-jazzy-rclpy ros-jazzy-tf2-ros ros-jazzy-slam-toolbox
# Rust 1.89 이상을 설치한 뒤 저장소 루트에서:
bash host_ws/scripts/build_robot.sh
```

Mac에서 생성한 `.dylib`는 Linux에서 사용할 수 없으므로 **실제로 실행할 Linux 호스트에서 한 번 빌드**해야 합니다. `build_robot.sh`는 ROS 노드 전체를 colcon 빌드하지 않고, 필요한 Rust 인식·탐색 공유 라이브러리 `.so`만 만듭니다.

## 센서 입력 확인 (이동 명령을 발행하지 않음)

```bash
# 이미 SLAM이 실행 중이면:
bash host_ws/scripts/run_robot.sh --check
# SLAM을 함께 시작해야 하면:
bash host_ws/scripts/run_robot.sh --with-slam --check
```

`ready: true`는 /scan, /odom, 카메라 영상·보정값, /map, map↔base와 카메라 optical TF가 유효한 시각에 들어왔다는 의미입니다. 실제 주행 성공을 의미하지 않습니다. 기본 동작도 `--check`이며 속도 publisher를 만들지 않습니다. SLAM을 함께 시작할 때는 다른 SLAM/로컬라이제이션 노드를 동시에 실행하지 마세요. 기본 SLAM 설정은 `slam.yaml`이고, 비표준 scan/프레임을 쓴다면 그 파일도 맞춰야 합니다.

## 실제 알고리즘 실행

아래 `0.10`은 **마커 검은 외곽 정사각형의 실제 한 변 길이(m)의 예시**입니다. 출력지 전체/흰 테두리 크기가 아닙니다. 실제 마커 사전과 크기를 맞추세요. ID마다 크기가 다르면 `robot_config.json`의 `marker_sizes`에 `{"8": 0.25, "19": 0.20, "29": 0.28}` 같은 형식으로 실제 값을 적습니다. 지정한 ID 이외에도 인식하려면 `--marker-size`를 공통 크기로 지정할 수 있습니다. 기본 사전은 5X5_50이고 `--dictionary DICT_6X6_250` 또는 `DICT_ARUCO_ORIGINAL`로 바꿀 수 있습니다.

```bash
# V1: 실제 방의 내벽 사이 실측 가로/세로를 지정 (아래 치수도 예시)
bash host_ws/scripts/run_robot.sh --with-slam --execute \
  --room-width 19.85 --room-height 11.05 --marker-size 0.10

```

이미 SLAM이 실행 중이면 `--with-slam`을 빼세요. **Ctrl-C**로 정지합니다. `--time-limit 600`은 실제 실행 시간 제한입니다. 결과는 `~/team7_robot_runs/날짜-시간-v1/`에 지도 PGM/YAML, 마커 map 좌표, 이벤트와 상태로 저장됩니다.

## 기본 입력/출력

| 항목 | 기본값 |
|---|---|
| 라이다 | `/scan` (sensor_msgs/LaserScan) |
| 주행거리 입력/상태 확인 | `/odom` (nav_msgs/Odometry) |
| 위치 | TF `map` → `base_link`, 최신 유효 시각 |
| 지도 | `/map` (nav_msgs/OccupancyGrid, SLAM 출력) |
| 영상 | `/oakd/rgb/preview/image_raw/compressed` |
| 보정값 | `/oakd/rgb/preview/camera_info` |
| 카메라 외부 파라미터 | 영상 시각의 `map` → CameraInfo optical frame TF |
| 속도 출력 (Jazzy) | `/cmd_vel` (geometry_msgs/TwistStamped) |

토픽·프레임은 `--scan-topic`, `--image-topic`, `--camera-info-topic`, `--map-frame`, `--base-frame` 등으로 바꿉니다. `--help`로 전체 옵션을 보세요. Humble 등의 unstamped 명령 구성에서는 `--cmd-type unstamped`를 명시하세요. 센서와 명령의 namespace도 옵션으로 실제 이름에 맞춰야 합니다.

카메라는 실제 K/D와 프레임 TF를 사용합니다. 시뮬레이터의 고정 화각이나 카메라 위치를 실물에 대입하지 않습니다. 라이다는 실제 laser→base TF를 적용한 후 미션으로 전달합니다. 지도와 위치는 SLAM에서 받고 Gazebo의 정확한 위치는 쓰지 않습니다. 마커 ID와 모서리 검출은 기존 Rust 코드를 재사용합니다.

## 주행 조건과 현재 한계

- 속도는 최대 전진 0.18 m/s, 회전 0.5 rad/s로 제한하며 후진은 보내지 않습니다.
- 별도 ROS 타이머가 명령/센서/TF 신선도를 검사합니다. 명령이 0.5초 이상 끊기거나 센서가 오래되면 정지 명령을 보냅니다. 전방 0.45 m 이내이면 전진을 막고, 주변 0.22 m 이내이면 회전도 막습니다.
- 다른 `/cmd_vel` publisher가 있으면 명령 출력을 정지합니다. Nav2/teleop과 이 미션을 동시에 제어기로 실행하지 마세요. 하드웨어의 기존 bumper/cliff 동작은 유지됩니다.
- V1은 사각형 방 크기를 알고 있고 SLAM 좌표축과 방 벽이 평행하다는 가정입니다. 초기 로봇 방향을 벽과 맞춰야 합니다.
- 이 실행기는 기존 Python A*와 속도 제어를 실물 입력에 연결한 버전입니다. Nav2/MPPI의 전체 주행 스택을 사용하는 것이 아닙니다. 마커 정면 위치 추정이 흔들리거나 목표가 장애물 확장 영역에 걸리는 기존 한계는 남아 있습니다.
- 개발 Mac에서는 메시지 변환·정지 조건·기존 시뮬레이션 회귀를 검사했습니다. **ROS 2 런타임 연결과 실제 TurtleBot4 주행은 아직 검증하지 않았습니다.** 먼저 `--check`로 실제 입력과 프레임을 확인해야 합니다.
