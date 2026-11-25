# 실제 TurtleBot4 실행 (V1)

## 기존 rust_base 데모에서 V1 실행

`host_ws/scripts/run_rust_base_v1.sh`는 기존 빌드의 `run_demo.sh`를 실행합니다. V1은 기존 SLAM, Nav2, 충돌 감시와 rosbag을 사용하고 `/navigate_to_pose`와 `/spin`으로 이동·회전합니다. V1 필터를 통과한 마커를 처음 관측하면 2.5초간 Nav2 주행 속도를 일반 주행 속도의 80%로 제한합니다. 전진 상한은 0.18 m/s이고, `velocity_guard`가 후진 명령을 0으로 제한합니다.

실행기는 찾은 작업공간에 `team7_viz`가 빌드되어 있으면 해당 지도 뷰어를 읽기 전용으로 엽니다. 뷰어는 지도, 라이다와 RGB 영상을 표시하며, `/team7/v1/targets`를 읽어 V1이 처음 관측한 ID를 **Candidates**에, 위치를 확인한 ID를 **Confirmed**에 표시합니다. V1 뷰어 설정은 실행 폴더에 저장됩니다. `team7_viz`가 없거나 실행에 실패하면 RViz가 열립니다. 필요하면 `rust_base`의 `host_ws/README.md`에서 `team7_viz` 빌드 방법을 확인하세요.

기본 위쪽 보기의 중심은 `base_link`를 따라가며, 지도는 `map` 좌표계를 사용합니다. RViz의 **V1 ArUco Targets**에서 첫 관측은 주황색, 두 프레임에서 위치가 일치한 마커는 초록색, 방문을 마친 마커는 파란색으로 표시됩니다. RViz를 따로 열면 Fixed Frame을 `map`으로 두고 **MarkerArray**를 추가한 뒤 `/team7/v1/targets`와 **Transient Local**을 선택하세요.

출발 대기 중 `waiting_for_startup`에 아직 필요한 입력이 표시됩니다. `waiting_for_wall_path`는 로봇 좌표, 지도 범위와 로봇 위치의 지도 상태를 함께 기록합니다.

ROS 2 Jazzy와 `rust_base` 빌드 의존성이 설치된 Ubuntu 호스트의 `mapping_algo_sj` 저장소 루트에서 실행합니다.

```bash
bash host_ws/scripts/run_rust_base_v1.sh
```

실행기는 `rust_base` 커밋 `a7eda3e`를 사용합니다. 같은 커밋의 체크아웃을 찾으면 재사용합니다. 경로를 직접 알려주려면 `TEAM7_RUST_BASE`를 설정하세요. 같은 커밋의 체크아웃이 없으면 옆에 `<저장소 이름>-rust_base-a7eda3e` 작업 트리를 만듭니다. 그 작업공간의 첫 실행에서 `colcon`으로 `team7_nodes`와 `team7_bringup`을 빌드합니다. V1 Rust 라이브러리도 처음 필요할 때 빌드합니다. 빌드 도구와 ROS 패키지는 `rust_base` 브랜치의 `host_ws/README.md`에 나온 Ubuntu 준비 절차에 따라 설치합니다. V1에는 아래에 정리된 Python 패키지도 필요합니다.

인자 없이 실행하면 스크립트에 설정된 마커 한 변 0.10 m와 ID 8·19·29를 사용합니다. 실제 값이 다르면 스크립트 상단의 값을 바꾸세요. 실행기는 센서·지도·TF·Nav2와 `velocity_guard`를 확인한 뒤 같은 스택에서 주행을 시작합니다. 데모 실행기는 기본 75초 안에 연결이 준비되지 않으면 주행 없이 종료합니다. 연결만 확인하려면 `--check`를 실행하세요. 명령줄에서 설정을 직접 지정할 때는 `--execute`와 마커 설정을 함께 전달합니다. 지도 뷰어를 끄려면 그 명령에 `--no-rviz`를 추가하세요. 도크에서 꺼낸 로봇을 라이다가 벽을 볼 수 있는 시작 위치에 놓으세요. V1은 자동 언도킹과 odom 초기화를 건너뜁니다.

V1은 라이다에서 벽 한 면이 확인되면 접근한 뒤 약 0.8 m를 띄우고 따라갑니다. 짧은 이동 목표를 이미 관측한 지도의 빈 공간에 놓으면서 SLAM 지도를 넓힙니다. 벽을 따라 5 m 이동할 때마다 제자리에서 한 바퀴 돌며 마커를 찾고 이어서 둘레를 돕니다. 이어지는 벽이 확인되면 그 벽을 따라가고, 네 모서리를 지나 처음 벽에 접근한 지점으로 돌아오면 둘레 순회가 끝납니다. 벽이나 안전한 중간 목표를 찾지 못하면 정지한 채 다음 입력을 기다립니다.

한 바퀴를 마치면 `rust_base`의 탐색기가 방 안의 미탐색 경계와 카메라 시점을 찾아갑니다. 목표에 더 가까워지지 않는 주행은 취소하고 다음 후보를 찾습니다. 탐색 중 발견한 마커는 V1이 지도에 저장합니다. 둘레 주행이나 내부 탐색 중 서로 다른 마커 세 개가 확인되면 진행 중인 목표를 취소하고, 현재 위치에서 가장 가까운 미방문 마커부터 정면으로 접근해 마커를 바라봅니다.

실행기는 RGB에 정렬된 `sensor_msgs/msg/Image` 깊이 영상을 자동으로 찾습니다. 필요하면 `TEAM7_DEPTH_TOPIC`과 `TEAM7_DEPTH_CAMERA_INFO_TOPIC`으로 토픽을 지정할 수 있습니다. 같은 광학 프레임에서 촬영 시각이 가까운 유효 깊이값으로 ArUco 거리와 지도 좌표를 보정합니다. 깊이 영상이 없거나 거리값이 맞지 않으면 RGB로 계속 추정합니다. 각 마커에 처음 깊이 보정이 적용되면 `mission.log`에 `marker_depth_used`가 기록됩니다.

`Ctrl+C`를 누르면 진행 중인 주행을 취소하고, 이번 실행이 시작한 탐색기, ROS 노드, 지도 뷰어와 rosbag을 정리합니다. 정상 종료에 응답하지 않는 하위 프로세스는 유예 시간 뒤 종료합니다. 실행 결과는 `~/team7_robot_runs/*-rust-base-v1/`에 저장됩니다. 실행이 중단되면 `mission.log`에서 연결·주행 상태를 확인하고, ROS 스택 초기 오류는 `stack.log`에서 확인하세요. 자체 뷰어 로그는 `team7_viz.log`에 저장됩니다. `rust_base/`에는 rosbag과 데모 로그, `mission/`에는 V1 지도·마커·이벤트가 들어 있습니다. ROS 2 Jazzy와 실물 로봇을 통한 실행 검증은 아직 하지 않았습니다.

## 독립 실행

`host_ws/scripts/run_robot.sh`는 실제 로봇의 ROS 2 입력으로 기존 V1 미션을 실행합니다. Gazebo를 실행하지 않으며 Gazebo Python 모듈도 필요하지 않습니다. 실행 장소는 **로봇의 ROS 토픽에 연결된 Ubuntu 24.04 / ROS 2 Jazzy 호스트**, 또는 같은 패키지가 설치된 로봇의 Pi입니다. ROS 2가 없는 Mac에서는 이 실물 실행기를 직접 실행할 수 없습니다. Mac 시뮬레이터는 기존 명령을 그대로 사용하세요.

## 준비와 빌드 (Ubuntu에서 최초 한 번)

로봇은 기본 TurtleBot4 bringup으로 라이다·카메라·odom·TF를 제공해야 합니다. 호스트의 ROS_DOMAIN_ID와 RMW 설정은 로봇과 맞추고, 이미지 시각의 TF 조회를 위해 시계도 동기화하세요. 관련 기존 자료는 `Reference/tb4_timesync/`에 있습니다. 로봇이 도킹 상태라 센서가 꺼져 있다면 별도로 언도킹해야 합니다. 이 스크립트는 자동 언도킹/odom 초기화/펌웨어 안전 설정 변경을 하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-numpy python3-opencv python3-yaml ros-jazzy-rclpy ros-jazzy-tf2-ros ros-jazzy-slam-toolbox ros-jazzy-visualization-msgs
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

아래 `0.10`은 **마커 검은 외곽 정사각형의 실제 한 변 길이(m)의 예시**입니다. 출력지 전체/흰 테두리 크기가 아닙니다. 실제 마커 사전과 크기를 맞추세요. ID마다 크기가 다르면 `robot_config.json`의 `marker_sizes`에 `{"8": 0.25, "19": 0.20, "29": 0.28}` 같은 형식으로 실제 값을 적습니다. 설치된 세 ID를 `--marker-ids 8 19 29`처럼 지정해 다른 태그를 배제할 수 있습니다. ID를 지정하지 않고 `marker_sizes`에 세 크기를 설정하면 그 키를 허용 ID로 사용합니다. `--marker-size`만 지정하면 선택한 ArUco 사전의 유효 ID를 받습니다. V1은 영상에서 위치를 찾은 QR 심볼과 겹치는 후보를 제외합니다. 목표 ID가 지정된 경우에만 기존 Rust 검출기의 1비트 보정을 허용합니다.

서로 다른 마커 세 개가 확인되면 순회 중인 목표를 취소하고, 현재 위치에서 가장 가까운 미방문 마커의 정면으로 이동합니다. 도착하면 마커 중심을 바라봅니다. 세 개를 확인하지 못하면 제한 시간 내에서 둘레 탐색을 계속합니다. 기본 사전은 5X5_50이고 `--dictionary DICT_6X6_250` 또는 `DICT_ARUCO_ORIGINAL`로 바꿀 수 있습니다.

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
- 개발 Mac에서는 메시지 변환·정지 조건·기존 시뮬레이션 회귀를 검사했습니다. **ROS 2 런타임 연결과 실제 TurtleBot4 주행은 아직 검증하지 않았습니다.** 연결만 확인하려면 `--check`를 사용하세요.
