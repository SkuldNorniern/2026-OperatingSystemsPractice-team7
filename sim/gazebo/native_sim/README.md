# track1 native Gazebo mission (ROS 2 없이 실행)

```sh
cd ~/2026-OperatingSystemsPractice-team7/sim/gazebo
./setup_native.sh  # 최초 한 번: 프로젝트 가상환경과 Python 의존성 설치
# V1: 네 모서리 순회 + 5 m마다 360° 회전
./run_native.sh
# GUI 없이 실행: ./run_native.sh --headless
# 제한 시간(실제 초): ./run_native.sh --headless --max-seconds 600
```

이 실행기는 자신의 서버/GUI/미션만 시작하고 Ctrl-C로 함께 종료합니다. macOS에서 서버와 GUI는 별도 프로세스로 실행됩니다. 저장소 위치마다 별도 Gazebo 통신 파티션을 사용하고, 동일 폴더의 중복 실행은 잠금 파일로 막습니다. 외부 도구로 연결할 때는 시작 로그의 `GZ_PARTITION` 값을 사용하세요.

카메라는 1280×960, 15 Hz로 설정했습니다. 기본 검증 실행 속도는 최대 실시간의 4배이며 PC 부하에 따라 느려집니다. 카메라는 GUI Plugins → Image Display → `/turtlebot4/oakd/rgb/image`로 확인합니다.

`track1.sdf`를 원본으로 읽어 `native_sim/track1_native.sdf`를 만듭니다. 원본 배치를 보존하며, 사각형 순회를 위해 남쪽 출입구를 실행 복사본에서 막습니다. 마커의 XY 배치는 유지하고, 인식 가능한 DICT_5X5_50 마커를 직립시킵니다. 패널은 GUI Transform으로 이동 가능하지만 중력에 넘어지지 않도록 static으로 둡니다. ID 8/19/29와 마커 외곽 변 길이는 config.json에 기록됩니다. 원본에는 손대지 않습니다.

## 실행 결과

기본 시뮬레이션 시간 제한은 600초입니다. `--max-sim-seconds 600`으로 시뮬레이션 시간 제한을, `--max-seconds 600`으로 실제 경과 시간 제한을 지정할 수 있습니다. 실행 결과는 `native_sim/runs/날짜-시간-v1/`에 저장됩니다. 이전 `--version v1` 명령도 호환되지만 현재 실행 가능한 알고리즘은 V1뿐입니다.

## V1 미션

1. 라이다의 축 방향 벽 클러스터와 알려진 방 크기로 방 경계를 추정합니다. 방이 초기 로봇 좌표계 축에 정렬됐다는 가정입니다.
2. 가장 가까운 벽에서 0.8 m 떨어진 위치로 접근합니다.
3. 마커 세 개가 확인될 때까지 사각형의 네 모서리를 반시계 방향으로 순회합니다. 모서리 자체에 충돌하지 않도록 벽에서 0.8 m 떨어져 갑니다.
4. 순회 중 누적 주행 거리 5 m마다 멈춰 360° 회전합니다. 회전 완료는 실제 yaw 변화량으로 판정합니다. 5 m 미만 잔여 구간은 다음 구간으로 이어 계산합니다.
5. 순회 중에도 계속 카메라로 마커를 검출하고 위치·정면 법선을 기록합니다.
6. 서로 다른 마커 세 개를 각각 두 프레임에서 확인하면 순회를 중단합니다. 현재 위치에서 가장 가까운 미방문 마커를 선택해 기본 0.8 m 앞 지점으로 이동하고 마커 중심을 바라봅니다. 각 방문 후 가까운 순서를 다시 계산합니다.

경로는 라이다 점유격자에 장애물 반경을 확장한 A*로 계획합니다. 입력이 끊기면 정지하며, 막힌 목표에 경로가 없으면 멈추고 로그에 남깁니다. 세 마커 발견 전에 시간 제한에 도달하면 `time_limit`으로 기록합니다.

## 결과와 한계

`native_sim/runs/날짜-시간/`에 events.jsonl, map.pgm/map.yaml, markers.json, status.json, 검출 시 camera_latest.jpg가 저장됩니다. marker_detected 시 해당 위치와 관측 횟수가 기록되며 저장 파일은 주기적으로 갱신됩니다. 좌표는 ideal_odometry가 제공하는 Gazebo 월드 좌표입니다.

이번 버전은 사용자가 설명한 미션을 별도로 구현한 Gazebo 전용 검증기입니다. Git 저장소의 C/Rust ROS 노드를 실행한 결과가 아닙니다. 로봇은 Gazebo VelocityControl로 속도를 직접 적용하는 간소화한 TurtleBot4 형태의 운동학 모델이고 카메라는 RGB 단안 카메라입니다. 실제 OAK-D RGB-D 전체 구성을 재현하지 않습니다. 위치는 Gazebo OdometryPublisher의 정확한 시뮬레이션 위치를 사용합니다. 지도는 라이다 스캔을 그 위치에 합성한 점유격자이며, 위치 오차를 보정하는 SLAM/루프클로저는 구현하지 않았습니다. 실물 입력은 별도의 `host_ws/robot_runtime` ROS 2 어댑터로 연결하며, 실제 주행 검증은 아직 필요합니다. ArUco PnP는 설정에 저장된 각 마커의 실제 크기를 사용합니다.

## 설치

Rust 1.89 이상이 필요합니다 (`brew install rust`). `setup_native.sh`가 저장소 루트의 Cargo workspace에서 `tb4_aruco_gazebo` 공유 라이브러리를 빌드합니다. Rust 인식기와 C ABI 연결 계층만 빌드하며 외부 crate 의존성은 없습니다.

이 통합본의 실행 대상은 Homebrew Gazebo Harmonic을 설치한 macOS입니다. Homebrew `gz.transport13` 네이티브 바인딩과 ABI가 맞는 Python 3.14를 사용합니다. 다른 호환 Python 경로는 `GAZEBO_PYTHON=/path/to/python ./setup_native.sh`로 지정할 수 있습니다. `.venv`에는 OpenCV/numpy/protobuf만 설치하고 Gazebo 바인딩은 Homebrew 설치에서 가져옵니다.

```sh
./setup_native.sh
```

## Rust ArUco 연결

`crates/tb4_aruco/`는 `rust_base` 브랜치에서 원본 소스와 테스트를 그대로 가져온 인식기입니다. 가져온 커밋은 해당 폴더의 `PROVENANCE.md`에 기록했습니다. `sim/gazebo/rust_aruco_bridge/`가 작은 C ABI를 제공하고, `native_sim/rust_detector.py`가 ctypes로 그 라이브러리를 호출합니다.

Gazebo RGB 영상 → 회색조 변환 → **Rust의 ID·모서리 검출** → 기존 위치 추정·매핑 순서입니다. 인식 사전은 `5X5_50`입니다. 실행 시작 로그에 `detector: rust_base/tb4_aruco`가 기록되며, 라이브러리가 없으면 오류로 중단합니다. OpenCV 인식기로 자동 전환하지 않습니다. OpenCV는 마커 이미지 생성, 영상 변환, PnP 위치 추정과 지도 이미지 처리에 계속 사용됩니다.

## 검증 상태

Rust 전환 후 원본 Rust 테스트 20개, Python 연결·미션 테스트 9개를 통과했습니다. 짧은 실제 Gazebo 실행에서 Rust 검출기로 ID 8/19/29 모두 검출, 벽 접근, 5 m 회전과 첫 모서리 도착을 확인했습니다. Rust 전환 후 전체 마커 방문 미션은 아직 끝까지 재검증하지 않았습니다.

통합 전 같은 track1 배치에서 OpenCV 인식기로 수행한 시뮬레이션에서는 네 모서리 목표와 시작 벽 복귀, 5 m 간격 회전, ID 8/19/29 검출과 좌표 저장을 확인했습니다. 마커 방문은 ID 8까지만 성공했고, ID 19의 추정 정면 목표가 장애물 확장 영역에 걸려 `blocked`로 중단됐습니다. 따라서 세 마커 방문 전체 성공은 아직 검증되지 않았습니다. 마커 정면 추정 및 접근 목표 선택은 후속 개선 항목입니다. 이 상태를 성공으로 보고하지 말고 실행 결과의 `status.json`을 확인하세요.

```sh
# 단위 테스트 (가상환경 설치 후)
.venv/bin/python -m unittest discover -s native_sim -p 'test_*.py' -v
# 가장 최근 실행 결과 요약과 map_with_markers.png 생성
.venv/bin/python native_sim/summarize.py
# 특정 실행 결과 요약
.venv/bin/python native_sim/summarize.py native_sim/runs/날짜-시간
```

`.venv/`, `native_sim/runs/`, 생성된 `track1_native.sdf`/`config.json`, 서버 로그와 잠금 파일은 Git 추적 대상에서 제외됩니다. 추적되는 `track1.sdf`를 GUI에서 편집·저장하면 다음 실행 시 그 배치를 읽습니다. 실행 복사본을 편집한 내용은 다음 실행에서 다시 생성됩니다.
