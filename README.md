# 2026-OperatingSystemsPractice-team7

TurtleBot 4 mapping and ArUco exploration project.

## Native Gazebo simulation (macOS, ROS 2 없이 실행)

`mapping_algo_sj`에는 직사각형 방 순회 알고리즘을 확인할 Gazebo 전용 실행기가 있습니다.
서버, GUI, 알고리즘을 별도 프로세스로 시작합니다.
마커 인식은 `rust_base`에서 가져온 `crates/tb4_aruco`를 사용합니다.
Rust 1.89 이상과 Homebrew Gazebo Harmonic이 필요합니다.

```sh
cd sim/gazebo
./setup_native.sh
./run_native.sh --version v1  # 네 모서리 순회 + 5 m 회전
./run_native.sh --version v2  # rust_base frontier + camera coverage
```

설치 조건, 알고리즘, 실행 결과와 한계는 [시뮬레이션 안내](sim/gazebo/native_sim/README.md)를 참고하세요.
이 시뮬레이션은 정확한 Gazebo 위치를 사용하는 운동학 모델이며, 실물용 ROS 2 노드나 SLAM 성능을 검증한 결과는 아닙니다.

## ROS 2 workspaces

- [Host workspace](host_ws/ros2_ws/README.md)
- [Robot workspace](rpi4_ws/ros2_ws/README.md)
