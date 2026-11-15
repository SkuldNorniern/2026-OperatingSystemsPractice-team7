# 장애물 및 끼임 관리 노드 — 담당 범위와 연결 초안

## 내 담당 범위

`nav_safety`는 **센서와 이동 의도를 받아 현재 상태를 판정하고 보고하는 C/rclc 노드**다.
현재 저장소 구성에 따라 **host 노트북**에서 실행한다. RPi4는 센서와 로봇 드라이버를
실행하고 `/scan`, `/odom`, TF를 제공한다.

이 문서는 우리 노드의 구현 명세와 이동관리 담당자에게 전달할 인터페이스 초안이다.
상대 노드가 완성되기 전에도 아래 입력을 가상으로 발행해 독립적으로 개발·검증할 수 있다.
토픽 이름과 메시지 형식은 연결 시 조정할 수 있으며, 팀 전체 합의가 끝났다는 의미는 아니다.

| 구분 | 책임 |
|---|---|
| 장애물·끼임 관리 (우리) | 장애물·지속 차단·무진행·입력 이상 판정, 상태/원인/관찰 시간 발행 |
| 이동관리 | 목표 실행·취소, 정상 대기/복구 상태 전달, 보고 해석, 재시도 횟수·전체 제한시간·목표 실패 결정 |
| Nav2 | 경로 계획, 경로 추종, 기본 회피·복구, 속도 계산 및 collision_monitor를 통한 최종 `/cmd_vel` 발행 |

우리 노드는 `/cmd_vel`을 발행하거나 Nav2 action을 호출하지 않는다. 정지·재계획을
명령하지 않으며, 복구 시도 횟수와 최종 목표 실패도 관리하지 않는다.

```text
RPi4 센서 (/scan, /odom, /tf, /tf_static) ─┐
                                        ├→ nav_safety → /team7/nav/safety → 이동관리
이동관리 (판정 활성 여부, 이동 의도) ─────┘                                  ↓
                                                                           Nav2
                                                                             ↓
                                                                          /cmd_vel
```

## 코드 위치

- `libtb4/include/tb4/nav_safety.h`: ROS에 의존하지 않는 판정 API와 상태/원인 정의
- `libtb4/src/nav_safety.c`: 장애물·무진행·입력 유효성 판정
- `libtb4/tests/test_nav_safety.c`: 독립 판정 테스트
- `host_ws/ros2_ws/src/team7_nodes/src/nav_safety.c`: ROS 입출력, TF 적용, 주기적 보고
- `host_ws/ros2_ws/src/team7_bringup/config/team7.yaml`: 판정 파라미터
- `host_ws/tests/test_nav_safety_ros.py`: 가상 입력을 이용한 ROS 연결 테스트

## 입력 토픽

| 토픽 | 타입 | 제공 주체 | 규칙 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR | 최신 timestamp, scan 프레임에서 base 프레임으로의 TF 필요 |
| `/odom` | `nav_msgs/msg/Odometry` | 로봇 | 기본 프레임 `odom` / `base_link` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 로봇 | 센서 위치·방향 변환 |
| `/team7/nav/active` | `std_msgs/msg/Bool` | 이동관리 | 1 Hz 이상. true=이동 진행을 판정할 구간, false=목표 없음/의도적 대기/복구로 판정 중지 |
| `/team7/nav/intent` | `geometry_msgs/msg/TwistStamped` | 이동관리 또는 연결 어댑터 | 10 Hz 권장. `base_link` 기준, linear.x와 angular.z의 이동 의도 |

active와 intent의 이름/형식은 우리 쪽 초안이다. 기존 이동관리 상태가 있다면 어댑터나
remapping을 통해 연결할 수 있다. active는 단순히 Nav2 goal이 존재한다는 뜻이 아니다.
정상적인 Wait/복구 중 정지를 끼임으로 판단하지 않도록 해당 구간은 false로 전달한다.

`intent`는 **충돌 대응으로 제한되기 전의 원하는 움직임**이다. 장애물 때문에 실제
속도가 0이 되더라도 움직이려는 의도가 남아 있다면 유지해야 한다. 최종 `/cmd_vel`을
그대로 연결하면 차단 시 0이 되어 진행·장애물 타이머가 초기화될 수 있다. Nav2 상류
명령도 정상 Wait 때 0이 될 수 있어, 어떤 명령을 의도로 전달할지는 이동관리 연결 시
확인한다. 의도 자체가 0이면 현재 구현은 무진행을 판단하지 않는다.

active와 intent는 reliable/volatile, depth 1이며 scan과 odom은 best effort/volatile,
depth 1로 구독한다. 입력이 없으면 정상 상태로 가정하지 않고 INPUT_STALE을 보고한다.

## 출력 토픽: 반환값에 해당하는 상태 보고

- 이름: `/team7/nav/safety`
- 타입: `std_msgs/msg/String`, 내용은 아래 JSON v2
- 주기: 10 Hz, 상태가 바뀌지 않아도 계속 발행
- QoS: reliable/volatile, depth 1

```json
{
  "version": 2,
  "sequence": 42,
  "monotonic_time": 12345.6,
  "state": "STUCK",
  "reason": "PERSISTENT_OBSTACLE",
  "observed_seconds": 5.2
}
```

| 필드 | 의미 |
|---|---|
| version | 인터페이스 버전. 현재 2 |
| sequence | 노드가 발행할 때 증가하는 번호. 재시작하면 초기화 |
| monotonic_time | 발행 노드의 monotonic 초. 로그용이며 다른 컴퓨터의 시각과 직접 비교하지 않음 |
| state | 아래 상태 중 하나 |
| reason | 판정 이유. 동일 state에서도 원인을 구분 |
| observed_seconds | BLOCKED/STUCK의 진행 없음 관찰 시간. 실제 진행이 인정되면 갱신하며 다른 상태에서는 0 |

| state | 의미 | reason |
|---|---|---|
| `IDLE` | active=false라 판정 중지 | `INACTIVE` |
| `CLEAR` | 현재 조건에서 막힘을 감지하지 않음. 주행 안전 보증이나 이동 명령은 아님 | `NONE` |
| `BLOCKED` | 의도한 이동 방향에 장애물 감지 | `OBSTACLE` |
| `STUCK` | 장애물이 감지되는 동안 실제 진행 없음, 또는 장애물 없이 이동 의도 대비 진행 없음 | `PERSISTENT_OBSTACLE`, `NO_LINEAR_PROGRESS`, `NO_ANGULAR_PROGRESS` |
| `INPUT_STALE` | 필요한 입력의 누락·만료·오류 | `ACTIVE_INPUT`, `COMMAND_INPUT`, `ODOM_INPUT`, `SCAN_INPUT`, `INVALID_VALUE` |

필요한 여러 입력이 동시에 잘못되면 active → command → odom → scan → 수치 검증 순서로
먼저 발견한 이유를 보고한다. 입력 이상/비활성화 후에는 관찰 시간을 다시 시작한다.
관찰 시간은 같은 상태의 보고가 시작된 이후 시간이 아니다. 장애물 최초 감지 또는 마지막
실제 진행 확인 이후의 시간이며, 장애물이 보여도 실제 진행이 확인되면 갱신한다.
예를 들어 진행 없이 BLOCKED가 5초 지속돼 STUCK이 되면 observed_seconds는 0으로 돌아가지
않는다. 장애물이 사라지면 장애물 없는 상태의 진행 관찰 구간을 새로 시작한다.

STUCK은 실패 latch가 아니라 **현재 관측 결과**다. 장애물이 사라지거나, 이동/회전 진행이
확인되거나, 이동 의도가 없어지면 현재 조건을 다시 평가한다. 외부 reset/ack가 필요하지
않고 반복 보고 횟수가 재시도 횟수가 되지도 않는다. 이전 초안(v1)의 FAILED,
stop_requested/replan_requested/failed/replans와 reset/replan_ack 토픽은 제거했다.

## 판정 기준과 초기 파라미터

| 항목 | 기본값 | 의미 |
|---|---|---|
| robot_radius / safety_margin | 0.19 / 0.05 m | 원형 footprint와 안전 여유 |
| lookahead | 1.2 s | 직선 이동 방향으로 속도 × 시간만큼 장애물 검사 영역 확장 |
| blocked_timeout | 5 s | 장애물이 감지되는 동안 실제 진행이 없으면 STUCK으로 보고하는 시간 |
| progress_timeout | 8 s | 이동/회전 의도가 있지만 진행이 없을 때 STUCK으로 보고하는 시간 |
| progress_distance | 0.05 m | 위치 진행 인정 거리 |
| progress_angle | 0.15 rad | 제자리 회전 진행 인정 각도 |
| min_linear / min_angular | 0.02 m/s / 0.05 rad/s | 이동·회전 의도 판정 기준 |
| input_timeout / active_timeout | 1 / 2 s | 데이터·의도 및 active heartbeat 유효 시간 |

전진/후진은 odom 위치의 변위로 판단한다. 목표까지의 거리가 늘어나는 정상 우회를
허용한다. 제자리 회전은 ±pi 경계를 고려한 각도 변화로 판단한다. 전진 의도가 있을 때
방향만 바뀌는 것은 위치 진행으로 인정하지 않는다. 선속도·각속도가 모두 기준 미만이면
CLEAR이며 무진행 타이머를 초기화한다.

전진↔회전 의도가 바뀌었다는 이유만으로 무진행 시간을 초기화하지 않는다. 각 관측 구간의
실제 움직임은 직전 의도에 따라 위치·각도 진행으로 나누어 누적한다. 명령이 자주 바뀌어도
작은 실제 움직임이 누적돼 기준을 넘으면 진행으로 인정한다. 전진 중 발생한 회전을 나중에
회전 의도로 바꿨다고 진행으로 소급 인정하지 않는다.

장애물이 검사 영역에 계속 보여도 실제 진행이 확인되면 BLOCKED 보고를 유지하면서 차단
타이머를 갱신한다. 움직이는 물체를 따라가거나 벽 옆에서 회전하는 상황을 장애물 관측
시간만으로 STUCK으로 승격하지 않는다. 진행이 멈춘 뒤 blocked_timeout이 지나면 STUCK이다.

scan 점은 timestamp에 맞는 TF로 base 프레임에 변환한다. 잘못된 메타데이터, TF 실패,
20% 초과 invalid range는 INPUT_STALE/SCAN_INPUT으로 보고한다. +inf는 센서의 no-return으로
취급하며 센서 드라이버도 이 규칙을 사용하는지 확인해야 한다.

하드웨어에서는 sensor/intent timestamp와 시스템 시각을 비교한다. 미래 0.2초 초과 또는
input_timeout 이상 오래된 메시지를 거부하며 수신 이후 timeout도 검사한다. 장치 간 시각
동기화가 필요하다. 프레임 이름과 주요 수치 유효성도 검사한다.

초기 거리·시간은 실기기에서 조정해야 한다. 원형 footprint와 직선 예측이며 곡선 궤적,
센서 사각지대·높이 밖 장애물, cliff/bumper, 바퀴 미끄러짐·반복 왕복은 별도 한계가 있다.
Nav2와 로봇의 기존 충돌 방지 기능을 유지한다.

## 이동관리 담당자에게 전달할 내용

> 우리 노드는 센서와 이동 의도로 CLEAR/BLOCKED/STUCK/INPUT_STALE을 판정하고,
> 상태·원인·관찰 시간을 `/team7/nav/safety`에 주기적으로 발행합니다.
> 이동관리에서는 판정 활성 여부와 이동 의도를 제공하고, 결과를 받아 Nav2 제어와
> 재시도·목표 실패를 결정해주세요. 노드 간 입력/출력 이름과 형식은 연결 시 조정 가능합니다.

연결할 때 확인할 항목은 세 가지다.

1. 이동관리의 실행/대기/복구 상태를 active로 어떻게 변환할지
2. 충돌 제한 전의 이동 의도를 어디서 제공할지
3. 토픽 이름과 String JSON 형식을 유지할지, 팀 공통 ROS msg로 바꿀지

서비스는 현재 필요하지 않아 추가하지 않았다. 계속 바뀌는 상태를 구독하는 토픽으로
독립 개발을 시작하고, 요청-응답이 필요한 기능이 생길 때 서비스 추가를 검토한다.

이동관리 처리 방식의 참고 사항 (우리 모듈이 강제하는 정책은 아님):

- 순간 BLOCKED마다 goal을 취소하지 않고 Nav2의 회피·감속 기회를 준다.
- STUCK은 재계획 실행 명령이 아니다. Nav2의 복구 상태·결과와 전체 제한시간을 함께 본다.
- 외부 재시도를 한다면 기존 goal 종료 및 정지를 확인한 뒤 새 요청을 보낸다.
- 센서/상태 보고 중단 시 이동관리의 수신 시각 기반 watchdog으로 중단·재개를 결정한다.
- 기존 C explorer의 취소 완료 전 다음 goal 전송 가능성과 목표거리 감소 기반 timeout은
  이동관리 통합 시 검토할 사항이다. 이번 작업에서는 해당 코드를 변경하지 않았다.

## 빌드와 독립 검증

ROS 없이 판정 로직과 기존 libtb4 테스트:

```bash
cmake -S libtb4 -B build/nav-safety -DTB4_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/nav-safety
ctest --test-dir build/nav-safety --output-on-failure
```

ptrace 환경에서 LeakSanitizer가 실행되지 않으면 마지막 명령 앞에
`ASAN_OPTIONS=detect_leaks=0`을 붙인다. AddressSanitizer와 UBSan 검사는 유지된다.

ROS 노드는 기존 host_ws 의존성(rclc, sensor_msgs 및 기존 ArUco 패키지 등)을 설치한 뒤 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd host_ws/ros2_ws
colcon build --symlink-install --packages-select team7_nodes team7_bringup
source install/setup.bash
ros2 run team7_nodes nav_safety --ros-args \
  --params-file src/team7_bringup/config/team7.yaml
```

기존 launch에서는 `nav_safety:=true`로 켜며 기본값은 false다. 입력 없이 실행하면
INPUT_STALE/ACTIVE_INPUT이 나온다. 이동관리 코드가 없어도 테스트 발행자로 실행 가능하다.

simulation/bag replay에서는 별도 실행 시 `-p check_header_stamp:=false`를 지정한다.
전체 launch에서는 YAML의 해당 값을 false로 바꾼다. 절대 timestamp 지연 검사를 생략하고
monotonic 수신 timeout을 사용하므로, replay 일시 정지 중에는 INPUT_STALE이 된다.

ROS 연결 테스트는 저장소 루트에서 아래처럼 실행한다. 스크립트가 localhost 전용 도메인
177에서 테스트 노드를 실행하고 종료한다. 같은 도메인에서 다른 작업을 실행하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source host_ws/ros2_ws/install/setup.bash
python3 host_ws/tests/test_nav_safety_ros.py
```

다른 위치에 빌드한 실행 파일은 `--executable /absolute/path/to/nav_safety`로 지정한다.
테스트는 가상 scan/odom/의도로 정상 → 장애물 관측 중 진행 → 실제 차단, 자동 정상 복귀,
무진행, 입력 끊김, 비활성화, 출력 필드 및 `/cmd_vel` 발행자 부재를 확인한다.
노드 시작 전에 `/tf_static`을 발행해 늦게 구독하는 노드가 정적 TF를 받는지 검사한다.
별도 LiDAR 프레임의 회전·이동 변환으로 장애물 검사를 수행하고, 알 수 없는 프레임은
INPUT_STALE로 처리되는지 확인한다.

## 검증 범위

기존 libtb4 테스트 및 판정 테스트 539개 항목이 통과했다. 명령 전환만 있는 정지 상태,
정상 전진·회전 전환, 작은 움직임의 누적, 장애물 관측 중 실제 진행 및 진행 중단을 포함한다.
AddressSanitizer/UBSan을 사용했고,
ptrace 제한으로 leak 검사는 비활성화했다. ROS 노드 임시 빌드와 위 가상 ROS 입력 테스트도
통과했으며, reset/ack 구독과 `/cmd_vel` 발행이 없음을 확인했다. 현재 환경의
전체 team7_nodes colcon 빌드는 기존 의존성 미설치로 확인하지 못했으며, 공식 Jazzy rclc를
`/tmp`에 빌드한 환경에서 우리 노드를 별도로 컴파일·링크한다. 시스템 패키지는 변경하지
않는다. 실제 TurtleBot4 동작과 이동관리 노드 통합은 아직 검증하지 않았다.
