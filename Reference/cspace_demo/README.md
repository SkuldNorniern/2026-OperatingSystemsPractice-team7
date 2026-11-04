# C-space 계산 수업용 데모 코드

Lec11 슬라이드 "From workspace to C-space", "Computing C-space", "C-space with rotation"과 짝을 이루는 수업용 데모입니다.
추후 sampling-based planning 강의 뒤에 낼 C-space 과제의 '예고편' 역할을 합니다.

## 설치 및 실행

추가 패키지 없이 `numpy`와 `matplotlib`만 사용합니다. Ubuntu 24.04 / ROS 2 Jazzy 환경에서는 `sudo apt install python3-numpy python3-matplotlib`로 설치하세요. Ubuntu 24.04는 시스템 Python에 `pip install`을 막아 두었으므로(PEP 668) apt 설치를 권장합니다. pip를 쓰려면 가상환경(`python3 -m venv`)을 만든 뒤 설치하세요.

macOS / Windows에서도 그대로 실행됩니다(운영체제별 코드 없음). macOS 예시:

```bash
python3 -m venv .venv && source .venv/bin/activate   # Homebrew Python도 시스템 pip 설치를 막아 두었으므로 가상환경 권장
pip install numpy matplotlib
```

```bash
python3 demo1_minkowski_2d.py        # 2D: Minkowski sum으로 C-obstacle 계산 (theta 슬라이더)
python3 demo2_cspace_3d.py           # 3D: theta 단면을 쌓은 (x, y, theta) C-space
python3 demo3_two_link_arm.py        # 2-link 로봇팔: 충돌 검사로 계산한 (theta1, theta2) C-space
```

모든 스크립트는 `--save 파일명.png`로 실행하면 창 없이 그림만 저장합니다(슬라이드용 그림 제작에 사용).

| 파일 | 내용 |
|---|---|
| `cspace_common.py` | 공통 함수: 회전/이동, 볼록 껍질, Minkowski sum, SAT 충돌 검사, 예제 장면 |
| `demo1_minkowski_2d.py` | 고정 방향 삼각형 로봇의 workspace ↔ C-space 비교 |
| `demo2_cspace_3d.py` | 회전 포함 3D C-space (단면 적층) |
| `demo3_two_link_arm.py` | 2-link 팔의 토러스 C-space, start–goal 직선 경로 충돌 검사 |

## 수업 진행 제안

### Demo 1: Minkowski sum (슬라이드 29–33 직후)
- 조작: 두 그림 중 아무 곳이나 클릭·드래그해 기준점 q를 움직이고, 아래 슬라이더로 θ를 바꿉니다. `g` 키는 brute-force 충돌 샘플(빨간 점) 표시를 켜고 끕니다.
- 보여줄 것
  1. 자홍색 영역(`(−A) ⊕ B`)에 기준점이 들어가는 순간 왼쪽 로봇이 빨갛게 변함 → "로봇을 점으로, 장애물을 부풀려서" 본다는 의미
  2. `g`를 눌러 brute-force 샘플이 자홍색 영역과 정확히 겹치는 것 확인 → Minkowski sum이 '모든 q를 하나씩 검사한 결과'와 같다
  3. 점선 사각형: workspace 경계도 C-space에서는 줄어든다(슬라이드 25)
  4. θ를 바꾸면 C-obstacle 모양이 달라짐 → 회전이 있으면 θ마다 다시 계산해야 한다
- 질문 예시
  - 기준점을 삼각형의 다른 꼭짓점으로 바꾸면 C-obstacle은 어떻게 달라질까? (답: 모양은 같고 평행이동만 됨)
  - 장애물이 볼록이 아니면 이 함수를 그대로 쓸 수 있을까? (답: 볼록 조각으로 분해 후 각각 계산해 합집합)

### Demo 2: 3D C-space (슬라이드 34–36)
- θ 단면을 쌓으면 C-obstacle이 θ축을 따라 비틀린 입체가 됨. 마우스로 회전해 보며 슬라이드 그림과 비교합니다.
- 질문 예시: θ = −180°와 +180° 단면은 왜 같아야 할까? (C-space = ℝ² × S¹)

### Demo 3: 2-link 로봇팔 (sampling-based 강의로 넘어가는 다리)
- 조작: 오른쪽 C-space를 클릭하면 해당 자세가 왼쪽에 나타납니다. `s`/`g`로 현재 자세를 start/goal로 지정하고 `l`을 누르면 C-space 직선 경로를 촘촘히 충돌 검사합니다.
- 보여줄 것
  1. 다각형 로봇과 달리 링크가 연결된 팔은 C-obstacle을 공식 하나로 구하기 어렵다 → 격자 위 모든 점을 충돌 검사(1° 간격 = 129,600회)
  2. workspace에서는 단순한 사각형 장애물이 C-space에서는 휘어진 띠 모양
  3. C-space는 토러스: 왼쪽·오른쪽, 위·아래 경계가 이어져 있음
  4. 직선 경로 검사(`l`) = sampling-based planner의 **local planner / edge checking**과 같은 연산
- 질문 예시
  - 관절이 6개면 같은 1° 격자는 몇 개의 점일까? (360⁶ ≈ 2.2 × 10¹⁵) → C-space를 '전부' 계산하지 않고 필요한 곳만 샘플링하는 방법이 필요 → 다음 강의
  - `--res 5`로 실행하면 무엇이 달라질까? 좁은 틈은 어떻게 될까? (해상도 문제, A* 과제와 연결)

## 추후 C-space 과제와의 연결 (아이디어)
- `cspace_common.py`의 함수를 그대로 제공하고, 학생은 다음을 구현
  1. 볼록이 아닌 장애물(볼록 분해) 또는 회전 포함 C-space 계산
  2. `demo3`의 충돌 검사 함수를 이용한 PRM 또는 RRT 구현 (C-space 격자 없이 `collision()`과 직선 경로 검사만 사용)
  3. 격자로 계산한 C-space 위 A* 결과와 PRM/RRT 결과 비교 (해상도, 계산 시간, 경로 길이)
