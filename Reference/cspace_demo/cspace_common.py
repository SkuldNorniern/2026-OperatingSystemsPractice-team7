"""
cspace_common.py
C-space 계산용 공통 함수 (numpy만 사용, 추가 패키지 불필요)

- 모든 다각형은 (M, 2) numpy 배열, 꼭짓점은 반시계(CCW) 순서의 '볼록' 다각형으로 가정
- 로봇 형상 A는 로봇 자신의 좌표계에서 정의하고, 기준점(reference point)이 원점에 오도록 둔다
  -> configuration q = (x, y, theta) 는 '기준점의 위치 + 로봇의 회전각'
"""
import numpy as np


# ---------------------------------------------------------------------------
# 기본 기하
# ---------------------------------------------------------------------------
def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def transform(poly, q):
    """로봇 좌표계의 다각형 poly 를 configuration q=(x, y, theta) 로 옮긴 workspace 좌표"""
    x, y, th = q
    return poly @ rot(th).T + np.array([x, y])


def convex_hull(points):
    """Andrew's monotone chain. 반시계 순서의 볼록 껍질 반환"""
    pts = np.unique(np.asarray(points, dtype=float), axis=0)
    if len(pts) <= 2:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))
    return np.array(lower[:-1] + upper[:-1])


def edge_normals(poly):
    """다각형 각 변의 (정규화되지 않은) 법선 벡터, shape (M, 2)"""
    e = np.roll(poly, -1, axis=0) - poly
    return np.stack([-e[:, 1], e[:, 0]], axis=1)


# ---------------------------------------------------------------------------
# C-obstacle 계산 (Minkowski sum)
# ---------------------------------------------------------------------------
def minkowski_cobs(robot_local, obstacle, theta=0.0):
    """
    방향 theta 로 고정된 로봇에 대한 C-obstacle
        C_obs = { q | (A(theta) + q) ∩ B ≠ ∅ } = (-A(theta)) ⊕ B
    볼록 다각형끼리는 모든 꼭짓점 쌍의 b - a 의 볼록 껍질과 같다.
    """
    A = robot_local @ rot(theta).T
    diffs = obstacle[:, None, :] - A[None, :, :]          # b - a, shape (Mb, Ma, 2)
    return convex_hull(diffs.reshape(-1, 2))


def workspace_cspace_bounds(robot_local, theta, ws):
    """로봇 전체가 직사각형 workspace ws=(xmin, xmax, ymin, ymax) 안에 있도록 하는 기준점 범위"""
    A = robot_local @ rot(theta).T
    xmin, xmax, ymin, ymax = ws
    return (xmin - A[:, 0].min(), xmax - A[:, 0].max(),
            ymin - A[:, 1].min(), ymax - A[:, 1].max())


# ---------------------------------------------------------------------------
# 충돌 검사 (Separating Axis Theorem, 벡터화)
# ---------------------------------------------------------------------------
def collide_translated(robot_local, theta, obstacle, Q):
    """
    방향 theta 인 로봇을 여러 위치 Q (N, 2) 에 놓았을 때 obstacle 과 충돌하는지 (N,) bool
    (Minkowski 결과를 '샘플링으로 검증'하는 용도: brute-force 방식)
    """
    A = robot_local @ rot(theta).T
    axes = np.vstack([edge_normals(A), edge_normals(obstacle)])     # (K, 2)
    pa = A @ axes.T                                                 # (Ma, K)
    pb = obstacle @ axes.T                                          # (Mb, K)
    shift = Q @ axes.T                                              # (N, K)
    a_min, a_max = pa.min(0) + shift, pa.max(0) + shift             # (N, K)
    b_min, b_max = pb.min(0), pb.max(0)                             # (K,)
    separated = (a_max < b_min) | (b_max < a_min)                   # (N, K)
    return ~separated.any(axis=1)


def segments_collide_polygon(P0, P1, poly):
    """선분들 [P0_i, P1_i] (N, 2) 이 볼록 다각형 poly 와 교차하는지 (N,) bool  (SAT)"""
    d = P1 - P0
    seg_n = np.stack([-d[:, 1], d[:, 0]], axis=1)                   # 선분마다 법선 1개 (N, 2)
    poly_n = edge_normals(poly)                                     # (M, 2)

    # (1) 다각형 변의 법선 축
    ps0, ps1 = P0 @ poly_n.T, P1 @ poly_n.T                         # (N, M)
    pp = poly @ poly_n.T                                            # (Mv, M)
    s_min, s_max = np.minimum(ps0, ps1), np.maximum(ps0, ps1)
    sep1 = (s_max < pp.min(0)) | (pp.max(0) < s_min)                # (N, M)

    # (2) 선분의 법선 축
    s_proj = np.einsum('ij,ij->i', P0, seg_n)                       # 선분은 이 축에서 한 점 (N,)
    pv = poly @ seg_n.T                                             # (Mv, N)
    sep2 = (s_proj < pv.min(0)) | (pv.max(0) < s_proj)              # (N,)

    return ~(sep1.any(axis=1) | sep2)


# ---------------------------------------------------------------------------
# 예제 장면
# ---------------------------------------------------------------------------
WS = (0.0, 10.0, 0.0, 10.0)

# 삼각형 로봇: 기준점(원점) = 한 꼭짓점 (슬라이드의 그림과 같은 설정)
ROBOT = np.array([[0.0, 0.0], [1.8, 0.45], [0.35, 1.1]])

OBSTACLES = [
    np.array([[6.0, 1.0], [9.0, 1.0], [9.0, 3.0], [6.0, 3.0]]),                 # O1: 직사각형
    np.array([[3.0, 5.0], [4.6, 5.4], [5.0, 7.0], [3.8, 8.2], [2.6, 7.0]]),     # O2: 오각형
]


def ccw(poly):
    """꼭짓점을 반시계 순서로 정렬 (볼록 다각형 가정)"""
    c = poly.mean(0)
    ang = np.arctan2(poly[:, 1] - c[1], poly[:, 0] - c[0])
    return poly[np.argsort(ang)]


def free_keys(*keys):
    """matplotlib 기본 단축키와 겹치는 키를 데모용으로 비워 둔다.
    (기본값: s=그림 저장, g=격자 표시, l=y축 log 스케일 등)"""
    import matplotlib as mpl
    for name in [k for k in mpl.rcParams if k.startswith('keymap.')]:
        mpl.rcParams[name] = [k for k in mpl.rcParams[name] if k not in keys]
