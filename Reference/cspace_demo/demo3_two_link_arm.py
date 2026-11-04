"""
demo3_two_link_arm.py
2-link 평면 로봇팔의 C-space (theta1, theta2) 를 충돌 검사로 계산하는 데모

- 관절각 격자 (기본 1° 간격, 360 x 360 = 129,600 configuration) 마다 두 링크(선분)와
  장애물(볼록 다각형)의 충돌 여부를 검사해 C_obs 를 그린다.
  -> 다각형 로봇과 달리 Minkowski sum 으로 바로 구할 수 없어 '샘플링 + 충돌 검사'로 계산
- C-space 는 토러스(T^2): theta = -180° 와 +180° 가 같은 점 (좌우, 위아래 경계가 이어짐)

조작
  - 오른쪽 C-space 클릭 : 해당 관절각의 로봇팔을 왼쪽 workspace 에 표시
  - s / g : 현재 configuration 을 start / goal 로 지정
  - l     : start -> goal 을 C-space 에서 '직선'으로 잇고, 그 위를 촘촘히 충돌 검사
            (sampling-based planner 의 local planner / edge checking 과 같은 연산)

실행:  python3 demo3_two_link_arm.py [--res 1.0] [--save c.png]
"""
import argparse
import numpy as np
import matplotlib

from cspace_common import ccw, segments_collide_polygon

ap = argparse.ArgumentParser()
ap.add_argument('--res', type=float, default=1.0, help='관절각 격자 간격 (deg)')
ap.add_argument('--save', default=None)
args = ap.parse_args()
if args.save:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cspace_common import free_keys
free_keys('s', 'g', 'l')   # matplotlib 기본 단축키(s: 저장, g: 격자, l: log 스케일)와 충돌 방지
from matplotlib.patches import Polygon

L1, L2 = 2.0, 1.5
OBS = [ccw(np.array(p, float)) for p in [
    [[1.6, 1.2], [2.6, 1.2], [2.6, 2.2], [1.6, 2.2]],
    [[-2.8, -0.6], [-1.8, -0.6], [-1.8, 0.6], [-2.8, 0.6]],
    [[0.6, -2.9], [1.8, -2.4], [1.2, -1.6]],
]]


def fk(t1, t2):
    """관절각(rad) -> 관절 위치 (base, elbow, tip)"""
    p0 = np.zeros(np.shape(t1) + (2,))
    p1 = np.stack([L1 * np.cos(t1), L1 * np.sin(t1)], -1)
    p2 = p1 + np.stack([L2 * np.cos(t1 + t2), L2 * np.sin(t1 + t2)], -1)
    return p0, p1, p2


def collision(t1, t2):
    t1, t2 = np.atleast_1d(t1), np.atleast_1d(t2)
    p0, p1, p2 = fk(t1, t2)
    hit = np.zeros(t1.shape, bool)
    for o in OBS:
        hit |= segments_collide_polygon(p0, p1, o) | segments_collide_polygon(p1, p2, o)
    return hit


# ---------- C-space 계산 (격자 + 충돌 검사)
deg = np.arange(-180, 180, args.res)
T1, T2 = np.meshgrid(np.deg2rad(deg), np.deg2rad(deg), indexing='xy')   # 행: theta2, 열: theta1
C_OBS = collision(T1.ravel(), T2.ravel()).reshape(T1.shape)
print(f'grid {len(deg)} x {len(deg)} = {C_OBS.size} configs, occupied {C_OBS.mean()*100:.1f}%')

state = {'q': np.deg2rad([30.0, 60.0]), 'start': None, 'goal': None, 'line': None}

fig, (axw, axc) = plt.subplots(1, 2, figsize=(13, 6.3))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def straight_line(qa, qb, step=np.deg2rad(0.5)):
    """토러스 위 최단 방향 직선 보간 (각 관절은 짧은 쪽으로 회전)"""
    d = wrap(qb - qa)
    n = int(np.ceil(np.abs(d).max() / step)) + 1
    s = np.linspace(0, 1, n)[:, None]
    return wrap(qa + s * d)


def draw():
    q = state['q']
    axw.clear(); axc.clear()
    # workspace
    axw.set_xlim(-4, 4); axw.set_ylim(-4, 4); axw.set_aspect('equal'); axw.grid(alpha=0.2)
    for i, o in enumerate(OBS):
        axw.add_patch(Polygon(o, fc='0.65', ec='k'))
        axw.text(*o.mean(0), f'$O_{i+1}$', ha='center', va='center')
    hit = collision(*q)[0]
    p0, p1, p2 = fk(*q)
    axw.plot(*np.array([p0, p1, p2]).T, '-o', lw=5, color=('tab:red' if hit else 'tab:blue'))
    if state['line'] is not None:      # 직선 경로를 따라가는 팔의 모습 (일부)
        for t in state['line'][::max(1, len(state['line']) // 12)]:
            a, b, c = fk(*t)
            axw.plot(*np.array([a, b, c]).T, '-', lw=1, color='0.4', alpha=0.5)
    axw.set_title('Workspace: q = (%.0f°, %.0f°)  %s' %
                  (*np.rad2deg(q), 'COLLISION' if hit else 'free'),
                  color=('tab:red' if hit else 'k'))
    # C-space
    axc.imshow(C_OBS, origin='lower', extent=[-180, 180, -180, 180], cmap='Greys', alpha=0.75)
    axc.set_xlabel(r'$\theta_1$ [deg]'); axc.set_ylabel(r'$\theta_2$ [deg]')
    axc.set_title(r'C-space $T^2$ (black: $C_{obs}$, res %.1f°)' % args.res)
    axc.plot(*np.rad2deg(q), 'o', ms=9, mec='k', color=('tab:red' if hit else 'tab:blue'))
    for key, c in (('start', 'tab:green'), ('goal', 'tab:orange')):
        if state[key] is not None:
            axc.plot(*np.rad2deg(state[key]), 's', ms=10, color=c, mec='k', label=key)
    if state['line'] is not None:
        L = np.rad2deg(state['line'])
        bad = collision(state['line'][:, 0], state['line'][:, 1])
        axc.plot(L[~bad, 0], L[~bad, 1], '.', ms=2, color='tab:green')
        axc.plot(L[bad, 0], L[bad, 1], '.', ms=3, color='tab:red')
        axc.set_title(axc.get_title() + ('\nstraight line: COLLISION (%d pts)' % bad.sum() if bad.any()
                                          else '\nstraight line: collision-free'))
    if state['start'] is not None or state['goal'] is not None:
        axc.legend(loc='upper right', fontsize=8)
    fig.canvas.draw_idle()


def on_click(ev):
    if ev.inaxes is axc and ev.xdata is not None:
        state['q'] = np.deg2rad([ev.xdata, ev.ydata]); draw()


def on_key(ev):
    if ev.key in ('s', 'g'):
        state['start' if ev.key == 's' else 'goal'] = state['q'].copy(); state['line'] = None
    elif ev.key == 'l' and state['start'] is not None and state['goal'] is not None:
        state['line'] = straight_line(state['start'], state['goal'])
    draw()


fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('key_press_event', on_key)

if args.save:   # 저장 모드: 예시 start/goal 과 직선 경로까지 그려서 저장
    state['start'], state['goal'] = np.deg2rad([0.0, 0.0]), np.deg2rad([120.0, -60.0])
    state['q'] = state['start']; state['line'] = straight_line(state['start'], state['goal'])
draw()
if args.save:
    fig.savefig(args.save, dpi=130)
else:
    plt.show()
