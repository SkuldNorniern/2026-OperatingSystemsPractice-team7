"""
demo1_minkowski_2d.py
Workspace  <->  C-space (x, y) 비교 데모 : 방향 theta 를 고정한 삼각형 로봇

왼쪽 : workspace (장애물 O1, O2 와 로봇)
오른쪽: 해당 theta 에서의 C-space
        - 자홍색 : Minkowski sum 으로 계산한 C-obstacle  (-A(theta)) ⊕ B
        - 점선    : 로봇이 workspace 밖으로 나가지 않는 기준점 범위
        - 빨간 점 : brute-force 충돌 검사 결과 ('g' 키로 켜기/끄기) -> Minkowski 결과와 일치하는지 확인

조작
  - 두 그림 중 아무 곳이나 클릭/드래그 : 로봇 기준점 q 이동 (충돌이면 로봇이 빨간색)
  - 아래 슬라이더 : 로봇 방향 theta 변경  -> C-obstacle 모양이 바뀌는 것 확인
  - g : brute-force 샘플 표시 토글

실행:  python3 demo1_minkowski_2d.py            (창 띄우기)
       python3 demo1_minkowski_2d.py --save a.png  (그림 파일로 저장)
"""
import argparse
import numpy as np
import matplotlib
from matplotlib.patches import Polygon, Rectangle

from cspace_common import (ROBOT, OBSTACLES, WS, ccw, transform, minkowski_cobs,
                           workspace_cspace_bounds, collide_translated)

ap = argparse.ArgumentParser()
ap.add_argument('--save', default=None, help='그림을 파일로 저장하고 종료')
ap.add_argument('--theta', type=float, default=0.0, help='초기 방향 (deg)')
args = ap.parse_args()
if args.save:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cspace_common import free_keys
free_keys('g')   # matplotlib 기본 단축키(s: 저장, g: 격자, l: log 스케일)와 충돌 방지
from matplotlib.widgets import Slider

OBS = [ccw(o) for o in OBSTACLES]
state = {'q': np.array([1.0, 1.5]), 'theta': np.deg2rad(args.theta), 'grid': bool(args.save)}

fig, (axw, axc) = plt.subplots(1, 2, figsize=(13, 6.4))
plt.subplots_adjust(bottom=0.17, wspace=0.18)
ax_sl = fig.add_axes([0.25, 0.05, 0.5, 0.03])
sl = Slider(ax_sl, 'theta [deg]', -180, 180, valinit=args.theta)


def in_collision(q, th):
    return any(collide_translated(ROBOT, th, o, q[None, :])[0] for o in OBS)


def inside_ws(q, th):
    xmin, xmax, ymin, ymax = workspace_cspace_bounds(ROBOT, th, WS)
    return xmin <= q[0] <= xmax and ymin <= q[1] <= ymax


def draw():
    th, q = state['theta'], state['q']
    for ax in (axw, axc):
        ax.clear()
        ax.set_xlim(WS[0] - 1, WS[1] + 1); ax.set_ylim(WS[2] - 1, WS[3] + 1)
        ax.set_aspect('equal'); ax.grid(alpha=0.2)

    # ---------- workspace
    axw.set_title('Workspace  W')
    axw.add_patch(Rectangle((WS[0], WS[2]), WS[1] - WS[0], WS[3] - WS[2], fill=False, lw=2))
    for i, o in enumerate(OBS):
        axw.add_patch(Polygon(o, fc='0.65', ec='k'))
        axw.text(*o.mean(0), f'$O_{i+1}$', ha='center', va='center', fontsize=13)
    hit = in_collision(q, th) or not inside_ws(q, th)
    R = transform(ROBOT, (q[0], q[1], th))
    axw.add_patch(Polygon(R, fc=('tab:red' if hit else 'tab:blue'), alpha=0.6, ec='k'))
    axw.plot(*q, 'ko', ms=6)
    axw.text(q[0] + 0.15, q[1] - 0.45, 'reference point', fontsize=8)

    # ---------- C-space
    axc.set_title(r'C-space slice at $\theta$ = %.0f°   (magenta: $(-A)\oplus B$)' % np.rad2deg(th))
    xmin, xmax, ymin, ymax = workspace_cspace_bounds(ROBOT, th, WS)
    axc.add_patch(Rectangle((WS[0], WS[2]), WS[1] - WS[0], WS[3] - WS[2], fill=False, lw=1, ec='0.6'))
    axc.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, lw=2, ls='--'))
    for i, o in enumerate(OBS):
        axc.add_patch(Polygon(minkowski_cobs(ROBOT, o, th), fc='magenta', alpha=0.55, ec='purple'))
        axc.add_patch(Polygon(o, fc='0.65', ec='k', alpha=0.8))
    if state['grid']:
        xs, ys = np.meshgrid(np.linspace(WS[0], WS[1], 90), np.linspace(WS[2], WS[3], 90))
        Q = np.c_[xs.ravel(), ys.ravel()]
        bad = np.zeros(len(Q), bool)
        for o in OBS:
            bad |= collide_translated(ROBOT, th, o, Q)
        axc.plot(Q[bad, 0], Q[bad, 1], 'r.', ms=2, label='brute-force collision samples')
        axc.legend(loc='upper right', fontsize=8)
    axc.plot(*q, 'o', color=('tab:red' if hit else 'tab:blue'), ms=9, mec='k')
    fig.suptitle('q = (%.2f, %.2f, %.0f°)   ->   %s' %
                 (q[0], q[1], np.rad2deg(th), 'COLLISION' if hit else 'free'),
                 color=('tab:red' if hit else 'k'))
    fig.canvas.draw_idle()


def on_click(ev):
    if ev.inaxes in (axw, axc) and ev.xdata is not None and (ev.button == 1):
        state['q'] = np.array([ev.xdata, ev.ydata]); draw()


def on_key(ev):
    if ev.key == 'g':
        state['grid'] = not state['grid']; draw()


sl.on_changed(lambda v: (state.__setitem__('theta', np.deg2rad(v)), draw()))
fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('motion_notify_event', lambda ev: on_click(ev) if ev.button == 1 else None)
fig.canvas.mpl_connect('key_press_event', on_key)
draw()

if args.save:
    fig.savefig(args.save, dpi=130)
else:
    plt.show()
