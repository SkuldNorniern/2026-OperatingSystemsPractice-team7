"""
demo2_cspace_3d.py
회전까지 포함한 C-space (x, y, theta) 를 theta 단면별로 계산해 3D로 쌓아 보여주는 데모

- 각 theta 마다 C-obstacle = (-A(theta)) ⊕ B 를 계산 (demo1 과 동일한 함수)
- 단면들을 theta 축으로 쌓으면, 회전에 따라 비틀리며 변하는 3차원 C-obstacle 이 된다
  (강의 슬라이드 'C-space with rotation' 그림에 해당)
- theta 와 theta ± 2π 는 같은 방향이므로 C-space 는 R^2 × S^1 (위아래가 연결된 공간)

실행:  python3 demo2_cspace_3d.py [--n 36] [--save b.png]
"""
import argparse
import numpy as np
import matplotlib

from cspace_common import ROBOT, OBSTACLES, WS, ccw, minkowski_cobs

ap = argparse.ArgumentParser()
ap.add_argument('--n', type=int, default=36, help='theta 단면 개수')
ap.add_argument('--save', default=None)
args = ap.parse_args()
if args.save:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OBS = [ccw(o) for o in OBSTACLES]
thetas = np.linspace(-np.pi, np.pi, args.n, endpoint=False)
colors = ['magenta', 'darkorange']

fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection='3d')

for k, o in enumerate(OBS):
    prev = None
    for th in thetas:
        P = minkowski_cobs(ROBOT, o, th)
        z = np.full(len(P), np.rad2deg(th))
        ax.add_collection3d(Poly3DCollection([np.c_[P, z]], facecolor=colors[k], alpha=0.18,
                                             edgecolor=colors[k], linewidth=0.4))
    # 바닥(theta = -180°)에 실제 장애물 표시
    ax.add_collection3d(Poly3DCollection([np.c_[o, np.full(len(o), -180)]], facecolor='0.5', alpha=0.9))

ax.set_xlim(WS[0] - 2, WS[1]); ax.set_ylim(WS[2] - 2, WS[3]); ax.set_zlim(-180, 180)
ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel(r'$\theta$ [deg]')
ax.set_title('C-obstacles in (x, y, θ): stacked slices of (−A(θ)) ⊕ B\n'
             '(gray at bottom: workspace obstacles O1, O2)')
ax.view_init(elev=22, azim=-60)

if args.save:
    fig.savefig(args.save, dpi=130)
else:
    plt.show()
