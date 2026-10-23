#!/usr/bin/env python3
"""Draw the README figures from the logged trial trajectories.

Usage:
    python3 make_figures.py <dir with traj_*.csv> <output dir>

Fig. 1  headon_example.png   head-on scenario, one run per method: paths + robot speed
Fig. 2  crossing_speed.png   crossing scenario, robot speed before contact (all contact runs)

Robot speed = per-step speed from the logged map pose, 0.5 s median (removes the
8-17 cm jumps of AMCL corrections), then 1 s moving mean.
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402

TRAJ, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)

C_MPPI, C_V1, C_V2, C_WORKER = '#52514e', '#6250d6', '#1baf7a', '#eb6834'
GRID = '#dddddd'
plt.rcParams.update({
    'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix', 'font.size': 9,
    'axes.titlesize': 9, 'axes.labelsize': 9, 'legend.fontsize': 8,
    'xtick.labelsize': 8, 'ytick.labelsize': 8, 'axes.linewidth': 0.6,
    'xtick.major.width': 0.6, 'ytick.major.width': 0.6,
    'xtick.direction': 'in', 'ytick.direction': 'in',
    'savefig.facecolor': 'white', 'figure.facecolor': 'white',
})


def load(name):
    """Trajectory of one trial, with robot speed; None for an empty log."""
    rows = list(csv.DictReader(open(f'{TRAJ}/traj_{name}.csv')))
    if not rows:
        return None
    a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    step = np.hypot(np.diff(a['robot_x']), np.diff(a['robot_y'])) / np.maximum(np.diff(a['t']), 1e-3)
    step = np.r_[step[0], step]
    pad = np.pad(step, 5, mode='edge')
    med = np.array([np.median(pad[i:i + 11]) for i in range(len(step))])
    a['speed'] = np.convolve(np.pad(med, 10, mode='edge'), np.ones(21) / 21, mode='valid')
    return a


def footprint(ax, x, y, yaw, color):
    rect = Rectangle((-0.41, -0.31), 0.82, 0.62, facecolor=color, alpha=0.25,
                     edgecolor=color, lw=0.9)
    rect.set_transform(Affine2D().rotate(yaw).translate(x, y) + ax.transData)
    ax.add_patch(rect)


def clean(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.grid(axis='y', color=GRID, lw=0.5)
    ax.set_axisbelow(True)


# ------------------------------------------------------------------ Fig. 1
runs = [('head_on_mppi_1', '(a) MPPI', C_MPPI),
        ('head_on_mppi_pred_1', '(b) MPPI + critic v1', C_V1),
        ('head_on_mppi_pred2_1', '(c) MPPI + critic v2', C_V2)]
fig = plt.figure(figsize=(7.16, 5.6))
gs = fig.add_gridspec(2, 3, height_ratios=[2.2, 1], hspace=0.38, wspace=0.08)
ax_v = fig.add_subplot(gs[1, :])
for j, (name, label, color) in enumerate(runs):
    a = load(name)
    ax = fig.add_subplot(gs[0, j])
    i = int(np.argmin(a['worker_1_clearance']))           # closest approach
    t0 = a['t'][i]
    win = (a['t'] > t0 - 25) & (a['t'] <= a['t'][min(i + 200, len(a['t']) - 1)])
    walking = np.r_[False, np.hypot(np.diff(a['worker_1_x']), np.diff(a['worker_1_y'])) > 1e-4] & win
    ax.plot(a['robot_x'][win], a['robot_y'][win], color=color, lw=1.4)
    ax.plot(a['worker_1_x'][walking], a['worker_1_y'][walking], color=C_WORKER, lw=1.2, ls=(0, (4, 2)))
    footprint(ax, a['robot_x'][i], a['robot_y'][i], a['robot_yaw'][i], color)
    ax.add_patch(Circle((a['worker_1_x'][i], a['worker_1_y'][i]), 0.25, facecolor=C_WORKER,
                        alpha=0.3, edgecolor=C_WORKER, lw=0.9))
    if a['worker_1_clearance'][i] <= 0.02:
        ax.plot(a['robot_x'][i], a['robot_y'][i], marker='x', ms=7, mew=1.6, color='black')
    ax.set_xlim(-4.5, 1.5)
    ax.set_ylim(0.5, 9.5)
    ax.set_aspect('equal')
    ax.set_xticks([-4, -2, 0])
    ax.set_yticks([2, 4, 6, 8])
    ax.grid(color=GRID, lw=0.5)
    ax.set_axisbelow(True)
    ax.set_xlabel('$x$ (m)')
    if j == 0:
        ax.set_ylabel('$y$ (m)')
        ax.legend(handles=[Line2D([], [], color='black', lw=1.4, label='robot'),
                           Line2D([], [], color=C_WORKER, lw=1.2, ls=(0, (4, 2)), label='worker')],
                  loc='lower left', frameon=False, handlelength=2.2)
    else:
        ax.set_yticklabels([])
    ax.set_title(label, loc='left')
    rel = a['t'] - t0
    sel = (rel > -12) & (rel < 5)
    ax_v.plot(rel[sel], a['speed'][sel], color=color, lw=1.2, label=label[4:])
ax_v.axvline(0, color='#999999', lw=0.7, ls=(0, (3, 2)))
ax_v.set_xlim(-12, 5)
ax_v.set_ylim(0, 0.4)
ax_v.set_xlabel('Time relative to closest approach (s)')
ax_v.set_ylabel('Speed (m/s)')
ax_v.set_title('(d)', loc='left')
ax_v.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), frameon=False, ncol=3, borderaxespad=0.2)
clean(ax_v)
fig.savefig(f'{OUT}/headon_example.png', dpi=300, bbox_inches='tight')

# ------------------------------------------------------------------ Fig. 2
fig, ax = plt.subplots(figsize=(4.6, 2.7))
grid_t = np.arange(-8, 0.001, 0.1)
for method, color, label in [('mppi', C_MPPI, 'MPPI'), ('mppi_pred2', C_V2, 'MPPI + critic v2')]:
    curves = []
    for path in sorted(glob.glob(f'{TRAJ}/traj_crossing_{method}_*.csv')):
        name = os.path.basename(path)[5:-4]
        if name.replace('crossing_', '').rsplit('_', 1)[0] != method:
            continue                                           # mppi_* would match mppi_pred2
        a = load(name)
        if a is None or len(a['t']) < 200 or a['worker_1_clearance'][-1] > 0.02:
            continue                                           # contact trials only
        curves.append(np.interp(grid_t, a['t'] - a['t'][-1], a['speed']))
    curves = np.array(curves)
    q25, q50, q75 = np.percentile(curves, [25, 50, 75], axis=0)
    ax.fill_between(grid_t, q25, q75, color=color, alpha=0.18, lw=0)
    ax.plot(grid_t, q50, color=color, lw=1.4, label=f'{label} ($n$ = {len(curves)})')
ax.set_xlim(-8, 0)
ax.set_ylim(0, 0.4)
ax.set_xlabel('Time before contact (s)')
ax.set_ylabel('Speed (m/s)')
ax.legend(loc='lower left', frameon=False)
clean(ax)
fig.savefig(f'{OUT}/crossing_speed.png', dpi=300, bbox_inches='tight')
print('written', OUT)
