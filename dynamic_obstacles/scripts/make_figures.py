#!/usr/bin/env python3
"""Draw the README figures from the trial trajectories.

Usage: python3 make_figures.py <dir with traj_*.csv> <dir with warehouse.pgm> <output dir>
"""
import csv, math, glob, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D
from PIL import Image

TRAJ = sys.argv[1]; MAPDIR = sys.argv[2]; OUT = sys.argv[3]
SURF, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
BLUE, ORANGE = '#2a78d6', '#eb6834'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK,
                     'axes.facecolor': SURF, 'figure.facecolor': SURF, 'font.family': 'DejaVu Sans'})

def load(name):
    r = list(csv.DictReader(open(f'{TRAJ}/traj_{name}.csv')))
    if not r:
        return None
    a = {k: np.array([float(x[k]) for x in r]) for k in r[0]}
    # speed: per-step speed, then a 0.5 s (11-sample) median. AMCL corrections move the map
    # pose by 8-17 cm over 2-3 samples; the median removes them, real motion is kept.
    step = np.hypot(np.diff(a['robot_x']), np.diff(a['robot_y'])) / np.maximum(np.diff(a['t']), 1e-3)
    step = np.r_[step[0], step]
    pad = np.pad(step, 5, mode='edge')
    med = np.array([np.median(pad[i:i + 11]) for i in range(len(step))])
    a['speed'] = np.convolve(np.pad(med, 10, mode='edge'), np.ones(21) / 21, mode='valid')  # 1 s mean
    return a

img = np.array(Image.open(f'{MAPDIR}/warehouse.pgm'))
res, ox, oy = 0.05, -15.0, -25.0
h, w = img.shape
ext = [ox, ox + w * res, oy, oy + h * res]

def draw_map(ax):
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=ext, origin='upper', alpha=0.35, interpolation='nearest')

def footprint(ax, x, y, yaw, color):
    rect = Rectangle((-0.41, -0.31), 0.82, 0.62, facecolor=color, alpha=0.25, edgecolor=color, lw=1.5)
    rect.set_transform(Affine2D().rotate(yaw).translate(x, y) + ax.transData)
    ax.add_patch(rect)

# ---------------- Figure 1: head-on, MPPI / critic v1 / critic v2 -----------------
runs = [('head_on_mppi_1', 'MPPI', 'avoids, min clearance 0.68 m', '#52514e'),
        ('head_on_mppi_pred_1', 'MPPI + critic v1', 'freezes in the path → contact', '#6250d6'),
        ('head_on_mppi_pred2_1', 'MPPI + critic v2', 'avoids, min clearance 0.83 m', '#1baf7a')]
fig = plt.figure(figsize=(12, 7.2))
gs = fig.add_gridspec(2, 3, height_ratios=[2.3, 1], hspace=0.22, wspace=0.12)
speed_ax = fig.add_subplot(gs[1, :])
for j, (name, title, note, mcol) in enumerate(runs):
    a = load(name)
    ax = fig.add_subplot(gs[0, j])
    draw_map(ax)
    i = int(np.argmin(a['worker_1_clearance']))
    t0 = a['t'][i]
    win = (a['t'] > t0 - 25) & (a['t'] <= a['t'][min(i + 200, len(a['t']) - 1)])
    moved = np.hypot(np.diff(a['worker_1_x']), np.diff(a['worker_1_y'])) > 1e-4
    wk = np.r_[False, moved] & win
    ax.plot(a['robot_x'][win], a['robot_y'][win], color=mcol, lw=2.2, solid_capstyle='round', label='robot')
    ax.plot(a['worker_1_x'][wk], a['worker_1_y'][wk], color=ORANGE, lw=2, ls=(0, (4, 2)), label='worker')
    footprint(ax, a['robot_x'][i], a['robot_y'][i], a['robot_yaw'][i], mcol)
    ax.add_patch(Circle((a['worker_1_x'][i], a['worker_1_y'][i]), 0.25, facecolor=ORANGE, alpha=0.35, edgecolor=ORANGE, lw=1.5))
    if a['worker_1_clearance'][i] <= 0.02:
        ax.plot(a['robot_x'][i], a['robot_y'][i], marker='x', ms=11, mew=2.5, color=INK)
    ax.set_xlim(-4.5, 1.5); ax.set_ylim(0.5, 9.5); ax.set_aspect('equal')
    ax.set_title(f'{title}\n', fontsize=11, fontweight='bold', color=INK, loc='left')
    ax.text(0, 1.015, note, transform=ax.transAxes, fontsize=9.5, color=INK2, va='bottom')
    ax.set_xticks([-4, -2, 0]); ax.set_yticks([2, 4, 6, 8])
    ax.tick_params(length=0, labelsize=8)
    ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
    if j == 0:
        ax.set_ylabel('y (m)')
    else:
        ax.set_yticklabels([])
    ax.set_xlabel('x (m)', fontsize=9)
    ax.legend(loc='lower left', frameon=False, fontsize=9)
    rel = a['t'] - t0
    sel = (rel > -12) & (rel < 5)
    speed_ax.plot(rel[sel], a['speed'][sel], lw=2, color=mcol, label=title)
speed_ax.axvline(0, color=GRID, lw=1)
speed_ax.text(0.12, 0.395, 'closest approach', color=INK2, fontsize=9, va='top')
speed_ax.set_xlabel('time relative to closest approach to the worker (s)')
speed_ax.set_ylabel('robot speed (m/s)')
speed_ax.set_ylim(0, 0.4); speed_ax.set_xlim(-12, 5)
speed_ax.grid(axis='y', color=GRID, lw=0.8); speed_ax.set_axisbelow(True)
for s in ('top', 'right'): speed_ax.spines[s].set_visible(False)
speed_ax.legend(loc='lower left', frameon=False, ncol=3, fontsize=9)
fig.suptitle('Head-on scenario: the first critic freezes in the worker\'s path, the terminal extension fixes it',
             x=0.125, ha='left', fontsize=12.5, fontweight='bold', y=0.985)
fig.text(0.125, 0.935, 'One example run per method, worker walking towards the loaded robot at 0.6 m/s. '
         'Shapes show robot footprint and worker at the closest approach; × = contact.', fontsize=9.5, color=INK2)
fig.savefig(f'{OUT}/headon_example.png', dpi=130, bbox_inches='tight')

# ---------------- Figure 2: crossing contacts, speed before contact -----------------
fig, ax = plt.subplots(figsize=(10, 4.3))
grid_t = np.arange(-8, 0.01, 0.1)
summary = {}
for method, color, label in [('mppi', '#52514e', 'MPPI'), ('mppi_pred2', '#1baf7a', 'MPPI + critic v2')]:
    curves = []
    for f in sorted(glob.glob(f'{TRAJ}/traj_crossing_{method}_*.csv')):
        name = f.split('traj_')[1][:-4]
        if not name.split('_')[-1].isdigit() or name.replace('crossing_', '').rsplit('_', 1)[0] != method:
            continue
        a = load(name)
        if a is None or a['worker_1_clearance'][-1] > 0.02 or len(a['t']) < 200:
            continue
        rel = a['t'] - a['t'][-1]
        curves.append(np.interp(grid_t, rel, a['speed']))
        ax.plot(grid_t, curves[-1], color=color, lw=1, alpha=0.28)
    curves = np.array(curves)
    med = np.median(curves, axis=0)
    ax.plot(grid_t, med, color=color, lw=2.6, label=f'{label}: median of {len(curves)} contact runs')
    summary[method] = len(curves)
ax.set_xlim(-8, 0); ax.set_ylim(0, 0.42)
ax.set_xlabel('time before contact (s)'); ax.set_ylabel('robot speed (m/s)')
ax.grid(axis='y', color=GRID, lw=0.8); ax.set_axisbelow(True)
for s in ('top', 'right'): ax.spines[s].set_visible(False)
ax.legend(loc='lower left', frameon=False, fontsize=9.5)
ax.set_title('Crossing scenario: plain MPPI keeps full speed; the critic brakes, yet contact still happens',
             loc='left', fontsize=11.5, fontweight='bold', pad=22)
ax.text(0, 1.03, 'Robot speed in the 8 s before contact, every crossing trial that ended in contact (thin lines = single runs).',
        transform=ax.transAxes, fontsize=9.5, color=INK2)
fig.savefig(f'{OUT}/crossing_speed.png', dpi=130, bbox_inches='tight')
print(summary)
