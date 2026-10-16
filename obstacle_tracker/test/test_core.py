"""Offline check of the tracker on a synthetic scan (no ROS needed).

A cylinder (radius 0.25 m, like the simulated worker) walks across in front
of a 360-degree lidar at 0.6 m/s. We check that the tracker finds exactly one
object and that its position and velocity converge to the truth.

Run:  python3 -m pytest test/test_core.py -q   (or just python3 test/test_core.py)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from obstacle_tracker.core import (MultiTracker, cluster_centre,  # noqa: E402
                                   cluster_points, points_from_scan)

N_BEAMS, RANGE_MAX = 360, 12.0
ANG_MIN, ANG_INC = -math.pi, 2 * math.pi / N_BEAMS


def ray_circle(ox, oy, a, cx, cy, r):
    dx, dy = math.cos(a), math.sin(a)
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def fake_scan(cx, cy, rng, radius=0.25, noise=0.01):
    ranges = []
    for i in range(N_BEAMS):
        a = ANG_MIN + i * ANG_INC
        d = ray_circle(0.0, 0.0, a, cx, cy, radius)
        ranges.append(d + rng.normal(0, noise) if d < RANGE_MAX else math.inf)
    return ranges


def run(offset):
    rng = np.random.default_rng(0)
    trk = MultiTracker(gate=1.0, accel_std=1.0, meas_std=0.08,
                       init_pos_std=0.3, init_vel_std=1.0, max_missed_time=1.0)
    errs, verrs, counts = [], [], []
    for k in range(100):                       # 10 s at 10 Hz
        t = 0.1 * k
        cx, cy = -3.0 + 0.6 * t, 3.0           # walks along +x, 3 m in front
        pts, _ = points_from_scan(fake_scan(cx, cy, rng), ANG_MIN, ANG_INC,
                                  0.1, RANGE_MAX, 8.0, 0.0, 0.0, 0.0)
        cl = cluster_points(pts, gap=0.2, min_points=3, max_width=0.8, wrap=True)
        dets = [cluster_centre(p, (0.0, 0.0), offset) for p in cl]
        tracks = trk.step(t, dets)
        counts.append(len(tracks))
        if t >= 2.0 and tracks:               # after the filter has settled
            tr = tracks[0]
            errs.append(math.hypot(tr.x[0] - cx, tr.x[1] - cy))
            verrs.append(math.hypot(tr.x[2] - 0.6, tr.x[3] - 0.0))
    return max(counts), float(np.mean(errs)), float(np.mean(verrs))


def test_single_object_converges():
    n, pos_err, vel_err = run(offset=0.0)
    assert n == 1
    assert vel_err < 0.1        # m/s
    assert pos_err < 0.25       # m, includes the near-side bias


def test_offset_reduces_bias():
    _, err0, _ = run(offset=0.0)
    _, err1, _ = run(offset=0.15)
    assert err1 < err0


if __name__ == '__main__':
    for off in (0.0, 0.10, 0.15, 0.20):
        n, pe, ve = run(off)
        print(f'offset={off:.2f}  max tracks={n}  mean pos err={pe:.3f} m  '
              f'mean vel err={ve:.3f} m/s')
