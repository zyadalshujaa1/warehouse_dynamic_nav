"""Pure (ROS-free) parts of the tracker, so they can be unit-tested.

Pipeline for one laser scan
---------------------------
1. points_from_scan : ranges -> 2D points in the map frame
2. static filter     : drop points on walls/shelves of the static map
                       (and points on the robot itself)
3. cluster_points    : split the remaining points into objects
                       (consecutive points closer than `gap` belong together)
4. KalmanTrack       : one constant-velocity Kalman filter per object,
                       state [x, y, vx, vy] in the map frame
5. associate         : greedy nearest-neighbour matching of detections
                       to predicted tracks, inside a gate

Method references (standard, not new):
- Constant-velocity Kalman filter with white-noise acceleration:
  Bar-Shalom, Li, Kirubarajan, "Estimation with Applications to Tracking
  and Navigation", Wiley 2001, ch. 6.
- Breaking a 2D scan into segments by the distance between consecutive
  points ("jump distance" segmentation) is the usual first step for
  2D lidar object detection.
"""
import math

import numpy as np


# ------------------------------------------------------------- geometry
def points_from_scan(ranges, angle_min, angle_inc, range_min, range_max,
                     max_range, sx, sy, syaw):
    """Return (N, 2) map-frame points and their scan indices.

    (sx, sy, syaw) is the pose of the laser frame in the map frame.
    """
    r = np.asarray(ranges, dtype=float)
    idx = np.arange(r.size)
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_range))
    r, idx = r[ok], idx[ok]
    a = syaw + angle_min + idx * angle_inc
    pts = np.column_stack((sx + r * np.cos(a), sy + r * np.sin(a)))
    return pts, idx


def in_robot_box(pts, rx, ry, ryaw, half_length, half_width):
    """Boolean mask: points inside the (inflated) robot rectangle."""
    c, s = math.cos(ryaw), math.sin(ryaw)
    dx, dy = pts[:, 0] - rx, pts[:, 1] - ry
    px, py = c * dx + s * dy, -s * dx + c * dy
    return (np.abs(px) <= half_length) & (np.abs(py) <= half_width)


class StaticMap:
    """Occupied cells of the static map, dilated by `margin` metres.

    Assumes the map origin has zero rotation (true for maps saved by
    slam_toolbox / map_server in this project).
    """

    def __init__(self, data, width, height, resolution, ox, oy,
                 margin, occupied_threshold=65):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        occ = grid >= occupied_threshold
        n = int(math.ceil(margin / resolution))
        pad = np.pad(occ, n)
        dil = np.zeros_like(occ)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if dx * dx + dy * dy <= n * n:
                    dil |= pad[n + dy:n + dy + height, n + dx:n + dx + width]
        self.mask = dil
        self.res, self.ox, self.oy = resolution, ox, oy
        self.w, self.h = width, height

    def is_static(self, pts):
        """Points on (dilated) occupied cells or outside the map."""
        cx = np.floor((pts[:, 0] - self.ox) / self.res).astype(int)
        cy = np.floor((pts[:, 1] - self.oy) / self.res).astype(int)
        inside = (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)
        out = np.ones(len(pts), dtype=bool)
        out[inside] = self.mask[cy[inside], cx[inside]]
        return out


def cluster_points(pts, gap, min_points, max_width, wrap=False):
    """Split ordered scan points into clusters.

    A new cluster starts when the distance to the previous point is larger
    than `gap`. With `wrap` (360-degree lidar) the last and first clusters
    are merged if they touch. Clusters with fewer than `min_points` points
    or wider than `max_width` (a wall left over by the static filter) are
    dropped. Only the first two columns are used as x, y; extra columns are
    carried along. Returns a list of (k, d) arrays.
    """
    if len(pts) == 0:
        return []
    clusters, cur = [], [0]
    for i in range(1, len(pts)):
        if np.hypot(*(pts[i, :2] - pts[i - 1, :2])) > gap:
            clusters.append(cur)
            cur = [i]
        else:
            cur.append(i)
    clusters.append(cur)
    if wrap and len(clusters) > 1 and np.hypot(*(pts[0, :2] - pts[-1, :2])) <= gap:
        clusters[0] = clusters.pop() + clusters[0]

    out = []
    for c in clusters:
        p = pts[c]
        if len(p) < min_points:
            continue
        if np.hypot(*(p[:, :2].max(axis=0) - p[:, :2].min(axis=0))) > max_width:
            continue
        out.append(p)
    return out


def cluster_centre(p, sensor_xy, offset):
    """Mean of the cluster, pushed `offset` metres away from the sensor.

    The laser only sees the near side of an object, so the plain mean is
    biased towards the sensor; `offset` compensates (0 = no correction).
    """
    m = p.mean(axis=0)
    if offset > 0.0:
        d = m - np.asarray(sensor_xy)
        n = np.hypot(*d)
        if n > 1e-6:
            m = m + offset * d / n
    return m


# ------------------------------------------------------------ tracking
class KalmanTrack:
    """Constant-velocity Kalman filter, state [x, y, vx, vy]."""

    H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    def __init__(self, tid, z, t, pos_std, vel_std):
        self.id = tid
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([pos_std ** 2, pos_std ** 2, vel_std ** 2, vel_std ** 2])
        self.t = t
        self.last_seen = t
        self.hits = 1

    def predict(self, t, accel_std):
        dt = t - self.t
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + (accel_std ** 2) * (G @ G.T)
        self.t = t

    def update(self, z, meas_std):
        R = np.eye(2) * meas_std ** 2
        y = np.asarray(z) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.last_seen = self.t
        self.hits += 1

    def position_at(self, t):
        """Constant-velocity extrapolation (used later for prediction)."""
        dt = t - self.t
        return self.x[0] + self.x[2] * dt, self.x[1] + self.x[3] * dt


def associate(tracks, detections, gate):
    """Greedy nearest-neighbour. Returns (pairs, unmatched_detection_ids)."""
    pairs, used_t, used_d = [], set(), set()
    cand = []
    for ti, tr in enumerate(tracks):
        for di, d in enumerate(detections):
            dist = math.hypot(d[0] - tr.x[0], d[1] - tr.x[1])
            if dist <= gate:
                cand.append((dist, ti, di))
    for _, ti, di in sorted(cand):
        if ti in used_t or di in used_d:
            continue
        pairs.append((ti, di))
        used_t.add(ti)
        used_d.add(di)
    return pairs, [d for d in range(len(detections)) if d not in used_d]


class MultiTracker:
    def __init__(self, gate, accel_std, meas_std, init_pos_std, init_vel_std,
                 max_missed_time):
        self.gate, self.accel_std, self.meas_std = gate, accel_std, meas_std
        self.init_pos_std, self.init_vel_std = init_pos_std, init_vel_std
        self.max_missed_time = max_missed_time
        self.tracks = []
        self.next_id = 1

    def step(self, t, detections):
        for tr in self.tracks:
            tr.predict(t, self.accel_std)
        pairs, new = associate(self.tracks, detections, self.gate)
        for ti, di in pairs:
            self.tracks[ti].update(detections[di], self.meas_std)
        for di in new:
            self.tracks.append(KalmanTrack(self.next_id, detections[di], t,
                                           self.init_pos_std, self.init_vel_std))
            self.next_id += 1
        self.tracks = [tr for tr in self.tracks
                       if t - tr.last_seen <= self.max_missed_time]
        return self.tracks
