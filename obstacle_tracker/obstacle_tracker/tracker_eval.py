"""Compare the tracker output with the true worker position.

Truth   : /workers/<worker>/pose  (published by dynamic_obstacles/worker_mover;
          exact, because the worker has no gravity or friction and moves at
          the commanded velocity)
Tracker : /tracked_obstacles      (obstacle_tracker)

Every 0.1 s the track nearest to the worker is taken, moved to the truth time
with its own velocity, and compared.

Coverage is reported against what the laser could actually see: for each scan
the node counts the laser points that fall on the worker, and follows them
through the same filters the tracker uses (static map, robot body, clustering,
read from the tracker's config/tracker.yaml). So when the worker is lost it
also says at which stage.

Stop with Ctrl+C: prints a summary and writes
<results_dir>/tracker_eval_<tag>.csv
"""
import collections
import csv
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_tracker.core import StaticMap, cluster_points, in_robot_box, points_from_scan

WORKER_RADIUS = 0.25
NEAR = WORKER_RADIUS + 0.15     # a laser point this close to the worker centre is on the worker


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tracker_config():
    path = os.path.join(get_package_share_directory('obstacle_tracker'), 'config', 'tracker.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['obstacle_tracker']['ros__parameters']


class TrackerEval(Node):

    def __init__(self):
        super().__init__('tracker_eval',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('worker', 'worker_1')
        p('tag', 'test')
        p('match_radius', 1.0)     # m, a track further than this is not the worker
        p('min_speed', 0.1)        # m/s, velocity error only while the worker walks
        p('results_dir', os.path.expanduser('~/logistic_ws/results'))
        p('map_frame', 'map')
        p('robot_frame', 'base_footprint')
        g = lambda n: self.get_parameter(n).value
        self.tag, self.match_radius, self.min_speed = g('tag'), g('match_radius'), g('min_speed')
        self.results_dir = os.path.expanduser(g('results_dir'))
        self.map_frame, self.robot_frame = g('map_frame'), g('robot_frame')
        self.cfg = tracker_config()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = collections.deque()           # (t, x, y), last ~0.5 s
        self.tracks, self.tracks_t = [], None      # tracks of the latest scan
        self.static_map = None
        self.diag = None                           # laser diagnostics of the latest scan
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, f'/workers/{g("worker")}/pose', self.on_truth, 50)
        self.create_subscription(Odometry, '/tracked_obstacles', self.on_track, 50)
        self.create_subscription(OccupancyGrid, self.cfg.get('map_topic', '/map'), self.on_map, map_qos)
        self.create_subscription(LaserScan, self.cfg.get('scan_topic', '/scan'), self.on_scan, 10)
        self.rows = []
        self.create_timer(0.1, self.sample)
        self.get_logger().info('tracker_eval running, press Ctrl+C to finish')

    # ----------------------------------------------------------- inputs
    def on_truth(self, msg):
        self.truth.append((stamp_s(msg), msg.pose.position.x, msg.pose.position.y))
        while self.truth and self.truth[-1][0] - self.truth[0][0] > 0.5:
            self.truth.popleft()

    def on_track(self, msg):
        t = stamp_s(msg)
        if self.tracks_t is None or t > self.tracks_t:
            self.tracks, self.tracks_t = [], t
        tw = msg.twist.twist.linear
        self.tracks.append((msg.pose.pose.position.x, msg.pose.pose.position.y, tw.x, tw.y))

    def on_map(self, msg):
        o = msg.info.origin
        self.static_map = StaticMap(msg.data, msg.info.width, msg.info.height,
                                    msg.info.resolution, o.position.x, o.position.y,
                                    self.cfg['map_margin'])

    def lookup(self, frame, stamp=None):
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                tr = self.tf_buffer.lookup_transform(self.map_frame, frame, t).transform
                return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            except TransformException:
                continue
        return None

    def on_scan(self, scan):
        """Follow the laser points on the worker through the tracker's filters."""
        if not self.truth or self.static_map is None:
            return
        c = self.cfg
        stamp = Time.from_msg(scan.header.stamp)
        sensor = self.lookup(scan.header.frame_id, stamp)
        robot = self.lookup(self.robot_frame, stamp)
        if sensor is None or robot is None:
            return
        _, wx, wy = self.truth[-1]
        pts, _ = points_from_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                  scan.range_min, scan.range_max, c['max_range'], *sensor)
        if len(pts) == 0:
            self.diag = (0, 0, 0, 0, 0.0)
            return
        near = np.hypot(pts[:, 0] - wx, pts[:, 1] - wy) < NEAR
        not_static = ~self.static_map.is_static(pts)
        not_robot = ~in_robot_box(pts, *robot,
                                  c['robot_half_length'] + c['robot_margin'],
                                  c['robot_half_width'] + c['robot_margin'])
        raw = int(near.sum())
        after_static = int((near & not_static).sum())
        kept = not_static & not_robot
        after_robot = int((near & kept).sum())
        # cluster exactly as the tracker does, but without dropping anything,
        # then look at the cluster that holds the worker's points
        kpts, knear = pts[kept], near[kept]
        cl_n, cl_w = 0, 0.0
        if knear.any():
            order = np.arange(len(kpts))
            full_circle = abs(scan.angle_increment * len(scan.ranges)) > 6.2
            for cl in cluster_points(np.column_stack((kpts, order)), c['cluster_gap'], 1, math.inf,
                                     wrap=full_circle):
                idx = cl[:, 2].astype(int)
                if knear[idx].sum() > 0 and len(idx) > cl_n:
                    p = kpts[idx]
                    cl_n, cl_w = len(idx), float(np.hypot(*(p.max(axis=0) - p.min(axis=0))))
        self.diag = (raw, after_static, after_robot, cl_n, cl_w)

    # ---------------------------------------------------------- sampling
    def sample(self):
        if len(self.truth) < 2:
            return
        robot = self.lookup(self.robot_frame)
        if robot is None:
            return
        t, x, y = self.truth[-1]
        t0, x0, y0 = self.truth[0]
        vx, vy = ((x - x0) / (t - t0), (y - y0) / (t - t0)) if t > t0 else (0.0, 0.0)
        dist_robot = math.hypot(x - robot[0], y - robot[1])

        best = None
        if self.tracks and self.tracks_t is not None and t - self.tracks_t < 0.5:
            dt = t - self.tracks_t
            for tx, ty, tvx, tvy in self.tracks:
                px, py = tx + tvx * dt, ty + tvy * dt       # move track to truth time
                d = math.hypot(px - x, py - y)
                if d <= self.match_radius and (best is None or d < best[0]):
                    best = (d, px, py, tvx, tvy)

        row = [round(t, 2), round(x, 3), round(y, 3), round(vx, 3), round(vy, 3),
               round(dist_robot, 2)]
        if best:
            d, px, py, tvx, tvy = best
            row += [round(px, 3), round(py, 3), round(tvx, 3), round(tvy, 3), round(d, 3),
                    round(math.hypot(tvx - vx, tvy - vy), 3)]
        else:
            row += [''] * 6
        diag = self.diag if self.diag is not None else ('', '', '', '', '')
        row += [diag[0], diag[1], diag[2], diag[3], round(diag[4], 2) if diag[4] != '' else '']
        self.rows.append(row)

    # ------------------------------------------------------------ summary
    def why_lost(self, r):
        c = self.cfg
        raw, a_static, a_robot, cl_n, cl_w = r[12:17]
        if a_static < c['min_points']:
            return 'removed by static-map filter'
        if a_robot < c['min_points']:
            return 'removed by robot-body filter'
        if cl_w > c['max_width']:
            return f'merged into a cluster wider than {c["max_width"]} m'
        if cl_n < c['min_points']:
            return 'cluster too small'
        return 'detected, but no confirmed track (tracking stage)'

    def finish(self):
        os.makedirs(self.results_dir, exist_ok=True)
        path = os.path.join(self.results_dir, f'tracker_eval_{self.tag}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'true_x', 'true_y', 'true_vx', 'true_vy', 'dist_robot',
                        'track_x', 'track_y', 'track_vx', 'track_vy', 'pos_err', 'vel_err',
                        'laser_raw', 'after_static', 'after_robot', 'cluster_n', 'cluster_width'])
            w.writerows(self.rows)

        mp = self.cfg['min_points']
        visible = [r for r in self.rows if r[12] != '' and r[12] >= mp]
        hit = [r for r in visible if r[10] != '']
        lost = [r for r in visible if r[10] == '']
        print(f'\n===== tracker evaluation ({self.tag}) =====')
        print(f'samples where the laser sees the worker (>= {mp} points): {len(visible)}')
        if visible:
            print(f'tracked (coverage): {len(hit)}/{len(visible)} = {100 * len(hit) / len(visible):.0f} %')
        if lost:
            reasons = collections.Counter(self.why_lost(r) for r in lost)
            print('lost samples, by stage:')
            for k, n in reasons.most_common():
                print(f'   {n:4d}  {k}')
        if hit:
            pe = np.array([r[10] for r in hit])
            bx = np.mean([r[6] - r[1] for r in hit])
            by = np.mean([r[7] - r[2] for r in hit])
            print(f'position error  mean {pe.mean():.3f}  median {np.median(pe):.3f}  '
                  f'p95 {np.percentile(pe, 95):.3f}  max {pe.max():.3f}  m')
            print(f'mean offset (track - truth): x {bx:+.3f}  y {by:+.3f}  m')
        moving = [r for r in hit if math.hypot(r[3], r[4]) >= self.min_speed]
        if moving:
            ve = np.array([r[11] for r in moving])
            print(f'velocity error while walking ({len(moving)} samples)  '
                  f'mean {ve.mean():.3f}  median {np.median(ve):.3f}  '
                  f'p95 {np.percentile(ve, 95):.3f}  m/s')
        print(f'saved {path}')


def main():
    rclpy.init()
    node = TrackerEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
