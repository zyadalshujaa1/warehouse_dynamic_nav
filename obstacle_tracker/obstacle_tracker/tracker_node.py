"""Track moving obstacles (people) from a 2D laser scan.

Subscribes : /scan (LaserScan), /map (OccupancyGrid, transient local), TF
Publishes  : /tracked_obstacles          nav_msgs/Odometry, one message per
                                         confirmed track per scan. Frame map;
                                         child_frame_id is also map, so the
                                         twist is the velocity in the map frame.
                                         Covariances come from the Kalman filter.
             /tracked_obstacles/markers  visualization_msgs/MarkerArray (RViz)

See core.py for the algorithm.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformException, TransformListener

from obstacle_tracker.core import (MultiTracker, StaticMap, cluster_centre,
                                   cluster_points, in_robot_box, points_from_scan)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrackerNode(Node):

    def __init__(self):
        super().__init__('obstacle_tracker',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('robot_frame', 'base_footprint')
        p('max_range', 8.0)          # m, ignore points further away
        p('map_margin', 0.20)        # m, points this close to a map wall are static
        p('robot_half_length', 0.41)
        p('robot_half_width', 0.31)
        p('robot_margin', 0.10)      # m, points on the robot / carried shelf
        p('cluster_gap', 0.20)       # m
        p('min_points', 3)
        p('max_width', 0.80)         # m, wider clusters are not people
        p('centre_offset', 0.20)     # m, see core.cluster_centre
        p('gate', 1.0)               # m, association gate
        p('accel_std', 1.0)          # m/s^2, process noise
        p('meas_std', 0.08)          # m, measurement noise
        p('init_pos_std', 0.3)
        p('init_vel_std', 1.0)
        p('confirm_hits', 3)         # publish a track after this many updates
        p('max_missed_time', 1.0)    # s, delete a track not seen for this long

        g = lambda n: self.get_parameter(n).value
        self.cfg = {n: g(n) for n in (
            'map_frame', 'robot_frame', 'max_range', 'map_margin',
            'robot_half_length', 'robot_half_width', 'robot_margin',
            'cluster_gap', 'min_points', 'max_width', 'centre_offset', 'confirm_hits')}
        self.tracker = MultiTracker(g('gate'), g('accel_std'), g('meas_std'),
                                    g('init_pos_std'), g('init_vel_std'),
                                    g('max_missed_time'))
        self.static_map = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, map_qos)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.pub = self.create_publisher(Odometry, '/tracked_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tracked_obstacles/markers', 10)
        self.last_warn = 0.0
        self.get_logger().info('obstacle_tracker started, waiting for /map and scans')

    # ----------------------------------------------------------- inputs
    def on_map(self, msg):
        o = msg.info.origin
        if abs(yaw_of(o.orientation)) > 1e-3:
            self.get_logger().warn('Map origin is rotated; static filter assumes it is not')
        self.static_map = StaticMap(msg.data, msg.info.width, msg.info.height,
                                    msg.info.resolution, o.position.x, o.position.y,
                                    self.cfg['map_margin'])
        self.get_logger().info(f'Static map received: {msg.info.width} x {msg.info.height} '
                               f'cells at {msg.info.resolution} m')

    def lookup(self, frame, stamp):
        """Pose (x, y, yaw) of `frame` in the map frame at `stamp` (fallback: latest)."""
        for t in (stamp, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(self.cfg['map_frame'], frame, t)
                tr = tf.transform
                return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            except TransformException:
                continue
        return None

    def on_scan(self, scan):
        c = self.cfg
        stamp = Time.from_msg(scan.header.stamp)
        t = stamp.nanoseconds * 1e-9
        sensor = self.lookup(scan.header.frame_id, stamp)
        robot = self.lookup(c['robot_frame'], stamp)
        if sensor is None or robot is None or self.static_map is None:
            if t - self.last_warn > 5.0:
                self.last_warn = t
                self.get_logger().warn(f'Waiting: TF sensor={sensor is not None} '
                                       f'robot={robot is not None} map={self.static_map is not None}')
            return

        pts, _ = points_from_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                  scan.range_min, scan.range_max, c['max_range'], *sensor)
        if len(pts):
            keep = ~self.static_map.is_static(pts)
            keep &= ~in_robot_box(pts, *robot,
                                  c['robot_half_length'] + c['robot_margin'],
                                  c['robot_half_width'] + c['robot_margin'])
            pts = pts[keep]
        full_circle = abs(scan.angle_increment * len(scan.ranges)) > 6.2
        clusters = cluster_points(pts, c['cluster_gap'], c['min_points'],
                                  c['max_width'], wrap=full_circle)
        dets = [cluster_centre(p, sensor[:2], c['centre_offset']) for p in clusters]
        tracks = self.tracker.step(t, dets)
        confirmed = [tr for tr in tracks
                     if tr.hits >= c['confirm_hits'] and tr.last_seen == t]
        self.publish(scan.header.stamp, confirmed)

    # ---------------------------------------------------------- outputs
    def publish(self, stamp, tracks):
        frame = self.cfg['map_frame']
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for tr in tracks:
            x, y, vx, vy = tr.x
            o = Odometry()
            o.header.stamp, o.header.frame_id = stamp, frame
            o.child_frame_id = frame
            o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
            yaw = math.atan2(vy, vx)
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            o.twist.twist.linear.x, o.twist.twist.linear.y = float(vx), float(vy)
            P = tr.P
            pc, tc = [0.0] * 36, [0.0] * 36
            pc[0], pc[1], pc[6], pc[7] = P[0, 0], P[0, 1], P[1, 0], P[1, 1]
            tc[0], tc[1], tc[6], tc[7] = P[2, 2], P[2, 3], P[3, 2], P[3, 3]
            o.pose.covariance = [float(v) for v in pc]
            o.twist.covariance = [float(v) for v in tc]
            self.pub.publish(o)
            markers.markers += self.track_markers(stamp, frame, tr)
        self.marker_pub.publish(markers)

    @staticmethod
    def track_markers(stamp, frame, tr):
        x, y, vx, vy = (float(v) for v in tr.x)
        body = Marker()
        body.header.stamp, body.header.frame_id = stamp, frame
        body.ns, body.id, body.type = 'body', tr.id, Marker.CYLINDER
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.85
        body.pose.orientation.w = 1.0
        body.scale.x, body.scale.y, body.scale.z = 0.5, 0.5, 1.7
        body.color.r, body.color.g, body.color.b, body.color.a = 0.1, 0.6, 1.0, 0.6
        arrow = Marker()
        arrow.header = body.header
        arrow.ns, arrow.id, arrow.type = 'velocity_1s', tr.id, Marker.ARROW
        arrow.points = [Point(x=x, y=y, z=0.1), Point(x=x + vx, y=y + vy, z=0.1)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.12, 0.12
        arrow.color.r, arrow.color.g, arrow.color.a = 1.0, 0.1, 1.0
        text = Marker()
        text.header = body.header
        text.ns, text.id, text.type = 'label', tr.id, Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 2.0
        text.scale.z = 0.3
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'#{tr.id} {math.hypot(vx, vy):.2f} m/s'
        return [body, arrow, text]


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
