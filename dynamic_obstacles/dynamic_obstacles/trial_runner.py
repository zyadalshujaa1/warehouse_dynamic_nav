"""Run one logistics mission and record how the robot dealt with the workers.

What it does
------------
1. Waits for the mission action server (/cycle_perform) and for localization.
2. Sends the mission goal and waits for the result (or a timeout).
3. At 20 Hz it records the robot pose (TF map -> base_footprint) and the
   position of every worker (/workers/<name>/pose, published by worker_mover), and
   computes the clearance between the robot footprint and each worker:

       clearance = distance(worker centre, robot rectangle) - worker radius

   clearance <= contact_tolerance counts as a collision.
4. Appends one row to <results_dir>/summary.csv and writes the full time
   series to <results_dir>/traj_<scenario>_<method>_<run>.csv.

Assumption: the map frame coincides with the Gazebo world frame (the map was
built from this world and AMCL starts at the spawn pose). The node logs the
robot's start pose so this can be checked.
"""
import csv
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

from logistic_msg.action import MoveRobot


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def distance_to_rectangle(px, py, half_length, half_width):
    """Distance from a point (robot frame) to the robot rectangle; 0 inside."""
    dx = max(abs(px) - half_length, 0.0)
    dy = max(abs(py) - half_width, 0.0)
    return math.hypot(dx, dy)


class TrialRunner(Node):

    def __init__(self):
        super().__init__('trial_runner',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('scenario', 'unknown')
        p('method', 'dwb')
        p('run', 1)
        p('workers', ['worker_1'])
        p('worker_radius', 0.25)
        p('half_length', 0.41)        # robot footprint, from the Nav2 config
        p('half_width', 0.31)
        p('contact_tolerance', 0.02)  # m
        p('stop_on_collision', True)  # end the trial at the first contact
        p('timeout', 600.0)           # s of simulation time
        p('settle', 5.0)              # s to wait after localization is up
        p('results_dir', os.path.expanduser('~/logistic_ws/results'))
        p('map_frame', 'map')
        p('robot_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value
        self.scenario, self.method, self.run = g('scenario'), g('method'), g('run')
        self.workers = list(g('workers'))
        self.r_worker, self.hl, self.hw = g('worker_radius'), g('half_length'), g('half_width')
        self.contact_tol, self.timeout, self.settle = g('contact_tolerance'), g('timeout'), g('settle')
        self.stop_on_collision = g('stop_on_collision')
        self.results_dir = os.path.expanduser(g('results_dir'))
        self.map_frame, self.robot_frame = g('map_frame'), g('robot_frame')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.worker_pose = {}      # name -> (x, y)
        self.worker_start = {}     # name -> first (x, y), to check it moved
        for w in self.workers:
            self.create_subscription(PoseStamped, f'/workers/{w}/pose',
                                     lambda msg, w=w: self.on_worker(w, msg), 10)

        self.client = ActionClient(self, MoveRobot, 'cycle_perform')
        self.phase = 'wait_server'
        self.t_phase = None
        self.t_goal = None
        self.goal_handle = None
        self.outcome = None        # 'succeeded', 'aborted', 'canceled', 'timeout', 'rejected'
        self.min_clearance = math.inf
        self.t_min_clearance = None
        self.collision_events = 0
        self.in_contact = False
        self.rows = []
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Trial: scenario={self.scenario} method={self.method} run={self.run}')

    # ---------------------------------------------------------------- inputs
    def on_worker(self, name, msg):
        xy = (msg.pose.position.x, msg.pose.position.y)
        self.worker_pose[name] = xy
        self.worker_start.setdefault(name, xy)

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = self.now()
        if now == 0.0 or self.done:
            return
        pose = self.robot_pose()

        if self.phase == 'wait_server':
            if self.client.server_is_ready() and pose is not None:
                self.get_logger().info(
                    f'Robot start pose in map: x={pose[0]:.2f} y={pose[1]:.2f} '
                    f'yaw={pose[2]:.2f} (expected about -3.0, 0.0, 0.0)')
                self.phase, self.t_phase = 'settle', now
            return

        if self.phase == 'settle':
            if now - self.t_phase >= self.settle:
                self.send_goal(now)
            return

        if self.phase == 'running':
            if pose is not None:
                self.record(now, pose)
            if self.outcome is None and now - self.t_goal > self.timeout:
                self.get_logger().warn('Timeout: cancelling the mission')
                self.outcome = 'timeout'
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
            if self.outcome is not None:
                self.finish(now)

    def send_goal(self, now):
        self.phase, self.t_goal = 'running', now
        goal = MoveRobot.Goal()
        goal.action = 'Start'
        self.get_logger().info('Sending mission goal')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.outcome = 'rejected'
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        if self.outcome is not None:
            return
        status = future.result().status
        self.outcome = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                        GoalStatus.STATUS_ABORTED: 'aborted',
                        GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, f'status_{status}')

    # -------------------------------------------------------------- metrics
    def record(self, now, pose):
        rx, ry, ryaw = pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        row = [round(now - self.t_goal, 3), round(rx, 3), round(ry, 3), round(ryaw, 3)]
        contact_now = False
        for w in self.workers:
            if w not in self.worker_pose:
                row += ['', '', '']
                continue
            wx, wy = self.worker_pose[w]
            # worker centre in the robot frame
            dx, dy = wx - rx, wy - ry
            px, py = c * dx + s * dy, -s * dx + c * dy
            clearance = distance_to_rectangle(px, py, self.hl, self.hw) - self.r_worker
            row += [round(wx, 3), round(wy, 3), round(clearance, 3)]
            if clearance < self.min_clearance:
                self.min_clearance, self.t_min_clearance = clearance, now - self.t_goal
            contact_now = contact_now or clearance <= self.contact_tol
        if contact_now and not self.in_contact:
            self.collision_events += 1
            self.get_logger().warn('Contact with a worker')
            if self.stop_on_collision and self.outcome is None:
                # A trial counts as failed at the first contact; what happens
                # afterwards (the worker pushing the robot) is not measured.
                self.outcome = 'collision'
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
        self.in_contact = contact_now
        self.rows.append(row)

    def finish(self, now):
        self.done = True
        duration = now - self.t_goal
        moved = all(math.dist(self.worker_start.get(w, (0, 0)), self.worker_pose.get(w, (0, 0))) > 0.5
                    for w in self.workers)
        os.makedirs(self.results_dir, exist_ok=True)

        tag = f'{self.scenario}_{self.method}_{self.run}'
        with open(os.path.join(self.results_dir, f'traj_{tag}.csv'), 'w', newline='') as f:
            wr = csv.writer(f)
            header = ['t', 'robot_x', 'robot_y', 'robot_yaw']
            for w in self.workers:
                header += [f'{w}_x', f'{w}_y', f'{w}_clearance']
            wr.writerow(header)
            wr.writerows(self.rows)

        summary = os.path.join(self.results_dir, 'summary.csv')
        new_file = not os.path.exists(summary)
        min_c = round(self.min_clearance, 3) if math.isfinite(self.min_clearance) else ''
        with open(summary, 'a', newline='') as f:
            wr = csv.writer(f)
            if new_file:
                wr.writerow(['scenario', 'method', 'run', 'outcome', 'mission_time_s',
                             'min_clearance_m', 'time_of_min_s', 'collisions', 'worker_moved'])
            wr.writerow([self.scenario, self.method, self.run, self.outcome, round(duration, 1),
                         min_c, round(self.t_min_clearance or 0.0, 1),
                         self.collision_events, moved])

        self.get_logger().info(
            f'RESULT {tag}: outcome={self.outcome} time={duration:.1f}s '
            f'min_clearance={min_c} m collisions={self.collision_events} worker_moved={moved}')
        self.get_logger().info(f'Saved to {self.results_dir}')


def main():
    rclpy.init()
    node = TrialRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
