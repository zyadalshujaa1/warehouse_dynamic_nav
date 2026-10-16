"""Move one simulated worker along a straight line.

Behaviour
---------
1. WAITING  : the worker stands still until the robot comes within
              `trigger_radius` of (trigger_x, trigger_y) in the map frame.
              If trigger_radius <= 0 the worker starts immediately.
2. WALKING  : walks `distance` metres along (dir_x, dir_y) at `speed` m/s.
3. If `loop` is true it pauses, turns back and repeats forever.
   If `loop` is false it stops for good after one leg (DONE).

The trigger makes every run of a scenario start the same way relative to
the robot, so experiments are repeatable.

Timing uses the simulation clock (use_sim_time), so the walked distance is
correct even when Gazebo runs slower than real time.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException

WAITING, WALKING, PAUSED, DONE = 'WAITING', 'WALKING', 'PAUSED', 'DONE'


class WorkerMover(Node):

    def __init__(self):
        super().__init__('worker_mover')

        p = self.declare_parameter
        p('worker_name', 'worker_1')
        p('start_x', 0.0)           # spawn position (map = world frame)
        p('start_y', 0.0)
        p('dir_x', 1.0)
        p('dir_y', 0.0)
        p('speed', 0.6)            # m/s, typical walking pace
        p('distance', 6.0)         # m, length of one leg
        p('pause', 1.0)            # s, wait at each end (loop mode)
        p('loop', True)            # back and forth forever, or one leg only
        p('trigger_x', 0.0)        # map frame
        p('trigger_y', 0.0)
        p('trigger_radius', 0.0)   # <= 0 : start immediately
        p('map_frame', 'map')
        p('robot_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value
        self.name = g('worker_name')
        norm = math.hypot(g('dir_x'), g('dir_y')) or 1.0
        self.dir = (g('dir_x') / norm, g('dir_y') / norm)
        self.speed = g('speed')
        self.walk_time = g('distance') / self.speed
        self.pause = g('pause')
        self.loop = g('loop')
        self.trigger = (g('trigger_x'), g('trigger_y'))
        self.trigger_radius = g('trigger_radius')
        self.map_frame = g('map_frame')
        self.robot_frame = g('robot_frame')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, f'/model/{self.name}/cmd_vel', 10)
        # The worker floats without gravity or friction and VelocityControl
        # enforces the commanded velocity, so integrating the commands gives
        # its true position (also when it pushes into the robot).
        self.pos = [g('start_x'), g('start_y')]
        self.vel = (0.0, 0.0)
        self.t_last = None
        self.pose_pub = self.create_publisher(PoseStamped, f'/workers/{self.name}/pose', 10)

        self.state = WAITING if self.trigger_radius > 0 else WALKING
        self.sign = 1.0           # +1 forward leg, -1 return leg
        self.phase_start = None   # set once sim time is running
        self.last_dist = None
        self.last_report = 0.0
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        start = ('when robot is within %.2f m of (%.2f, %.2f)'
                 % (self.trigger_radius, *self.trigger)
                 if self.state == WAITING else 'immediately')
        self.get_logger().info(
            f'{self.name}: {g("distance")} m at {self.speed} m/s, '
            f'dir ({self.dir[0]:.2f}, {self.dir[1]:.2f}), '
            f'{"loop" if self.loop else "one pass"}, starts {start}')

    def robot_near_trigger(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time())
        except TransformException:
            self.last_dist = None  # localization not up yet
            return False
        dx = t.transform.translation.x - self.trigger[0]
        dy = t.transform.translation.y - self.trigger[1]
        self.last_dist = math.hypot(dx, dy)
        return self.last_dist <= self.trigger_radius

    def report_waiting(self, now):
        """Every 5 s of sim time, say how far the robot is from the trigger."""
        if now - self.last_report < 5.0:
            return
        self.last_report = now
        if self.last_dist is None:
            self.get_logger().warn(
                f'{self.name}: waiting, no TF {self.map_frame} -> {self.robot_frame} yet')
        else:
            self.get_logger().info(
                f'{self.name}: waiting, robot is {self.last_dist:.2f} m from the trigger '
                f'(starts at {self.trigger_radius:.2f} m)')

    def set_state(self, state, now):
        self.state, self.phase_start = state, now
        self.get_logger().info(f'{self.name}: {state}')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return  # /clock not received yet
        if self.phase_start is None:
            self.phase_start = now

        elapsed = now - self.phase_start
        if self.state == WAITING:
            if self.robot_near_trigger():
                self.set_state(WALKING, now)
            else:
                self.report_waiting(now)
        elif self.state == WALKING and elapsed >= self.walk_time:
            self.set_state(PAUSED if self.loop else DONE, now)
        elif self.state == PAUSED and elapsed >= self.pause:
            self.sign = -self.sign
            self.set_state(WALKING, now)

        # advance the position with the velocity commanded on the last tick
        if self.t_last is not None:
            dt = now - self.t_last
            self.pos[0] += self.vel[0] * dt
            self.pos[1] += self.vel[1] * dt
        self.t_last = now

        cmd = Twist()  # zero velocity unless walking
        if self.state == WALKING:
            cmd.linear.x = self.sign * self.speed * self.dir[0]
            cmd.linear.y = self.sign * self.speed * self.dir[1]
        self.vel = (cmd.linear.x, cmd.linear.y)
        self.pub.publish(cmd)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.pos
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = WorkerMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
