#!/usr/bin/env python3
"""给 Cartographer 建图用的自动巡航节点（作业 4.7 的辅助工具）。

它按给定的 waypoint 路线行驶，用 /odom 做位置闭环，用 /scan 做前方
障碍物保护；走完后原地转一圈，帮助 Cartographer 闭合回环。

默认路线是为 turtlebot3_world 的 3x3 圆柱阵列设计的：
  外圈一圈 + 中间 0.55 m 走廊 + 中心小环 + 终点旋转。

运行（三个终端）：
  1) ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  2) ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
  3) ros2 run tb3_course auto_mapper --ros-args -p use_sim_time:=true

如果地图还有缺口，可以修改 waypoints 参数，或者用 teleop 手动补几分钟。
waypoints 格式："x1,y1;x2,y2;..."
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    return max(low, min(high, value))


# turtlebot3_world 的墙大约在 x/y = ±2.2，圆柱在 ±1.1。
# 外圈走 ±1.65，离墙和圆柱都约 0.55 m；内圈走 ±0.55 的走廊。
DEFAULT_WAYPOINTS = (
    '-1.65,-0.5;-1.65,-1.65;1.65,-1.65;1.65,1.65;-1.65,1.65;'
    '-1.65,0.55;-0.55,0.55;0.55,0.55;0.55,0.0;0.55,-0.55;'
    '0.0,-0.55;-0.55,-0.55;-0.55,0.0;-0.55,0.55;1.65,0.55;'
    '1.65,-0.55;-1.65,-0.55;-1.65,0.55;-1.65,-0.5'
)


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('linear_speed', 0.13)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('waypoint_tolerance', 0.14)
        self.declare_parameter('obstacle_distance', 0.30)
        # 雷达装在车体上方，侧面 50° 以外会扫到自己的结构（约 0.12-0.19 m）。
        # 前方用 20° 窄扇区，近距紧急用 45° 扇区，刚好避开这些自车回波。
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('emergency_half_angle_deg', 45.0)
        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('spin_at_end', True)
        self.declare_parameter('max_duration', 900.0)

        raw = str(self.get_parameter('waypoints').value)
        self.waypoints = []
        for item in raw.split(';'):
            item = item.strip()
            if not item:
                continue
            x_str, y_str = item.split(',')
            self.waypoints.append((float(x_str), float(y_str)))

        self.pose = None
        self.scan = None
        self.index = 0
        self.state = 'WAIT_ODOM'
        self.state_start = self.get_clock().now()
        self.avoid_dir = 1.0
        self.finished = False
        self.started_at = self.get_clock().now()

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f'AutoMapper loaded {len(self.waypoints)} waypoints.')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q))

    def scan_cb(self, msg):
        self.scan = msg

    def sector_min(self, center_rad, half_rad):
        """返回以 center_rad 为中心、half_rad 为半宽扇区的最近距离。

        注意：LaserScan 的 angle_min 可能是 0，也可能是 -pi；
        所以不能直接用 (center - angle_min) / inc 取下标，
        必须把每一束的角度归一化到 [-pi, pi] 再比较。
        """
        if self.scan is None or not self.scan.ranges:
            return float('inf')
        inc = self.scan.angle_increment
        if inc <= 0.0:
            return float('inf')

        best = float('inf')
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r <= self.scan.range_min:
                continue
            angle = self.scan.angle_min + i * inc
            if abs(normalize_angle(angle - center_rad)) <= half_rad:
                best = min(best, r)
        return best

    def front_min(self):
        half = math.radians(
            float(self.get_parameter('front_half_angle_deg').value))
        return self.sector_min(0.0, half)

    def emergency_min(self):
        half = math.radians(
            float(self.get_parameter('emergency_half_angle_deg').value))
        return self.sector_min(0.0, half)

    def left_min(self):
        return self.sector_min(math.radians(90.0), math.radians(30.0))

    def right_min(self):
        return self.sector_min(math.radians(-90.0), math.radians(30.0))

    def stop(self):
        self.pub.publish(Twist())

    def elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def switch(self, state):
        self.state = state
        self.state_start = self.get_clock().now()

    def finish(self, message):
        if self.finished:
            return
        self.stop()
        self.finished = True
        self.timer.cancel()
        self.get_logger().info(message)
        rclpy.shutdown()

    def loop(self):
        if self.finished:
            return

        total = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        max_duration = float(self.get_parameter('max_duration').value)
        if total > max_duration:
            self.finish('Max duration reached, mapping tour stopped.')
            return

        if self.pose is None:
            return

        x, y, yaw = self.pose
        speed = float(self.get_parameter('linear_speed').value)
        angular = float(self.get_parameter('angular_speed').value)
        tol = float(self.get_parameter('waypoint_tolerance').value)
        obstacle = float(self.get_parameter('obstacle_distance').value)
        emergency = float(self.get_parameter('emergency_distance').value)

        if self.state == 'WAIT_ODOM':
            self.switch('GO')
            self.get_logger().info(
                f'Start pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
            return

        if self.state == 'GO':
            if self.index >= len(self.waypoints):
                if bool(self.get_parameter('spin_at_end').value):
                    self.stop()
                    self.switch('SPIN_END')
                    self.get_logger().info('Route done, spinning 360 deg ...')
                else:
                    self.finish('Mapping tour finished.')
                return

            tx, ty = self.waypoints[self.index]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                self.stop()
                self.index += 1
                if self.index < len(self.waypoints):
                    self.get_logger().info(
                        f'Waypoint {self.index}/{len(self.waypoints)} done, '
                        f'next=({self.waypoints[self.index][0]:.2f}, '
                        f'{self.waypoints[self.index][1]:.2f})')
                return

            heading = math.atan2(dy, dx)
            error = normalize_angle(heading - yaw)
            front = self.front_min()
            emergency_front = self.emergency_min()

            cmd = Twist()
            if front < obstacle or emergency_front < emergency:
                self.stop()
                self.avoid_dir = 1.0 if self.left_min() > self.right_min() \
                    else -1.0
                self.switch('AVOID')
                self.get_logger().info(
                    f'Obstacle: front={front:.2f} m, '
                    f'emergency={emergency_front:.2f} m; avoiding to '
                    f'{"left" if self.avoid_dir > 0 else "right"}.')
                return

            if abs(error) > 0.5:
                cmd.angular.z = clamp(1.5 * error, -angular, angular)
            else:
                cmd.linear.x = speed
                cmd.angular.z = clamp(1.5 * error, -angular, angular)
            self.pub.publish(cmd)
            return

        if self.state == 'AVOID':
            cmd = Twist()
            cmd.angular.z = self.avoid_dir * angular
            self.pub.publish(cmd)
            front = self.front_min()
            emergency_front = self.emergency_min()
            if front > obstacle + 0.12 and emergency_front > emergency + 0.005:
                self.stop()
                self.switch('GO')
                self.get_logger().info(
                    f'Path clear (front={front:.2f}, '
                    f'emergency={emergency_front:.2f}), resume route.')
            elif self.elapsed() > 6.0:
                self.stop()
                self.switch('GO')
                self.get_logger().warn('Avoid timeout, resume route anyway.')
            return

        if self.state == 'SPIN_END':
            cmd = Twist()
            cmd.angular.z = angular
            self.pub.publish(cmd)
            spin_time = 2.0 * math.pi / max(0.05, angular)
            if self.elapsed() >= spin_time:
                self.finish('Mapping tour finished.')


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
