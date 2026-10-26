#!/usr/bin/env python3
"""Breadcrumb Leader-Follower（沿 leader 的安全轨迹跟随）。

与 leader_follower.py 的区别：
  - leader_follower.py 追“leader 当前位姿后方 0.6 m”的点，转弯时会切内角；
  - 本节点记录 leader 走过的 odom 轨迹，目标点取“沿轨迹落后 0.6 m”的点，
    因此 follower 会沿 leader 已经验证过的安全路径走，适合穿过圆柱阵列。

leader 建议由 auto_mapper（带 /scan 避障）或 Nav2 驱动；follower 订阅：
  /TB3_1/odom  /TB3_2/odom  /TB3_2/scan
发布：
  /TB3_2/cmd_vel

运行示例（leader 由 auto_mapper 驱动）：
  ros2 run tb3_course trail_follower --ros-args -p use_sim_time:=true \
      -p follow_distance:=0.6
"""

import math

import rclpy
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


class TrailFollower(Node):
    def __init__(self):
        super().__init__('trail_follower')
        self.declare_parameter('follow_distance', 0.6)
        self.declare_parameter('trail_resolution', 0.05)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('rotate_threshold', 0.5)
        self.declare_parameter('min_distance', 0.45)
        self.declare_parameter('obstacle_distance', 0.25)
        self.declare_parameter('timeout_sec', 1.0)

        self.leader = None
        self.follower = None
        self.scan = None
        self.last_leader_time = None
        self.trail = []

        self.sub1 = self.create_subscription(
            Odometry, '/TB3_1/odom', self.leader_cb, 10)
        self.sub2 = self.create_subscription(
            Odometry, '/TB3_2/odom', self.follower_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/TB3_2/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('trail_follower started.')

    def leader_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.leader = (p.x, p.y, yaw_from_quaternion(q))
        self.last_leader_time = self.get_clock().now()

        resolution = float(self.get_parameter('trail_resolution').value)
        if not self.trail:
            self.trail.append((p.x, p.y))
        else:
            lx, ly = self.trail[-1]
            if math.hypot(p.x - lx, p.y - ly) >= resolution:
                self.trail.append((p.x, p.y))
                # 只保留最近 2000 个点（约 100 m），避免无限增长
                if len(self.trail) > 2000:
                    self.trail = self.trail[-1000:]

    def follower_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.follower = (p.x, p.y, yaw_from_quaternion(q))

    def scan_cb(self, msg):
        self.scan = msg

    def sector_min(self, center_rad, half_rad):
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
        return self.sector_min(0.0, math.radians(25.0))

    def target_on_trail(self, distance):
        if not self.trail:
            return None
        remaining = distance
        for i in range(len(self.trail) - 1, 0, -1):
            p1 = self.trail[i]
            p0 = self.trail[i - 1]
            seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if seg > 1e-9 and seg >= remaining:
                ratio = remaining / seg
                return (p1[0] + (p0[0] - p1[0]) * ratio,
                        p1[1] + (p0[1] - p1[1]) * ratio)
            remaining -= seg
        return self.trail[0]

    def stop(self):
        self.pub.publish(Twist())

    def loop(self):
        if self.leader is None or self.follower is None:
            return

        timeout = float(self.get_parameter('timeout_sec').value)
        age = (self.get_clock().now() - self.last_leader_time).nanoseconds / 1e9
        if age > timeout:
            self.stop()
            self.get_logger().warn(
                'Leader odom timeout, follower stopped.',
                throttle_duration_sec=2.0)
            return

        lx, ly, _ = self.leader
        fx, fy, fyaw = self.follower
        min_distance = float(self.get_parameter('min_distance').value)
        separation = math.hypot(fx - lx, fy - ly)
        if separation < min_distance:
            self.stop()
            self.get_logger().warn(
                f'Too close to leader ({separation:.2f} m), stopped.',
                throttle_duration_sec=1.0)
            return

        follow_distance = float(self.get_parameter('follow_distance').value)
        target = self.target_on_trail(follow_distance)
        if target is None:
            return
        tx, ty = target
        dx, dy = tx - fx, ty - fy
        rho = math.hypot(dx, dy)
        alpha = normalize_angle(math.atan2(dy, dx) - fyaw)

        max_v = float(self.get_parameter('max_linear').value)
        max_w = float(self.get_parameter('max_angular').value)
        kp_v = float(self.get_parameter('kp_linear').value)
        kp_w = float(self.get_parameter('kp_angular').value)
        rotate_threshold = float(self.get_parameter('rotate_threshold').value)

        cmd = Twist()
        if abs(alpha) > rotate_threshold:
            cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)
        else:
            if rho > 0.03:
                cmd.linear.x = clamp(kp_v * rho, 0.0, max_v) * max(
                    0.0, math.cos(alpha))
            cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)

        # 安全层：follower 前方太近就停车（沿 leader 轨迹一般不会触发）
        obstacle = float(self.get_parameter('obstacle_distance').value)
        if cmd.linear.x > 0.0 and self.front_min() < obstacle:
            cmd.linear.x = 0.0

        self.pub.publish(cmd)
        self.get_logger().info(
            f'target=({tx:.2f},{ty:.2f}) rho={rho:.2f} alpha={alpha:.2f} '
            f'follower=({fx:.2f},{fy:.2f})',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TrailFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
