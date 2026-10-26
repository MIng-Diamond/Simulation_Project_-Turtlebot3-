#!/usr/bin/env python3
"""闭环 Leader-Follower（教程 6.7 的进阶题）。

目标：让 follower (TB3_2) 始终停在 leader (TB3_1) 后方 follow_distance 米处。

控制思路：
  1. 由 leader 的 odom 位姿计算目标点：
         target = leader_xy - follow_distance * [cos(yaw), sin(yaw)]
  2. 计算 follower 到目标点的距离 rho 和方位误差 alpha；
  3. 方位误差大时先原地转，误差小时边走边修正（P 控制）。

运行：
    ros2 run tb3_course leader_follower --ros-args \
        -p use_sim_time:=true -p follow_distance:=0.6
另一个终端遥控 leader：
    ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1

坐标系说明同 robot_distance.py：默认两车 odom 都是世界坐标；
如果是编码器里程计，请设置 tb3_1/tb3_2 的 offset 参数。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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


class LeaderFollower(Node):
    def __init__(self):
        super().__init__('leader_follower')

        self.declare_parameter('follow_distance', 0.6)
        self.declare_parameter('min_distance', 0.45)
        self.declare_parameter('max_linear', 0.18)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('kp_linear', 1.2)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('rotate_threshold', 0.5)
        self.declare_parameter('timeout_sec', 1.0)
        self.declare_parameter('tb3_1_offset_x', 0.0)
        self.declare_parameter('tb3_1_offset_y', 0.0)
        self.declare_parameter('tb3_2_offset_x', 0.0)
        self.declare_parameter('tb3_2_offset_y', 0.0)

        self.leader = None
        self.follower = None
        self.last_leader_time = None

        self.sub1 = self.create_subscription(
            Odometry, '/TB3_1/odom', self.leader_cb, 10)
        self.sub2 = self.create_subscription(
            Odometry, '/TB3_2/odom', self.follower_cb, 10)
        self.pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('leader_follower started.')

    def leader_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.leader = (p.x, p.y, yaw_from_quaternion(q))
        self.last_leader_time = self.get_clock().now()

    def follower_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.follower = (p.x, p.y, yaw_from_quaternion(q))

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

        ox1 = float(self.get_parameter('tb3_1_offset_x').value)
        oy1 = float(self.get_parameter('tb3_1_offset_y').value)
        ox2 = float(self.get_parameter('tb3_2_offset_x').value)
        oy2 = float(self.get_parameter('tb3_2_offset_y').value)

        lx = self.leader[0] + ox1
        ly = self.leader[1] + oy1
        lyaw = self.leader[2]
        fx = self.follower[0] + ox2
        fy = self.follower[1] + oy2
        fyaw = self.follower[2]

        # 安全距离：急转弯时 P 控制器容易“切内道”，离 leader 太近就直接停，
        # 避免两车碰撞。正常直行时距离应保持在 follow_distance 附近。
        separation = math.hypot(fx - lx, fy - ly)
        min_distance = float(self.get_parameter('min_distance').value)
        if separation < min_distance:
            self.stop()
            self.get_logger().warn(
                f'Too close to leader ({separation:.2f} m < '
                f'{min_distance:.2f} m), follower stopped.',
                throttle_duration_sec=1.0)
            return

        distance = float(self.get_parameter('follow_distance').value)
        target_x = lx - distance * math.cos(lyaw)
        target_y = ly - distance * math.sin(lyaw)

        dx = target_x - fx
        dy = target_y - fy
        rho = math.hypot(dx, dy)
        alpha = normalize_angle(math.atan2(dy, dx) - fyaw)

        max_v = float(self.get_parameter('max_linear').value)
        max_w = float(self.get_parameter('max_angular').value)
        kp_v = float(self.get_parameter('kp_linear').value)
        kp_w = float(self.get_parameter('kp_angular').value)
        rotate_threshold = float(self.get_parameter('rotate_threshold').value)

        cmd = Twist()
        if abs(alpha) > rotate_threshold:
            # 方位差较大：原地转向目标点
            cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)
        else:
            # 边走边修正；rho 很小时停住，避免抖动
            if rho > 0.03:
                cmd.linear.x = clamp(kp_v * rho, 0.0, max_v) * max(
                    0.0, math.cos(alpha))
            cmd.angular.z = clamp(kp_w * alpha, -max_w, max_w)

        self.pub.publish(cmd)

        self.get_logger().info(
            f'leader=({lx:.2f},{ly:.2f}) follower=({fx:.2f},{fy:.2f}) '
            f'target=({target_x:.2f},{target_y:.2f}) rho={rho:.2f} '
            f'alpha={alpha:.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollower()
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
