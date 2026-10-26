#!/usr/bin/env python3
"""基于 /odom 闭环走正方形（教程 3.9 的拓展题）。

与 move_square.py 的开环计时不同，这里每一步都用里程计反馈：
  1. 记录本边起点，向前走到“离起点 side_length 米”；
  2. 用当前 yaw 计算目标 yaw = 当前 yaw + 90°，原地转到目标；
  3. 重复四次，最后输出回到起点的闭合误差。

运行（先启动 Gazebo）：
    ros2 run tb3_course square_odom --ros-args -p use_sim_time:=true
    ros2 run tb3_course square_odom --ros-args \
        -p use_sim_time:=true -p side_length:=0.8 -p linear_speed:=0.12

注意：/odom 是里程计坐标，不是 map 坐标；本节点只使用相对位移，
所以单机器人和多机器人场景都能用。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """四元数 -> 偏航角（只关心平面运动）。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    """把角度归一化到 [-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SquareOdom(Node):
    def __init__(self):
        super().__init__('square_odom')
        self.declare_parameter('side_length', 0.5)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('turn_angle', math.pi / 2.0)
        self.declare_parameter('tolerance', 0.005)
        self.declare_parameter('kp_heading', 1.2)
        self.declare_parameter('max_heading_correction', 0.3)
        self.declare_parameter('max_duration', 180.0)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.pose = None
        self.state = 'WAIT_ODOM'
        self.side_index = 0
        self.initial_xy = None
        self.segment_start_xy = None
        self.side_yaw = 0.0
        self.target_yaw = 0.0
        self.finished = False
        self.started_at = self.get_clock().now()

        self.get_logger().info('square_odom started, waiting for /odom ...')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q))

    def publish_stop(self):
        self.pub.publish(Twist())

    def finish(self, message):
        self.publish_stop()
        self.finished = True
        self.get_logger().info(message)

    def loop(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        max_duration = float(self.get_parameter('max_duration').value)
        if elapsed > max_duration:
            self.finish('Timeout: square not finished, stopped.')
            return

        if self.pose is None:
            return

        x, y, yaw = self.pose
        side = float(self.get_parameter('side_length').value)
        speed = float(self.get_parameter('linear_speed').value)
        angular = float(self.get_parameter('angular_speed').value)
        turn = float(self.get_parameter('turn_angle').value)
        tol = float(self.get_parameter('tolerance').value)
        kp_heading = float(self.get_parameter('kp_heading').value)
        max_correction = float(
            self.get_parameter('max_heading_correction').value)

        if self.state == 'WAIT_ODOM':
            self.initial_xy = (x, y)
            self.segment_start_xy = (x, y)
            self.side_yaw = yaw
            self.state = 'FORWARD'
            self.get_logger().info(
                f'Start pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
            return

        if self.state == 'FORWARD':
            dist = math.hypot(x - self.segment_start_xy[0],
                              y - self.segment_start_xy[1])
            cmd = Twist()
            if dist < side - tol:
                cmd.linear.x = speed
                # 直行时用航向反馈抵消左右轮差带来的漂移
                heading_error = normalize_angle(self.side_yaw - yaw)
                cmd.angular.z = max(
                    -max_correction,
                    min(max_correction, kp_heading * heading_error))
                self.pub.publish(cmd)
                return

            self.publish_stop()
            self.side_index += 1
            if self.side_index >= 4:
                ex = x - self.initial_xy[0]
                ey = y - self.initial_xy[1]
                self.finish(
                    f'Finished 4 sides. Final pose=({x:.3f}, {y:.3f}), '
                    f'closure error={math.hypot(ex, ey):.3f} m')
                return

            # 以本边名义航向为基准转 90°，避免误差逐边累积
            self.target_yaw = normalize_angle(self.side_yaw + turn)
            self.state = 'TURN'
            self.get_logger().info(
                f'Side {self.side_index}/4 done at ({x:.3f}, {y:.3f}), '
                f'turning to yaw={self.target_yaw:.3f}')
            return

        if self.state == 'TURN':
            error = normalize_angle(self.target_yaw - yaw)
            cmd = Twist()
            if abs(error) > tol:
                # 接近目标时减速，避免冲过 90°
                cmd.angular.z = max(-angular, min(angular, 1.5 * error))
                self.pub.publish(cmd)
                return

            self.publish_stop()
            self.segment_start_xy = (x, y)
            self.side_yaw = self.target_yaw
            self.state = 'FORWARD'
            self.get_logger().info(
                f'Turn done, start side {self.side_index + 1}/4, '
                f'target yaw={self.side_yaw:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = SquareOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
