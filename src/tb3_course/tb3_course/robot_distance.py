#!/usr/bin/env python3
"""计算两台 TurtleBot3 的欧氏距离。对应教程 6.9。

订阅：
  /TB3_1/odom
  /TB3_2/odom
输出：
  终端日志 + /tb3_distance (std_msgs/msg/Float32)

关于坐标系：
  Gazebo 的 libgazebo_ros_diff_drive 默认 odometry_source=world，
  此时两车的 odom 都是世界坐标，直接相减即可。
  如果你的版本输出的是从各自起点开始的编码器里程计，请把
  tb3_1_offset_x/y、tb3_2_offset_x/y 设为两车的初始 spawn 位置
  （本课程 multi_robot_2.launch.py 默认是 -2.0,-0.5 和 0.5,-2.0）。

运行：
    ros2 run tb3_course robot_distance --ros-args -p use_sim_time:=true
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class RobotDistance(Node):
    def __init__(self):
        super().__init__('robot_distance')

        self.declare_parameter('tb3_1_offset_x', 0.0)
        self.declare_parameter('tb3_1_offset_y', 0.0)
        self.declare_parameter('tb3_2_offset_x', 0.0)
        self.declare_parameter('tb3_2_offset_y', 0.0)

        self.pose1 = None
        self.pose2 = None

        self.sub1 = self.create_subscription(
            Odometry, '/TB3_1/odom', self.odom1_cb, 10)
        self.sub2 = self.create_subscription(
            Odometry, '/TB3_2/odom', self.odom2_cb, 10)
        self.pub = self.create_publisher(Float32, '/tb3_distance', 10)
        self.timer = self.create_timer(1.0, self.report)

        self.get_logger().info('Waiting for /TB3_1/odom and /TB3_2/odom ...')

    def odom1_cb(self, msg):
        self.pose1 = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def odom2_cb(self, msg):
        self.pose2 = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def report(self):
        if self.pose1 is None or self.pose2 is None:
            self.get_logger().info(
                'Waiting for both odom topics ...',
                throttle_duration_sec=5.0)
            return

        ox1 = float(self.get_parameter('tb3_1_offset_x').value)
        oy1 = float(self.get_parameter('tb3_1_offset_y').value)
        ox2 = float(self.get_parameter('tb3_2_offset_x').value)
        oy2 = float(self.get_parameter('tb3_2_offset_y').value)

        x1, y1 = self.pose1[0] + ox1, self.pose1[1] + oy1
        x2, y2 = self.pose2[0] + ox2, self.pose2[1] + oy2
        distance = math.hypot(x2 - x1, y2 - y1)

        self.pub.publish(Float32(data=float(distance)))
        self.get_logger().info(
            f'TB3_1=({x1:.2f}, {y1:.2f})  TB3_2=({x2:.2f}, {y2:.2f})  '
            f'distance={distance:.3f} m')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
