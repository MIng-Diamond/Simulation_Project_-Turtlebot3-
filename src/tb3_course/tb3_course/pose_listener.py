#!/usr/bin/env python3
"""订阅 turtlesim 的 /turtle1/pose，打印位姿。对应教程 2.8。"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_cb, 10)

    def pose_cb(self, msg):
        # turtlesim 的 pose 频率较高，限制为每秒最多两条，避免刷屏
        self.get_logger().info(
            f'x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
