#!/usr/bin/env python3
"""让 turtlesim 画圆。

对应教程 2.7。相比教程原版，这里把半径和线速度做成 ROS 参数，
方便完成 2.9 的“修改圆半径”练习：

    ros2 run tb3_course circle
    ros2 run tb3_course circle --ros-args -p radius:=0.5
    ros2 param set /circle_publisher radius 2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('rate_hz', 10.0)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        rate = max(0.1, float(self.get_parameter('rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)
        self.get_logger().info(
            'circle publisher started: v=%.2f m/s, radius parameter (r=v/w)'
            % float(self.get_parameter('linear_speed').value))

    def timer_cb(self):
        radius = max(0.01, float(self.get_parameter('radius').value))
        speed = float(self.get_parameter('linear_speed').value)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = speed / radius
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
