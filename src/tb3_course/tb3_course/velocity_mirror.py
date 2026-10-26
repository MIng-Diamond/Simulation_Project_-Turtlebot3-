#!/usr/bin/env python3
"""把 TB3_1 的速度指令镜像给 TB3_2。对应教程 6.6。

先启动两台机器人（多机器人 launch），再运行：
    ros2 run tb3_course velocity_mirror
另一个终端遥控 leader：
    ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1

注意：这是“开环镜像”，不是闭环队形保持；两台车动力学有差异时会漂移。
闭环版本见 leader_follower.py。
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityMirror(Node):
    def __init__(self):
        super().__init__('velocity_mirror')
        self.declare_parameter('scale', 0.9)
        # 看门狗：leader 停止发命令后，follower 必须在 0.5 s 内停车，
        # 否则最后一条非零 cmd_vel 会一直生效，车会撞墙/侧翻。
        self.declare_parameter('cmd_timeout', 0.5)

        self.sub = self.create_subscription(
            Twist, '/TB3_1/cmd_vel', self.cmd_cb, 10)
        self.pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.watchdog)
        self.last_cmd_time = None

        self.get_logger().info(
            'Mirroring /TB3_1/cmd_vel -> /TB3_2/cmd_vel, '
            f'scale={float(self.get_parameter("scale").value):.2f}, '
            f'cmd_timeout={float(self.get_parameter("cmd_timeout").value):.2f}s')

    def cmd_cb(self, msg):
        scale = float(self.get_parameter('scale').value)
        out = Twist()
        out.linear.x = msg.linear.x * scale
        out.linear.y = msg.linear.y * scale
        out.linear.z = msg.linear.z * scale
        out.angular.x = msg.angular.x * scale
        out.angular.y = msg.angular.y * scale
        out.angular.z = msg.angular.z * scale
        self.pub.publish(out)
        self.last_cmd_time = time.monotonic()

    def watchdog(self):
        if self.last_cmd_time is None:
            return
        timeout = float(self.get_parameter('cmd_timeout').value)
        if time.monotonic() - self.last_cmd_time > timeout:
            self.pub.publish(Twist())
            self.last_cmd_time = None
            self.get_logger().info('Leader cmd timeout, follower stopped.',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
