#!/usr/bin/env python3
"""开环走近似正方形。对应教程 3.7。

这是按时间开环控制：走 side_time 秒、转 turn_time 秒，重复四次。
由于轮子打滑、加速过程等因素，它只能得到“近似”正方形。
闭环版本见 square_odom.py。

运行示例（必须先启动 Gazebo）：
    ros2 run tb3_course move_square --ros-args -p use_sim_time:=true
    ros2 run tb3_course move_square --ros-args \
        -p use_sim_time:=true -p linear_speed:=0.12 -p side_time:=5.0
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        self.declare_parameter('side_time', 3.0)
        self.declare_parameter('turn_time', 2.0)
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', math.pi / 4.0)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.side_index = 0
        self.state = 'FORWARD'
        self.state_start = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.loop)

    def elapsed(self):
        dt = self.get_clock().now() - self.state_start
        return dt.nanoseconds / 1e9

    def switch_state(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()

    def loop(self):
        cmd = Twist()

        if self.side_index >= 4:
            self.pub.publish(cmd)
            self.get_logger().info('Square finished.')
            self.timer.cancel()
            return

        side_time = float(self.get_parameter('side_time').value)
        turn_time = float(self.get_parameter('turn_time').value)
        speed = float(self.get_parameter('linear_speed').value)
        angular = float(self.get_parameter('angular_speed').value)

        if self.state == 'FORWARD':
            cmd.linear.x = speed
            if self.elapsed() >= side_time:
                self.switch_state('TURN')
        elif self.state == 'TURN':
            cmd.angular.z = angular
            if self.elapsed() >= turn_time:
                self.side_index += 1
                self.get_logger().info(
                    f'Finished side {self.side_index}/4')
                self.switch_state('FORWARD')

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveSquare()
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
