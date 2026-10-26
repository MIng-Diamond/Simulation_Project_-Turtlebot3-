#!/usr/bin/env python3
"""通过 Nav2 的 NavigateToPose Action 发送单个导航目标。对应教程 5.5。

运行示例（先启动 Nav2 并完成初始位姿）：
    ros2 run tb3_course nav_goal --ros-args \
        -p use_sim_time:=true \
        -p goal_x:=1.0 -p goal_y:=0.5 -p goal_yaw:=0.0

坐标系是 map。goal 坐标要根据实际地图的自由空间调整。
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class NavGoal(Node):
    def __init__(self):
        super().__init__('nav_goal')

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.5, self.start_once)
        self.started = False

    def start_once(self):
        if self.started:
            return

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                'Waiting for NavigateToPose action server...',
                throttle_duration_sec=2.0)
            return

        self.started = True
        self.timer.cancel()

        x = float(self.get_parameter('goal_x').value)
        y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # stamp=0：让 Nav2 使用最新 TF，避免仿真时间未同步时目标时间戳过旧
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Send goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {fb.distance_remaining:.2f} m',
            throttle_duration_sec=1.0)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation finished, status={status}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
