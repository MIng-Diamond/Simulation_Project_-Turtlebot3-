#!/usr/bin/env python3
"""按顺序执行多个 Nav2 目标，支持循环多圈。对应教程 5.6。

运行示例（先启动 Nav2 并完成初始位姿）：
    ros2 run tb3_course patrol --ros-args -p use_sim_time:=true
    ros2 run tb3_course patrol --ros-args -p use_sim_time:=true -p loops:=2

默认路线是按 turtlebot3_world 原点附近选的点，实际使用时请根据地图
自由空间调整 self.goals；目标点落在障碍物上会被 Nav2 拒绝。
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol')
        self.declare_parameter('loops', 1)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # (x, y, yaw)，map 坐标系。
        # 本课程保存的地图以建图起点为 map 原点，therefore 外圈矩形
        # 采用 (0.35,0) → (3.65,0) → (3.65,2.15) → (0.35,2.15)，
        # 距墙和圆柱约 0.55 m，四点均经过 Nav2 实测可达。
        self.goals = [
            (0.35, 0.0, 0.0),
            (3.65, 0.0, 0.0),
            (3.65, 2.15, math.pi / 2.0),
            (0.35, 2.15, math.pi),
        ]
        self.index = 0
        self.loop = 0
        self.started = False
        self.timer = self.create_timer(0.5, self.try_start)

    def try_start(self):
        if self.started:
            return
        if self.client.wait_for_server(timeout_sec=0.1):
            self.started = True
            self.timer.cancel()
            self.send_current_goal()

    def send_current_goal(self):
        x, y, yaw = self.goals[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # stamp=0 表示让 Nav2 使用最新 TF。若节点启动时 /clock 还没同步，
        # 用 get_clock().now() 会得到旧时间，Nav2 会报 TF 外推错误。
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        loops = max(1, int(self.get_parameter('loops').value))
        self.get_logger().info(
            f'Loop {self.loop + 1}/{loops}, '
            f'goal {self.index + 1}/{len(self.goals)}: ({x:.2f}, {y:.2f})')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected, stop patrol.')
            rclpy.shutdown()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Goal failed, status={status}. Stop patrol.')
            rclpy.shutdown()
            return

        self.index += 1
        if self.index >= len(self.goals):
            self.index = 0
            self.loop += 1

        loops = max(1, int(self.get_parameter('loops').value))
        if self.loop >= loops:
            self.get_logger().info('Patrol finished.')
            rclpy.shutdown()
        else:
            self.send_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = Patrol()
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
