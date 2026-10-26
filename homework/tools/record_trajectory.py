#!/usr/bin/env python3
"""把 turtlesim 位姿或 odom 轨迹记录成 CSV。

用法示例：
  # 记录 turtlesim 的 /turtle1/pose，10 秒
  python3 record_trajectory.py --type pose --topic /turtle1/pose \
      --duration 10 --output ~/turtlebot3_ws/homework/data/circle_r1.csv

  # 记录 Gazebo 的 /odom，25 秒
  python3 record_trajectory.py --type odom --topic /odom \
      --duration 25 --output ~/turtlebot3_ws/homework/data/square_odom.csv

CSV 格式：t,x,y,yaw
"""

import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, args):
        super().__init__('trajectory_recorder')
        self.args = args
        self.start = time.monotonic()
        self.file = open(args.output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t', 'x', 'y', 'yaw'])
        self.done = False

        if args.type == 'pose':
            self.sub = self.create_subscription(
                Pose, args.topic, self.pose_cb, 10)
        else:
            self.sub = self.create_subscription(
                Odometry, args.topic, self.odom_cb, 10)

        self.timer = self.create_timer(0.1, self.check_done)
        self.get_logger().info(
            f'recording {args.topic} ({args.type}) -> {args.output}')

    def write(self, x, y, yaw):
        t = time.monotonic() - self.start
        self.writer.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}',
                              f'{yaw:.4f}'])

    def pose_cb(self, msg):
        self.write(msg.x, msg.y, msg.theta)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.write(p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def check_done(self):
        if self.args.duration > 0:
            if time.monotonic() - self.start >= self.args.duration:
                self.get_logger().info('duration reached, stop.')
                self.file.close()
                self.timer.cancel()
                self.done = True

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', choices=['pose', 'odom'], default='pose')
    parser.add_argument('--topic', default='/turtle1/pose')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='0 means until Ctrl+C')
    parser.add_argument('--output', required=True)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Recorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
