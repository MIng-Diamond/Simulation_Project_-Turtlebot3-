#!/usr/bin/env python3
"""记录 Nav2 的 /plan 路径到 CSV。

用法：
  python3 record_path.py --output ~/turtlebot3_ws/homework/data/nav_paths.csv

每收到一条 Path 消息，path_id 加 1；CSV 列为 path_id,index,x,y。
Ctrl+C 退出。
"""

import argparse
import csv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PathRecorder(Node):
    def __init__(self, output):
        super().__init__('path_recorder')
        self.file = open(output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['path_id', 'index', 'x', 'y'])
        self.path_id = 0
        self.sub = self.create_subscription(
            Path, '/plan', self.path_cb, 10)
        self.get_logger().info(f'recording /plan -> {output}')

    def path_cb(self, msg):
        self.path_id += 1
        for i, pose in enumerate(msg.poses):
            p = pose.pose.position
            self.writer.writerow([self.path_id, i, f'{p.x:.4f}', f'{p.y:.4f}'])
        self.file.flush()
        self.get_logger().info(
            f'path {self.path_id}: {len(msg.poses)} poses '
            f'(frame={msg.header.frame_id})')

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PathRecorder(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
