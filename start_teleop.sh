#!/bin/bash
# 键盘遥控 TurtleBot3（Ctrl+C 退出）。
set -e

source "$(dirname "$0")/tb3_env.sh"
exec ros2 run turtlebot3_teleop teleop_keyboard
