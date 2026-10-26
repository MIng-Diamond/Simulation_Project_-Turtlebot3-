#!/bin/bash
# 启动两台 TurtleBot3（TB3_1 / TB3_2）。
# 用法：
#   bash start_multi_robot.sh                         # turtlebot3_world
#   bash start_multi_robot.sh empty_world.world       # 空世界（做跟随实验）
set -e

source "$(dirname "$0")/tb3_env.sh"
WORLD="${1:-turtlebot3_world.world}"
exec ros2 launch tb3_course multi_robot_2.launch.py world:="$WORLD"
