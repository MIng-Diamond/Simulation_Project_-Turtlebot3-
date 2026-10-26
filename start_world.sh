#!/bin/bash
# 启动 TurtleBot3 Gazebo 世界。
# 用法：
#   bash start_world.sh                  # 默认 turtlebot3_world（建图用）
#   bash start_world.sh empty_world      # 空世界
#   bash start_world.sh turtlebot3_house # House（拓展）
set -e

source "$(dirname "$0")/tb3_env.sh"
WORLD="${1:-turtlebot3_world}"
echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL, world=$WORLD"
exec ros2 launch turtlebot3_gazebo "${WORLD}.launch.py"
