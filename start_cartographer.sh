#!/bin/bash
# 启动 Cartographer 2D SLAM（需要先启动 Gazebo 世界）。
set -e

source "$(dirname "$0")/tb3_env.sh"
exec ros2 launch turtlebot3_cartographer cartographer.launch.py \
    use_sim_time:=True
