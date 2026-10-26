#!/bin/bash
# 启动 Nav2（需要先启动 Gazebo 世界，并准备好地图）。
# 用法：
#   bash start_nav2.sh
#   bash start_nav2.sh /path/to/map.yaml
set -e

source "$(dirname "$0")/tb3_env.sh"
MAP="${1:-$HOME/maps/tb3_world.yaml}"
echo "map=$MAP"
exec ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=True map:="$MAP"
