#!/bin/bash
# 每个新终端先执行：
#   source ~/turtlebot3_ws/tb3_env.sh
#
# 作用：
#   1. 加载 ROS 2 Humble 和本工作空间；
#   2. 设置 TURTLEBOT3_MODEL=burger；
#   3. 补上 Gazebo Classic 自带的模型路径。
#
# 第 3 点很关键：gazebo_ros 的 launch 默认把 GAZEBO_MODEL_PATH 设成
# “TB3 包内的 models”，不含 /usr/share/gazebo-11/models。empty_world /
# turtlebot3_world 里的 ground_plane、sun 找不到时，Gazebo 会去
# models.gazebosim.org 联网下载；网络不通就会卡在加载世界，
# /spawn_entity 服务永远不出现，机器人也就 spawn 不出来。

source /opt/ros/humble/setup.bash 2>/dev/null || true

if [ -f "$HOME/turtlebot3_ws/install/setup.bash" ]; then
    source "$HOME/turtlebot3_ws/install/setup.bash"
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"

# 可选：如果 Gazebo GUI 很卡，取消下面两行注释，让 GUI 走 NVIDIA 独显。
# 注意：TB3 的激光雷达是 CPU ray，不依赖这个设置，不会出现 TB4 的 0.164m 问题。
# export __NV_PRIME_RENDER_OFFLOAD=1
# export __GLX_VENDOR_LIBRARY_NAME=nvidia
