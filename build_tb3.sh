#!/bin/bash
# 编译 tb3_course 并 source 当前 shell 的环境（用于交互式终端）
set -e

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb3_course
source install/setup.bash

echo
echo "tb3_course 编译完成。可用节点："
ros2 pkg executables tb3_course
