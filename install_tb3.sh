#!/bin/bash
# 安装 TurtleBot3 + Gazebo Classic（需要 sudo 密码）
# 只安装本课程需要的包，不动已有的 TurtleBot4 / Ignition 环境。
set -e

sudo apt update
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-fake-node

echo
echo "安装完成。检查："
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E '^turtlebot3' | sort
which gazebo
gazebo --version
