#!/bin/bash
# 以 root 身份运行（由 sudo 调用）：安装 TurtleBot3 + Gazebo Classic。
set -e

apt-get update
apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-fake-node
