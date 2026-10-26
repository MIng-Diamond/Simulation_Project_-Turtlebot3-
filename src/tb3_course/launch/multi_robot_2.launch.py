#!/usr/bin/env python3
"""两台 TurtleBot3 的多机器人 Gazebo 仿真。

基于官方 turtlebot3_gazebo/multi_robot.launch.py（Humble，默认 4 台），
按教程 6.4 改成 2 台，并把临时 SDF 的写入位置从 /opt/ros/... 改到
用户可写的 /tmp/tb3_multi，避免 apt 安装目录只读导致 PermissionError。

用法：
    export TURTLEBOT3_MODEL=burger
    ros2 launch tb3_course multi_robot_2.launch.py

话题/命名空间：
    /TB3_1/cmd_vel, /TB3_1/odom, /TB3_1/scan
    /TB3_2/cmd_vel, /TB3_2/odom, /TB3_2/scan
TF frame 前缀：TB3_1/..., TB3_2/...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace

import xml.etree.ElementTree as ET


def generate_launch_description():
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']

    number_of_robots = 2
    namespace = 'TB3'
    pose = [[-2.0, -0.5], [0.5, -2.0]]

    model_folder = 'turtlebot3_' + turtlebot3_model
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    save_path = '/tmp/tb3_multi/'
    os.makedirs(save_path, exist_ok=True)

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world',
                                     default='turtlebot3_world.world')

    world = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        world_name,
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd_list = []
    for count in range(number_of_robots):
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir,
                                 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace}_{count + 1}'
                }.items()
            )
        )

    spawn_turtlebot_cmd_list = []
    for count in range(number_of_robots):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        # 让每个机器人使用独立的 odom / base_footprint / scan frame
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}_{count + 1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}_{count + 1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}_{count + 1}/base_scan'

        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n' + urdf_modified
        sdf_file = f'{save_path}tb3_{count + 1}.sdf'
        with open(sdf_file, 'w') as file:
            file.write(urdf_modified)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir,
                                 'multi_spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(pose[count][0]),
                    'y_pose': str(pose[count][1]),
                    'robot_name': f'{turtlebot3_model}_{count + 1}',
                    'namespace': f'{namespace}_{count + 1}',
                    'sdf_path': sdf_file
                }.items()
            )
        )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='turtlebot3_world.world',
        description='World file name under turtlebot3_gazebo/worlds'))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: [
                os.remove(f'{save_path}tb3_{count + 1}.sdf')
                for count in range(number_of_robots)
            ]
        )
    ))

    for count, spawn_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([
            PushRosNamespace(f'{namespace}_{count}'),
            robot_state_publisher_cmd_list[count - 1],
            spawn_cmd
        ]))

    return ld
