#!/usr/bin/env python3
"""
A* 寻路演示启动
加载带障碍物的 world，5 秒后启动 A* 导航节点自动规划路径。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'diff_car'

    # 机器人状态发布
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # 加载带障碍物的 world
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'demo_obstacle.world')
        }.items()
    )

    # 生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 加载控制器
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    # A* 导航节点延迟启动，等仿真稳定
    astar_navigator = Node(
        package=package_name,
        executable='astar_navigator',
        output='screen'
    )

    delayed_astar_navigator = TimerAction(
        period=5.0,
        actions=[astar_navigator]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        delayed_astar_navigator,
    ])
