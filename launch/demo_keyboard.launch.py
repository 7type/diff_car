#!/usr/bin/env python3
"""
键盘控制演示启动
把 Gazebo、差速控制器、键盘控制节点一起拉起来，5 秒后键盘才生效，给仿真留启动时间。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'diff_car'

    # 机器人状态发布，解析 xacro 并发布 robot_description
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # 启动 Gazebo 空场景，extra_gazebo_args 用来传参数文件
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # 从 topic 生成机器人实体并放到仿真里
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 加载差速控制器和里程计广播器
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

    # 键盘控制节点延迟 5 秒启动，不然 Gazebo 还没准备好按键会没反应
    keyboard_control = Node(
        package=package_name,
        executable='keyboard_control',
        output='screen'
    )

    delayed_keyboard_control = TimerAction(
        period=5.0,
        actions=[keyboard_control]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        delayed_keyboard_control,
    ])
