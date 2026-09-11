#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-13
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    follow_pkg_path = FindPackageShare('hex_ros_demo_arm_follow')

    enable_keyboard_arg = DeclareLaunchArgument(
        name='enable_keyboard',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to launch the shared keyboard teleoperation node')

    # Left group arguments.
    left_master_robot_host_arg = DeclareLaunchArgument(
        name='left_master_robot_host',
        default_value='198.168.100.1',
        description='Left master robot controller IP address')
    left_master_robot_port_arg = DeclareLaunchArgument(
        name='left_master_robot_port',
        default_value='9439',
        description='Left master robot controller WebSocket port')
    left_slave_robot_host_arg = DeclareLaunchArgument(
        name='left_slave_robot_host',
        default_value='172.18.23.100',
        description='Left slave robot controller IP address')
    left_slave_robot_port_arg = DeclareLaunchArgument(
        name='left_slave_robot_port',
        default_value='8439',
        description='Left slave robot controller WebSocket port')
    left_robot_grip_type_arg = DeclareLaunchArgument(
        name='left_robot_grip_type',
        default_value='empty',
        choices=['gp100', 'gp80', 'gr100', 'empty'],
        description='Left slave robot grip type')
    left_robot_type_arg = DeclareLaunchArgument(
        name='left_robot_type',
        default_value='archer',
        choices=['archer', 'firefly'],
        description='Left slave robot arm type')

    # Right group arguments.
    right_master_robot_host_arg = DeclareLaunchArgument(
        name='right_master_robot_host',
        default_value='198.168.100.1',
        description='Right master robot controller IP address')
    right_master_robot_port_arg = DeclareLaunchArgument(
        name='right_master_robot_port',
        default_value='8439',
        description='Right master robot controller WebSocket port')
    right_slave_robot_host_arg = DeclareLaunchArgument(
        name='right_slave_robot_host',
        default_value='172.18.23.100',
        description='Right slave robot controller IP address')
    right_slave_robot_port_arg = DeclareLaunchArgument(
        name='right_slave_robot_port',
        default_value='9439',
        description='Right slave robot controller WebSocket port')
    right_robot_grip_type_arg = DeclareLaunchArgument(
        name='right_robot_grip_type',
        default_value='empty',
        choices=['gp100', 'gp80', 'gr100', 'empty'],
        description='Right slave robot grip type')
    right_robot_type_arg = DeclareLaunchArgument(
        name='right_robot_type',
        default_value='archer',
        choices=['archer', 'firefly'],
        description='Right slave robot arm type')

    follow_launch_path = PathJoinSubstitution(
        [follow_pkg_path, 'hello2real_follow.launch.py'])

    left_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follow_launch_path),
        launch_arguments={
            'master_robot_host': LaunchConfiguration('left_master_robot_host'),
            'master_robot_port': LaunchConfiguration('left_master_robot_port'),
            'slave_robot_host': LaunchConfiguration('left_slave_robot_host'),
            'slave_robot_port': LaunchConfiguration('left_slave_robot_port'),
            'robot_grip_type': LaunchConfiguration('left_robot_grip_type'),
            'robot_type': LaunchConfiguration('left_robot_type'),
            'enable_keyboard': 'false',
        }.items(),
    )

    right_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follow_launch_path),
        launch_arguments={
            'master_robot_host': LaunchConfiguration('right_master_robot_host'),
            'master_robot_port': LaunchConfiguration('right_master_robot_port'),
            'slave_robot_host': LaunchConfiguration('right_slave_robot_host'),
            'slave_robot_port': LaunchConfiguration('right_slave_robot_port'),
            'robot_grip_type': LaunchConfiguration('right_robot_grip_type'),
            'robot_type': LaunchConfiguration('right_robot_type'),
            'enable_keyboard': 'false',
        }.items(),
    )

    # Keep one shared keyboard node outside both arm namespaces.
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, 'teleop_keyboard.launch.py'])),
        condition=IfCondition(LaunchConfiguration('enable_keyboard')),
    )

    return LaunchDescription([
        enable_keyboard_arg,
        left_master_robot_host_arg,
        left_master_robot_port_arg,
        left_slave_robot_host_arg,
        left_slave_robot_port_arg,
        left_robot_grip_type_arg,
        left_robot_type_arg,
        right_master_robot_host_arg,
        right_master_robot_port_arg,
        right_slave_robot_host_arg,
        right_slave_robot_port_arg,
        right_robot_grip_type_arg,
        right_robot_type_arg,
        GroupAction([
            PushRosNamespace('left'),
            left_follow_launch,
        ]),
        GroupAction([
            PushRosNamespace('right'),
            right_follow_launch,
        ]),
        keyboard_launch,
    ])
