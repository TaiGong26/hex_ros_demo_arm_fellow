#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-06
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_pkg_path = FindPackageShare('hex_ros_robot_arm')
    sim_pkg_path = FindPackageShare('hex_ros_sim_archer_y6')
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    follow_pkg_path = FindPackageShare('hex_ros_demo_arm_follow')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_archer_y6')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    master_robot_host_arg = DeclareLaunchArgument(
        name='master_robot_host',
        default_value='192.168.1.100',
        description='Master robot controller IP address')
    master_robot_port_arg = DeclareLaunchArgument(
        name='master_robot_port',
        default_value='8439',
        description='Master robot controller WebSocket port')
    viewer_arg = DeclareLaunchArgument(
        name='viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on mujoco viewer (slave sim)')
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to turn on rviz')

    # ------------------------------------------------------------------
    # Master robot (real hello, read-only, namespaced /master/*)
    # ------------------------------------------------------------------
    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arm_pkg_path, "hello.launch.py"])),
        launch_arguments={
            'robot_host': LaunchConfiguration('master_robot_host'),
            'robot_port': LaunchConfiguration('master_robot_port'),
        }.items(),
    )

    # ------------------------------------------------------------------
    # Slave sim (namespaced /slave/*)
    # ------------------------------------------------------------------
    slave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg_path, "sim_archer_y6.launch.py"])),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration('rviz'),
            'test': 'false',
        }.items(),
    )

    # ------------------------------------------------------------------
    # Keyboard teleop
    # ------------------------------------------------------------------
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])), )

    # ------------------------------------------------------------------
    # Follow node (real master + sim slave -> use_sim_time)
    # ------------------------------------------------------------------
    follow_param_path = PathJoinSubstitution(
        [follow_pkg_path, "config", "ros2", "arm_follow.yaml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_comp.urdf"])
    follow_node = Node(
        package='hex_ros_demo_arm_follow',
        executable='arm_follow',
        name='arm_follow',
        output="screen",
        emulate_tty=True,
        parameters=[
            follow_param_path,
            {
                "model_urdf": ParameterValue(urdf_file_path, value_type=str),
                "use_sim_time": True,
            },
        ],
        remappings=[
            ('master/manip_state', 'master/manip_state'),
            ('slave/manip_state', 'slave/manip_state'),
            ('slave/manip_ctrl', 'slave/manip_ctrl'),
            ('teleop_keyboard_state', 'teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        # arguments
        master_robot_host_arg,
        master_robot_port_arg,
        viewer_arg,
        rviz_arg,
        # master (real hello) / slave (sim) instances
        GroupAction([PushRosNamespace('master'), master_launch]),
        GroupAction([PushRosNamespace('slave'), slave_launch]),
        # utilities
        keyboard_launch,
        follow_node,
    ])
