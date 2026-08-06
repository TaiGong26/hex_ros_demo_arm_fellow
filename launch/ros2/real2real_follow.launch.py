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
    slave_robot_host_arg = DeclareLaunchArgument(
        name='slave_robot_host',
        default_value='192.168.1.101',
        description='Slave robot controller IP address')
    slave_robot_port_arg = DeclareLaunchArgument(
        name='slave_robot_port',
        default_value='8439',
        description='Slave robot controller WebSocket port')
    robot_grip_type_arg = DeclareLaunchArgument(
        name='robot_grip_type',
        default_value='empty',
        choices=['gp80', 'gr100', 'empty'],
        description='Grip type: gp80 (1-DoF) or empty (0-DoF)')

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
    # Slave robot (real archer, namespaced /slave/*)
    # ------------------------------------------------------------------
    slave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arm_pkg_path, "archer.launch.py"])),
        launch_arguments={
            'robot_host': LaunchConfiguration('slave_robot_host'),
            'robot_port': LaunchConfiguration('slave_robot_port'),
            'robot_grip_type': LaunchConfiguration('robot_grip_type'),
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
    # Follow node (real: no use_sim_time)
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
        slave_robot_host_arg,
        slave_robot_port_arg,
        robot_grip_type_arg,
        # master (real hello) / slave (real archer) instances
        GroupAction([PushRosNamespace('master'), master_launch]),
        GroupAction([PushRosNamespace('slave'), slave_launch]),
        # utilities
        keyboard_launch,
        follow_node,
    ])
