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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
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
        default_value='172.18.0.201',
        description='Master robot controller IP address')
    master_robot_port_arg = DeclareLaunchArgument(
        name='master_robot_port',
        default_value='9439',
        description='Master robot controller WebSocket port')
    slave_robot_host_arg = DeclareLaunchArgument(
        name='slave_robot_host',
        default_value='172.18.23.100',
        description='Slave robot controller IP address')
    slave_robot_port_arg = DeclareLaunchArgument(
        name='slave_robot_port',
        default_value='8439',
        description='Slave robot controller WebSocket port')
    robot_grip_type_arg = DeclareLaunchArgument(
        name='robot_grip_type',
        default_value='empty',
        choices=['gp100', 'gp80', 'gr100', 'empty'],
        description='Grip type: gp100/gp80/gr100 (1-DoF) or empty (0-DoF)')
    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value='archer',
        choices=['archer', 'firefly'],
        description='Robot arm type: archer or firefly')
    enable_keyboard_arg = DeclareLaunchArgument(
        name='enable_keyboard',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to launch the keyboard teleoperation node')

    # robot launch file name: "archer.launch.py" / "firefly.launch.py"
    robot_launch_file = PythonExpression(
        ['"', LaunchConfiguration('robot_type'), '.launch.py"'])

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
            PathJoinSubstitution([arm_pkg_path, robot_launch_file])),
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
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])),
        condition=IfCondition(LaunchConfiguration('enable_keyboard')),
    )

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
                "robot_grip_type": LaunchConfiguration('robot_grip_type'),
            },
        ],
        remappings=[
            ('master/manip_state', 'master/manip_state'),
            ('master/joy_state', 'master/joy_state'),
            ('slave/manip_state', 'slave/manip_state'),
            ('slave/manip_ctrl', 'slave/manip_ctrl'),
            ('teleop_keyboard_state', '/teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        # arguments
        master_robot_host_arg,
        master_robot_port_arg,
        slave_robot_host_arg,
        slave_robot_port_arg,
        robot_grip_type_arg,
        robot_type_arg,
        enable_keyboard_arg,
        # master (real hello) / slave (real archer) instances
        GroupAction([PushRosNamespace('master'), master_launch]),
        GroupAction([PushRosNamespace('slave'), slave_launch]),
        # utilities
        keyboard_launch,
        follow_node,
    ])
