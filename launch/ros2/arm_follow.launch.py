#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-06
################################################################

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    follow_pkg_path = FindPackageShare('hex_ros_demo_arm_follow')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_archer_y6')

    # arm_follow node
    follow_param_path = PathJoinSubstitution(
        [follow_pkg_path, "config", "ros2", "arm_follow.yaml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_comp.urdf"])

    arm_follow_node = Node(
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
            ('master/joy_state', 'master/joy_state'),
            ('slave/manip_state', 'slave/manip_state'),
            ('slave/manip_ctrl', 'slave/manip_ctrl'),
            ('teleop_keyboard_state', '/teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        arm_follow_node,
    ])
