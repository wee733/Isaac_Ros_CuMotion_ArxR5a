# Copyright (c) 2023-2025 ARXrobotics
# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Spawn the ARX R5A description in Gazebo Classic."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def replace_package_path_in_urdf(urdf_path, package_path):
    """Create a temporary URDF with absolute mesh paths for Gazebo."""
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        urdf_content = urdf_file.read()
    urdf_content = urdf_content.replace('package://R5a', package_path)

    temporary_urdf_path = '/tmp/temp_R5a.urdf'
    with open(temporary_urdf_path, 'w', encoding='utf-8') as urdf_file:
        urdf_file.write(urdf_content)
    return temporary_urdf_path


def generate_launch_description():
    """Create the Gazebo launch description."""
    package_name = 'R5a'
    package_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(package_share, 'urdf', 'R5a.urdf')
    temporary_urdf_path = replace_package_path_in_urdf(
        urdf_model_path, package_share
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ],
        output='screen',
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'R5a', '-file', temporary_urdf_path],
        output='screen',
    )

    return LaunchDescription([gazebo, spawn_entity])
