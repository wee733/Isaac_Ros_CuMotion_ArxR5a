# Copyright (c) 2023-2025 ARXrobotics
# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Display the ARX R5A description in RViz."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the robot description display launch description."""
    package_share = get_package_share_directory('R5a')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(package_share, 'rviz', 'view.rviz'),
            description='Full path to the RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Start robot_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub',
            default_value='True',
            description='Start joint_state_publisher_gui.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(package_share, 'urdf', 'R5a.urdf'),
            description='Full path to the R5A URDF.',
        ),
    ]

    robot_state_publisher = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
    )
    joint_state_publisher = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        arguments=[urdf_file],
    )
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription(
        [
            *declared_arguments,
            joint_state_publisher,
            robot_state_publisher,
            rviz,
        ]
    )
