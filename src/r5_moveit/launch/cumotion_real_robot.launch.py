# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch the real R5A hardware, cuMotion planner, MoveIt, and RViz."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the complete real-robot cuMotion launch description."""
    launch_dir = PathJoinSubstitution(
        [FindPackageShare('r5_moveit'), 'launch']
    )
    read_esdf_world = LaunchConfiguration('read_esdf_world')

    def include(name, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([launch_dir, name])
            ),
            launch_arguments=(launch_arguments or {}).items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument('read_esdf_world', default_value='False'),
            include('hardware.launch.py'),
            TimerAction(
                period=3.0,
                actions=[
                    include(
                        'cumotion_planner.launch.py',
                        {'read_esdf_world': read_esdf_world},
                    )
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[include('cumotion_move_group.launch.py')],
            ),
        ]
    )
