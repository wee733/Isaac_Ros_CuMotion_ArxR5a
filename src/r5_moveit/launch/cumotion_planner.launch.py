# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch the Isaac ROS 4.3 cuMotion planner for the ARX R5A."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _prepend_existing(paths, variable_name):
    values = [path for path in paths if os.path.exists(path)]
    current_value = os.environ.get(variable_name, '')
    if current_value:
        values.append(current_value)
    return os.pathsep.join(values)


def generate_launch_description() -> LaunchDescription:
    """Create the R5A cuMotion planner launch description."""
    r5_share = get_package_share_directory('r5_moveit')
    cumotion_share = get_package_share_directory('isaac_ros_cumotion')

    xrdf_file_path = LaunchConfiguration('xrdf_file_path')
    urdf_file_path = LaunchConfiguration('urdf_file_path')
    tool_frame = LaunchConfiguration('tool_frame')
    read_esdf_world = LaunchConfiguration('read_esdf_world')

    # Debian console scripts use /usr/bin/python3, while the pip shims install
    # torch and related packages in the managed Isaac ROS virtualenv.
    isaac_ros_venv = '/var/lib/isaac-ros-cli/isaac-ros'
    venv_site_packages = os.path.join(
        isaac_ros_venv, 'lib', 'python3.12', 'site-packages'
    )
    ros_overlay = os.path.join(
        isaac_ros_venv, 'ros-deb-overlay', 'opt', 'ros', 'jazzy'
    )
    system_manipulator_utils = os.path.join(
        '/opt/ros/jazzy/lib/python3.12/site-packages',
        'isaac_manipulator_ros_python_utils',
    )
    overlay_paths = (
        [] if os.path.exists(system_manipulator_utils) else [ros_overlay]
    )
    python_path = _prepend_existing(
        [
            *[os.path.join(path, 'lib', 'python3.12', 'site-packages')
              for path in overlay_paths],
            venv_site_packages,
        ],
        'PYTHONPATH',
    )
    library_path = _prepend_existing(
        [os.path.join(path, 'lib') for path in overlay_paths],
        'LD_LIBRARY_PATH',
    )
    ament_prefix_path = _prepend_existing(
        overlay_paths, 'AMENT_PREFIX_PATH'
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                cumotion_share, 'launch', 'isaac_ros_cumotion.launch.py'
            )
        ),
        launch_arguments={
            # These are the Isaac ROS 4.3 names, not the 4.5 documentation.
            'cumotion_planner.robot': xrdf_file_path,
            'cumotion_planner.urdf_path': urdf_file_path,
            'cumotion_planner.tool_frame': tool_frame,
            'cumotion_planner.read_esdf_world': read_esdf_world,
            'cumotion_planner.add_ground_plane': 'False',
            'cumotion_planner.override_moveit_scaling_factors': 'False',
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'xrdf_file_path',
                default_value=os.path.join(r5_share, 'config', 'r5a.xrdf'),
            ),
            DeclareLaunchArgument(
                'urdf_file_path',
                default_value=os.path.join(
                    r5_share, 'config', 'r5a_cumotion.urdf'
                ),
            ),
            DeclareLaunchArgument('tool_frame', default_value='link6'),
            DeclareLaunchArgument('read_esdf_world', default_value='False'),
            SetEnvironmentVariable('PYTHONPATH', python_path),
            SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
            SetEnvironmentVariable('AMENT_PREFIX_PATH', ament_prefix_path),
            planner,
        ]
    )
