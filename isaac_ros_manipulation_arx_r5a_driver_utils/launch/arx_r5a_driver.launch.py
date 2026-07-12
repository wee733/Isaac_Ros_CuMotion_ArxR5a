# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch the ARX R5A driver adapter, MoveIt, and optional cuMotion."""

from isaac_ros_manipulation_arx_r5a_driver_utils import (
    ArxR5aDriverConfig,
    ArxR5aDriverUtils,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


DESCRIPTION_PACKAGE = 'isaac_ros_manipulation_arx_r5a_robot_description'


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and construct the complete driver stack."""
    driver_config = ArxR5aDriverConfig(context)
    arx = ArxR5aDriverUtils(driver_config)

    actions = []
    if driver_config.start_vendor_driver:
        actions.append(arx.get_vendor_driver_launch())

    actions.append(arx.get_robot_state_publisher())
    actions.extend(arx.get_robot_control_nodes())

    if driver_config.start_cumotion:
        actions.extend(arx.get_cumotion_actions())

    move_group, moveit_config = arx.get_moveit_group_node()
    actions.append(move_group)
    if driver_config.start_rviz:
        actions.append(arx.get_rviz_node(moveit_config))
    return actions


def generate_launch_description():
    """Generate the official-style ARX R5A driver launch description."""
    description_share = FindPackageShare(DESCRIPTION_PACKAGE)

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use the ROS simulation clock.',
        ),
        DeclareLaunchArgument(
            'start_vendor_driver',
            default_value='False',
            description='Include the official ARX single-arm driver launch.',
        ),
        DeclareLaunchArgument(
            'start_cumotion',
            default_value='True',
            description='Start cuMotion and make it the default arm planner.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='True',
            description='Start RViz with the MoveIt motion-planning panel.',
        ),
        DeclareLaunchArgument(
            'read_esdf_world',
            default_value='False',
            description='Read the nvblox ESDF world for obstacle avoidance.',
        ),
        DeclareLaunchArgument(
            'configure_isaac_ros_43_environment',
            default_value='True',
            description='Add Isaac ROS CLI Python paths required by 4.3 deb installs.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Log level for MoveIt.',
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=PathJoinSubstitution(
                [description_share, 'urdf', 'r5a.urdf.xacro']
            ),
        ),
        DeclareLaunchArgument(
            'srdf_path',
            default_value=PathJoinSubstitution(
                [description_share, 'srdf', 'r5a.srdf']
            ),
        ),
        DeclareLaunchArgument(
            'initial_positions_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'initial_positions.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'joint_limits_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'joint_limits.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'kinematics_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'kinematics.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'moveit_controllers_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'moveit_controllers.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'ros2_controllers_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'ros2_controllers.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution(
                [description_share, 'config', 'moveit.rviz']
            ),
        ),
        DeclareLaunchArgument(
            'cumotion_urdf_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'urdf', 'r5a_cumotion.urdf']
            ),
        ),
        DeclareLaunchArgument(
            'cumotion_xrdf_file_path',
            default_value=PathJoinSubstitution(
                [description_share, 'xrdf', 'r5a.xrdf']
            ),
        ),
        DeclareLaunchArgument(
            'cumotion_tool_frame',
            default_value='link6',
            description='End-effector frame used by cuMotion.',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
