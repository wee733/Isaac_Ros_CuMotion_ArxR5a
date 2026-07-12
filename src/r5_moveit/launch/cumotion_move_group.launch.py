# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch MoveIt with cuMotion as the default R5A planning pipeline."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def generate_launch_description() -> LaunchDescription:
    """Create the cuMotion-enabled MoveIt and RViz launch description."""
    moveit_config = (
        MoveItConfigsBuilder('R5a', package_name='r5_moveit')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    r5_share = get_package_share_directory('r5_moveit')
    config_path = os.path.join(
        r5_share, 'config', 'isaac_ros_cumotion_planning.yaml'
    )
    with open(config_path, 'r', encoding='utf-8') as config_file:
        cumotion_config = yaml.safe_load(config_file)

    moveit_config.planning_pipelines['planning_pipelines'].insert(
        0, 'isaac_ros_cumotion'
    )
    moveit_config.planning_pipelines[
        'isaac_ros_cumotion'
    ] = cumotion_config
    moveit_config.planning_pipelines[
        'default_planning_pipeline'
    ] = 'isaac_ros_cumotion'

    controllers = {
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        ),
        'moveit_simple_controller_manager': {
            'controller_names': [
                'manipulator_controller',
                'gripper_controller',
            ],
            'manipulator_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'joint1',
                    'joint2',
                    'joint3',
                    'joint4',
                    'joint5',
                    'joint6',
                ],
            },
            'gripper_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'joints': ['joint7', 'joint8'],
            },
        },
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            controllers,
            {'use_sim_time': False},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(r5_share, 'config', 'moveit.rviz'),
        ],
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': False}],
    )

    return LaunchDescription([move_group_node, rviz_node])
