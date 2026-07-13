# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Official-style robot controller utilities for the ARX R5A."""

import os

from ament_index_python.packages import get_package_share_directory

from isaac_ros_manipulation_arx_r5a_driver_utils.robot_description import (
    get_robot_description_contents,
)

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from moveit_configs_utils import MoveItConfigsBuilder

import yaml


DESCRIPTION_PACKAGE = 'isaac_ros_manipulation_arx_r5a_robot_description'


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


class ArxR5aDriverUtils:
    """Build robot-state, MoveIt, cuMotion, and ros2_control launch actions."""

    def __init__(self, driver_config):
        self.driver_config = driver_config
        self.robot_description_content = get_robot_description_contents(
            driver_config.urdf_path,
            driver_config.initial_positions_file_path,
        )
        self.robot_description = {
            'robot_description': self.robot_description_content,
        }

    def get_robot_state_publisher(self) -> Node:
        """Return robot_state_publisher with the real R5A description."""
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                self.robot_description,
                {'use_sim_time': self.driver_config.use_sim_time},
            ],
        )

    def get_robot_control_nodes(self):
        """Return ros2_control and controller spawner nodes."""
        controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                self.robot_description,
                ParameterFile(
                    self.driver_config.ros2_controllers_file_path,
                    allow_substs=True,
                ),
                {'use_sim_time': self.driver_config.use_sim_time},
            ],
        )

        def spawner(controller_name):
            return Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller_name,
                    '--controller-manager',
                    '/controller_manager',
                    '--controller-manager-timeout',
                    '30',
                ],
            )

        return [
            controller_manager,
            spawner('joint_state_broadcaster'),
            spawner('manipulator_controller'),
            spawner('gripper_controller'),
        ]

    def get_moveit_group_node(self):
        """Return move_group and the matching MoveIt configuration bundle."""
        config = self.driver_config
        moveit_config = (
            MoveItConfigsBuilder('r5a', package_name=DESCRIPTION_PACKAGE)
            .robot_description(
                file_path=config.urdf_path,
                mappings={
                    'initial_positions_file': config.initial_positions_file_path,
                },
            )
            .robot_description_semantic(file_path=config.srdf_path)
            .robot_description_kinematics(file_path=config.kinematics_file_path)
            .joint_limits(file_path=config.joint_limits_file_path)
            .trajectory_execution(file_path=config.moveit_controllers_file_path)
            .planning_pipelines(
                default_planning_pipeline='ompl',
                pipelines=['ompl'],
            )
            .to_moveit_configs()
        )

        if config.start_cumotion:
            cumotion_config_path = os.path.join(
                get_package_share_directory('isaac_ros_cumotion_moveit'),
                'config',
                'isaac_ros_cumotion_planning.yaml',
            )
            cumotion_config = _load_yaml(cumotion_config_path)
            moveit_config.planning_pipelines['planning_pipelines'].insert(
                0, 'isaac_ros_cumotion'
            )
            moveit_config.planning_pipelines[
                'isaac_ros_cumotion'
            ] = cumotion_config
            moveit_config.planning_pipelines[
                'default_planning_pipeline'
            ] = 'isaac_ros_cumotion'

        move_group = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': config.use_sim_time},
            ],
            arguments=['--ros-args', '--log-level', config.log_level],
        )
        return move_group, moveit_config

    def get_rviz_node(self, moveit_config) -> Node:
        """Return RViz configured with the same MoveIt model."""
        return Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_moveit',
            output='screen',
            arguments=['-d', self.driver_config.rviz_config_file],
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': self.driver_config.use_sim_time},
            ],
        )

    def get_vendor_driver_launch(self):
        """Return the official ARX single-arm driver launch action."""
        driver_launch = os.path.join(
            get_package_share_directory('arx_r5_controller'),
            'launch',
            'open_single_arm.launch.py',
        )
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch)
        )

    def get_cumotion_actions(self):
        """Return the Isaac ROS 4.5 cuMotion planner launch action."""
        cumotion_share = get_package_share_directory('isaac_ros_cumotion')
        config = self.driver_config
        launch_arguments = {
            'cumotion_action_server.xrdf_file_path': (
                config.cumotion_xrdf_file_path
            ),
            'cumotion_action_server.urdf_file_path': (
                config.cumotion_urdf_file_path
            ),
            'cumotion_action_server.tool_frame': config.cumotion_tool_frame,
            'cumotion_action_server.time_dilation_factor': (
                config.cumotion_time_dilation_factor
            ),
            'cumotion_action_server.read_esdf_world': str(
                config.read_esdf_world
            ),
            'cumotion_action_server.add_ground_plane': 'False',
            'cumotion_action_server.override_moveit_scaling_factors': 'False',
        }

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        cumotion_share,
                        'launch',
                        'isaac_ros_cumotion.launch.py',
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )
        ]
