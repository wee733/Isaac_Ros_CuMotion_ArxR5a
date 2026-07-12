# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Official-style robot controller utilities for the ARX R5A."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from isaac_ros_manipulation_arx_r5a_driver_utils.robot_description import (
    get_robot_description_contents,
)

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from moveit_configs_utils import MoveItConfigsBuilder

import yaml


DESCRIPTION_PACKAGE = 'isaac_ros_manipulation_arx_r5a_robot_description'


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def _prepend_existing(paths, variable_name):
    values = [path for path in paths if os.path.exists(path)]
    current_value = os.environ.get(variable_name, '')
    if current_value:
        values.append(current_value)
    return os.pathsep.join(values)


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
        """Return environment and planner actions for installed cuMotion version."""
        actions = []
        cumotion_share = get_package_share_directory('isaac_ros_cumotion')
        package_xml = ET.parse(os.path.join(cumotion_share, 'package.xml'))
        version = package_xml.getroot().findtext('version', default='0.0.0')
        major_minor = tuple(int(part) for part in version.split('.')[:2])

        if (
            major_minor < (4, 5)
            and self.driver_config.configure_isaac_ros_43_environment
        ):
            actions.extend(self._get_isaac_ros_43_environment_actions())

        config = self.driver_config
        if major_minor >= (4, 5):
            launch_arguments = {
                'cumotion_action_server.xrdf_file_path': (
                    config.cumotion_xrdf_file_path
                ),
                'cumotion_action_server.urdf_file_path': (
                    config.cumotion_urdf_file_path
                ),
                'cumotion_action_server.tool_frame': config.cumotion_tool_frame,
                'cumotion_action_server.read_esdf_world': str(
                    config.read_esdf_world
                ),
                'cumotion_action_server.add_ground_plane': 'False',
                'cumotion_action_server.override_moveit_scaling_factors': 'False',
            }
        else:
            launch_arguments = {
                'cumotion_planner.robot': config.cumotion_xrdf_file_path,
                'cumotion_planner.urdf_path': config.cumotion_urdf_file_path,
                'cumotion_planner.tool_frame': config.cumotion_tool_frame,
                'cumotion_planner.read_esdf_world': str(config.read_esdf_world),
                'cumotion_planner.add_ground_plane': 'False',
                'cumotion_planner.override_moveit_scaling_factors': 'False',
            }

        actions.append(
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
        )
        return actions

    @staticmethod
    def _get_isaac_ros_43_environment_actions():
        isaac_ros_venv = '/var/lib/isaac-ros-cli/isaac-ros'
        venv_site_packages = os.path.join(
            isaac_ros_venv,
            'lib',
            'python3.12',
            'site-packages',
        )
        ros_overlay = os.path.join(
            isaac_ros_venv,
            'ros-deb-overlay',
            'opt',
            'ros',
            'jazzy',
        )
        system_utils = os.path.join(
            '/opt/ros/jazzy/lib/python3.12/site-packages',
            'isaac_manipulator_ros_python_utils',
        )
        overlay_paths = [] if os.path.exists(system_utils) else [ros_overlay]
        python_path = _prepend_existing(
            [
                *[
                    os.path.join(path, 'lib', 'python3.12', 'site-packages')
                    for path in overlay_paths
                ],
                venv_site_packages,
            ],
            'PYTHONPATH',
        )
        library_path = _prepend_existing(
            [os.path.join(path, 'lib') for path in overlay_paths],
            'LD_LIBRARY_PATH',
        )
        ament_prefix_path = _prepend_existing(
            overlay_paths,
            'AMENT_PREFIX_PATH',
        )
        return [
            SetEnvironmentVariable('PYTHONPATH', python_path),
            SetEnvironmentVariable('LD_LIBRARY_PATH', library_path),
            SetEnvironmentVariable('AMENT_PREFIX_PATH', ament_prefix_path),
        ]
