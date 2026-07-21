# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch-time configuration for the ARX R5A driver stack."""

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration


def _get_string(context: LaunchContext, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _get_bool(context: LaunchContext, name: str) -> bool:
    value = _get_string(context, name).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'Launch argument {name} must be a boolean, got {value!r}')


class ArxR5aDriverConfig:
    """Resolved launch configuration consumed by :class:`ArxR5aDriverUtils`."""

    def __init__(self, context: LaunchContext):
        self.use_sim_time = _get_bool(context, 'use_sim_time')
        self.start_vendor_driver = _get_bool(context, 'start_vendor_driver')
        self.start_cumotion = _get_bool(context, 'start_cumotion')
        self.enable_cumotion_moveit_plugin = _get_bool(
            context, 'enable_cumotion_moveit_plugin'
        )
        self.start_rviz = _get_bool(context, 'start_rviz')
        self.read_esdf_world = _get_bool(context, 'read_esdf_world')

        self.urdf_path = _get_string(context, 'urdf_path')
        self.srdf_path = _get_string(context, 'srdf_path')
        self.initial_positions_file_path = _get_string(
            context, 'initial_positions_file_path'
        )
        self.joint_limits_file_path = _get_string(
            context, 'joint_limits_file_path'
        )
        self.kinematics_file_path = _get_string(
            context, 'kinematics_file_path'
        )
        self.moveit_controllers_file_path = _get_string(
            context, 'moveit_controllers_file_path'
        )
        self.ros2_controllers_file_path = _get_string(
            context, 'ros2_controllers_file_path'
        )
        self.rviz_config_file = _get_string(context, 'rviz_config_file')
        self.cumotion_urdf_file_path = _get_string(
            context, 'cumotion_urdf_file_path'
        )
        self.cumotion_xrdf_file_path = _get_string(
            context, 'cumotion_xrdf_file_path'
        )
        self.cumotion_tool_frame = _get_string(context, 'cumotion_tool_frame')
        self.cumotion_time_dilation_factor = _get_string(
            context, 'cumotion_time_dilation_factor'
        )
        self.log_level = _get_string(context, 'log_level')
