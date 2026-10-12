# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Tests for the cuMotion launch flag compatibility contract."""

import importlib.util
from pathlib import Path

from isaac_ros_manipulation_arx_r5a_driver_utils import ArxR5aDriverConfig

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import pytest


LAUNCH_FILE = (
    Path(__file__).parents[1] / 'launch' / 'arx_r5a_driver.launch.py'
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'arx_r5a_driver_launch', LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _resolve_cumotion_flags(start_override=None, plugin_override=None):
    module = _load_launch_module()
    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()
    if start_override is not None:
        context.launch_configurations['start_cumotion'] = start_override
    if plugin_override is not None:
        context.launch_configurations[
            'enable_cumotion_moveit_plugin'
        ] = plugin_override

    declarations['start_cumotion'].execute(context)
    declarations['enable_cumotion_moveit_plugin'].execute(context)
    return (
        context.perform_substitution(LaunchConfiguration('start_cumotion')),
        context.perform_substitution(
            LaunchConfiguration('enable_cumotion_moveit_plugin')
        ),
    )


def test_cumotion_launch_flag_combinations():
    """Plugin follows start_cumotion by default and can be set independently."""
    assert _resolve_cumotion_flags() == ('True', 'True')
    assert _resolve_cumotion_flags('False') == ('False', 'False')
    assert _resolve_cumotion_flags('False', 'True') == ('False', 'True')
    assert _resolve_cumotion_flags('True', 'False') == ('True', 'False')


@pytest.mark.parametrize('factor', ['0', '-0.1', '1.1', 'nan', 'inf', 'fast'])
def test_invalid_speed_factor_is_rejected(factor):
    """Reject invalid speed settings before creating any launch actions."""
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations['cumotion_time_dilation_factor'] = factor
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    with pytest.raises(ValueError):
        ArxR5aDriverConfig(context)


def test_default_config_and_standalone_xacro():
    """Resolve installed paths and expand xacro without a working-directory default."""
    from isaac_ros_manipulation_arx_r5a_driver_utils.robot_description import (
        get_robot_description_contents,
    )
    import xacro

    context = LaunchContext()
    for entity in _load_launch_module().generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    config = ArxR5aDriverConfig(context)
    assert config.cumotion_time_dilation_factor == '1.0'
    assert config.start_cumotion and config.enable_cumotion_moveit_plugin
    explicit = get_robot_description_contents(
        config.urdf_path, config.initial_positions_file_path
    )
    assert xacro.process_file(config.urdf_path).toxml() == explicit


def test_cumotion_tool_frame_must_match_xrdf(monkeypatch):
    """Reject a frame override that the upstream launch would silently ignore."""
    from isaac_ros_manipulation_arx_r5a_driver_utils import ArxR5aDriverUtils
    from isaac_ros_manipulation_arx_r5a_driver_utils import arx_r5a_driver_utils

    context = LaunchContext()
    for entity in _load_launch_module().generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    config = ArxR5aDriverConfig(context)
    arx = ArxR5aDriverUtils(config)
    monkeypatch.setattr(
        arx_r5a_driver_utils, 'get_package_share_directory', lambda _: '/unused'
    )
    arguments = dict(arx.get_cumotion_actions()[0].launch_arguments)
    assert 'cumotion_action_server.tool_frame' not in arguments
    assert arguments['cumotion_action_server.xrdf_file_path'] == config.cumotion_xrdf_file_path
    config.cumotion_tool_frame = 'missing_tool'
    with pytest.raises(ValueError, match='missing_tool'):
        arx.get_cumotion_actions()
