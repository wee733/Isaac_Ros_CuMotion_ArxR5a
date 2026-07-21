# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Tests for the cuMotion launch flag compatibility contract."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
    """Keep legacy defaults with independent server/plugin ownership."""
    assert _resolve_cumotion_flags() == ('True', 'True')
    assert _resolve_cumotion_flags('False') == ('False', 'False')
    assert _resolve_cumotion_flags('False', 'True') == ('False', 'True')
    assert _resolve_cumotion_flags('True', 'False') == ('True', 'False')
