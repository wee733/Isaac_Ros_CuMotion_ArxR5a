# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Validate XRDF constraints required by Isaac ROS cuMotion 4.5."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
XRDF_PATH = PACKAGE_ROOT / 'xrdf' / 'r5a.xrdf'
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'r5a_cumotion.urdf'


def _load_xrdf():
    with XRDF_PATH.open(encoding='utf-8') as xrdf_file:
        return yaml.safe_load(xrdf_file)


def test_collision_sphere_radii_are_positive():
    """Require every XRDF collision sphere to have a positive radius."""
    xrdf = _load_xrdf()
    for geometry in xrdf['geometry'].values():
        for spheres in geometry['spheres'].values():
            assert all(sphere['radius'] > 0.0 for sphere in spheres)


def test_mimic_joints_have_no_default_position():
    """Keep URDF mimic joints out of XRDF auxiliary default positions."""
    urdf = ET.parse(URDF_PATH).getroot()
    mimic_joints = {
        joint.attrib['name']
        for joint in urdf.findall('joint')
        if joint.find('mimic') is not None
    }
    default_joints = set(_load_xrdf()['default_joint_positions'])
    assert mimic_joints.isdisjoint(default_joints)


def test_pick_and_place_attachment_frames_exist():
    """Require the ROS TF and XRDF frames used by object attachment."""
    xrdf = _load_xrdf()
    add_frames = [
        modifier['add_frame']
        for modifier in xrdf['modifiers']
        if 'add_frame' in modifier
    ]
    attached_object = next(
        frame for frame in add_frames
        if frame['frame_name'] == 'attached_object'
    )
    assert attached_object['parent_frame_name'] == 'grasp_frame'
    assert xrdf['tool_frames'][0] == 'link6'
    assert xrdf['collision']['buffer_distance']['attached_object'] > 0.0

    for urdf_path in (
        PACKAGE_ROOT / 'urdf' / 'r5a.urdf',
        PACKAGE_ROOT / 'urdf' / 'r5a_cumotion.urdf',
    ):
        urdf = ET.parse(urdf_path).getroot()
        grasp_joint = next(
            joint for joint in urdf.findall('joint')
            if joint.attrib['name'] == 'grasp_frame_joint'
        )
        assert grasp_joint.attrib['type'] == 'fixed'
        assert grasp_joint.find('parent').attrib['link'] == 'link6'
        assert grasp_joint.find('child').attrib['link'] == 'grasp_frame'


def test_gripper_collision_spheres_cover_both_fingers():
    """Keep both moving fingers in robot segmentation and collision checking."""
    xrdf = _load_xrdf()
    sphere_groups = xrdf['geometry']['r5a_collision_spheres']['spheres']
    assert sphere_groups['link7']
    assert sphere_groups['link8']
