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
