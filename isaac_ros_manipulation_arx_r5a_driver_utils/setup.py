# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

from glob import glob

from setuptools import setup


package_name = 'isaac_ros_manipulation_arx_r5a_driver_utils'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wee733',
    maintainer_email='wee733@users.noreply.github.com',
    description=(
        'Official-style launch and configuration utilities for ARX R5A '
        'manipulation.'
    ),
    license='BSD-3-Clause',
    tests_require=['pytest'],
)
