from glob import glob

from setuptools import setup


package_name = 'R5a'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name + '/urdf/', glob('urdf/*')),
        ('share/' + package_name + '/rviz/', glob('rviz/*')),
        (
            'share/' + package_name + '/meshes/collision/',
            glob('meshes/collision/*'),
        ),
        (
            'share/' + package_name + '/meshes/visual/',
            glob('meshes/visual/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wee733',
    maintainer_email='wee733@users.noreply.github.com',
    description='URDF description and mesh assets for the ARX R5A.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
