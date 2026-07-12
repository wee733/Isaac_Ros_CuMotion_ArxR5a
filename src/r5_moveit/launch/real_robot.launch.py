from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare("r5_moveit"), "launch"])

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "hardware.launch.py"])
        )
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [launch_dir, "move_group_with_controllers.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            hardware,
            TimerAction(period=3.0, actions=[move_group]),
        ]
    )
