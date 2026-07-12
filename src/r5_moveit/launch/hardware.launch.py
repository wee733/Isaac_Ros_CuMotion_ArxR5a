from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("r5_moveit")
    urdf_xacro = PathJoinSubstitution([share, "config", "R5a.urdf.xacro"])
    init_yaml = PathJoinSubstitution([share, "config", "initial_positions.yaml"])
    controllers_yaml = PathJoinSubstitution([share, "config", "ros2_controllers.yaml"])

    robot_description = {
        "robot_description": Command(
            ["xacro ", urdf_xacro, " initial_positions_file:=", init_yaml]
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": False}],
        output="screen",
    )

    def spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "30",
            ],
            output="screen",
        )

    return LaunchDescription(
        [
            rsp,
            controller_manager,
            spawner("joint_state_broadcaster"),
            spawner("manipulator_controller"),
            spawner("gripper_controller"),
        ]
    )
