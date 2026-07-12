from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description() -> LaunchDescription:
    # 1) MoveIt 基础配置（URDF/SRDF/kinematics/OMPL等）
    moveit_config = (
        MoveItConfigsBuilder("R5a", package_name="r5_moveit").to_moveit_configs()
    )

    # 2) 直接用字典把控制器映射喂给 move_group（等价于 moveit_controllers.yaml）
    controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["manipulator_controller", "gripper_controller"],
            "manipulator_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": ["joint1","joint2","joint3","joint4","joint5","joint6"],
            },
            "gripper_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "joints": ["joint7","joint8"],
            },
        },
    }

    # 3) move_group（带上机器人模型+控制器映射）
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), controllers, {"use_sim_time": False}],
    )

    # 4) RViz2（同样注入机器人模型参数 + 自带的 rviz 配置）
    rviz_config = PathJoinSubstitution([FindPackageShare("r5_moveit"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    return LaunchDescription([move_group_node, rviz_node])

