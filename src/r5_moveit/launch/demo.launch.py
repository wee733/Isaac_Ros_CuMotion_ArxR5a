from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("R5a", package_name="r5_moveit").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs()
    return generate_demo_launch(moveit_config)
