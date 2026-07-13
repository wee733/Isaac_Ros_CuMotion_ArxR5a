# Isaac ROS Manipulation Support for ARX R5A

[中文说明](README.zh-CN.md)

Official-style robot integration packages for running the ARX R5A with
ROS 2 control, MoveIt 2, and NVIDIA Isaac ROS cuMotion. The repository follows
the Isaac ROS Manipulation "Bring Your Own Robot" package boundaries while
reusing the official ARX ROS 2 driver for CAN communication.

> [!IMPORTANT]
> This is a community integration, not an official ARX Robotics or NVIDIA
> release. It targets ROS 2 Jazzy and Isaac ROS 4.5.0 or newer.

## Architecture

```text
MoveIt API / RViz
        |
        v
MoveIt move_group -- isaac_ros_cumotion_moveit -- cuMotion / cuRobo (GPU)
        |
        v
JointTrajectoryController
        |
        v
ArxR5aSystem -- /arm_cmd, /arm_status -- official ARX driver -- CAN
```

MoveIt remains the standard planning and execution API. With cuMotion enabled,
the six arm joints use NVIDIA's GPU planner and OMPL remains available as a
fallback. The gripper is controlled independently through a `GripperCommand`
action, matching the Isaac ROS Manipulation behavior-tree interface.

## Package Layout

```text
isaac_ros_manipulation_arx_r5a_robot_description/
  urdf/   meshes/   srdf/   xrdf/   config/

isaac_ros_manipulation_arx_r5a_ros2_control/
  ARX topic-to-ros2_control hardware adapter

isaac_ros_manipulation_arx_r5a_driver_utils/
  config class, robot utilities, launch files, and workflow parameters
```

The official [ARX R5 repository](https://github.com/ARXroboticsX/R5) remains an
external dependency. Its driver owns USB/CAN communication and publishes
`/arm_status` while accepting `/arm_cmd`.

## Requirements

- ARX R5A with a configured CAN adapter and emergency stop
- Ubuntu 24.04 and ROS 2 Jazzy
- MoveIt 2 and ros2_control
- NVIDIA GPU supported by Isaac ROS
- Isaac ROS 4.5 or newer, including cuMotion and the cuMotion MoveIt plugin
- Built ARX packages: `arx_r5_controller` and `arx5_arm_msg`

Build from the colcon workspace that contains this repository:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

When this repository itself is the workspace root, replace `--from-paths src`
with `--from-paths .`.

## CAN and Vendor Driver

The tested SLCAN configuration uses `can1` at 1 Mbit/s:

```bash
sudo slcand -o -f -s8 /dev/arxcan1 can1
sudo ip link set can1 up
ip -details -statistics link show can1
```

Run the official driver separately to keep its console output isolated:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
ros2 topic hz /arm_status
```

## Launch MoveIt and cuMotion

Inside the activated Isaac ROS environment:

```bash
isaac-ros activate
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/workspace/install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py
```

The launch defaults are:

| Argument | Default | Purpose |
|---|---:|---|
| `start_vendor_driver` | `False` | Include the official ARX driver launch |
| `start_cumotion` | `True` | Register cuMotion as the default arm planner |
| `start_rviz` | `True` | Start MoveIt RViz |
| `read_esdf_world` | `False` | Read dynamic obstacles from nvblox |
| `cumotion_time_dilation_factor` | `1.0` | Preserve the validated full-speed cuMotion timing |

Examples:

```bash
# MoveIt + OMPL only
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_cumotion:=False

# Include the official ARX driver in the same launch
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_vendor_driver:=True

# Enable nvblox ESDF obstacle avoidance
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py read_esdf_world:=True
```

## Planning Groups

- `manipulator`: `joint1` through `joint6`; defaults to
  `isaac_ros_cumotion` when enabled.
- cuMotion tool frame: `link6`.
- MoveIt controller action: `manipulator_controller/follow_joint_trajectory`.
- Independent gripper action: `gripper_controller/gripper_cmd`.
- Gripper positions: open `0.044`, intermediate `0.015`, closed `0.0` meters.

Control the gripper independently of MoveIt:

```bash
# Open; use 0.015 for an intermediate grasp or 0.0 to close.
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.044, max_effort: 0.0}}"
```

The XRDF in `xrdf/r5a.xrdf` contains the c-space, acceleration and jerk limits,
tool frame, collision spheres, and self-collision ignore rules. Revalidate it
after changing the camera, payload, gripper, or robot geometry.

## Migration from the Previous Layout

```text
r5_moveit                                  -> robot_description + driver_utils
arx5_ros2_control                          -> isaac_ros_manipulation_arx_r5a_ros2_control
ros2 launch r5_moveit cumotion_real_robot  -> ros2 launch ... arx_r5a_driver.launch.py
```

The arm planning group, ARX topics, joint limits, and XRDF are preserved. The
gripper is intentionally outside MoveIt and uses the `GripperCommand` action
expected by Isaac ROS Manipulation because the ARX driver exposes one physical
command and one feedback value.

## Safety

Clear the workspace, verify feedback, keep the emergency stop available, and
test small trajectories before large motions. Collision models and GPU
planning do not replace physical supervision or hardware limits.

## License

BSD 3-Clause. See [NOTICE.md](NOTICE.md) for ARX, MoveIt, and NVIDIA attribution.
