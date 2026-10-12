<div align="center">

# ARX R5A × Isaac ROS cuMotion

**GPU-accelerated motion planning for the ARX R5A arm with MoveIt 2 and ros2_control**

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Isaac ROS 4.5](https://img.shields.io/badge/Isaac%20ROS-4.5-76B900?logo=nvidia)](https://nvidia-isaac-ros.github.io/v/release-4.5/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-1E5AA8)](https://moveit.ai/)
[![Platform](https://img.shields.io/badge/platform-Jetson%20AGX%20Thor%20%7C%20Ubuntu%2024.04-informational)](#requirements)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](LICENSE)

English | [简体中文](README.zh-CN.md)

</div>

## Overview

This repository connects a physical **ARX R5A** to **NVIDIA Isaac ROS cuMotion**.
Plan in RViz or through the MoveIt API, and the trajectory is executed on the real
arm through ros2_control. The ARX vendor driver keeps ownership of CAN
communication; this repository supplies the robot model, planning configuration,
and a hardware adapter that speaks the vendor's ROS 2 topics.

```text
RViz / MoveIt client → move_group → Isaac ROS cuMotion → trajectory
                          ↓
               joint_trajectory_controller
                          ↓
                    ArxR5aSystem  ── /arm_cmd ──▶  ARX vendor driver ↔ CAN ↔ R5A
                                  ◀─ /arm_status ─
```

**Features**

- cuMotion registered as MoveIt's default planning pipeline, with OMPL as an option
- URDF, SRDF, and XRDF (collision spheres, tool frames) for the R5A
- `ArxR5aSystem` ros2_control plugin with command rate and lead limiting
- Arm trajectory and `GripperCommand` actions
- A plan-only CLI to confirm that requests are served by cuMotion

> Community project, not affiliated with ARX Robotics or NVIDIA.
> Camera perception, nvblox, and pick-and-place live in the separate
> [manipulation overlay](https://github.com/wee733/isaac_ros_manipulation_arx_r5a).

## Requirements

| Component | Version |
|---|---|
| Hardware | ARX R5A, Jetson AGX Thor (reference) or another CUDA-capable NVIDIA GPU |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy |
| Isaac ROS | 4.5, with `isaac_ros_cumotion` and `isaac_ros_cumotion_moveit` |
| ARX driver | [ARXroboticsX/R5](https://github.com/ARXroboticsX/R5) ROS 2 packages `arx5_arm_msg` and `arx_r5_controller` |

## Quick Start

### 1. Prepare dependencies

1. Set up the [Isaac ROS 4.5 environment](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/index.html)
   and install cuMotion following the
   [cuMotion MoveIt quickstart](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html).
2. Clone [ARXroboticsX/R5](https://github.com/ARXroboticsX/R5) (for example to
   `~/workspaces/R5`) and make `ROS2/R5_ws/src/arxmsgros2` and
   `ROS2/R5_ws/src/ARX_R5_ros2_V7` available to your workspace. Before building it,
   apply two changes:
   - `arx_r5_controller/src/R5Controller.cpp`: the four `joint_pos` / `joint_vel`
     copy loops use `i <= 7` on seven-element arrays. Change them to `i < 7`.
   - On Jetson Thor, `can1` collides with the native CAN interface. Set the
     interface to `slcan1` in both `ARX_CAN/arx_can/arx_can1.sh` and
     `arx_r5_controller/config/single_arm.yaml` (`arm_can_id`).

### 2. Build

```bash
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
git clone https://github.com/wee733/Isaac_Ros_CuMotion_ArxR5a.git \
  "$ISAAC_ROS_WS/src/Isaac_Ros_CuMotion_ArxR5a"

cd "$ISAAC_ROS_WS"
isaac-ros activate          # enter the Isaac ROS 4.5 environment
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

### 3. Run

**Terminal 1 — CAN**

```bash
cd ~/workspaces/R5/ARX_CAN/arx_can
./arx_can1.sh
```

**Terminal 2 — ARX driver** (publishes `/arm_status`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

**Terminal 3 — MoveIt, cuMotion, and RViz**

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
source install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py cumotion_time_dilation_factor:=0.1
```

In RViz, select the `manipulator` planning group, drag the goal, then click
**Plan** or **Plan & Execute**. `0.1` is a conservative speed factor for first
runs; raise it toward `1.0` once motion looks right.

### 4. Verify cuMotion planning

```bash
ros2 control list_controllers
ros2 param get /move_group default_planning_pipeline   # isaac_ros_cumotion
ros2 run isaac_ros_manipulation_arx_r5a_driver_utils plan_joint_target \
  --joints 0.0 1.0 1.5 0.0 0.0 0.0
```

`plan_joint_target` requests a joint-space plan from the `isaac_ros_cumotion`
pipeline and prints the result as JSON. It never executes the trajectory and
exits nonzero if planning fails.

## Launch Options

| Argument | Default | Description |
|---|---|---|
| `start_vendor_driver` | `False` | Also launch the ARX driver from this launch file |
| `start_cumotion` | `True` | Start the cuMotion action server |
| `enable_cumotion_moveit_plugin` | follows `start_cumotion` | Use cuMotion as MoveIt's default pipeline |
| `start_rviz` | `True` | Start RViz with the MoveIt panel |
| `read_esdf_world` | `False` | Use an nvblox ESDF for obstacle avoidance |
| `cumotion_time_dilation_factor` | `1.0` | Trajectory speed factor in `(0, 1]` |
| `cumotion_tool_frame` | `link6` | Tool frame; must be listed in the XRDF |

Use `start_cumotion:=False` for OMPL only, or
`start_cumotion:=False enable_cumotion_moveit_plugin:=True` when another launch
owns the cuMotion server. Run with `--show-args` for file-path overrides.

## Interfaces

| Interface | Value |
|---|---|
| Planning group | `manipulator` (`joint1`–`joint6`, radians) |
| Base / tool frame | `base_link` / `link6`; `grasp_frame` for object attachment |
| Arm action | `/manipulator_controller/follow_joint_trajectory` |
| Gripper action | `/gripper_controller/gripper_cmd` (`0.0` closed, `0.044` m open) |
| Vendor topics | `/arm_status` (`RobotStatus`), `/arm_cmd` (`RobotCmd`) |

## Repository Layout

| Package | Contents |
|---|---|
| [`isaac_ros_manipulation_arx_r5a_robot_description`](isaac_ros_manipulation_arx_r5a_robot_description) | URDF/xacro, meshes, SRDF, XRDF, MoveIt and controller configs |
| [`isaac_ros_manipulation_arx_r5a_ros2_control`](isaac_ros_manipulation_arx_r5a_ros2_control) | `ArxR5aSystem` hardware plugin (C++) |
| [`isaac_ros_manipulation_arx_r5a_driver_utils`](isaac_ros_manipulation_arx_r5a_driver_utils) | Launch file, launch helpers, overlay params, `plan_joint_target` |

`r5a.urdf` is used by MoveIt, with velocity limits from `config/joint_limits.yaml`.
`r5a_cumotion.urdf` is the same model with those limits written in, because
cuMotion reads them from the URDF. A unit test keeps the two in sync.

## Contributing

Issues and pull requests are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) for
code style and the checks to run.

## License

[BSD 3-Clause](LICENSE). The robot description and meshes are derived from
[ARXroboticsX/R5](https://github.com/ARXroboticsX/R5) (BSD 3-Clause). NVIDIA
Isaac ROS and the ARX SDK are external dependencies under their own licenses.
