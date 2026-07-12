# Isaac ROS cuMotion for ARX R5A

[中文说明](README.zh-CN.md)

ROS 2 control, MoveIt 2, and NVIDIA Isaac ROS cuMotion integration for the
ARX R5A manipulator. The repository connects the official ARX ROS 2 driver to
`joint_trajectory_controller`, supports the six arm joints and two-finger
gripper, and provides a cuMotion XRDF collision model.

> [!IMPORTANT]
> This is a community project, not an official ARX Robotics or NVIDIA release.
> It has been tested with ROS 2 Jazzy and Isaac ROS 4.3.0. Isaac ROS 4.5 uses
> different cuMotion launch parameter names and is not yet validated here.

## Features

- Real-hardware `ros2_control` bridge for `/arm_status` and `/arm_cmd`
- MoveIt 2 planning and execution through OMPL
- cuMotion planning for `joint1` through `joint6`
- XRDF with collision spheres for the R5A model
- Gripper named states: `open`, `jia`, and `close`
- Optional nvblox ESDF world input

## Repository Layout

```text
src/R5a/                  ARX R5A description and meshes
src/arx5_ros2_control/    ros2_control hardware interface
src/r5_moveit/            MoveIt, cuMotion, XRDF, and launch configuration
```

## Prerequisites

- ARX R5A and a configured CAN adapter
- Ubuntu 24.04 with ROS 2 Jazzy and MoveIt 2
- NVIDIA GPU supported by Isaac ROS
- Isaac ROS 4.3.0 with `isaac_ros_cumotion` and
  `isaac_ros_cumotion_moveit`
- The [official ARX R5 repository](https://github.com/ARXroboticsX/R5), built
  to provide `arx_r5_controller` and `arx5_arm_msg`

Build the official driver first, then build this workspace in an activated
Isaac ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Hardware Setup

Confirm the adapter path before creating the CAN interface. The tested SLCAN
configuration uses channel `can1` and 1 Mbit/s (`-s8`):

```bash
sudo slcand -o -f -s8 /dev/arxcan1 can1
sudo ip link set can1 up
ip -details -statistics link show can1
```

Start the official driver in a dedicated terminal and verify fresh status:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
ros2 topic hz /arm_status
```

## Run with MoveIt 2

For OMPL planning without cuMotion:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source install/setup.bash
ros2 launch r5_moveit real_robot.launch.py
```

## Run with cuMotion

Keep the official driver running separately. In the Isaac ROS environment:

```bash
isaac-ros activate
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/this/repository/install/setup.bash
ros2 launch r5_moveit cumotion_real_robot.launch.py
```

RViz defaults to the `manipulator` group and `isaac_ros_cumotion` pipeline.
For the `gripper` group, select the `ompl` pipeline. cuMotion intentionally
controls only `joint1` through `joint6`; its tool frame is `link6`.

To consume an existing nvblox ESDF world:

```bash
ros2 launch r5_moveit cumotion_real_robot.launch.py read_esdf_world:=True
```

## Configuration Notes

- `src/r5_moveit/config/r5a.xrdf` defines cuMotion c-space and collision
  spheres. Revalidate it if the tool, camera, or robot geometry changes.
- `r5a_cumotion.urdf` is a cuMotion-specific URDF with usable velocity limits;
  the original description URDF remains separate.
- Joint limits and execution tolerances are starting points for the tested
  R5A. Validate them against your firmware and payload.
- The hardware interface suppresses commands until fresh `/arm_status` data is
  available and limits command lead relative to measured state.

## Troubleshooting

- `joint1 is not in list`: check `/joint_states` names and ensure the current
  hardware state is available before planning.
- `INVALID_START_STATE_SELF_COLLISION`: inspect the current state in RViz and
  validate the XRDF collision spheres for your hardware.
- Endpoint velocity rejected: restart the controller after changing
  `ros2_controllers.yaml`; cuMotion can leave tiny floating-point residuals.
- Execution timeout: compare the planned duration with real joint tracking
  before increasing the limits in `moveit_controllers.yaml`.

## Safety

Use an emergency stop, clear the workspace, verify joint feedback, and test
small motions before executing large trajectories. Collision models do not
replace physical supervision or hardware limits.

## License and Attribution

Released under the BSD 3-Clause License. The R5A description and driver-facing
interfaces are derived from the official ARX R5 project. See [NOTICE.md](NOTICE.md)
for third-party attribution.
