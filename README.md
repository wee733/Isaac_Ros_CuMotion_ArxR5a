# Isaac ROS 4.5 Manipulation for ARX R5A

[中文说明](README.zh-CN.md)

Community integration for running a physical ARX R5A with ros2_control,
MoveIt 2, RViz, and NVIDIA Isaac ROS cuMotion. The ARX vendor driver continues
to own CAN communication; this repository supplies the robot model, hardware
adapter, and planning configuration.

Tested on Jetson AGX Thor, Ubuntu 24.04, ROS 2 Jazzy, and Isaac ROS 4.5.0.

> This is a community integration, not an official ARX Robotics or NVIDIA
> release.

## Quick Start

These commands assume the tested layout below and an already-built workspace:

```text
~/workspaces/isaac_ros-dev
~/workspaces/R5
```

### Terminal 1 — CAN

```bash
cd ~/workspaces/R5/ARX_CAN/arx_can
./arx_can1.sh
```

Keep it running. The tested configuration uses `/dev/arxcan1`, SocketCAN
interface `slcan1`, and `arm_can_id: slcan1`. Do not start a second `slcand`.

### Terminal 2 — ARX driver

The vendor C++ driver runs on the host and does not need `isaac-ros activate`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

Keep it running. A working driver continuously publishes `/arm_status`.

### Terminal 3 — MoveIt, cuMotion, and RViz

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
```

After the prompt changes to `(isaac-ros)`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  cumotion_time_dilation_factor:=0.1
```

RViz starts automatically. Select planning group `manipulator`, choose a
target, then click **Plan** or **Plan & Execute**. The tested startup speed
factor is `0.1`; it can be raised to `1.0` later.

### Verify

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 topic hz /arm_status
ros2 control list_controllers
ros2 action list -t | grep -E 'cumotion|follow_joint_trajectory|gripper_cmd'
```

`joint_state_broadcaster`, `manipulator_controller`, and `gripper_controller`
should all be `active`.

## Build

Place this repository and the required ARX ROS 2 packages in
`~/workspaces/isaac_ros-dev/src`, then run:

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
```

Inside the Isaac ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

## Useful Options

The launch defaults already enable cuMotion and RViz. Common overrides are:

| Argument | Default | Purpose |
|---|---:|---|
| `start_vendor_driver` | `False` | Start the ARX driver from this launch |
| `start_cumotion` | `True` | Start the standalone cuMotion action server |
| `enable_cumotion_moveit_plugin` | value of `start_cumotion` | Make cuMotion the default MoveIt planning pipeline |
| `start_rviz` | `True` | Start MoveIt RViz |
| `read_esdf_world` | `False` | Read an nvblox ESDF obstacle world |
| `cumotion_time_dilation_factor` | `1.0` | Trajectory speed factor `(0, 1]` |

Keeping the plugin default coupled to `start_cumotion` preserves the existing
`start_cumotion:=False` OMPL-only workflow. Set the two arguments explicitly
when a separate launch owns the cuMotion server or calls its action directly.

No custom ROS domain is required. If the machine has another DDS system, set
the following in both Terminal 2 and Terminal 3 before launching:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Interfaces

- Planning group: `manipulator` (`joint1` through `joint6`)
- cuMotion tool frame: `link6`
- ARX topics: `/arm_status`, `/arm_cmd`
- Arm action: `/manipulator_controller/follow_joint_trajectory`
- Gripper action: `/gripper_controller/gripper_cmd`
- Gripper positions: open `0.044`, intermediate `0.015`, closed `0.0` metres

## Complete Manipulation Overlay

The maintained camera, perception, behavior-tree, nvblox, and pick-and-place
workflow lives in the separate patch-free overlay repository:

https://github.com/wee733/isaac_ros_manipulation_arx_r5a

This repository remains responsible for the ARX robot description, XRDF,
ros2_control hardware, MoveIt integration, and standalone cuMotion operation.

## Troubleshooting

- `START_STATE_INVALID` with `joint2` outside `[0, 3]`: pull the current branch,
  rebuild, and source the workspace. The current MoveIt and cuMotion models
  allow the measured near-home tolerance down to `-0.01 rad`.
- No `/arm_status`: check `slcan1`, `arm_can_id`, and that only one `slcand`
  process exists.
- MoveIt cannot see the driver: both ROS terminals must use the same domain,
  discovery range, and RMW implementation.
- The KDL `base_link has an inertia` warning does not block this fixed-base
  model from planning.

Detailed staged hardware checks are in
[SAFETY_VALIDATION.md](SAFETY_VALIDATION.md).

## License

BSD 3-Clause. See [NOTICE.md](NOTICE.md) for third-party attribution.
