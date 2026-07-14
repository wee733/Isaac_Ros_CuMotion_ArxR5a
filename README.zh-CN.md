# ARX R5A 的 Isaac ROS 4.5 Manipulation 适配

[English](README.md)

本仓库用于在真实 ARX R5A 上运行 ros2_control、MoveIt 2、RViz 和 NVIDIA
Isaac ROS cuMotion。CAN 通信仍由 ARX 厂商驱动负责；本仓库提供机器人模型、
硬件适配器和规划配置。

已在 Jetson AGX Thor、Ubuntu 24.04、ROS 2 Jazzy 和 Isaac ROS 4.5.0 上验证。

> 这是社区适配项目，并非 ARX Robotics 或 NVIDIA 官方发布。

## 快速启动

以下命令按照已验证的默认目录编写，并假设工作区已经完成编译：

```text
~/workspaces/isaac_ros-dev
~/workspaces/R5
```

### 终端 1：启动 CAN

```bash
cd ~/workspaces/R5/ARX_CAN/arx_can
./arx_can1.sh
```

保持该终端运行。已验证配置为 `/dev/arxcan1`、SocketCAN 接口 `slcan1`，
厂商配置为 `arm_can_id: slcan1`。不要再启动第二个 `slcand`。

### 终端 2：启动 ARX 驱动

厂商 C++ 驱动运行在宿主机，不需要进入 `isaac-ros activate`。

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

保持该终端运行。驱动正常时会持续发布 `/arm_status`。

### 终端 3：启动 MoveIt、cuMotion 和 RViz

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
```

终端提示符变为 `(isaac-ros)` 后执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  cumotion_time_dilation_factor:=0.1
```

RViz 会自动启动。选择 `manipulator` 规划组，设置目标后点击 **Plan** 或
**Plan & Execute**。已验证的初始速度因子是 `0.1`，之后可以提高到 `1.0`。

### 验证

在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 topic hz /arm_status
ros2 control list_controllers
ros2 action list -t | grep -E 'cumotion|follow_joint_trajectory|gripper_cmd'
```

`joint_state_broadcaster`、`manipulator_controller` 和 `gripper_controller`
都应显示为 `active`。

## 编译

将本仓库和所需的 ARX ROS 2 包放入 `~/workspaces/isaac_ros-dev/src`，然后执行：

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
```

进入 Isaac ROS 环境后：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

## 常用参数

默认 launch 已经启用 cuMotion 和 RViz。常用覆盖参数如下：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `start_vendor_driver` | `False` | 是否从该 launch 启动 ARX 驱动 |
| `start_cumotion` | `True` | 是否使用 cuMotion 规划机械臂 |
| `start_rviz` | `True` | 是否启动 MoveIt RViz |
| `read_esdf_world` | `False` | 是否读取 nvblox ESDF 障碍物世界 |
| `cumotion_time_dilation_factor` | `1.0` | 轨迹速度因子 `(0, 1]` |

默认不需要设置 ROS domain。如果本机还有其他 DDS 系统，可在终端 2 和终端 3
启动前同时设置：

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 接口

- 规划组：`manipulator`（`joint1` 到 `joint6`）
- cuMotion 末端坐标系：`link6`
- ARX 话题：`/arm_status`、`/arm_cmd`
- 机械臂 Action：`/manipulator_controller/follow_joint_trajectory`
- 夹爪 Action：`/gripper_controller/gripper_cmd`
- 夹爪位置：张开 `0.044`、中间 `0.015`、闭合 `0.0` 米

## 常见问题

- `START_STATE_INVALID`，并提示 `joint2` 超出 `[0, 3]`：拉取当前分支、重新
  编译并 source 工作区。当前 MoveIt 和 cuMotion 模型已允许零位附近反馈达到
  `-0.01 rad`。
- 没有 `/arm_status`：检查 `slcan1`、`arm_can_id`，并确认只有一个 `slcand`。
- MoveIt 看不到驱动：两个 ROS 终端必须使用相同的 domain、发现范围和 RMW。
- KDL 的 `base_link has an inertia` 警告不会阻止当前固定基座模型进行规划。

完整的实机分阶段检查见 [SAFETY_VALIDATION.md](SAFETY_VALIDATION.md)。

## 许可

本项目采用 BSD 3-Clause License，第三方来源见 [NOTICE.md](NOTICE.md)。
