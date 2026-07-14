# ARX R5A 的 Isaac ROS Manipulation 适配

[English](README.md)

本仓库按照 NVIDIA Isaac ROS Manipulation 的 “Bring Your Own Robot” 结构，
为 ARX R5A 提供 ros2_control、MoveIt 2 和 GPU cuMotion 规划支持。底层
USB/CAN 通信继续使用 ARX 官方驱动，本仓库不复制或重写厂商驱动。

> [!IMPORTANT]
> 这是社区适配项目，并非 ARX Robotics 或 NVIDIA 官方发布。当前已在
> ROS 2 Jazzy 与 Isaac ROS 4.5.0 上验证，并要求 Isaac ROS 4.5.0 或更高版本。

> [!CAUTION]
> 不要把“启动厂商驱动”当作只读联通测试。本机所用 ARX 厂商库在初始化时会
> 使能电机并执行回零流程，机械臂可能在 ROS 话题出现之前就开始运动。
> 此外，即使 `start_vendor_driver:=False`，本 launch 仍会激活 ros2_control；
> 只要收到新鲜的 `/arm_status`，适配器就会以 `mode=5`（位置控制）持续发布
> `/arm_cmd`，机械臂可能立即上力保持。首次接入前必须阅读
> [安全启动与分阶段验收](SAFETY_VALIDATION.md)。

## 软件结构

```text
MoveIt API / RViz
        ↓
move_group → isaac_ros_cumotion_moveit → cuMotion/cuRobo GPU
        ↓
JointTrajectoryController
        ↓
ArxR5aSystem → /arm_cmd、/arm_status → ARX 官方驱动 → CAN
```

MoveIt 仍然负责标准规划接口、场景和轨迹执行。启用 cuMotion 后，六轴机械臂
使用 NVIDIA GPU 规划；OMPL 作为备用管线，夹爪通过独立 Action 控制。

## 包结构

```text
isaac_ros_manipulation_arx_r5a_robot_description/
  urdf/   meshes/   srdf/   xrdf/   config/

isaac_ros_manipulation_arx_r5a_ros2_control/
  ARX 话题与 ros2_control 之间的硬件适配器

isaac_ros_manipulation_arx_r5a_driver_utils/
  配置类、机器人启动工具、launch 和工作流参数
```

ARX 官方 [R5 仓库](https://github.com/ARXroboticsX/R5) 是外部依赖，负责
发布 `/arm_status`、接收 `/arm_cmd` 并通过 CAN 控制电机。

## 编译

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

如果本仓库本身就是 colcon 工作区根目录，将 `--from-paths src` 改为
`--from-paths .`。

## CAN 与官方驱动

总线接口名必须和厂商驱动的 `arm_can_id` 完全一致，速率为 1 Mbit/s。Jetson
AGX Thor 自带名为 `can0`、`can1` 等的原生 CAN 接口，因此 USB SLCAN 不应再
尝试占用 `can1`；可以使用 `slcan1`，并同步修改厂商驱动配置。

USB SLCAN 示例：

```bash
sudo slcand -o -f -s8 /dev/arxcan1 slcan1
sudo ip link set slcan1 up
ip -details -statistics link show slcan1
```

使用 Thor 原生 CAN 时，应按平台说明配置对应接口为 1 Mbit/s，再确认物理
收发器、终端电阻和实际接线。不要同时猜测使用原生 `can1` 和 USB `slcan1`。

> [!WARNING]
> 以下厂商 launch 会使能电机并可能自动回零，只能在人员现场、工作区清空、
> 急停可立即触发且已确认回零路径安全时运行。它不是状态只读命令。

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
ros2 topic hz /arm_status
```

首次连接不要使用 `start_vendor_driver:=True` 把所有节点一次启动；先按
[安全启动与分阶段验收](SAFETY_VALIDATION.md) 分别验证 CAN、厂商初始化、
位置保持和小幅运动。

## 启动 MoveIt 与 cuMotion

在 Isaac ROS 环境中执行：

```bash
isaac-ros activate
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/workspace/install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py
```

默认参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `start_vendor_driver` | `False` | 是否同时启动 ARX 官方驱动 |
| `start_cumotion` | `True` | 是否将 cuMotion 设为默认机械臂规划器 |
| `start_rviz` | `True` | 是否启动 MoveIt RViz |
| `read_esdf_world` | `False` | 是否读取 nvblox 动态障碍物 |
| `cumotion_time_dilation_factor` | `1.0` | 保持已验证的 cuMotion 全速轨迹时序 |

`start_vendor_driver=False` 只表示本 launch 不负责创建厂商节点，并不等于
“不会向实体机械臂发命令”。如果同一 ROS 域中已经有厂商驱动订阅 `/arm_cmd`，
且任意节点或 rosbag 正在发布 `/arm_status`，适配器会进入位置保持输出。
离线测试必须使用与实体驱动隔离的 `ROS_DOMAIN_ID`。

常用模式：

```bash
# 只使用 MoveIt + OMPL
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_cumotion:=False

# 仅限已完成有人监护验收的系统：会使能并可能自动回零
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_vendor_driver:=True

# 接入 nvblox ESDF 动态避障
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py read_esdf_world:=True
```

## 规划组与接口

- `manipulator`：`joint1` 到 `joint6`，默认使用 `isaac_ros_cumotion`。
- cuMotion 末端坐标系：`link6`。
- MoveIt 控制器 Action：`manipulator_controller/follow_joint_trajectory`。
- 夹爪不进入 MoveIt 规划，通过独立的 `gripper_controller/gripper_cmd`
  Action 控制。
- 夹爪位置：张开 `0.044`、中间 `0.015`、闭合 `0.0` 米。

完成实机分阶段验收后，夹爪可以独立于 MoveIt 控制。该 action 会产生实体运动：

```bash
# 张开；中间夹持改为 0.015，闭合改为 0.0。
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.044, max_effort: 0.0}}"
```

`xrdf/r5a.xrdf` 包含关节空间、加速度与 jerk 限制、工具坐标系、碰撞球和
自碰撞忽略规则。更换相机、负载、夹爪或机械结构后必须重新校验。

## 旧结构迁移

```text
r5_moveit         → robot_description + driver_utils
arx5_ros2_control → isaac_ros_manipulation_arx_r5a_ros2_control
```

规划组、控制器 Action、ARX 话题、关节限制、XRDF、夹爪状态和执行容差均未改变。

## 安全与许可

执行前清空工作区、确认关节反馈和急停，从小幅运动开始验证。当前
`RobotStatus` 不包含急停、使能或故障码，适配器也不能验证这些状态；正常关闭时
发送一次保护命令不能替代硬件急停。默认 cuMotion 配置不读取 ESDF，并关闭地面
平面，因此桌面、地面和现场障碍物不会自动进入规划。

完整的隔离离线启动、被动 CAN 检查、有人监护厂商回零、位置保持、小幅轨迹和
cuMotion 执行门槛见 [安全启动与分阶段验收](SAFETY_VALIDATION.md)。GPU 规划和
碰撞模型不能替代现场监护与硬件限位。本项目采用 BSD 3-Clause License，第三方
来源见 [NOTICE.md](NOTICE.md)。
