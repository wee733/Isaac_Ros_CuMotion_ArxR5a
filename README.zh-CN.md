# ARX R5A 的 Isaac ROS Manipulation 适配

[English](README.md)

本仓库按照 NVIDIA Isaac ROS Manipulation 的 “Bring Your Own Robot” 结构，
为 ARX R5A 提供 ros2_control、MoveIt 2 和 GPU cuMotion 规划支持。底层
USB/CAN 通信继续使用 ARX 官方驱动，本仓库不复制或重写厂商驱动。

> [!IMPORTANT]
> 这是社区适配项目，并非 ARX Robotics 或 NVIDIA 官方发布。当前已在
> ROS 2 Jazzy 与 Isaac ROS 4.3.0 上验证，同时能够根据已安装版本切换
> Isaac ROS 4.3/4.5 的 cuMotion 参数名称。

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
使用 NVIDIA GPU 规划；OMPL 作为备用管线，并用于双指夹爪规划。

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

已验证的 SLCAN 配置为 `can1`、1 Mbit/s：

```bash
sudo slcand -o -f -s8 /dev/arxcan1 can1
sudo ip link set can1 up
ip -details -statistics link show can1
```

建议单独终端运行官方驱动，避免其输出污染 MoveIt/cuMotion 日志：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
ros2 topic hz /arm_status
```

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
| `configure_isaac_ros_43_environment` | `True` | 补充 Isaac ROS 4.3 Python 路径 |

常用模式：

```bash
# 只使用 MoveIt + OMPL
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_cumotion:=False

# 同时启动 ARX 官方驱动
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py start_vendor_driver:=True

# 接入 nvblox ESDF 动态避障
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py read_esdf_world:=True
```

Isaac ROS 4.5 环境已经正确提供 Python 依赖时，可设置：

```bash
configure_isaac_ros_43_environment:=False
```

## 规划组与接口

- `manipulator`：`joint1` 到 `joint6`，默认使用 `isaac_ros_cumotion`。
- cuMotion 末端坐标系：`link6`。
- MoveIt 控制器 Action：`manipulator_controller/follow_joint_trajectory`。
- 夹爪不进入 MoveIt 规划，通过独立的 `gripper_controller/gripper_cmd`
  Action 控制。
- 夹爪位置：张开 `0.044`、中间 `0.015`、闭合 `0.0` 米。

夹爪可以独立于 MoveIt 控制：

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

执行前清空工作区、确认关节反馈和急停，从小幅运动开始验证。GPU 规划和碰撞模型
不能替代现场监护与硬件限位。本项目采用 BSD 3-Clause License，第三方来源见
[NOTICE.md](NOTICE.md)。
