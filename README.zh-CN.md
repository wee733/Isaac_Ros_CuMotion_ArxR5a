<div align="center">

# ARX R5A × Isaac ROS cuMotion

**基于 MoveIt 2 与 ros2_control，为 ARX R5A 机械臂提供 GPU 加速运动规划**

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Isaac ROS 4.5](https://img.shields.io/badge/Isaac%20ROS-4.5-76B900?logo=nvidia)](https://nvidia-isaac-ros.github.io/v/release-4.5/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-1E5AA8)](https://moveit.ai/)
[![Platform](https://img.shields.io/badge/platform-Jetson%20AGX%20Thor%20%7C%20Ubuntu%2024.04-informational)](#环境要求)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](LICENSE)

[English](README.md) | 简体中文

</div>

## 简介

本仓库将实体 **ARX R5A** 接入 **NVIDIA Isaac ROS cuMotion**。在 RViz 或通过 MoveIt
API 发起规划后，轨迹经 ros2_control 在真实机械臂上执行。CAN 通信仍由 ARX 厂商驱动
负责；本仓库提供机器人模型、规划配置，以及对接厂商 ROS 2 话题的硬件适配器。

```text
RViz / MoveIt 客户端 → move_group → Isaac ROS cuMotion → 轨迹
                          ↓
               joint_trajectory_controller
                          ↓
                    ArxR5aSystem  ── /arm_cmd ──▶  ARX 厂商驱动 ↔ CAN ↔ R5A
                                  ◀─ /arm_status ─
```

**功能**

- cuMotion 注册为 MoveIt 默认规划管线，可切换为 OMPL
- R5A 的 URDF、SRDF 与 XRDF（碰撞球、工具坐标系）
- `ArxR5aSystem` ros2_control 插件，带指令速度与领先量限制
- 机械臂轨迹 Action 与 `GripperCommand` 夹爪 Action
- 只规划不执行的命令行工具，用于确认请求由 cuMotion 完成

> 社区项目，与 ARX Robotics 和 NVIDIA 无隶属关系。
> 相机感知、nvblox 和抓放流程见独立的
> [manipulation overlay](https://github.com/wee733/isaac_ros_manipulation_arx_r5a)。

## 环境要求

| 组件 | 版本 |
|---|---|
| 硬件 | ARX R5A；Jetson AGX Thor（参考平台）或其他支持 CUDA 的 NVIDIA GPU |
| 系统 / ROS | Ubuntu 24.04、ROS 2 Jazzy |
| Isaac ROS | 4.5，含 `isaac_ros_cumotion` 与 `isaac_ros_cumotion_moveit` |
| ARX 驱动 | [ARXroboticsX/R5](https://github.com/ARXroboticsX/R5) 的 ROS 2 包 `arx5_arm_msg` 与 `arx_r5_controller` |

## 快速开始

### 1. 准备依赖

1. 按 [Isaac ROS 4.5 环境文档](https://nvidia-isaac-ros.github.io/v/release-4.5/getting_started/index.html)
   配置环境，再按 [cuMotion MoveIt 快速入门](https://nvidia-isaac-ros.github.io/v/release-4.5/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html)
   安装 cuMotion。
2. 克隆 [ARXroboticsX/R5](https://github.com/ARXroboticsX/R5)（例如放在 `~/workspaces/R5`），
   让工作区能找到 `ROS2/R5_ws/src/arxmsgros2` 和 `ROS2/R5_ws/src/ARX_R5_ros2_V7`。
   编译前做两处修改：
   - `arx_r5_controller/src/R5Controller.cpp`：拷贝 `joint_pos` / `joint_vel` 的四处循环
     对七元素数组使用了 `i <= 7`，改为 `i < 7`。
   - Jetson Thor 上 `can1` 与原生 CAN 接口冲突。将 `ARX_CAN/arx_can/arx_can1.sh` 和
     `arx_r5_controller/config/single_arm.yaml`（`arm_can_id`）中的接口都改为 `slcan1`。

### 2. 编译

```bash
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
git clone https://github.com/wee733/Isaac_Ros_CuMotion_ArxR5a.git \
  "$ISAAC_ROS_WS/src/Isaac_Ros_CuMotion_ArxR5a"

cd "$ISAAC_ROS_WS"
isaac-ros activate          # 进入 Isaac ROS 4.5 环境
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install \
  --packages-up-to isaac_ros_manipulation_arx_r5a_driver_utils
source install/setup.bash
```

### 3. 运行

**终端 1：CAN**

```bash
cd ~/workspaces/R5/ARX_CAN/arx_can
./arx_can1.sh
```

**终端 2：ARX 驱动**（发布 `/arm_status`）

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

**终端 3：MoveIt、cuMotion 与 RViz**

```bash
cd ~/workspaces/isaac_ros-dev
isaac-ros activate
source install/setup.bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py cumotion_time_dilation_factor:=0.1
```

在 RViz 中选择 `manipulator` 规划组，拖动目标后点击 **Plan** 或 **Plan & Execute**。
`0.1` 是首次运行的保守速度因子，确认运动正常后可逐步提高到 `1.0`。

### 4. 验证 cuMotion 规划

```bash
ros2 control list_controllers
ros2 param get /move_group default_planning_pipeline   # isaac_ros_cumotion
ros2 run isaac_ros_manipulation_arx_r5a_driver_utils plan_joint_target \
  --joints 0.0 1.0 1.5 0.0 0.0 0.0
```

`plan_joint_target` 向 `isaac_ros_cumotion` 管线请求一次关节空间规划，并以 JSON 输出结果。
它不会执行轨迹，规划失败时返回非零退出码。

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_vendor_driver` | `False` | 由本 launch 同时启动 ARX 驱动 |
| `start_cumotion` | `True` | 启动 cuMotion Action Server |
| `enable_cumotion_moveit_plugin` | 跟随 `start_cumotion` | 将 cuMotion 设为 MoveIt 默认管线 |
| `start_rviz` | `True` | 启动带 MoveIt 面板的 RViz |
| `read_esdf_world` | `False` | 使用 nvblox ESDF 进行避障 |
| `cumotion_time_dilation_factor` | `1.0` | 轨迹速度因子，范围 `(0, 1]` |
| `cumotion_tool_frame` | `link6` | 工具坐标系，必须在 XRDF 中列出 |

只用 OMPL 时设 `start_cumotion:=False`；cuMotion Server 由其他 launch 启动时用
`start_cumotion:=False enable_cumotion_moveit_plugin:=True`。文件路径覆盖参数见 `--show-args`。

## 接口

| 接口 | 值 |
|---|---|
| 规划组 | `manipulator`（`joint1`–`joint6`，弧度） |
| 基座 / 工具坐标系 | `base_link` / `link6`；`grasp_frame` 用于物体附着 |
| 机械臂 Action | `/manipulator_controller/follow_joint_trajectory` |
| 夹爪 Action | `/gripper_controller/gripper_cmd`（`0.0` 闭合，`0.044` 米张开） |
| 厂商话题 | `/arm_status`（`RobotStatus`）、`/arm_cmd`（`RobotCmd`） |

## 项目结构

| 软件包 | 内容 |
|---|---|
| [`isaac_ros_manipulation_arx_r5a_robot_description`](isaac_ros_manipulation_arx_r5a_robot_description) | URDF/xacro、网格、SRDF、XRDF、MoveIt 与控制器配置 |
| [`isaac_ros_manipulation_arx_r5a_ros2_control`](isaac_ros_manipulation_arx_r5a_ros2_control) | `ArxR5aSystem` 硬件插件（C++） |
| [`isaac_ros_manipulation_arx_r5a_driver_utils`](isaac_ros_manipulation_arx_r5a_driver_utils) | launch 文件、启动辅助代码、overlay 参数、`plan_joint_target` |

MoveIt 使用 `r5a.urdf`，速度限位来自 `config/joint_limits.yaml`。`r5a_cumotion.urdf` 是同一
模型并直接写入这些限位，因为 cuMotion 从 URDF 读取。单元测试会保证两者一致。

## 参与贡献

欢迎提交 Issue 和 Pull Request。代码规范和需要运行的检查见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 许可

[BSD 3-Clause](LICENSE)。机器人描述和网格派生自 [ARXroboticsX/R5](https://github.com/ARXroboticsX/R5)
（BSD 3-Clause）。NVIDIA Isaac ROS 和 ARX SDK 为外部依赖，遵循各自许可。
