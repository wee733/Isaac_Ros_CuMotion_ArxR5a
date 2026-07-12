# ARX R5A 的 Isaac ROS cuMotion 集成

[English](README.md)

本仓库为 ARX R5A 提供 ROS 2 control、MoveIt 2 和 NVIDIA Isaac ROS
cuMotion 集成。它通过官方 ARX ROS 2 驱动的 `/arm_status` 与 `/arm_cmd`
话题控制六个机械臂关节和双指夹爪。

> [!IMPORTANT]
> 这是社区项目，并非 ARX Robotics 或 NVIDIA 官方发布。当前已在 ROS 2
> Jazzy 和 Isaac ROS 4.3.0 上完成实机测试；Isaac ROS 4.5 的 cuMotion
> 参数名称不同，目前尚未验证。

## 目录结构

```text
src/R5a/                  R5A 描述文件与网格模型
src/arx5_ros2_control/    ros2_control 硬件接口
src/r5_moveit/            MoveIt、cuMotion、XRDF 与启动配置
```

## 环境要求

- ARX R5A、CAN 适配器及急停装置
- Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2
- 支持 Isaac ROS 的 NVIDIA GPU
- Isaac ROS 4.3.0、`isaac_ros_cumotion`、`isaac_ros_cumotion_moveit`
- 已编译的 [ARX 官方 R5 驱动](https://github.com/ARXroboticsX/R5)，用于提供
  `arx_r5_controller` 与 `arx5_arm_msg`

先编译官方驱动，再在 Isaac ROS 环境中编译本仓库：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## CAN 与官方驱动

以下是已验证的 `can1`、1 Mbit/s SLCAN 配置：

```bash
sudo slcand -o -f -s8 /dev/arxcan1 can1
sudo ip link set can1 up
ip -details -statistics link show can1
```

在单独终端启动官方驱动，并确认状态持续更新：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
ros2 launch arx_r5_controller open_single_arm.launch.py
ros2 topic hz /arm_status
```

## 启动 MoveIt 2

只使用 OMPL 时：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source install/setup.bash
ros2 launch r5_moveit real_robot.launch.py
```

## 启动 cuMotion

保持官方驱动单独运行，在 Isaac ROS 环境中执行：

```bash
isaac-ros activate
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/this/repository/install/setup.bash
ros2 launch r5_moveit cumotion_real_robot.launch.py
```

RViz 默认使用 `manipulator` 规划组和 `isaac_ros_cumotion` 管线。夹爪组
`gripper` 需要切换到 `ompl`，可使用 `open`、`jia`、`close` 三个命名状态。
cuMotion 只规划 `joint1` 到 `joint6`，末端坐标系为 `link6`。

已有 nvblox ESDF 环境时可启用：

```bash
ros2 launch r5_moveit cumotion_real_robot.launch.py read_esdf_world:=True
```

## 配置说明

- `src/r5_moveit/config/r5a.xrdf` 包含 cuMotion 关节空间和碰撞球；更换相机、
  末端工具或机械结构后必须重新校验。
- `r5a_cumotion.urdf` 是 cuMotion 专用速度上限版本，原始 URDF 独立保留。
- 当前关节限制、执行时间和跟踪容差来自已测试的 R5A，请结合固件与负载验证。
- 硬件接口只有收到新鲜 `/arm_status` 后才发送命令，并限制命令超前量。

## 常见问题

- `joint1 is not in list`：检查 `/joint_states` 的关节名和当前状态时间戳。
- `INVALID_START_STATE_SELF_COLLISION`：在 RViz 检查实机姿态，并重新验证 XRDF。
- 末点速度非零被拒绝：修改控制器配置后必须重启 ros2_control。
- 执行超时：先比较规划时间与实机跟踪速度，不要直接关闭超时监控。

## 安全与许可

执行前清空工作区、确认急停可用并从小幅运动开始。碰撞模型不能替代现场监护
和硬件限位。本项目采用 BSD 3-Clause License，第三方来源见 [NOTICE.md](NOTICE.md)。
