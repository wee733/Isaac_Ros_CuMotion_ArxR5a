# ARX R5A 安全启动与分阶段验收

本文用于区分可以无人值守完成的离线软件验证，以及必须由人员在机械臂旁完成的
实体硬件验收。它不是 ARX 官方安全手册；厂商手册、硬件急停和现场风险评估始终
具有更高优先级。

## 必须先知道的行为

1. **厂商驱动启动不是只读操作。** 对目标 Thor 当前安装的 ARX arm64 预编译库
   进行静态检查后确认，其初始化路径会打开 SocketCAN、重复使能电机、进入保护，
   随后执行回零流程。仅启动 `open_single_arm.launch.py` 就可能产生实体运动。
2. **`start_vendor_driver:=False` 不代表没有控制输出。** 它只是不由本 launch
   创建厂商节点。ros2_control 及控制器仍会启动；收到新鲜 `/arm_status` 后，适配器
   会以 200 Hz 发布 `/arm_cmd`，其中 `mode=5` 是厂商位置控制模式。
3. **ROS 状态不足以证明机械臂安全。** 当前 `RobotStatus` 只有位置、速度和电流，
   没有急停、使能、保护模式、在线状态或故障码。底层库虽有错误码接口，厂商 ROS
   节点并未将其发布给本适配器。
4. **状态来源没有身份或时间戳校验。** 适配器按消息到达时间判断反馈是否新鲜，
   不验证 `header.stamp`。在实体驱动所在 ROS 域回放含 `/arm_status` 的 rosbag，可能
   触发位置控制输出。
5. **软件停止不是硬件断能。** 正常停用时只能尝试发送一次保护命令；进程崩溃、
   网络/DDS 中断或强制结束时，不能依赖该命令送达，也不能确认电机已经 disable。
6. **默认规划世界不包含现场环境。** 默认 `read_esdf_world=False`，cuMotion launch
   又设置 `add_ground_plane=False`。桌面、地面、相机支架和周围人员不会自动成为
   障碍物。

因此，禁止在无人监护状态下启动厂商驱动、激活实体控制链或执行轨迹。

## 验收门槛总览

| 阶段 | 目标 | 是否允许无人值守 | 进入下一阶段的最低条件 |
|---|---|---:|---|
| 0 | 源码、依赖、模型和 launch 静态检查 | 是 | 不运行厂商节点，不接触 CAN 状态 |
| 1 | 隔离 ROS 域中的离线启动 | 是 | `/arm_cmd` 无实体订阅者，实体驱动不在该域 |
| 2 | 被动 CAN 链路检查 | 否 | 接口唯一、1 Mbit/s、无 bus-off/持续错误 |
| 3 | 厂商使能和自动回零 | 否 | 人员现场，回零路径安全，急停已实测可用 |
| 4 | ros2_control 当前位置保持 | 否 | 模型反馈一致，无启动跳变，停止流程已验证 |
| 5 | 单关节极小幅运动 | 否 | 低速、低加速度、可取消、急停有效 |
| 6 | cuMotion 规划与小幅执行 | 否 | 先只规划，碰撞环境已校验，再执行小轨迹 |

任何阶段失败都应退回物理急停/断能状态，不应通过放宽限制掩盖故障。

## 阶段 0：静态检查

本阶段不启动 ROS 节点，不打开 CAN，不改变网络接口：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/isaac_ros-dev/install/setup.bash

ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py --show-args
xacro \
  /path/to/isaac_ros_manipulation_arx_r5a_robot_description/urdf/r5a.urdf.xacro \
  initial_positions_file:=/path/to/initial_positions.yaml > /tmp/r5a.expanded.urdf
check_urdf /tmp/r5a.expanded.urdf
```

还应检查：

- `arm_can_id` 与计划使用的 SocketCAN 接口完全一致；
- 关节方向、零位、机械上下限和实体 R5A 一致；
- XRDF 碰撞球覆盖当前夹爪、相机和负载；
- 不存在另一个会发布 `/arm_status` 或订阅 `/arm_cmd` 的测试进程；
- 要回放的 rosbag 不包含 `/arm_status`、`/arm_cmd` 或控制器 action 目标。

`ros2 bag info <bag-path>` 可以只读检查 bag 的话题清单。

## 阶段 1：隔离 ROS 域离线启动

离线测试必须使用一个确认未被实体驱动使用的 ROS 域。下面的 `231` 只是示例，
应先确认现场没有其他进程使用它：

```bash
export ROS_DOMAIN_ID=231
source /opt/ros/jazzy/setup.bash
source /path/to/R5/ROS2/R5_ws/install/setup.bash
source /path/to/isaac_ros-dev/install/setup.bash

ros2 node list
ros2 topic info /arm_status --verbose
ros2 topic info /arm_cmd --verbose
```

预期在启动前没有实体节点、没有 `/arm_status` 发布者，也没有 `/arm_cmd` 订阅者。
随后可以进行最小离线启动：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  start_vendor_driver:=False \
  start_cumotion:=False \
  start_rviz:=False
```

该进程会创建 ros2_control 和控制器，但在没有 `/arm_status` 时硬件适配器应抑制
命令。另一个相同 `ROS_DOMAIN_ID` 的终端中确认：

```bash
ros2 topic info /arm_status --verbose
ros2 topic info /arm_cmd --verbose
ros2 control list_controllers
```

离线通过条件：

- `/arm_status` 没有发布者；
- `/arm_cmd` 可以有本适配器的发布者，但绝不能有实体厂商驱动订阅者；
- 没有启动 `R5Controller`、`slcand` 或改变任何实体 CAN 接口；
- 退出后系统中没有残留控制节点。

如果无法证明 ROS 域隔离，则只能做阶段 0，不能启动该 launch。

## 阶段 2：被动 CAN 链路检查

本阶段必须有人员在机械臂旁，执行器保持断能或物理急停状态。先识别实际连接，
不能同时猜测两个接口：

```bash
ip -details -statistics link show type can
ls -l /dev/arxcan* /dev/ttyACM* 2>/dev/null
```

### Thor 原生 CAN

如果机械臂确实接在 Thor 原生 `can1` 的收发器上，可按平台硬件文档配置 1 Mbit/s：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can1 up
ip -details -statistics link show can1
timeout 10 candump -L can1
```

### USB SLCAN

Thor 已占用原生名称 `can1`，USB SLCAN 应使用不同名称，例如 `slcan1`：

```bash
readlink -f /dev/arxcan1
udevadm info --query=property --name=/dev/arxcan1
sudo modprobe slcan
sudo slcand -o -f -s8 /dev/arxcan1 slcan1
sudo ip link set slcan1 up
ip -details -statistics link show slcan1
timeout 10 candump -L slcan1
```

厂商配置中的 `arm_can_id` 必须相应为 `can1` 或 `slcan1`。当前适配 launch
包含厂商 launch 时不会替它重写这个参数，因此应检查厂商工作区中实际安装的
`single_arm.yaml`，不能只看源码副本。

`candump` 是被动监听；本阶段禁止使用 `cansend`、键盘控制器、厂商驱动或任何
会轮询/使能电机的程序。被动监听没有报文并不能单独证明链路故障，因为部分设备
可能只响应主动查询；它只用于发现已有流量、bus-off 和明显接线错误。

通过条件：接口唯一、名称匹配、状态不是 bus-off、错误计数不持续增加，并且物理
收发器、CAN-H/CAN-L、共地和两端终端电阻均经现场确认。

## 阶段 3：有人监护的厂商初始化和回零

这是第一个会使能并可能移动机械臂的阶段。必须满足：

- 操作员站在急停旁，且已在断能条件下确认急停机构有效；
- 机械臂周围及预计回零路径完全清空；
- 末端负载、相机和线束不会在回零时碰撞或拉扯；
- 底座固定，供电、CAN 和终端电阻稳定；
- 同一 ROS 域中没有本适配器、MoveIt、rosbag 或其他 `/arm_cmd` 发布者；
- 已明确接受厂商驱动启动时的使能和自动回零行为。

先确认没有命令发布者：

```bash
ros2 topic info /arm_cmd --verbose
```

然后才可在现场启动厂商驱动：

```bash
ros2 launch arx_r5_controller open_single_arm.launch.py
```

出现任何非预期方向、速度、噪声、过流迹象或线束拉扯时，立即使用物理急停，
不要等待 ROS 命令。自动回零完成并稳定后，才检查：

```bash
ros2 topic hz /arm_status
ros2 topic echo /arm_status --once
```

这些命令只能证明 ROS 收到数值，不能证明急停、使能和故障状态正常。还必须使用
厂商规定的硬件指示、诊断工具或现场流程确认。

首次连接不要使用本仓库的 `start_vendor_driver:=True`；它会把厂商初始化、控制器
激活和规划栈并行启动，不利于分辨自动回零和适配器位置保持行为。

## 阶段 4：当前位置保持

仅在厂商回零完成、机械臂静止且关节反馈与模型一致后进行。保持厂商驱动独立运行，
再启动不含 cuMotion 和 RViz 的适配层：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  start_vendor_driver:=False \
  start_cumotion:=False \
  start_rviz:=False
```

收到新鲜反馈后，适配器会自动发布 `mode=5` 当前位置命令。预期现象是机械臂进入
位置保持并可能明显变硬；这不是纯观察模式。操作员必须持续监护启动瞬间。

通过条件：

- `/joint_states` 与 `/arm_status` 的六轴顺序、方向和单位一致；
- 启动时没有关节跳变，目标与反馈差值处于预先批准的小阈值内；
- `/arm_status` 中断超过 250 ms 时适配器停止继续发送新命令；
- 取消、正常关闭和物理急停均已分别验证；
- 停止后通过硬件状态确认，而不是假设一次 `mode=2` 已可靠断能。

不要使用 `kill -9` 作为正常停止方法。

## 阶段 5：单关节极小幅运动

首次运动仍先禁用 cuMotion，只使用 MoveIt 的规划预览。RViz 中先点击 **Plan**，
不要直接点击 **Plan & Execute**：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  start_vendor_driver:=False \
  start_cumotion:=False \
  start_rviz:=True
```

初始测试建议：

- 一次只改变一个关节；
- 位移不超过经现场批准的极小量，例如 `0.01` 到 `0.02 rad`；
- RViz 速度与加速度缩放先设为 `0.05` 或更低；
- 目标远离机械限位和奇异位形；
- 操作员在执行全过程握住急停；
- 先验证 action 取消，再增加运动范围。

仓库默认关节速度/加速度缩放和 cuMotion time dilation 为 `1.0`，这适合已验收系统，
不适合作为首次实体测试值。

## 阶段 6：cuMotion 规划与小幅执行

先只启用规划并查看轨迹，不执行：

```bash
ros2 launch isaac_ros_manipulation_arx_r5a_driver_utils \
  arx_r5a_driver.launch.py \
  start_vendor_driver:=False \
  start_cumotion:=True \
  start_rviz:=True \
  cumotion_time_dilation_factor:=0.1
```

执行前逐项确认：

- 当前六轴状态在 URDF/XRDF 关节范围内，且零位和方向一致；
- 轨迹在 RViz 中连续、没有接近机械限位或环境障碍；
- XRDF 碰撞球与当前实体结构一致；
- 桌面、地面、相机、线束和夹具已经显式加入规划场景，或由已验收的 nvblox ESDF
  提供；默认配置不会自动完成这一步；
- 仍从单关节、小位移和低时间缩放开始；
- MoveIt/控制器取消与物理急停均已在前一阶段验证。

只有先完成“规划而不执行”的人工审查，才可在现场执行第一条 cuMotion 小轨迹。

## 停止和故障处理

计划内停止应先取消当前轨迹，再停止规划/控制进程，最后停止厂商驱动，并通过
厂商硬件状态确认机械臂处于安全模式。软件退出过程中发送的保护命令不能替代：

- 物理急停；
- 电机断能；
- 厂商规定的故障复位和锁定流程。

如果 `/arm_status` 丢失、CAN 进入 bus-off、关节发生跳变、控制进程崩溃或 ROS 域
出现未知节点，应立即停止验收并使用物理安全措施，不要在带电状态下反复重启节点。

## 当前已知的实机阻塞项

在把该集成作为生产系统之前，至少应解决：

- 厂商驱动启动时不可选择地自动使能/回零；
- 缺少显式人工 arm/disarm 门控和默认 inactive 控制器模式；
- ROS 状态缺少急停、使能、保护模式、在线与故障码；
- 反馈只按到达时间判新鲜，无法区分真实硬件、rosbag 和错误发布者；
- 进程异常退出时没有可证明的硬件断能；
- 厂商上游 `R5Controller.cpp` 原本对长度 7 的状态数组使用 `i <= 7`，存在越界
  访问风险；本机源码已修为 `i < 7`，后续更新或重新克隆时必须保留并复核该修复；
- Thor 原生 `can1` 与 USB SLCAN 接口名/厂商 `arm_can_id` 可能冲突；
- 默认没有地面和现场环境碰撞模型。

这些问题不妨碍隔离环境中的编译和规划验证，但在修复或由厂商正式确认前，不能把
软件启动成功视为实体机械臂的安全验收完成。

## English summary

- Starting the vendor `R5Controller` is **not** read-only: the vendor library on
  the target system enables motors and runs a homing sequence during startup.
- `start_vendor_driver:=False` only prevents this launch file from creating the
  vendor node. A fresh `/arm_status` still causes the adapter to publish
  `/arm_cmd` at 200 Hz in vendor `mode=5` (position control).
- Offline tests must use a ROS domain that is isolated from every physical
  `/arm_cmd` subscriber. Never replay `/arm_status` in the hardware domain.
- Passive CAN inspection, vendor homing, position hold, small motion, and
  cuMotion execution are separate gates. Every hardware gate requires an
  operator at the robot with a tested, reachable physical E-stop.
- The current ROS status message does not expose E-stop, enable, protection, or
  fault state. Normal process shutdown is not proof of motor disable.
- The default planner has no ESDF world or ground plane. Validate the real
  environment and collision model before any execution.
