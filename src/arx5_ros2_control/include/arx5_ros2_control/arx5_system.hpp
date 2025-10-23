#pragma once
/**
 * arx5_ros2_control/arx5_system.hpp
 *
 * 核心思路（和你的接口严格对齐）：
 * - 受控关节：joint1..joint8（前6=手臂；joint7/joint8=两指）。
 * - 读状态 read():
 *     /arm_status (RobotStatus) -> 填充 8 个关节的 position/velocity（若有电流可做 effort）。
 * - 写命令 write():
 *     来自 JTC 的 8 个位置 setpoint：
 *       - joint1..joint6 -> RobotCmd.joint_pos[0..5]
 *       - joint7/joint8  -> 合并成一个标量（默认：两指“平均”）-> RobotCmd.gripper
 *     其余 RobotCmd 字段：mode=参数（你写 5），end_pos 填 0（若固件忽略即可）。
 *
 * 重要：插值、定时、容差全由 joint_trajectory_controller 负责。本插件只做“周期映射”。
 */

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arx5_arm_msg/msg/robot_cmd.hpp>
#include <arx5_arm_msg/msg/robot_status.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace arx5_ros2_control {

class Arx5System : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Arx5System)

  // —— 生命周期回调（由 controller_manager 调用）——
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  // —— 导出接口（告诉 ros2_control 我有哪些读/写接口）——
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // —— 硬件 I/O ——（控制循环里被周期调用）
  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ========== ROS 通信 ==========
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<arx5_arm_msg::msg::RobotCmd>::SharedPtr cmd_pub_;

  // ★ 只持有一个执行器实例，不开线程
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

  // ========== 参数 ==========
  std::string status_topic_{"/arm_status"};
  std::string cmd_topic_{"/arm_cmd"};
  int mode_{5};                 // RobotCmd.mode，按你的固件协议，默认 5

  // 将 joint7/joint8 如何合并成 gripper：
  // 0=平均；1=取左指(joint7)；2=取右指(joint8)；3=取最大；4=取最小
  int gripper_merge_policy_{0};

  // ========== 接口缓存 ==========
  std::vector<std::string> joint_names_; // 期望长度=8，对应 joint1..joint8
  std::vector<double> pos_;              // 关节实际位置（来自状态）
  std::vector<double> vel_;              // 关节实际速度（来自状态）
  std::vector<double> cmd_pos_;          // 关节目标位置（来自控制器）

  // ========== 状态订阅缓存（线程安全）==========
  std::mutex mtx_;
  arx5_arm_msg::msg::RobotStatus last_status_;
  bool status_ready_{false};

  // 工具：获取关节数量（便于显式）
  inline size_t nj() const { return joint_names_.size(); } // 预期 8

  // 将 joint7/joint8 合并成一个 gripper 值
  double merge_gripper(double j7, double j8) const;
};

}  // namespace arx5_ros2_control
