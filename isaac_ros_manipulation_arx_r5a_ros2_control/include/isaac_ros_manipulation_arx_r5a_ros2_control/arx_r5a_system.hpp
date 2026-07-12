// Copyright (c) 2026 wee733
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <arx5_arm_msg/msg/robot_cmd.hpp>
#include <arx5_arm_msg/msg/robot_status.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace isaac_ros_manipulation_arx_r5a_ros2_control
{

class ArxR5aSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArxR5aSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<arx5_arm_msg::msg::RobotCmd>::SharedPtr cmd_pub_;

  std::string status_topic_{"/arm_status"};
  std::string cmd_topic_{"/arm_cmd"};
  int mode_{5};
  double gripper_travel_{0.044};
  double gripper_feedback_max_{5.0};
  double gripper_command_max_{5.0};
  double max_arm_command_velocity_{1.0};
  double max_gripper_command_velocity_{0.02};
  double max_arm_command_lead_{0.10};
  double max_gripper_command_lead_{0.005};
  std::chrono::milliseconds status_timeout_{250};

  // 0=average, 1=joint7, 2=joint8, 3=max, 4=min.
  int gripper_merge_policy_{0};

  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> cmd_pos_;
  std::vector<double> last_sent_pos_;

  std::mutex mtx_;
  arx5_arm_msg::msg::RobotStatus last_status_;
  bool status_ready_{false};
  bool command_initialized_{false};
  std::chrono::steady_clock::time_point last_status_time_{};

  inline size_t nj() const {return joint_names_.size();}

  double merge_gripper(double j7, double j8) const;
};

}  // namespace isaac_ros_manipulation_arx_r5a_ros2_control
