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

/// ros2_control system that bridges position interfaces to the ARX vendor driver.
///
/// The vendor driver owns CAN communication. It publishes RobotStatus on
/// `status_topic` and accepts RobotCmd on `cmd_topic`. joint1..joint6 are the
/// arm. joint7 commands the single vendor gripper value; joint8 is the mimic
/// finger and only reports state.
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
  /// Convert a gripper opening in metres to the vendor command scale.
  double to_vendor_gripper(double position) const;
  /// True when a status message arrived within `status_timeout_`. Requires `mtx_`.
  bool status_fresh() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<arx5_arm_msg::msg::RobotCmd>::SharedPtr cmd_pub_;

  // Hardware parameters from the URDF <ros2_control> block.
  std::string status_topic_{"/arm_status"};
  std::string cmd_topic_{"/arm_cmd"};
  int mode_{5};  // RobotCmd::mode while active; 5 is vendor POSITION_CONTROL.
  double gripper_travel_{0.044};  // Finger opening in metres at full vendor scale.
  double gripper_feedback_max_{5.0};  // Vendor feedback value at full opening.
  double gripper_command_max_{5.0};  // Vendor command value at full opening.
  double max_arm_command_velocity_{1.0};  // rad/s
  double max_arm_command_lead_{0.10};  // rad ahead of the measured position
  std::chrono::milliseconds status_timeout_{250};

  std::vector<std::string> joint_names_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> cmd_pos_;
  std::vector<double> last_sent_pos_;

  // Guards the fields written by the status subscription callback.
  std::mutex mtx_;
  arx5_arm_msg::msg::RobotStatus last_status_;
  bool status_ready_{false};
  bool command_initialized_{false};
  std::chrono::steady_clock::time_point last_status_time_{};
};

}  // namespace isaac_ros_manipulation_arx_r5a_ros2_control
