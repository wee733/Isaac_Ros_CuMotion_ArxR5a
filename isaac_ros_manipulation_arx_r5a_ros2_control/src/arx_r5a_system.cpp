// Copyright (c) 2026 wee733
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include "isaac_ros_manipulation_arx_r5a_ros2_control/arx_r5a_system.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace isaac_ros_manipulation_arx_r5a_ros2_control
{

double ArxR5aSystem::merge_gripper(double joint7, double joint8) const
{
  switch (gripper_merge_policy_) {
    case 1:
      return joint7;
    case 2:
      return joint8;
    case 3:
      return std::max(joint7, joint8);
    case 4:
      return std::min(joint7, joint8);
    default:
      return 0.5 * (joint7 + joint8);
  }
}

CallbackReturn ArxR5aSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ArxR5aSystem"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }

  node_ = get_node();

  const auto & hardware_info = get_hardware_info();
  const auto & parameters = hardware_info.hardware_parameters;
  if (auto it = parameters.find("status_topic"); it != parameters.end()) {
    status_topic_ = it->second;
  }
  if (auto it = parameters.find("cmd_topic"); it != parameters.end()) {
    cmd_topic_ = it->second;
  }

  auto read_double = [this, &parameters](const char * name, double & value) {
      const auto it = parameters.find(name);
      if (it == parameters.end()) {
        return true;
      }
      try {
        value = std::stod(it->second);
        return true;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid %s: %s", name, error.what());
        return false;
      }
    };
  auto read_int = [this, &parameters](const char * name, int & value) {
      const auto it = parameters.find(name);
      if (it == parameters.end()) {
        return true;
      }
      try {
        value = std::stoi(it->second);
        return true;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid %s: %s", name, error.what());
        return false;
      }
    };

  int status_timeout_ms = static_cast<int>(status_timeout_.count());
  const bool parameters_valid =
    read_int("mode", mode_) &&
    read_int("gripper_merge_policy", gripper_merge_policy_) &&
    read_int("status_timeout_ms", status_timeout_ms) &&
    read_double("gripper_travel", gripper_travel_) &&
    read_double("gripper_feedback_max", gripper_feedback_max_) &&
    read_double("gripper_command_max", gripper_command_max_) &&
    read_double("max_arm_command_velocity", max_arm_command_velocity_) &&
    read_double("max_arm_command_lead", max_arm_command_lead_);
  status_timeout_ = std::chrono::milliseconds(status_timeout_ms);

  if (!parameters_valid || status_timeout_ms <= 0 || gripper_travel_ <= 0.0 ||
    gripper_feedback_max_ <= 0.0 || gripper_command_max_ <= 0.0 ||
    max_arm_command_velocity_ <= 0.0 || max_arm_command_lead_ <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Hardware scale and limit parameters must be positive");
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : hardware_info.joints) {
    joint_names_.push_back(joint.name);
  }
  if (joint_names_.size() != 8) {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected 8 joints (joint1..joint8), got %zu",
      joint_names_.size());
    return CallbackReturn::ERROR;
  }

  pos_.assign(joint_names_.size(), 0.0);
  vel_.assign(joint_names_.size(), 0.0);
  cmd_pos_.assign(joint_names_.size(), 0.0);
  last_sent_pos_.assign(joint_names_.size(), 0.0);

  status_sub_ = node_->create_subscription<arx5_arm_msg::msg::RobotStatus>(
    status_topic_, rclcpp::SensorDataQoS().keep_last(5),
    [this](arx5_arm_msg::msg::RobotStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mtx_);
      last_status_ = *message;
      status_ready_ = true;
      last_status_time_ = std::chrono::steady_clock::now();
    });
  cmd_pub_ = node_->create_publisher<arx5_arm_msg::msg::RobotCmd>(
    cmd_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    node_->get_logger(),
    "Initialized: arm_limit=%.3f rad/s, arm_lead=%.3f rad, "
    "gripper_feedback_max=%.1f",
    max_arm_command_velocity_, max_arm_command_lead_,
    gripper_feedback_max_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArxR5aSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mtx_);
  command_initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "ArxR5aSystem activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArxR5aSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (cmd_pub_ && command_initialized_) {
    arx5_arm_msg::msg::RobotCmd command;
    command.header.stamp = node_->get_clock()->now();
    command.end_pos.fill(0.0);
    for (size_t index = 0; index < 6; ++index) {
      command.joint_pos[index] = pos_[index];
    }
    const double gripper_position = std::clamp(
      merge_gripper(pos_[6], pos_[7]), 0.0, gripper_travel_);
    command.gripper = gripper_position / gripper_travel_ * gripper_command_max_;
    command.mode = 2;
    cmd_pub_->publish(command);
  }
  command_initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "ArxR5aSystem deactivated");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArxR5aSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(nj() * 2);
  for (size_t index = 0; index < nj(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &pos_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &vel_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ArxR5aSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  const auto & joints = get_hardware_info().joints;
  for (size_t index = 0; index < joints.size(); ++index) {
    for (const auto & command_interface : joints[index].command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(
          joint_names_[index], hardware_interface::HW_IF_POSITION, &cmd_pos_[index]);
      }
    }
  }
  return interfaces;
}

return_type ArxR5aSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!status_ready_) {
    return return_type::OK;
  }

  for (size_t index = 0; index < 6; ++index) {
    pos_[index] = last_status_.joint_pos[index];
    vel_[index] = last_status_.joint_vel[index];
  }

  const double gripper_feedback = std::clamp(
    last_status_.joint_pos[6], 0.0, gripper_feedback_max_);
  const double gripper_position =
    gripper_feedback / gripper_feedback_max_ * gripper_travel_;
  const double gripper_velocity =
    last_status_.joint_vel[6] / gripper_feedback_max_ * gripper_travel_;
  pos_[6] = gripper_position;
  pos_[7] = gripper_position;
  vel_[6] = gripper_velocity;
  vel_[7] = gripper_velocity;

  const bool status_fresh =
    std::chrono::steady_clock::now() - last_status_time_ <= status_timeout_;
  if (!command_initialized_ && status_fresh) {
    cmd_pos_ = pos_;
    last_sent_pos_ = pos_;
    command_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Commands initialized from hardware state");
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Hardware state: [%.3f %.3f %.3f %.3f %.3f %.3f], gripper=%.3f m",
    pos_[0], pos_[1], pos_[2], pos_[3], pos_[4], pos_[5], pos_[6]);
  return return_type::OK;
}

return_type ArxR5aSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const bool status_stale = !status_ready_ ||
      std::chrono::steady_clock::now() - last_status_time_ > status_timeout_;
    if (!command_initialized_ || status_stale) {
      if (status_stale) {
        command_initialized_ = false;
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Suppressing commands until fresh hardware status is available");
      return return_type::OK;
    }
  }

  double seconds = period.seconds();
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    seconds = 0.005;
  }

  arx5_arm_msg::msg::RobotCmd command;
  command.header.stamp = node_->get_clock()->now();
  command.end_pos.fill(0.0);
  command.mode = mode_;

  const double arm_max_step = max_arm_command_velocity_ * seconds;
  for (size_t index = 0; index < 6; ++index) {
    if (!std::isfinite(cmd_pos_[index]) || !std::isfinite(pos_[index])) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Suppressing non-finite arm command");
      return return_type::ERROR;
    }
    const double step = std::clamp(
      cmd_pos_[index] - last_sent_pos_[index], -arm_max_step, arm_max_step);
    last_sent_pos_[index] = std::clamp(
      last_sent_pos_[index] + step,
      pos_[index] - max_arm_command_lead_,
      pos_[index] + max_arm_command_lead_);
    command.joint_pos[index] = last_sent_pos_[index];
  }

  if (!std::isfinite(cmd_pos_[6]) || !std::isfinite(cmd_pos_[7])) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Suppressing non-finite gripper command");
    return return_type::ERROR;
  }
  const double requested_gripper = std::clamp(
    merge_gripper(cmd_pos_[6], cmd_pos_[7]), 0.0, gripper_travel_);
  last_sent_pos_[6] = requested_gripper;
  last_sent_pos_[7] = requested_gripper;
  command.gripper = requested_gripper / gripper_travel_ * gripper_command_max_;

  cmd_pub_->publish(command);
  return return_type::OK;
}

}  // namespace isaac_ros_manipulation_arx_r5a_ros2_control

PLUGINLIB_EXPORT_CLASS(
  isaac_ros_manipulation_arx_r5a_ros2_control::ArxR5aSystem, hardware_interface::SystemInterface)
