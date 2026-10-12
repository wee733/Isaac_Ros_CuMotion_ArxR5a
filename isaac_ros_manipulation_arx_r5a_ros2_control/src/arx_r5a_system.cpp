// Copyright (c) 2026 wee733
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include "isaac_ros_manipulation_arx_r5a_ros2_control/arx_r5a_system.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <hardware_interface/lexical_casts.hpp>
#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace isaac_ros_manipulation_arx_r5a_ros2_control
{

namespace
{

// joint1..joint6 are the arm; joint7 and joint8 are the gripper fingers.
constexpr size_t kArmJointCount = 6;
constexpr size_t kJointCount = 8;
constexpr size_t kJoint7 = 6;
constexpr size_t kJoint8 = 7;

// RobotCmd::mode for the vendor PROTECT state (arx_r5_src InterfacesThread::state).
constexpr int kVendorModeProtect = 2;

// Used when the controller manager reports an invalid period (200 Hz update rate).
constexpr double kFallbackPeriodSeconds = 0.005;

// Read an optional hardware parameter. A missing parameter keeps its default.
template<typename ParameterMap, typename T>
bool read_parameter(
  const ParameterMap & parameters, const char * name, T & value,
  const rclcpp::Logger & logger)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return true;
  }
  try {
    // hardware_interface parsers are locale-independent and reject trailing characters.
    if constexpr (std::is_same_v<T, int>) {
      value = hardware_interface::stoi32(it->second);
    } else if constexpr (std::is_same_v<T, double>) {
      value = hardware_interface::stod(it->second);
      if (!std::isfinite(value)) {
        throw std::invalid_argument("expected a finite number");
      }
    } else {
      value = it->second;
    }
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Invalid %s: %s", name, error.what());
    return false;
  }
}

}  // namespace

double ArxR5aSystem::to_vendor_gripper(double position) const
{
  return position / gripper_travel_ * gripper_command_max_;
}

bool ArxR5aSystem::status_fresh() const
{
  return status_ready_ &&
         std::chrono::steady_clock::now() - last_status_time_ <= status_timeout_;
}

CallbackReturn ArxR5aSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ArxR5aSystem"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }

  node_ = get_node();
  const auto logger = node_->get_logger();
  const auto & hardware_info = get_hardware_info();
  const auto & parameters = hardware_info.hardware_parameters;

  int status_timeout_ms = static_cast<int>(status_timeout_.count());
  const bool parameters_valid =
    read_parameter(parameters, "status_topic", status_topic_, logger) &&
    read_parameter(parameters, "cmd_topic", cmd_topic_, logger) &&
    read_parameter(parameters, "mode", mode_, logger) &&
    read_parameter(parameters, "status_timeout_ms", status_timeout_ms, logger) &&
    read_parameter(parameters, "gripper_travel", gripper_travel_, logger) &&
    read_parameter(parameters, "gripper_feedback_max", gripper_feedback_max_, logger) &&
    read_parameter(parameters, "gripper_command_max", gripper_command_max_, logger) &&
    read_parameter(
    parameters, "max_arm_command_velocity", max_arm_command_velocity_, logger) &&
    read_parameter(parameters, "max_arm_command_lead", max_arm_command_lead_, logger);
  if (!parameters_valid) {
    return CallbackReturn::ERROR;
  }

  status_timeout_ = std::chrono::milliseconds(status_timeout_ms);

  if (status_timeout_ms <= 0 || gripper_travel_ <= 0.0 ||
    gripper_feedback_max_ <= 0.0 || gripper_command_max_ <= 0.0 ||
    max_arm_command_velocity_ <= 0.0 || max_arm_command_lead_ <= 0.0)
  {
    RCLCPP_ERROR(logger, "Hardware scale and limit parameters must be positive");
    return CallbackReturn::ERROR;
  }

  if (hardware_info.joints.size() != kJointCount) {
    RCLCPP_ERROR(
      logger, "Expected %zu joints (joint1..joint8), got %zu",
      kJointCount, hardware_info.joints.size());
    return CallbackReturn::ERROR;
  }
  // Vendor message arrays have no joint names; their order is part of the ABI.
  joint_names_.clear();
  for (size_t index = 0; index < kJointCount; ++index) {
    const auto expected = "joint" + std::to_string(index + 1);
    const auto & joint = hardware_info.joints[index];
    if (joint.name != expected) {
      RCLCPP_ERROR(
        logger, "Expected %s at index %zu, got %s",
        expected.c_str(), index, joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  pos_.assign(kJointCount, 0.0);
  vel_.assign(kJointCount, 0.0);
  cmd_pos_.assign(kJointCount, 0.0);
  last_sent_pos_.assign(kJointCount, 0.0);

  status_sub_ = node_->create_subscription<arx5_arm_msg::msg::RobotStatus>(
    status_topic_, rclcpp::SensorDataQoS().keep_last(5),
    [this](arx5_arm_msg::msg::RobotStatus::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mtx_);
      const auto finite = [](double value) {return std::isfinite(value);};
      if (!std::all_of(message->joint_pos.begin(), message->joint_pos.end(), finite) ||
      !std::all_of(message->joint_vel.begin(), message->joint_vel.end(), finite))
      {
        status_ready_ = false;
        command_initialized_ = false;
        RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Rejected non-finite ARX status; waiting for valid feedback");
        return;
      }
      last_status_ = *message;
      status_ready_ = true;
      last_status_time_ = std::chrono::steady_clock::now();
    });
  cmd_pub_ = node_->create_publisher<arx5_arm_msg::msg::RobotCmd>(
    cmd_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    logger,
    "Initialized: arm_limit=%.3f rad/s, arm_lead=%.3f rad, "
    "gripper_feedback_max=%.1f",
    max_arm_command_velocity_, max_arm_command_lead_,
    gripper_feedback_max_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArxR5aSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mtx_);
  // The first fresh status in read() seeds the commands from the measured pose.
  command_initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "ArxR5aSystem activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArxR5aSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (cmd_pub_ && command_initialized_) {
    // Send one final command that holds the measured pose in vendor PROTECT mode.
    arx5_arm_msg::msg::RobotCmd command;
    command.header.stamp = node_->get_clock()->now();
    command.end_pos.fill(0.0);
    for (size_t index = 0; index < kArmJointCount; ++index) {
      command.joint_pos[index] = pos_[index];
    }
    const double gripper_position = std::clamp(pos_[kJoint7], 0.0, gripper_travel_);
    command.gripper = to_vendor_gripper(gripper_position);
    command.mode = kVendorModeProtect;
    cmd_pub_->publish(command);
  }
  command_initialized_ = false;
  RCLCPP_INFO(node_->get_logger(), "ArxR5aSystem deactivated");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArxR5aSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2);
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &pos_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &vel_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ArxR5aSystem::export_command_interfaces()
{
  // Only joints that declare a position command interface in the URDF get one;
  // joint8 mirrors joint7 and is state-only.
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

  for (size_t index = 0; index < kArmJointCount; ++index) {
    pos_[index] = last_status_.joint_pos[index];
    vel_[index] = last_status_.joint_vel[index];
  }

  // The vendor reports one gripper value in [0, gripper_feedback_max]; both
  // fingers report the same opening in metres.
  const double gripper_feedback = std::clamp(
    last_status_.joint_pos[kJoint7], 0.0, gripper_feedback_max_);
  const double gripper_position =
    gripper_feedback / gripper_feedback_max_ * gripper_travel_;
  const double gripper_velocity =
    last_status_.joint_vel[kJoint7] / gripper_feedback_max_ * gripper_travel_;
  pos_[kJoint7] = gripper_position;
  pos_[kJoint8] = gripper_position;
  vel_[kJoint7] = gripper_velocity;
  vel_[kJoint8] = gripper_velocity;

  if (!command_initialized_ && status_fresh()) {
    cmd_pos_ = pos_;
    last_sent_pos_ = pos_;
    command_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Commands initialized from hardware state");
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Hardware state: [%.3f %.3f %.3f %.3f %.3f %.3f], gripper=%.3f m",
    pos_[0], pos_[1], pos_[2], pos_[3], pos_[4], pos_[5], pos_[kJoint7]);
  return return_type::OK;
}

return_type ArxR5aSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const bool status_stale = !status_fresh();
    if (!command_initialized_ || status_stale) {
      // Re-seed from the measured pose once status resumes.
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
    seconds = kFallbackPeriodSeconds;
  }

  arx5_arm_msg::msg::RobotCmd command;
  command.header.stamp = node_->get_clock()->now();
  command.end_pos.fill(0.0);
  command.mode = mode_;

  // Rate-limit each arm joint toward its target and keep the command within
  // max_arm_command_lead of the measured position. These bounds apply to
  // outgoing commands, not to the physical response of the vendor controller.
  const double arm_max_step = max_arm_command_velocity_ * seconds;
  for (size_t index = 0; index < kArmJointCount; ++index) {
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

  if (!std::isfinite(cmd_pos_[kJoint7])) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Suppressing non-finite gripper command");
    return return_type::ERROR;
  }
  const double requested_gripper = std::clamp(cmd_pos_[kJoint7], 0.0, gripper_travel_);
  last_sent_pos_[kJoint7] = requested_gripper;
  last_sent_pos_[kJoint8] = requested_gripper;
  command.gripper = to_vendor_gripper(requested_gripper);

  cmd_pub_->publish(command);
  return return_type::OK;
}

}  // namespace isaac_ros_manipulation_arx_r5a_ros2_control

PLUGINLIB_EXPORT_CLASS(
  isaac_ros_manipulation_arx_r5a_ros2_control::ArxR5aSystem, hardware_interface::SystemInterface)
