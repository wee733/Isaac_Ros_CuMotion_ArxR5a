#include "arx5_ros2_control/arx5_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace arx5_ros2_control {

// ========== 工具：两指值如何合并为 gripper ==========
double Arx5System::merge_gripper(double j7, double j8) const {
  switch (gripper_merge_policy_) {
    case 1: return j7;                   // 只用 joint7
    case 2: return j8;                   // 只用 joint8
    case 3: return std::max(j7, j8);     // 取大
    case 4: return std::min(j7, j8);     // 取小
    default: return 0.5 * (j7 + j8);     // 默认：平均
  }
}

// ========== 生命周期：初始化 ==========
CallbackReturn Arx5System::on_init(const hardware_interface::HardwareInfo & info) {
  // 让基类先处理（会把 <ros2_control> 里的 joint/param 读进来）
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("Arx5System"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }

  auto options = rclcpp::NodeOptions()
    .context(rclcpp::contexts::get_global_default_context())
    .use_intra_process_comms(false)
    .automatically_declare_parameters_from_overrides(true);
  node_ = rclcpp::Node::make_shared("arx5_system", options);


  // 读插件参数（在 URDF 的 <hardware><param ...> 里设置）
  const auto & p = info.hardware_parameters;
  if (auto it = p.find("status_topic"); it != p.end()) status_topic_ = it->second;
  if (auto it = p.find("cmd_topic");    it != p.end()) cmd_topic_    = it->second;
  if (auto it = p.find("mode");         it != p.end()) try { mode_ = std::stoi(it->second); } catch(...) {}
  if (auto it = p.find("gripper_merge_policy"); it != p.end()) try { gripper_merge_policy_ = std::stoi(it->second); } catch(...) {}

  // 关节列表（顺序即控制器使用的顺序）
  joint_names_.reserve(info.joints.size());
  for (const auto & j : info.joints) joint_names_.push_back(j.name);

  // 分配缓存（只导出 position/velocity 两种状态接口，和 position 命令接口）
  const size_t N = joint_names_.size();      // 预期 8
  pos_.assign(N, 0.0);
  vel_.assign(N, 0.0);
  cmd_pos_.assign(N, 0.0);

  // 订阅状态：/arm_status
  status_sub_ = node_->create_subscription<arx5_arm_msg::msg::RobotStatus>(
      status_topic_, rclcpp::QoS(1200),
      [this](arx5_arm_msg::msg::RobotStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_status_ = *msg;
        status_ready_ = true;
      });

  // 发布命令：/arm_cmd
  cmd_pub_ = node_->create_publisher<arx5_arm_msg::msg::RobotCmd>(cmd_topic_, rclcpp::QoS(50));

  RCLCPP_INFO(node_->get_logger(),
              "Arx5System init ok. joints=%zu, cmd_topic=%s, status_topic=%s, mode=%d, merge_policy=%d",
              N, cmd_topic_.c_str(), status_topic_.c_str(), mode_, gripper_merge_policy_);

  // 输入健壮性提示（不致命，仅提示）
  if (N < 8) {
    RCLCPP_WARN(node_->get_logger(), "Expecting 8 joints (joint1..joint8). Found %zu.", N);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Arx5System::on_activate(const rclcpp_lifecycle::State &) {
  // 若你的下位机需要“切模式/使能”，可在这里发一次性命令
  RCLCPP_INFO(node_->get_logger(), "Arx5System activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Arx5System::on_deactivate(const rclcpp_lifecycle::State &) {
  // 若需要安全停车，可在这里发“保停/零速度”等
  RCLCPP_INFO(node_->get_logger(), "Arx5System deactivated");
  return CallbackReturn::SUCCESS;
}

// ========== 导出状态接口 ==========
std::vector<hardware_interface::StateInterface> Arx5System::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(nj() * 2);
  for (size_t i = 0; i < nj(); ++i) {
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    // 如果你在 URDF 里没声明 effort，就不要导出 effort，避免接口不匹配
    // si.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &eff_[i]);
  }
  return si;
}

// ========== 导出命令接口 ==========
std::vector<hardware_interface::CommandInterface> Arx5System::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(nj());
  for (size_t i = 0; i < nj(); ++i) {
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
  }
  return ci;
}

// ========== 从硬件读状态（由控制循环周期调用）==========
return_type Arx5System::read(const rclcpp::Time &, const rclcpp::Duration &) {

//   if (node_) {
//     rclcpp::spin_some(node_);
//   }

//   // 把 /arm_status 的 joint_pos/joint_vel 拷到 pos_/vel_
//   std::lock_guard<std::mutex> lk(mtx_);
//   if (!status_ready_) return return_type::OK;

//   // 期望 RobotStatus 里有 8 个关节（你的定义里确实有 joint_pos[7]=第8个）
//   const auto & jp = last_status_.joint_pos;
//   const auto & jv = last_status_.joint_vel;

//   const size_t Np = std::min(nj(), jp.size());
//   for (size_t i = 0; i < Np; ++i) pos_[i] = jp[i];

//   const size_t Nv = std::min(nj(), jv.size());
//   for (size_t i = 0; i < Nv; ++i) vel_[i] = jv[i];

//   return return_type::OK;

    // ★ 修法B的核心：保证订阅回调被执行
  if (node_) {
    rclcpp::spin_some(node_);
  }

  std::lock_guard<std::mutex> lk(mtx_);
  if (!status_ready_) {
    return hardware_interface::return_type::OK;
  }

  // 从缓存把状态拷到 ros2_control 的 state interfaces
  // 你的 RobotStatus: joint_pos[7], joint_vel[7]
  const auto &jp = last_status_.joint_pos;
  const auto &jv = last_status_.joint_vel;

  // 手臂六关节
  const size_t arm = std::min<size_t>(6, jp.size());
  for (size_t i = 0; i < arm; ++i) {
    pos_[i] = jp[i];
  }
  const size_t armv = std::min<size_t>(6, jv.size());
  for (size_t i = 0; i < armv; ++i) {
    vel_[i] = jv[i];
  }

  // 夹爪：状态里第7个（索引6） -> joint7/joint8
  if (jp.size() >= 7) {
    const double gpos = jp[6];
    pos_[6] = gpos;
    pos_[7] = gpos;
  }
  if (jv.size() >= 7) {
    const double gvel = jv[6];
    vel_[6] = gvel;
    vel_[7] = gvel;
  }

  RCLCPP_INFO_THROTTLE(
  node_->get_logger(), *node_->get_clock(), 2000,
  "JS write: pos0=%.3f pos1=%.3f pos2=%.3f g=%.3f",
  pos_[0], pos_[1], pos_[2], pos_[6]);
  return hardware_interface::return_type::OK;
}

// ========== 写命令到硬件（由控制循环周期调用）==========
return_type Arx5System::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // 保护：必须至少有手臂 6 轴 + 两指共 8 轴
  if (nj() < 8) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000 /*ms*/,
                          "Expected 8 joints (joint1..joint8). Got %zu.", nj());
    return return_type::ERROR;
  }

  arx5_arm_msg::msg::RobotCmd cmd;
  cmd.header.stamp = node_->get_clock()->now();

  // cmd.joint_pos 是 std::array<double,6>，逐个赋值即可
  for (size_t i = 0; i < 6; ++i) {
    cmd.joint_pos[i] = cmd_pos_[i];
  }

  // 两指合并为一个 gripper 值
  const double j7 = cmd_pos_[6];
  const double j8 = cmd_pos_[7];
  cmd.gripper = merge_gripper(j7, j8);

  // cmd.end_pos 也是 std::array<double,6>，统一清零
  cmd.end_pos.fill(0.0);

  cmd.mode = mode_;

  cmd_pub_->publish(cmd);
  return return_type::OK;
}


}  // namespace arx5_ros2_control

// 导出插件，供 pluginlib 发现
PLUGINLIB_EXPORT_CLASS(arx5_ros2_control::Arx5System, hardware_interface::SystemInterface)
