// Copyright 2023 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Ethan K. Gordon

#include "ada_hardware/isaacsim.hpp"

// ROS
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ada_hardware
{

// Ensure Shutdown
JacoIsaac::~JacoIsaac()
{
  // Do Nothing
}

// Init: Read info and configure command/state buffers
hardware_interface::CallbackReturn JacoIsaac::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  callback_states_positions_.resize(info_.joints.size(), 0.0);
  callback_states_velocities_.resize(info_.joints.size(), 0.0);
  callback_states_efforts_.resize(info_.joints.size(), 0.0);
  control_level_ = integration_level_t::kUNDEFINED;
  control_connected_ = {false, false};

  position_offsets_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Robot (Jaco 2) has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("JacoIsaac"), "Joint '%s' has %zu command interfaces. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("JacoIsaac"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("JacoIsaac"), "Joint '%s'has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("JacoIsaac"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Create Publishers and Subscriber
  std::string ns = info_.hardware_parameters["namespace"];
  node_ = std::make_shared<rclcpp::Node>("JacoIsaac");
  exec_.add_node(node_);
  pos_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/position_command", 10);
  vel_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/velocity_command", 10);
  eff_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/effort_command", 10);

  sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    ns + "/joint_states", 10, std::bind(&JacoIsaac::joint_callback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JacoIsaac::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JacoIsaac::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

// Mode Switching
// All joints must be the same mode.
hardware_interface::return_type JacoIsaac::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare stopping command modes
  std::vector<integration_level_t> old_modes = {};
  for (std::string key : stop_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        old_modes.push_back(integration_level_t::kPOSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        old_modes.push_back(integration_level_t::kVELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        old_modes.push_back(integration_level_t::kEFFORT);
      }
    }
  }

  // Handle Stop
  if (old_modes.size() > 0) {
    // Criterion: All hand or all arm or all joints must be stopped at the same time
    if (
      old_modes.size() != num_dofs_.first && old_modes.size() != num_dofs_.second &&
      old_modes.size() != info_.joints.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JacoIsaac"),
        "Must stop all hand, arm, or robot joints simultaneously.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: All joints must have the same (existing) command mode
    if (!std::all_of(old_modes.begin() + 1, old_modes.end(), [&](integration_level_t mode) {
          return mode == control_level_;
        })) {
      RCLCPP_ERROR(rclcpp::get_logger("JacoIsaac"), "All stopped joints must be in the same mode.");
      return hardware_interface::return_type::ERROR;
    }

    // Record removal of connected control
    if (old_modes.size() == num_dofs_.second) {
      control_connected_.second = false;
    } else if (old_modes.size() == num_dofs_.first) {
      control_connected_.first = false;
    } else {
      control_connected_.first = control_connected_.second = false;
    }
    if (!control_connected_.first && !control_connected_.second) {
      control_level_ = integration_level_t::kUNDEFINED;
    }
  }

  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(integration_level_t::kPOSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(integration_level_t::kVELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes.push_back(integration_level_t::kEFFORT);
      }
    }
  }

  // Handle Start
  if (new_modes.size() > 0) {
    // Criterion: All hand or all arm or all joints must be given new command mode at the same time
    if (
      new_modes.size() != num_dofs_.first && new_modes.size() != num_dofs_.second &&
      new_modes.size() != info_.joints.size()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("JacoIsaac"),
        "Must request all hand, arm, or robot joints simultaneously.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: All joints must have the same command mode
    if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_level_t mode) {
          return mode == new_modes[0];
        })) {
      RCLCPP_ERROR(rclcpp::get_logger("JacoIsaac"), "All joints must be the same command mode.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: Joints must not be in use.
    bool inUse = (new_modes.size() == num_dofs_.second && control_connected_.second) ||
                 (new_modes.size() == num_dofs_.first && control_connected_.first) ||
                 (control_connected_.first && control_connected_.second);
    if (inUse) {
      RCLCPP_ERROR(rclcpp::get_logger("JacoIsaac"), "Joints already in use.");
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: if finger joints only, must be the same mode as what's already here
    if (new_modes.size() == num_dofs_.second) {
      if (control_level_ != integration_level_t::kUNDEFINED && new_modes[0] != control_level_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("JacoIsaac"), "Hand controller can't override arm control mode.");
        return hardware_interface::return_type::ERROR;
      }
    }

    // Set the new command mode
    // Record addition of connected control
    if (new_modes.size() == num_dofs_.second) {
      control_connected_.second = true;
    } else if (new_modes.size() == num_dofs_.first) {
      control_connected_.first = true;
    } else {
      control_connected_.first = control_connected_.second = true;
    }
    if (control_connected_.first || control_connected_.second) {
      control_level_ = new_modes[0];
    }
  }

  return hardware_interface::return_type::OK;
}

// Configure: init api, make sure joint number matches robot
hardware_interface::CallbackReturn JacoIsaac::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Fix joint configuration
  num_dofs_.first = 6;   // 6 arm joints
  num_dofs_.second = 2;  // 2 finger joints

  if (hw_states_positions_.size() != num_dofs_.first + num_dofs_.second) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JacoIsaac"),
      "Provided number of joints (%ld) does not match those reported by the robot (%ld)",
      hw_states_positions_.size(), num_dofs_.first + num_dofs_.second);
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_offsets_.resize(num_dofs_.first, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate: start control api, angular control, 0 values
hardware_interface::CallbackReturn JacoIsaac::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize Default Values
  auto ret = read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("JacoIsaac"), "Could not read default position.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initializeOffsets()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    if (std::isnan(hw_states_positions_[i])) {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = hw_states_positions_[i];
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_efforts_[i])) {
      hw_commands_efforts_[i] = 0;
    }
    control_level_ = integration_level_t::kUNDEFINED;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// This makes the reported joint values to start within urdf limits
static const double hardcoded_pos_midpoints[7] = {0.0, M_PI, M_PI, 0.0, 0.0, 0.0, 0.0};
bool JacoIsaac::initializeOffsets()
{
  // Clear and re-read offsets
  position_offsets_.clear();
  position_offsets_.resize(num_dofs_.first, 0.0);
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    return false;
  }

  // Next, we wrap the positions so they are within -pi to pi of
  // the hardcoded midpoints, and add that to the offset.
  for (size_t i = 0; i < num_dofs_.first; i++) {
    while (hw_states_positions_[i] < hardcoded_pos_midpoints[i] - M_PI) {
      hw_states_positions_[i] += 2.0 * M_PI;
      position_offsets_[i] += 2.0 * M_PI;
    }
    while (hw_states_positions_[i] > hardcoded_pos_midpoints[i] + M_PI) {
      hw_states_positions_[i] -= 2.0 * M_PI;
      position_offsets_[i] -= 2.0 * M_PI;
    }
  }
  return true;
}

// Deactivate: stop trajectories, cartesian control, stop control api
hardware_interface::CallbackReturn JacoIsaac::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup: close api, make sure joint number matches robot
hardware_interface::CallbackReturn JacoIsaac::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Shutdown: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn JacoIsaac::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

// Error: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn JacoIsaac::on_error(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Write Operations
bool JacoIsaac::sendVelocityCommand(const std::vector<double> & command)
{
  auto message = sensor_msgs::msg::JointState();
  message.name = std::vector<std::string>();
  for (auto joint : info_.joints) {
    message.name.push_back(joint.name);
  }
  message.position = command;
  vel_pub_->publish(message);

  return true;
}

bool JacoIsaac::sendPositionCommand(const std::vector<double> & command)
{
  auto message = sensor_msgs::msg::JointState();
  message.name = std::vector<std::string>();
  for (auto joint : info_.joints) {
    message.name.push_back(joint.name);
  }
  message.position = command;
  pos_pub_->publish(message);

  return true;
}

bool JacoIsaac::sendEffortCommand(const std::vector<double> & command)
{
  auto message = sensor_msgs::msg::JointState();
  message.name = std::vector<std::string>();
  for (auto joint : info_.joints) {
    message.name.push_back(joint.name);
  }
  message.position = command;
  eff_pub_->publish(message);

  return true;
}

hardware_interface::return_type JacoIsaac::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::lock_guard<std::mutex> lock(mMutex);
  std::vector<double> zero(num_dofs_.first + num_dofs_.second, 0.0);
  bool ret = true;

  switch (control_level_) {
    case integration_level_t::kVELOCITY:
      ret = sendVelocityCommand(hw_commands_velocities_);
      break;
    case integration_level_t::kPOSITION:
      ret = sendPositionCommand(hw_commands_positions_);
      break;
    case integration_level_t::kEFFORT:
      ret = sendEffortCommand(hw_commands_efforts_);
      break;
    default:
      // Stop Bot (Position is best for)
      ret = sendPositionCommand(hw_commands_positions_);
  }

  return ret ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

// Read Operation
void JacoIsaac::joint_callback(const sensor_msgs::msg::JointState & msg)
{
  std::vector<std::string> joint_names = msg.name;
  std::vector<double> positions = msg.position;
  std::vector<double> velocities = msg.velocity;
  std::vector<double> efforts = msg.effort;

  for (size_t j = 0; j < joint_names.size(); j++) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (joint_names[j] == info_.joints[i].name) {
        callback_states_positions_[i] = positions[j];
        callback_states_velocities_[i] = velocities[j];
        callback_states_efforts_[i] = efforts[j];
      }
    }
  }
}

hardware_interface::return_type JacoIsaac::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  exec_.spin_once();

  hw_states_positions_[0] = callback_states_positions_[0] + position_offsets_[0];
  hw_states_positions_[1] = callback_states_positions_[1] + position_offsets_[1];
  hw_states_positions_[2] = callback_states_positions_[2] + position_offsets_[2];
  hw_states_positions_[3] = callback_states_positions_[3] + position_offsets_[3];
  hw_states_positions_[4] = callback_states_positions_[4] + position_offsets_[4];
  hw_states_positions_[5] = callback_states_positions_[5] + position_offsets_[5];
  hw_states_positions_[6] = callback_states_positions_[6];
  hw_states_positions_[7] = callback_states_positions_[7];

  hw_states_velocities_[0] = callback_states_velocities_[0];
  hw_states_velocities_[1] = callback_states_velocities_[1];
  hw_states_velocities_[2] = callback_states_velocities_[2];
  hw_states_velocities_[3] = callback_states_velocities_[3];
  hw_states_velocities_[4] = callback_states_velocities_[4];
  hw_states_velocities_[5] = callback_states_velocities_[5];
  hw_states_velocities_[6] = callback_states_velocities_[6];
  hw_states_velocities_[7] = callback_states_velocities_[7];

  hw_states_efforts_[0] = callback_states_efforts_[0];
  hw_states_efforts_[1] = callback_states_efforts_[1];
  hw_states_efforts_[2] = callback_states_efforts_[2];
  hw_states_efforts_[3] = callback_states_efforts_[3];
  hw_states_efforts_[4] = callback_states_efforts_[4];
  hw_states_efforts_[5] = callback_states_efforts_[5];
  hw_states_efforts_[6] = callback_states_efforts_[6];
  hw_states_efforts_[7] = callback_states_efforts_[7];

  return hardware_interface::return_type::OK;
}

};  // namespace ada_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ada_hardware::JacoIsaac, hardware_interface::SystemInterface)
