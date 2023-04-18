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

#include "ada_hardware/jaco2.hpp"

// kinova api
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <KinovaTypes.h>

// ROS
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ada_hardware
{

// Ensure Shutdown
Jaco2::~Jaco2()
{
  setTorqueMode(false);

  EraseAllTrajectories();

  SetCartesianControl();

  StopControlAPI();

  CloseAPI();
}

// Init: Read info and configure command/state buffers
hardware_interface::CallbackReturn Jaco2::on_init(const hardware_interface::HardwareInfo & info)
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
  control_level_ = integration_level_t::kUNDEFINED;

  position_offsets_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // JACO2 has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Jaco2"), "Joint '%s' has %zu command interfaces. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Jaco2"), "Joint '%s' has %s command interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Jaco2"), "Joint '%s'has %zu state interfaces. 3 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Jaco2"), "Joint '%s' has %s state interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Jaco2::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> Jaco2::export_command_interfaces()
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
hardware_interface::return_type Jaco2::prepare_command_mode_switch(
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
      return hardware_interface::return_type::ERROR;
    }

    // If arm or whole-bot, remove control_level
    if (old_modes.size() != num_dofs_.second) {
      // Criterion: All joints must have the same (existing) command mode
      if (!std::all_of(old_modes.begin() + 1, old_modes.end(), [&](integration_level_t mode) {
            return mode == control_level_;
          })) {
        return hardware_interface::return_type::ERROR;
      }
      // Stop motion
      {
        const std::lock_guard<std::mutex> lock(mMutex);
        int r = NO_ERROR_KINOVA;
        r = EraseAllTrajectories();
        if (r != NO_ERROR_KINOVA) {
          RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop robot : Error code %d", r);
          return hardware_interface::return_type::ERROR;
        }
      }
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
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: All joints must have the same command mode
    if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_level_t mode) {
          return mode == new_modes[0];
        })) {
      return hardware_interface::return_type::ERROR;
    }

    // Criterion: if finger joints only, must be the same mode as what's already here
    if (new_modes.size() == num_dofs_.second) {
      if (new_modes[0] != control_level_) {
        return hardware_interface::return_type::ERROR;
      }
    }
    // Criterion: Only one mode active at a time
    else if (control_level_ != integration_level_t::kUNDEFINED) {
      return hardware_interface::return_type::ERROR;
    }

    // Set the new command mode
    // Stop motion
    {
      const std::lock_guard<std::mutex> lock(mMutex);
      int r = NO_ERROR_KINOVA;
      r = EraseAllTrajectories();
      if (r != NO_ERROR_KINOVA) {
        RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop robot : Error code %d", r);
        return hardware_interface::return_type::ERROR;
      }
    }
    control_level_ = new_modes[0];
  }

  return hardware_interface::return_type::OK;
}

// Configure: init api, make sure joint number matches robot
hardware_interface::CallbackReturn Jaco2::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  int r = NO_ERROR_KINOVA;
  r = InitAPI();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not initialize API: Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = InitFingers();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not initialize Fingers: Error code %d", r);
    CloseAPI();
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = StartControlAPI();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not start API Control: Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = SetAngularControl();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not set angular control: Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = SetTorqueSafetyFactor(1.0f);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not send : Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  KinovaDevice robot;
  r = GetActiveDevice(robot);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not get Active Device: Error code %d", r);
    CloseAPI();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check joint configuration
  num_dofs_.first = 0;
  num_dofs_.second = 0;
  switch (robot.DeviceType) {
    case ROBOT_CONFIG_JACOV2_6DOF_ASSISTIVE:
    case ROBOT_CONFIG_JACOV2_6DOF_SERVICE:
      // 6 arm + 2 fingers
      num_dofs_.first = 6;
      num_dofs_.second = 2;
      break;
    case ROBOT_CONFIG_SPHERICAL_6DOF_SERVICE:
      // 6 arm + 3 fingers
      num_dofs_.first = 6;
      num_dofs_.second = 3;
      break;
    case ROBOT_CONFIG_SPHERICAL_7DOF_SERVICE:
      // 7 arm + 3 fingers
      num_dofs_.first = 7;
      num_dofs_.second = 3;
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("Jaco2"), "Robot Identity (%d) not supported", robot.DeviceType);
      CloseAPI();
      return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_states_positions_.size() != num_dofs_.first + num_dofs_.second) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"),
      "Provided number of joints (%ld) does not match those reported by the robot (%ld)",
      hw_states_positions_.size(), num_dofs_.first + num_dofs_.second);
    CloseAPI();
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_offsets_.resize(num_dofs_.first, 0.0);

  if (!setTorqueMode(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not set torque mode on configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate: start control api, angular control, 0 values
hardware_interface::CallbackReturn Jaco2::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize Default Values
  auto ret = read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not read default position.");
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

  if (!setTorqueMode(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not set torque mode on configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// This makes the reported joint values to start within urdf limits
static const double hardcoded_pos_midpoints[7] = {0.0, M_PI, M_PI, 0.0, 0.0, 0.0, 0.0};
bool Jaco2::initializeOffsets()
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
hardware_interface::CallbackReturn Jaco2::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!setTorqueMode(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop torque mode");
    return hardware_interface::CallbackReturn::ERROR;
  }

  int r = NO_ERROR_KINOVA;
  r = EraseAllTrajectories();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop trajectories; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup: close api, make sure joint number matches robot
hardware_interface::CallbackReturn Jaco2::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  int r = NO_ERROR_KINOVA;
  r = SetCartesianControl();
  if (r != NO_ERROR_KINOVA) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  r = StopControlAPI();
  if (r != NO_ERROR_KINOVA) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  r = CloseAPI();
  if (r != NO_ERROR_KINOVA) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Shutdown: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn Jaco2::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.label() == "active") {
    setTorqueMode(false);
    EraseAllTrajectories();
  }
  return on_cleanup(previous_state);
}

// Error: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn Jaco2::on_error(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  setTorqueMode(false);
  EraseAllTrajectories();
  SetCartesianControl();
  StopControlAPI();
  CloseAPI();

  return hardware_interface::CallbackReturn::SUCCESS;
}

/////// Inline Unit Conversions
inline static double degreesToRadians(double degrees) { return (M_PI / 180.0) * degrees; }

inline static double radiansToDegrees(double radians) { return (180.0 / M_PI) * radians; }

inline static double radiansToFingerTicks(double radians)
{
  return (6800.0 / 80) * radians * 180.0 /
         M_PI;  // this magic number was found in the kinova-ros code,
                // kinova_driver/src/kinova_arm.cpp
}

inline static double fingerTicksToRadians(double ticks)
{
  return ticks * (80 / 6800.0) * M_PI /
         180.0;  // this magic number was found in the kinova-ros code,
                 // kinova_driver/src/kinova_arm.cpp
}

// Write Operations
bool Jaco2::setTorqueMode(bool torqueMode)
{
  int r = NO_ERROR_KINOVA;
  EraseAllTrajectories();

  r = SwitchTrajectoryTorque(torqueMode ? TORQUE : POSITION);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not set torque mode : Error code %d", r);
    return false;
  }

  return true;
}

bool Jaco2::sendVelocityCommand(const std::vector<double> & command)
{
  // Check if in torque mode
  {
    int mode = 0;
    int r = NO_ERROR_KINOVA;
    r = GetTrajectoryTorqueMode(mode);
    if (r != NO_ERROR_KINOVA) {
      RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not get torque mode : Error code %d", r);
      return false;
    }
    if (mode) {
      RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "In torque mode. Dropping...");
      if (!setTorqueMode(false)) {
        RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not exit torque mode.");
        return true;
      }
    }
  }

  if (command.size() != num_dofs_.first + num_dofs_.second) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"), "Incorrect command size (%ld), expected (%ld)", command.size(),
      num_dofs_.first + num_dofs_.second);
    return false;
  }
  // Need to send an "advance trajectory" with a single point and the correct
  // settings Angular velocity

  AngularInfo joint_vel;
  joint_vel.InitStruct();
  joint_vel.Actuator1 = float(radiansToDegrees(command.at(0)));
  joint_vel.Actuator2 = float(radiansToDegrees(command.at(1)));
  joint_vel.Actuator3 = float(radiansToDegrees(command.at(2)));
  joint_vel.Actuator4 = float(radiansToDegrees(command.at(3)));
  joint_vel.Actuator5 = float(radiansToDegrees(command.at(4)));
  joint_vel.Actuator6 = float(radiansToDegrees(command.at(5)));
  int fingerStart = 6;
  if (num_dofs_.first > 6) {
    joint_vel.Actuator7 = float(radiansToDegrees(command.at(6)));
    fingerStart++;
  }

  TrajectoryPoint trajectory;
  trajectory.InitStruct();
  memset(&trajectory, 0, sizeof(trajectory));

  trajectory.Position.Type = ANGULAR_VELOCITY;
  trajectory.Position.HandMode = VELOCITY_MODE;
  trajectory.Position.Actuators = joint_vel;

  trajectory.Position.Fingers.Finger1 = float(radiansToFingerTicks(command.at(fingerStart)));
  trajectory.Position.Fingers.Finger2 = float(radiansToFingerTicks(command.at(fingerStart + 1)));
  if (num_dofs_.second > 2) {
    trajectory.Position.Fingers.Finger3 = float(radiansToFingerTicks(command.at(fingerStart + 2)));
  }

  int r = NO_ERROR_KINOVA;
  r = SendAdvanceTrajectory(trajectory);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"), "Could not send velocity command ; Error Code: %d", r);
    return false;
  }

  return true;
}

bool Jaco2::sendPositionCommand(const std::vector<double> & command)
{
  // Check if in torque mode
  {
    int mode = 0;
    int r = NO_ERROR_KINOVA;
    r = GetTrajectoryTorqueMode(mode);
    if (r != NO_ERROR_KINOVA) {
      RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not get torque mode : Error code %d", r);
      return false;
    }
    if (mode) {
      RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "In torque mode. Dropping...");
      if (!setTorqueMode(false)) {
        RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not exit torque mode.");
        return true;
      }
    }
  }

  static std::vector<double> prev_command;

  if (command.size() != num_dofs_.first + num_dofs_.second) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"), "Incorrect command size (%ld), expected (%ld)", command.size(),
      num_dofs_.first + num_dofs_.second);
    return false;
  }
  // Need to send an "advance trajectory" with a single point and the correct
  // settings Angular velocity

  AngularInfo joint_pos;
  joint_pos.InitStruct();
  joint_pos.Actuator1 = float(radiansToDegrees(command.at(0) - position_offsets_[0]));
  joint_pos.Actuator2 = float(radiansToDegrees(command.at(1) - position_offsets_[1]));
  joint_pos.Actuator3 = float(radiansToDegrees(command.at(2) - position_offsets_[2]));
  joint_pos.Actuator4 = float(radiansToDegrees(command.at(3) - position_offsets_[3]));
  joint_pos.Actuator5 = float(radiansToDegrees(command.at(4) - position_offsets_[4]));
  joint_pos.Actuator6 = float(radiansToDegrees(command.at(5) - position_offsets_[5]));
  int fingerStart = 6;
  if (num_dofs_.first > 6) {
    joint_pos.Actuator7 = float(radiansToDegrees(command.at(6) - position_offsets_[6]));
    fingerStart++;
  }

  TrajectoryPoint trajectory;
  trajectory.InitStruct();
  memset(&trajectory, 0, sizeof(trajectory));

  trajectory.Position.Delay = 0.0;
  trajectory.Position.Type = ANGULAR_POSITION;
  trajectory.Position.HandMode = POSITION_MODE;
  trajectory.Position.Actuators = joint_pos;

  trajectory.Position.Fingers.Finger1 = float(radiansToFingerTicks(command.at(fingerStart)));
  trajectory.Position.Fingers.Finger2 = float(radiansToFingerTicks(command.at(fingerStart + 1)));
  if (num_dofs_.second > 2) {
    trajectory.Position.Fingers.Finger3 = float(radiansToFingerTicks(command.at(fingerStart + 2)));
  }

  // Clear queue if new command
  if (command != prev_command) {
    EraseAllTrajectories();
    prev_command = command;
  }

  int r = NO_ERROR_KINOVA;
  r = SendAdvanceTrajectory(trajectory);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"), "Could not send position command ; Error Code: %d", r);
    return false;
  }

  return true;
}

bool Jaco2::sendEffortCommand(const std::vector<double> & command)
{
  // Check if in torque mode
  int mode = 0;
  int r = NO_ERROR_KINOVA;
  r = GetTrajectoryTorqueMode(mode);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not get torque mode : Error code %d", r);
    return false;
  }
  if (!mode) {
    RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Dropped out of torque mode. Retrying...");
    if (!setTorqueMode(true)) {
      RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not enter torque mode.");
      return true;
    }
  }

  // Send Torque Command
  float joint_eff[COMMAND_SIZE] = {0};
  std::copy(command.begin(), command.end(), joint_eff);

  r = SendAngularTorqueCommand(joint_eff);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not send effort command : Error code %d", r);
    return false;
  }

  return true;
}

hardware_interface::return_type Jaco2::write(
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
      // Stop Bot
      ret = sendVelocityCommand(zero);
  }

  return ret ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

// Read Operation
hardware_interface::return_type Jaco2::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // make sure that pos, vel, and eff are up to date.
  // TODO: If there is too much lag between calling read()
  // and getting the actual values back, we'll need to be
  // reading values constantly and storing them locally, so
  // at least there is a recent value available for the controller.

  AngularPosition arm_pos;
  AngularPosition arm_vel;
  AngularPosition arm_eff;

  // Requires 3 seperate calls to the USB
  int r = NO_ERROR_KINOVA;
  r = GetAngularPosition(arm_pos);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not read position; Error code %d", r);
    return hardware_interface::return_type::OK;
  }
  r = GetAngularVelocity(arm_vel);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not read velocity; Error code %d", r);
    return hardware_interface::return_type::OK;
  }
  r = GetAngularForce(arm_eff);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_WARN(rclcpp::get_logger("Jaco2"), "Could not read effort; Error code %d", r);
    return hardware_interface::return_type::OK;
  }

  hw_states_positions_[0] =
    degreesToRadians(double(arm_pos.Actuators.Actuator1)) + position_offsets_[0];
  hw_states_positions_[1] =
    degreesToRadians(double(arm_pos.Actuators.Actuator2)) + position_offsets_[1];
  hw_states_positions_[2] =
    degreesToRadians(double(arm_pos.Actuators.Actuator3)) + position_offsets_[2];
  hw_states_positions_[3] =
    degreesToRadians(double(arm_pos.Actuators.Actuator4)) + position_offsets_[3];
  hw_states_positions_[4] =
    degreesToRadians(double(arm_pos.Actuators.Actuator5)) + position_offsets_[4];
  hw_states_positions_[5] =
    degreesToRadians(double(arm_pos.Actuators.Actuator6)) + position_offsets_[5];
  if (num_dofs_.first > 6) {
    hw_states_positions_[6] =
      degreesToRadians(double(arm_pos.Actuators.Actuator7)) + position_offsets_[6];
    hw_states_positions_[7] = fingerTicksToRadians(double(arm_pos.Fingers.Finger1));
    hw_states_positions_[8] = fingerTicksToRadians(double(arm_pos.Fingers.Finger2));
  } else {
    hw_states_positions_[6] = fingerTicksToRadians(double(arm_pos.Fingers.Finger1));
    hw_states_positions_[7] = fingerTicksToRadians(double(arm_pos.Fingers.Finger2));
  }
  if (num_dofs_.second > 2) {
    hw_states_positions_[hw_states_positions_.size() - 1] =
      fingerTicksToRadians(double(arm_pos.Fingers.Finger3));
  }

  // According to kinova-ros, the reported values are half of the actual.
  hw_states_velocities_[0] = degreesToRadians(double(arm_vel.Actuators.Actuator1));
  hw_states_velocities_[1] = degreesToRadians(double(arm_vel.Actuators.Actuator2));
  hw_states_velocities_[2] = degreesToRadians(double(arm_vel.Actuators.Actuator3));
  hw_states_velocities_[3] = degreesToRadians(double(arm_vel.Actuators.Actuator4));
  hw_states_velocities_[4] = degreesToRadians(double(arm_vel.Actuators.Actuator5));
  hw_states_velocities_[5] = degreesToRadians(double(arm_vel.Actuators.Actuator6));
  if (num_dofs_.first > 6) {
    hw_states_velocities_[6] = degreesToRadians(double(arm_vel.Actuators.Actuator7));
    hw_states_velocities_[7] = fingerTicksToRadians(double(arm_vel.Fingers.Finger1));
    hw_states_velocities_[8] = fingerTicksToRadians(double(arm_vel.Fingers.Finger2));
  } else {
    hw_states_velocities_[6] = fingerTicksToRadians(double(arm_vel.Fingers.Finger1));
    hw_states_velocities_[7] = fingerTicksToRadians(double(arm_vel.Fingers.Finger2));
  }
  if (num_dofs_.second > 2) {
    hw_states_velocities_[hw_states_velocities_.size() - 1] =
      fingerTicksToRadians(double(arm_vel.Fingers.Finger3));
  }

  hw_states_efforts_[0] = arm_eff.Actuators.Actuator1;
  hw_states_efforts_[1] = arm_eff.Actuators.Actuator2;
  hw_states_efforts_[2] = arm_eff.Actuators.Actuator3;
  hw_states_efforts_[3] = arm_eff.Actuators.Actuator4;
  hw_states_efforts_[4] = arm_eff.Actuators.Actuator5;
  hw_states_efforts_[5] = arm_eff.Actuators.Actuator6;
  if (num_dofs_.first > 6) {
    hw_states_efforts_[6] = arm_eff.Actuators.Actuator7;
    hw_states_efforts_[7] = arm_eff.Fingers.Finger1;
    hw_states_efforts_[8] = arm_eff.Fingers.Finger2;
  } else {
    hw_states_efforts_[6] = arm_eff.Fingers.Finger1;
    hw_states_efforts_[7] = arm_eff.Fingers.Finger2;
  }
  if (num_dofs_.second > 2) {
    hw_states_efforts_[hw_states_efforts_.size() - 1] = arm_eff.Fingers.Finger3;
  }

  return hardware_interface::return_type::OK;
}

};  // namespace ada_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ada_hardware::Jaco2, hardware_interface::SystemInterface)
