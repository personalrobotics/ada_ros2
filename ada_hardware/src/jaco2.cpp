#include "ada_hardware/jaco2.hpp"

// kinova api
#include <Kinova.API.USBCommandLayerUbuntu.h>
#include <KinovaTypes.h>

namespace ada_hardware
{
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
  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);

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

  RobotIdentity robot;
  r = GetRobotIdentity(robot);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not get Robot Identity: Error code %d", r);
    CloseAPI();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check joint configuration
  size_t expected_dof = 0;
  switch (robot.RobotType) {
    case eRobotType_JacoV2_6DOF_Assistive:
    case eRobotType_JacoV2_6Dof_Service:
      expected_dof = 6 + 2;  // 6 arm + 2 fingers
      break;
    case eRobotType_Spherical_6DOF_Service:
      expected_dof = 6 + 3;  // 6 arm + 3 fingers
      break;
    case eRobotType_Spherical_7DOF_Service:
      expected_dof = 7 + 3;  // 7 arm + 3 fingers
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("Jaco2"), "Robot Identity (%d) not supported", robot.RobotType);
      CloseAPI();
      return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_states_positions_.size() != expected_dof) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Jaco2"),
      "Provided number of joints (%d) does not match those reported by the robot (%d)",
      hw_states_positions_.size(), expected_dof);
    CloseAPI();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate: start control api, angular control, 0 values
hardware_interface::CallbackReturn Jaco2::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  int r = NO_ERROR_KINOVA;
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

  // Initialize Default Values
  auto ret = read(rclcpp::Time(0), rclcpp::Duration(0));
  if (ret != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not read default position.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    if (std::isnan(hw_states_positions_[i])) {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i])) {
      hw_states_efforts_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = hw_states_positions_[i];
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i])) {
      hw_commands_accelerations_[i] = 0;
    }
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate: stop trajectories, cartesian control, stop control api
hardware_interface::CallbackReturn Jaco2::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!setTorqueMode(false)) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop torque mode; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  int r = NO_ERROR_KINOVA;
  r = EraseAllTrajectories();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop trajectories; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = SetCartesianControl();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not set Cartesian control; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  r = StopControlAPI();
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not stop control API; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup: close api, make sure joint number matches robot
hardware_interface::CallbackReturn Jaco2::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  CloseAPI();
}

// Shutdown: make sure deactivate + cleanup calls happen
hardware_interface::CallbackReturn Jaco2::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.label() == "active") {
    setTorqueMode(false);
    EraseAllTrajectories();
    SetCartesianControl();
    StopControlAPI();
  }
  CloseAPI();

  return hardware_interface::CallbackReturn::SUCCESS;
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
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not read position; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }
  r = GetAngularVelocity(arm_vel);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not read velocity; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
  }
  r = GetAngularForce(arm_eff);
  if (r != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(rclcpp::get_logger("Jaco2"), "Could not read effort; Error code %d", r);
    return hardware_interface::CallbackReturn::ERROR;
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
  hw_states_positions_[6] = fingerTicksToRadians(double(arm_pos.Fingers.Finger1));
  hw_states_positions_[7] = fingerTicksToRadians(double(arm_pos.Fingers.Finger2));

  // According to kinova-ros, the reported values are half of the actual.
  vel[0] = degreesToRadians(double(arm_vel.Actuators.Actuator1));
  vel[1] = degreesToRadians(double(arm_vel.Actuators.Actuator2));
  vel[2] = degreesToRadians(double(arm_vel.Actuators.Actuator3));
  vel[3] = degreesToRadians(double(arm_vel.Actuators.Actuator4));
  vel[4] = degreesToRadians(double(arm_vel.Actuators.Actuator5));
  vel[5] = degreesToRadians(double(arm_vel.Actuators.Actuator6));
  vel[6] = fingerTicksToRadians(double(arm_vel.Fingers.Finger1));
  vel[7] = fingerTicksToRadians(double(arm_vel.Fingers.Finger2));

  eff[0] = arm_torq.Actuators.Actuator1;
  eff[1] = arm_torq.Actuators.Actuator2;
  eff[2] = arm_torq.Actuators.Actuator3;
  eff[3] = arm_torq.Actuators.Actuator4;
  eff[4] = arm_torq.Actuators.Actuator5;
  eff[5] = arm_torq.Actuators.Actuator6;
  eff[6] = arm_torq.Fingers.Finger2;
  eff[7] = arm_torq.Fingers.Finger2;
}

};  // namespace ada_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ada_hardware::Jaco2, hardware_interface::SystemInterface)