#ifndef ADA_HARDWARE_JACO2_H_
#define ADA_HARDWARE_JACO2_H_

#include "hardware_interface/system_interface.hpp"

namespace ada_hardware
{

class Jaco2 : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Jaco2);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the joint command/states
  std::vector<double> position_offsets_;  // Handles SO(2) wrap
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  // Command Mode
  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum integration_level_t : std::uint8_t { UNDEFINED = 0, POSITION = 1, VELOCITY = 2, EFFORT = 3 };

  integration_level_t control_level_;

  //// Private Functions /////
  bool sendVelocityCommand(const std::vector<double> & command);
  bool sendPositionCommand(const std::vector<double> & command);
  bool sendEffortCommand(const std::vector<double> & command);
  bool setTorqueMode(bool torqueMode);

  bool initializeOffsets();

  inline double radiansToFingerTicks(double radians)
  {
    return (6800.0 / 80) * radians * 180.0 /
           M_PI;  // this magic number was found in the kinova-ros code,
                  // kinova_driver/src/kinova_arm.cpp
  }

  inline double fingerTicksToRadians(double ticks)
  {
    return ticks * (80 / 6800.0) * M_PI /
           180.0;  // this magic number was found in the kinova-ros code,
                   // kinova_driver/src/kinova_arm.cpp
  }

};  // End class Jaco2

};  // End namespace ada_hardware

#endif
