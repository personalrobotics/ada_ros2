#ifndef JACO_ROBOT_H
#define JACO_ROBOT_H

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>

// ros
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

// kinova api
#include <kinova/Kinova.API.USBCommandLayerUbuntu.h>
#include <kinova/KinovaTypes.h>

// c++
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std;

// This makes the reported joint values to start within urdf limits
static const double hardcoded_pos_midpoints[6] = {0.0, M_PI, M_PI,
                                                  0.0, 0.0,  0.0};
static const int num_full_dof = 8;
static const int num_arm_dof = 6;
static const int num_finger_dof = 2;

static const ROBOT_TYPE our_robot_type = JACOV2_6DOF_ASSISTIVE;

class JacoRobot : public hardware_interface::RobotHW {
public:
  JacoRobot(ros::NodeHandle nh);

  virtual ~JacoRobot();

  void initializeOffsets();

  ros::Time get_time(void);

  ros::Duration get_period(void);

  inline double degreesToRadians(double degrees);
  inline double radiansToDegrees(double radians);
  inline double radiansToFingerTicks(double radians);
  inline double fingerTicksToRadians(double ticks);

  void sendPositionCommand(const std::vector<double> &command);
  void sendVelocityCommand(const std::vector<double> &command);
  void sendTorqueCommand(const std::vector<double> &command);

  void write(void);
  void read(void);

  void checkForStall(void);

  // Gravcomp calibration functions
  std::vector<float> calcGravcompParams(void);
  bool zeroTorqueSensors(void);

  // Gravcomp functions
  // Return true if mode successfully switched.
  void enterGravComp(void);
  bool useGravcompForEStop(bool use,
                           std::vector<float> params = std::vector<float>());
  bool useGravcompForEStop(bool use, std::string fileName);
  bool setTorqueMode(bool torqueMode);
  bool eff_stall;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::JointModeInterface jm_interface;

  // Joint Commands
  vector<double> cmd_pos;
  vector<double> cmd_vel;
  vector<double> cmd_eff;
  vector<double> prev_cmd_pos;

  // Joint State
  vector<double> pos;
  vector<double> vel;
  vector<double> eff;

  // Parameters
  vector<double> pos_offsets;
  vector<double> soft_limits;

  // Switch between joint command type
  hardware_interface::JointCommandModes joint_mode;
  hardware_interface::JointCommandModes last_mode;

  // Only do grav comp if we have the parameters
  bool mUseGravComp;
  bool mInTorqueMode;
};

#endif
