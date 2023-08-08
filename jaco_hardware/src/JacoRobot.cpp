#include "JacoRobot.h"
#include <cmath> // std::abs

using namespace std;

JacoRobot::JacoRobot(ros::NodeHandle nh) {
  ROS_INFO("Starting to initialize jaco_hardware");
  int i;
  cmd_pos.resize(num_full_dof);
  cmd_vel.resize(num_full_dof);
  cmd_eff.resize(num_full_dof);

  pos.resize(num_full_dof);
  vel.resize(num_full_dof);
  eff.resize(num_full_dof);

  pos_offsets.resize(num_arm_dof);
  soft_limits.resize(num_full_dof);

  for (std::size_t i = 0; i < pos_offsets.size(); ++i)
    pos_offsets[i] = 0.0;

  for (std::size_t i = 0; i < cmd_vel.size(); ++i)
    cmd_vel[i] = 0.0;

  // connect and register the joint state interface.
  // this gives joint states (pos, vel, eff) back as an output.
  hardware_interface::JointStateHandle state_handle_base(
      "j2n6s200_joint_1", &pos[0], &vel[0], &eff[0]);
  hardware_interface::JointStateHandle state_handle_shoulder(
      "j2n6s200_joint_2", &pos[1], &vel[1], &eff[1]);
  hardware_interface::JointStateHandle state_handle_elbow(
      "j2n6s200_joint_3", &pos[2], &vel[2], &eff[2]);
  hardware_interface::JointStateHandle state_handle_wrist0(
      "j2n6s200_joint_4", &pos[3], &vel[3], &eff[3]);
  hardware_interface::JointStateHandle state_handle_wrist1(
      "j2n6s200_joint_5", &pos[4], &vel[4], &eff[4]);
  hardware_interface::JointStateHandle state_handle_wrist2(
      "j2n6s200_joint_6", &pos[5], &vel[5], &eff[5]);
  hardware_interface::JointStateHandle state_handle_finger0(
      "j2n6s200_joint_finger_1", &pos[6], &vel[6], &eff[6]);
  hardware_interface::JointStateHandle state_handle_finger1(
      "j2n6s200_joint_finger_2", &pos[7], &vel[7], &eff[7]);

  jnt_state_interface.registerHandle(state_handle_base);
  jnt_state_interface.registerHandle(state_handle_shoulder);
  jnt_state_interface.registerHandle(state_handle_elbow);
  jnt_state_interface.registerHandle(state_handle_wrist0);
  jnt_state_interface.registerHandle(state_handle_wrist1);
  jnt_state_interface.registerHandle(state_handle_wrist2);
  jnt_state_interface.registerHandle(state_handle_finger0);
  jnt_state_interface.registerHandle(state_handle_finger1);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  // this takes joint velocities in as a command.
  hardware_interface::JointHandle vel_handle_base(
      jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_vel[0]);
  hardware_interface::JointHandle vel_handle_shoulder(
      jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_vel[1]);
  hardware_interface::JointHandle vel_handle_elbow(
      jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_vel[2]);
  hardware_interface::JointHandle vel_handle_wrist0(
      jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_vel[3]);
  hardware_interface::JointHandle vel_handle_wrist1(
      jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_vel[4]);
  hardware_interface::JointHandle vel_handle_wrist2(
      jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_vel[5]);
  hardware_interface::JointHandle vel_handle_finger0(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_vel[6]);
  hardware_interface::JointHandle vel_handle_finger1(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_vel[7]);

  jnt_vel_interface.registerHandle(vel_handle_base);
  jnt_vel_interface.registerHandle(vel_handle_shoulder);
  jnt_vel_interface.registerHandle(vel_handle_elbow);
  jnt_vel_interface.registerHandle(vel_handle_wrist0);
  jnt_vel_interface.registerHandle(vel_handle_wrist1);
  jnt_vel_interface.registerHandle(vel_handle_wrist2);
  jnt_vel_interface.registerHandle(vel_handle_finger0);
  jnt_vel_interface.registerHandle(vel_handle_finger1);

  registerInterface(&jnt_vel_interface);

  // connect and register the joint position interface
  // this takes joint positions in as a command.
  hardware_interface::JointHandle pos_handle_base(
      jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_pos[0]);
  hardware_interface::JointHandle pos_handle_shoulder(
      jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_pos[1]);
  hardware_interface::JointHandle pos_handle_elbow(
      jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_pos[2]);
  hardware_interface::JointHandle pos_handle_wrist0(
      jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_pos[3]);
  hardware_interface::JointHandle pos_handle_wrist1(
      jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_pos[4]);
  hardware_interface::JointHandle pos_handle_wrist2(
      jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_pos[5]);
  hardware_interface::JointHandle pos_handle_finger0(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_pos[6]);
  hardware_interface::JointHandle pos_handle_finger1(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_pos[7]);

  jnt_pos_interface.registerHandle(pos_handle_base);
  jnt_pos_interface.registerHandle(pos_handle_shoulder);
  jnt_pos_interface.registerHandle(pos_handle_elbow);
  jnt_pos_interface.registerHandle(pos_handle_wrist0);
  jnt_pos_interface.registerHandle(pos_handle_wrist1);
  jnt_pos_interface.registerHandle(pos_handle_wrist2);
  jnt_pos_interface.registerHandle(pos_handle_finger0);
  jnt_pos_interface.registerHandle(pos_handle_finger1);

  registerInterface(&jnt_pos_interface);

  ROS_INFO("Register Effort Interface...");

  // connect and register the joint position interface
  // this takes joint effort in as a command.
  hardware_interface::JointHandle eff_handle_base(
      jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_eff[0]);
  hardware_interface::JointHandle eff_handle_shoulder(
      jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_eff[1]);
  hardware_interface::JointHandle eff_handle_elbow(
      jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_eff[2]);
  hardware_interface::JointHandle eff_handle_wrist0(
      jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_eff[3]);
  hardware_interface::JointHandle eff_handle_wrist1(
      jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_eff[4]);
  hardware_interface::JointHandle eff_handle_wrist2(
      jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_eff[5]);
  hardware_interface::JointHandle eff_handle_finger0(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_eff[6]);
  hardware_interface::JointHandle eff_handle_finger1(
      jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_eff[7]);

  jnt_eff_interface.registerHandle(eff_handle_base);
  jnt_eff_interface.registerHandle(eff_handle_shoulder);
  jnt_eff_interface.registerHandle(eff_handle_elbow);
  jnt_eff_interface.registerHandle(eff_handle_wrist0);
  jnt_eff_interface.registerHandle(eff_handle_wrist1);
  jnt_eff_interface.registerHandle(eff_handle_wrist2);
  jnt_eff_interface.registerHandle(eff_handle_finger0);
  jnt_eff_interface.registerHandle(eff_handle_finger1);

  registerInterface(&jnt_eff_interface);

  // connect and register the joint mode interface
  // this is needed to determine which control mode is needed
  hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
  jm_interface.registerHandle(mode_handle);
  registerInterface(&jm_interface);

  eff_stall = false;

  // Start Up Kinova API
  int r = NO_ERROR_KINOVA;

  ROS_INFO("Attempting to inialize API...");
  r = InitAPI();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not initialize API: Error code %d", r);
  }

  ROS_INFO("Attempting to initialize fingers...");
  r = InitFingers();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not initialize fingers: Error code %d", r);
  }

  ROS_INFO("Attempting to start API control of the robot...");
  r = StartControlAPI();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not start API Control: Error code %d", r);
  }

  ROS_INFO("Attempting to set angular control...");
  r = SetAngularControl();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not set angular control: Error code %d", r);
  }

  ROS_INFO("Attempting to set torque safety factor...");
  r = SetTorqueSafetyFactor(1.0f);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send : Error code %d", r);
  }

  // get soft limits from rosparams
  if (nh.hasParam("soft_limits/eff")) {
    nh.getParam("soft_limits/eff", soft_limits);
  } else {
    ROS_WARN("No JACO soft limits in param server! Using defaults.");
    const double defaults[] = {16, 16, 16, 10, 10, 10, 1.3, 1.3};
    soft_limits.assign(defaults, defaults + num_full_dof);
  }
  ROS_INFO("Set soft_limits for eff to: [%f,%f,%f,%f,%f,%f,%f,%f]",
           soft_limits[0], soft_limits[1], soft_limits[2], soft_limits[3],
           soft_limits[4], soft_limits[5], soft_limits[6], soft_limits[7]);

  // initialize default positions
  initializeOffsets();

  // Default to velocity mode
  // This will also clear out any trajectories at the beginning.
  joint_mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
  last_mode = hardware_interface::JointCommandModes::BEGIN;

  // Default to no grav comp
  mUseGravComp = false;
  mInTorqueMode = false;
}

JacoRobot::~JacoRobot() {
  int r = NO_ERROR_KINOVA;

  ROS_INFO("Erase all trajectories");
  r = EraseAllTrajectories();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not erase trajectories: Error code %d", r);
  }

  r = StopControlAPI();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not stop API Control: Error code %d", r);
  }
  r = CloseAPI();
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not close API Control: Error code %d", r);
  }

  ros::Duration(0.10).sleep();
}

void JacoRobot::initializeOffsets() {
  this->read();

  // Next, we wrap the positions so they are within -pi to pi of
  // the hardcoded midpoints, and add that to the offset.
  for (int i = 0; i < num_arm_dof; i++) {
    while (this->pos[i] < hardcoded_pos_midpoints[i] - M_PI) {
      this->pos[i] += 2.0 * M_PI;
      this->pos_offsets[i] += 2.0 * M_PI;
    }
    while (this->pos[i] > hardcoded_pos_midpoints[i] + M_PI) {
      this->pos[i] -= 2.0 * M_PI;
      this->pos_offsets[i] -= 2.0 * M_PI;
    }

    ROS_INFO("Joint %d: %f %f", i, this->pos[i], this->pos_offsets[i]);
  }
}

ros::Time JacoRobot::get_time(void) { return ros::Time::now(); }

ros::Duration JacoRobot::get_period(void) {
  // TODO(benwr): What is a reasonable period?
  // Here I've assumed  10ms
  return ros::Duration(0.01);
}

inline double JacoRobot::degreesToRadians(double degrees) {
  return (M_PI / 180.0) * degrees;
}

inline double JacoRobot::radiansToDegrees(double radians) {
  return (180.0 / M_PI) * radians;
}

inline double JacoRobot::radiansToFingerTicks(double radians) {
  return (6800.0 / 80) * radians * 180.0 /
         M_PI; // this magic number was found in the kinova-ros code,
               // kinova_driver/src/kinova_arm.cpp
}

inline double JacoRobot::fingerTicksToRadians(double ticks) {
  return ticks * (80 / 6800.0) * M_PI /
         180.0; // this magic number was found in the kinova-ros code,
                // kinova_driver/src/kinova_arm.cpp
}

bool JacoRobot::setTorqueMode(bool torqueMode) {

  int r = NO_ERROR_KINOVA;

  r = SwitchTrajectoryTorque(torqueMode ? TORQUE : POSITION);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send : Error code %d", r);
    return false;
  }

  mInTorqueMode = torqueMode;
  return true;
}

bool JacoRobot::useGravcompForEStop(bool use, std::string fileName) {
  if (fileName.empty()) {
    return false;
  }

  std::string path = ros::package::getPath("jaco_hardware") + "/" + fileName;
  std::vector<float> params;

  std::ifstream file(path);
  if (!file.is_open()) {
    ROS_ERROR("Could not open file: %s", path.c_str());
    return false;
  }
  while (!file.eof()) {
    float param;
    file >> param;
    params.push_back(param);
  }
  // We'll read the last element twice
  params.pop_back();
  file.close();
  std::ostringstream oss;
  std::copy(params.begin(), params.end() - 1,
            std::ostream_iterator<float>(oss, ","));
  oss << params.back();
  ROS_INFO_STREAM("Gravcomp Params: " << oss.str());
  return useGravcompForEStop(use, params);
}

bool JacoRobot::useGravcompForEStop(bool use, std::vector<float> params) {
  if (!use || params.size() == 0) {
    mUseGravComp = false;
    return false || !use;
  }

  float arr[OPTIMAL_Z_PARAM_SIZE] = {0};
  std::copy(params.begin(), params.end(), arr);

  int r = NO_ERROR_KINOVA;
  r = SetGravityOptimalZParam(arr);
  // Known error, see https://github.com/Kinovarobotics/kinova-ros/issues/114
  if (r != NO_ERROR_KINOVA && r != 2005) {
    ROS_ERROR("Could not send Z Params : Error code %d", r);
    mUseGravComp = false;
    return false;
  }
  r = SetGravityType(OPTIMAL);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send Gravity Type : Error code %d", r);
    mUseGravComp = false;
    return false;
  }

  mUseGravComp = true;
  return true;
}

std::vector<float> JacoRobot::calcGravcompParams() {
  double arr[OPTIMAL_Z_PARAM_SIZE] = {0};

  int r = NO_ERROR_KINOVA;
  r = RunGravityZEstimationSequence(our_robot_type, arr);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not run sequence : Error code %d", r);
    return std::vector<float>();
  }

  std::vector<float> ret(OPTIMAL_Z_PARAM_SIZE);
  for (int i = 0; i < OPTIMAL_Z_PARAM_SIZE; i++) {
    ret[i] = (float)arr[i];
  }
  return ret;
}

void JacoRobot::sendPositionCommand(const std::vector<double> &command) {
  if (mInTorqueMode) {
    if (!setTorqueMode(false)) {
      ROS_WARN("Could not exit torque mode.");
      return;
    }
    ROS_INFO("Exited torque mode.");
  }
  // Need to send an "advance trajectory" with a single point and the correct
  // settings Angular position
  AngularInfo joint_pos;
  joint_pos.InitStruct();
  

  joint_pos.Actuator1 = float(radiansToDegrees(command.at(0) - pos_offsets[0]));
  joint_pos.Actuator2 = float(radiansToDegrees(command.at(1) - pos_offsets[1]));
  joint_pos.Actuator3 = float(radiansToDegrees(command.at(2) - pos_offsets[2]));
  joint_pos.Actuator4 = float(radiansToDegrees(command.at(3) - pos_offsets[3]));
  joint_pos.Actuator5 = float(radiansToDegrees(command.at(4) - pos_offsets[4]));
  joint_pos.Actuator6 = float(radiansToDegrees(command.at(5) - pos_offsets[5]));

  TrajectoryPoint trajectory;
  trajectory.InitStruct();                    // initialize structure
  memset(&trajectory, 0, sizeof(trajectory)); // zero out the structure

  trajectory.Position.Delay = 0.0;
  trajectory.Position.HandMode = POSITION_MODE;
  trajectory.Position.Type = ANGULAR_POSITION;
  trajectory.Position.Actuators = joint_pos;

  trajectory.Position.Fingers.Finger1 =
      float(radiansToFingerTicks(command.at(6)));
  trajectory.Position.Fingers.Finger2 =
      float(radiansToFingerTicks(command.at(7)));

  // Clear FIFO if new command
  if(command != prev_cmd_pos) {
    EraseAllTrajectories();
    prev_cmd_pos = command;
  }

  int r = NO_ERROR_KINOVA;
  r = SendAdvanceTrajectory(trajectory);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send : Error code %d", r);
  }
}

bool JacoRobot::zeroTorqueSensors() {
  // Move to candlestick
  const double PI = 3.1415926535897932384626;
  std::vector<double> command(num_full_dof);
  for (int i = 0; i < num_arm_dof; i++) {
    command[i] = PI;
  }
  command[4] = 0.0;
  command[5] = 0.0;
  command[6] = PI;
  command[7] = PI;

  ROS_INFO("Moving to candlestick...");
  sendPositionCommand(command);
  // Wait for finish
  ros::Duration(10.0).sleep();

  // Zero all actuators
  // Actuator addresses are 16-21
  ROS_INFO("Executing torque zero...");
  for (int i = 16; i < 22; i++) {
    int r = NO_ERROR_KINOVA;
    r = SetTorqueZero(i);
    if (r != NO_ERROR_KINOVA) {
      ROS_ERROR("Could not set torque zero : Error code %d", r);
      return false;
    }
  }

  return true;
}

void JacoRobot::sendVelocityCommand(const std::vector<double> &command) {
  if (mInTorqueMode) {
    if (!setTorqueMode(false)) {
      ROS_WARN("Could not exit torque mode.");
      return;
    }
    ROS_INFO("Exited torque mode.");
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

  TrajectoryPoint trajectory;
  trajectory.InitStruct();
  memset(&trajectory, 0, sizeof(trajectory));

  trajectory.Position.Type = ANGULAR_VELOCITY;
  trajectory.Position.HandMode = VELOCITY_MODE;
  trajectory.Position.Actuators = joint_vel;

  trajectory.Position.Fingers.Finger1 =
      float(radiansToFingerTicks(command.at(6)));
  trajectory.Position.Fingers.Finger2 =
      float(radiansToFingerTicks(command.at(7)));

  int r = NO_ERROR_KINOVA;
  r = SendAdvanceTrajectory(trajectory);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send : Error code %d", r);
  }
}

void JacoRobot::sendTorqueCommand(const std::vector<double> &command) {
  // Check if in torque mode
  int mode = 0;
  GetTrajectoryTorqueMode(mode);
  if (!mode) {
    ROS_WARN("Dropped out of torque mode. Retrying...");
    mInTorqueMode = false;
  }

  if (!mInTorqueMode) {
    if (!setTorqueMode(true)) {
      ROS_WARN("Could not enter torque mode.");
      return;
    }
    ROS_INFO("Entered torque mode.");
  }

  float joint_eff[COMMAND_SIZE] = {0};
  std::copy(command.begin(), command.end(), joint_eff);

  int r = NO_ERROR_KINOVA;
  r = SendAngularTorqueCommand(joint_eff);
  if (r != NO_ERROR_KINOVA) {
    ROS_ERROR("Could not send : Error code %d", r);
  }
}

void JacoRobot::enterGravComp() {
  joint_mode = hardware_interface::JointCommandModes::EMERGENCY_STOP;
}

void JacoRobot::write(void) {
  // Clear all commands when switching modes
  if (last_mode != joint_mode) {
    EraseAllTrajectories();
    setTorqueMode(false);
    last_mode = joint_mode;
  }

  vector<double> zero(num_full_dof, 0.0);

  switch (joint_mode) {
  case hardware_interface::JointCommandModes::MODE_VELOCITY:
    sendVelocityCommand(cmd_vel);
    break;
  case hardware_interface::JointCommandModes::MODE_POSITION:
    sendPositionCommand(cmd_pos);
    break;
  case hardware_interface::JointCommandModes::MODE_EFFORT:
    sendTorqueCommand(cmd_eff);
    break;
  case hardware_interface::JointCommandModes::EMERGENCY_STOP:
    // Drop to gravity compensation
    if (mUseGravComp) {
      sendTorqueCommand(zero);
      break;
    }
  default:
    // Stop Bot
    sendVelocityCommand(zero);
  }
}

void JacoRobot::checkForStall(void) {
  // check soft limits.

  bool all_in_limits = true;
  for (int i = 0; i < num_full_dof; i++) {
    if (eff[i] < -soft_limits[i] || eff[i] > soft_limits[i]) {
      all_in_limits = false;
      ROS_WARN("Exceeded soft effort limits on joint %d. Limit=%f, Measured=%f",
               i, soft_limits[i], eff[i]);
    }
  }
}

void JacoRobot::read(void) {
  // make sure that pos, vel, and eff are up to date.
  // TODO: If there is too much lag between calling read()
  // and getting the actual values back, we'll need to be
  // reading values constantly and storing them locally, so
  // at least there is a recent value available for the controller.

  AngularPosition arm_pos;
  AngularPosition arm_vel;
  AngularPosition arm_torq;

  // Requires 3 seperate calls to the USB
  GetAngularPosition(arm_pos);
  GetAngularVelocity(arm_vel);
  GetAngularForce(arm_torq);

  pos[0] =
      degreesToRadians(double(arm_pos.Actuators.Actuator1)) + pos_offsets[0];
  pos[1] =
      degreesToRadians(double(arm_pos.Actuators.Actuator2)) + pos_offsets[1];
  pos[2] =
      degreesToRadians(double(arm_pos.Actuators.Actuator3)) + pos_offsets[2];
  pos[3] =
      degreesToRadians(double(arm_pos.Actuators.Actuator4)) + pos_offsets[3];
  pos[4] =
      degreesToRadians(double(arm_pos.Actuators.Actuator5)) + pos_offsets[4];
  pos[5] =
      degreesToRadians(double(arm_pos.Actuators.Actuator6)) + pos_offsets[5];
  pos[6] = fingerTicksToRadians(double(arm_pos.Fingers.Finger1));
  pos[7] = fingerTicksToRadians(double(arm_pos.Fingers.Finger2));

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
