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

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
}  // namespace

class TestJacoIsaac : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_jaco6dof_2finger_ =
      R"(
  <ros2_control name="j2n6s200" type="system">
    <hardware>
      <plugin>ada_hardware/JacoIsaac</plugin>
    </hardware>
    <joint name="j2n6s200_joint_1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_3">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_4">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_5">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_6">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_finger_1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="j2n6s200_joint_finger_2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";
    rclcpp::init(0, nullptr);
  }

  std::string hardware_system_jaco6dof_2finger_;
};

// Forward declaration
namespace hardware_interface
{
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestJacoIsaac;

  TestableResourceManager() : hardware_interface::ResourceManager() {}

  TestableResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
};

TEST_F(TestJacoIsaac, load_isaacsim_6dof_2finger)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_jaco6dof_2finger_ +
              ros2_control_test_assets::urdf_tail;
  try {
    TestableResourceManager rm(urdf);
    SUCCEED();
  } catch (std::exception const & err) {
    FAIL() << err.what();
  }
}
