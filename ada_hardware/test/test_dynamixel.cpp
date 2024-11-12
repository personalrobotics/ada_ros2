// Copyright 2024 Personal Robotics Lab, University of Washington
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
// Author: Jose Jaime

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include <gmock/gmock.h>
#include <string>

namespace {
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
} // namespace

class TestDynamixelHardware : public ::testing::Test {
protected:
  void SetUp() override {
    hardware_system_articulable_fork_ =
        R"(
  <ros2_control name="articulable_fork" type="system">
    <hardware>
      <plugin>ada_hardware/DynamixelHardware</plugin>
      <param name="usb_port">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="af_joint_1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="af_joint_2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";
  }

  std::string hardware_system_articulable_fork_;
};

// Forward declaration
namespace hardware_interface {
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager {
public:
  friend TestDynamixelHardware;

  TestableResourceManager() : hardware_interface::ResourceManager() {}

  TestableResourceManager(const std::string &urdf,
                          bool validate_interfaces = true,
                          bool activate_all = false)
      : hardware_interface::ResourceManager(urdf, validate_interfaces,
                                            activate_all) {}
};

TEST_F(TestDynamixelHardware, load_articulable_fork) {
  auto urdf = ros2_control_test_assets::urdf_head +
              hardware_system_articulable_fork_ +
              ros2_control_test_assets::urdf_tail;
  try {
    TestableResourceManager rm(urdf);
    SUCCEED();
  } catch (std::exception const &err) {
    FAIL() << err.what();
  }
}
