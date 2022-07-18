
#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestLangoIMUHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    lango_imu_hardware_interface_2dof_ =
        R"(
  <ros2_control name="LangoIMUHardwareInterface2dof" type="sensor">
    <hardware>
      <plugin>lango_imu_hardware_interface/LangoIMUHardwareInterface</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }

  std::string lango_imu_hardware_interface_2dof_;
};

TEST_F(TestLangoIMUHardwareInterface, load_lango_imu_hardware_interface_2dof)
{
  auto urdf =
      ros2_control_test_assets::urdf_head + lango_imu_hardware_interface_2dof_ + ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
