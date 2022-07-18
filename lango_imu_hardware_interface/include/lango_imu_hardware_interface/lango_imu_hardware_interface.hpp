
#ifndef LANGO_IMU_HARDWARE_INTERFACE__LANGO_IMU_HARDWARE_INTERFACE_HPP_
#define LANGO_IMU_HARDWARE_INTERFACE__LANGO_IMU_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "lango_imu_hardware_interface/visibility_control.h"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

namespace lango_imu_hardware_interface
{
class LangoIMUHardwareInterface
  : public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LangoIMUHardwareInterface);

  LANGO_IMU_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  LANGO_IMU_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  LANGO_IMU_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  LANGO_IMU_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  LANGO_IMU_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace lango_imu_hardware_interface

#endif  // LANGO_IMU_HARDWARE_INTERFACE__LANGO_IMU_HARDWARE_INTERFACE_HPP_
