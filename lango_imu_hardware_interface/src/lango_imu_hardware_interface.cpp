
#include <limits>
#include <vector>

#include "lango_imu_hardware_interface/lango_imu_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lango_imu_hardware_interface
{
hardware_interface::return_type LangoIMUHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> LangoIMUHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::return_type LangoIMUHardwareInterface::start()
{
  // TODO(anyone): prepare the robot to receive commands

  status_ = hardware_interface::status::STARTED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LangoIMUHardwareInterface::stop()
{
  // TODO(anyone): prepare the robot to stop receiving commands

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LangoIMUHardwareInterface::read()
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

}  // namespace lango_imu_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lango_imu_hardware_interface::LangoIMUHardwareInterface, hardware_interface::SensorInterface)
