// Copyright (c) 2022, ARES
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ODRIVE_CANSIMPLE_HARDWARE_INTERFACE__CANSIMPLE_HARDWARE_INTERFACE_HPP_
#define ODRIVE_CANSIMPLE_HARDWARE_INTERFACE__CANSIMPLE_HARDWARE_INTERFACE_HPP_

#include <linux/can.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "odrive_cansimple_hardware_interface/socketcan.hpp"
#include "odrive_cansimple_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "odrive_cansimple_hardware_interface/socketcan.hpp"
#include "odrive_cansimple_hardware_interface//cansimple_protocol.hpp"

namespace odrive_cansimple_hardware_interface
{
class CanSimpleHardwareInterface
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanSimpleHardwareInterface)

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  ODRIVE_CANSIMPLE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:

  void handleService();

  std::vector<CanSimpleAxis> axies_;
  std::unordered_map<std::string, std::shared_ptr<SocketCan>> can_devices_;
};

}  // namespace odrive_cansimple_hardware_interface

#endif  // ODRIVE_CANSIMPLE_HARDWARE_INTERFACE__CANSIMPLE_HARDWARE_INTERFACE_HPP_
