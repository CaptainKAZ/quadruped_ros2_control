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

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <thread>
#include <vector>
#include "odrive_cansimple_hardware_interface/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "odrive_cansimple_hardware_interface/cansimple_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace odrive_cansimple_hardware_interface
{
hardware_interface::return_type CanSimpleHardwareInterface::configure(const hardware_interface::HardwareInfo& info)
{
  // node_=std::make_shared<rclcpp::Node>("odrive_cansimple_hardware_interface");
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  axies_.resize(info_.joints.size(), CanSimpleAxis());
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    auto bus_name = info_.joints[i].parameters.at("can_bus");
    if (can_devices_.find(bus_name) == can_devices_.end())
    {
      can_devices_.emplace(bus_name, std::make_shared<SocketCan>());
      can_devices_.at(bus_name)->open(bus_name, 1e6);
    }
    auto axis_can_id = std::stoul(info_.joints[i].parameters.at("can_id"));
    auto axis_gear_ratio = std::stod(info_.joints[i].parameters.at("gear_ratio"));
    auto axis_motor_kt = std::stod(info_.joints[i].parameters.at("motor_kt"));
    axies_[i].configure(axis_gear_ratio, axis_motor_kt);
    axies_[i].attach(can_devices_.at(bus_name), axis_can_id);
  }

  status_ = hardware_interface::status::CONFIGURED;

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> CanSimpleHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, axies_[i].getStatePosPtr());
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, axies_[i].getStateVelPtr());
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, axies_[i].getStateEffPtr());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanSimpleHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(info_.joints[i].name, "hybrid_position", axies_[i].getCommandPosPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "hybird_velocity", axies_[i].getCommandVelPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "hybird_effort", axies_[i].getCommandEffPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "hybrid_kp", axies_[i].getCommandKpPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "hybrid_kd", axies_[i].getCommandKdPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "calibartiopn_new_pos", axies_[i].getCalibrationPosPtr());
    command_interfaces.emplace_back(info_.joints[i].name, "request_state", axies_[i].getRequestStatePtr());
    command_interfaces.emplace_back(info_.joints[i].name, "clear_error", axies_[i].getClearErrorPtr());
  }

  return command_interfaces;
}

hardware_interface::return_type CanSimpleHardwareInterface::start()
{
  for (auto& axis : axies_)
  {
    axis.setControllerMode(0x03,0x01);
    axis.setAxisRequestedState(0x08);
  }
  status_ = hardware_interface::status::STARTED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanSimpleHardwareInterface::stop()
{
  for (auto& axis : axies_)
  {
    axis.setAxisRequestedState(0x00);
  }
  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanSimpleHardwareInterface::read()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanSimpleHardwareInterface::write()
{
  for (auto& axis : axies_)
  {
    axis.writeCommand();
  }
  return hardware_interface::return_type::OK;
}

}  // namespace odrive_cansimple_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_cansimple_hardware_interface::CanSimpleHardwareInterface,
                       hardware_interface::SystemInterface)
