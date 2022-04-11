// Copyright (c) 2022, Zou Yuanhao
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

#ifndef JPOS_INITIALIZER_CONTROLLER__JPOS_INITIALIZER_HPP_
#define JPOS_INITIALIZER_CONTROLLER__JPOS_INITIALIZER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "jpos_initializer_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "tinysplinecxx.h"

#include "control_msgs/srv/query_calibration_state.hpp"

namespace jpos_initializer_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JPosInitializer : public controller_interface::ControllerInterface
{
public:
  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  JPosInitializer();

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  JPOS_INITIALIZER_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  mutable std::unordered_map<std::string,size_t> interface_map_{};
  std::vector<std::string> cheetah_joint_names_{};
  std::vector<double> init_jpos_{};
  std::vector<double> mid_jpos_{};
  std::vector<double> target_jpos_{};
  double dt_{};
  double duration_{};
  double kp_{};
  double kd_{};
  rclcpp::Time start_time_{};
  rclcpp::Time last_update_time_{};
  tinyspline::BSpline spline_{};
  using CalibrationState = control_msgs::srv::QueryCalibrationState;
  rclcpp::Service<CalibrationState>::SharedPtr calibrated_jpos_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_set_callback_handle_;
  enum ControllerState
  {
    INITIALIZED,
    MOVING,
    CALIBRATED
  }state_{};
};

}  // namespace jpos_initializer_controller

#endif  // JPOS_INITIALIZER_CONTROLLER__JPOS_INITIALIZER_HPP_
