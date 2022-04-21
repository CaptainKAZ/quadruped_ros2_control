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
#pragma once

#include <memory>
#include <rclcpp/time.hpp>
#include <string>
#include <urdf/model.h>
#include <vector>

#include "quadruped_controllers/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "urdf_parser/urdf_parser.h"

#include "quadruped_controllers/ros2_node_interface.hpp"
#include "quadruped_controllers/quadruped_interface.hpp"
#include "quadruped_controllers/leg_controller.hpp"
#include "quadruped_controllers/state_update.hpp"
#include "quadruped_controllers/pinocchio_solver.hpp"

namespace quadruped_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class QuadrupedControllerBase : public controller_interface::ControllerInterface
{
public:
  QUADRUPED_CONTROLLERS_PUBLIC
  QuadrupedControllerBase();

  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type update() override;

  QUADRUPED_CONTROLLERS_PUBLIC
  void print_state();

protected:
  void updateURDFModel(const std::string& from_node_name,const std::string& param_name);
  std::shared_ptr<urdf::ModelInterface> urdf_;
  std::shared_ptr<QuadrupedState> state_;
  std::shared_ptr<QuadrupedCommand> command_;
  std::shared_ptr<QuadrupedInterface> interface_;
  std::shared_ptr<PinocchioSolver> kine_solver_;
  std::shared_ptr<LegController> leg_controller_;
  std::shared_ptr<InterfaceStateUpdate> interface_update_;
  std::shared_ptr<FromGroundTruthStateUpdate> from_ground_truth_update_;
  std::shared_ptr<KinematicSolverStateUpdate> kinematic_solver_update_;
  std::shared_ptr<LinearKFPosVelEstimateUpdate> linear_kf_pos_vel_update_;
  std::vector<std::shared_ptr<StateUpdateBase>> state_updater_queue_;
  std::vector<std::shared_ptr<Ros2NodeInterfaceBase>> ros2_node_interface_queue_;
  rclcpp::Time start_time_;
};

}  // namespace quadruped_controllers

