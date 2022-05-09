// Copyright (c) 2022, Zou Yuanhao
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template) Inspired by https://github.com/qiayuanliao/cheetah_ros Licensed
// under the Apache License, Version 2.0 (the "License"); you may not use this
// file except in compliance with the License. You may obtain a copy of the
// License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>

#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/ros2_node_interface.hpp"
#include "quadruped_controllers/utils.hpp"

namespace quadruped_controllers {
QuadrupedControllerBase::QuadrupedControllerBase()
    : controller_interface::ControllerInterface() {}

controller_interface::return_type
QuadrupedControllerBase::init(const std::string &controller_name) {
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  state_ = std::make_shared<QuadrupedState>();
  command_ = std::make_shared<QuadrupedCommand>();
  interface_ = std::make_shared<QuadrupedInterface>();
  ros2_node_interface_queue_.emplace_back(
      std::make_shared<OdomTfPublisher>(get_node(), state_, command_));
  ros2_node_interface_queue_.emplace_back(
      std::make_shared<TwistSubscriber>(get_node(), state_, command_));
  ros2_node_interface_queue_.emplace_back(
      std::make_shared<GroundTruthSubscriber>(get_node(), state_, command_));
  auto ground_truth = std::dynamic_pointer_cast<GroundTruthSubscriber>(
                          ros2_node_interface_queue_.back())
                          ->getTruthState();
  from_ground_truth_update_ = std::make_shared<FromGroundTruthStateUpdate>(
      get_node(), state_, ground_truth);
  linear_kf_pos_vel_update_ =
      std::make_shared<LinearKFPosVelEstimateUpdate>(get_node(), state_);
  interface_update_ =
      std::make_shared<InterfaceStateUpdate>(get_node(), state_, interface_);
  updateURDFModel("robot_state_publisher", "robot_description");
  return controller_interface::return_type::OK;
}

CallbackReturn QuadrupedControllerBase::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  kine_solver_ = std::make_shared<PinocchioSolver>(urdf_);
  leg_controller_ =
      std::make_shared<LegController>(interface_, kine_solver_, state_);
  kinematic_solver_update_ =
      std::make_shared<KinematicSolverStateUpdate>(get_node(), state_, kine_solver_);
  // debug visualize
  p3d_pub_ = std::make_shared<P3dPublisher>(get_node(), state_, command_,
                                            "desired_foot_pos");
  line_pub_ = std::make_shared<LinePublisher>(get_node(), state_, command_,
                                              "velocity_line");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
QuadrupedControllerBase::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::ALL;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
QuadrupedControllerBase::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::ALL;

  return state_interfaces_config;
}

CallbackReturn QuadrupedControllerBase::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  interface_->assignLoanedInterfaces(state_interfaces_, command_interfaces_);
  state_updater_queue_.push_back(interface_update_);

  state_updater_queue_.push_back(kinematic_solver_update_);
  //state_updater_queue_.push_back(linear_kf_pos_vel_update_);
  // enable in cheater mode
  state_updater_queue_.push_back(from_ground_truth_update_);
  state_updater_queue_.push_back(kinematic_solver_update_);
  
  RCLCPP_INFO(get_node()->get_logger(), "Activated!!!!!!");
  start_time_ = get_node()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn QuadrupedControllerBase::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  interface_->releaseInterfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedControllerBase::update() {
  for (auto &ros2Interface : ros2_node_interface_queue_) {
    ros2Interface->update(get_node()->get_clock()->now());
  }
  for (auto &updater : state_updater_queue_) {
    updater->update(get_node()->get_clock()->now());
  }
  leg_controller_->writeInterface();
  //print_state();
  p3d_pub_->update(node_->now());
  line_pub_->update(node_->now());
  return controller_interface::return_type::OK;
}

void QuadrupedControllerBase::updateURDFModel(const std::string &from_node_name,
                                              const std::string &param_name) {
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      get_node(), from_node_name);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Interrupted while waiting for %s service. Exiting.",
                   from_node_name.c_str());
      return;
    }
    RCLCPP_ERROR(get_node()->get_logger(),
                 "%s service not available, waiting again...",
                 from_node_name.c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "connected to service!! %s",
              from_node_name.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    if (parameters_client->has_parameter(param_name)) {
      RCLCPP_INFO(get_node()->get_logger(), "param_name %s",
                  param_name.c_str());
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "%s does not exist",
                   param_name.c_str());
    }
    try {
      std::vector<rclcpp::Parameter> values =
          parameters_client->get_parameters({param_name});
      urdf_string = values[0].as_string();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "gazebo_ros2_control plugin is waiting for model"
                   " URDF in parameter [%s] on the ROS param server.",
                   param_name.c_str());
    }
    sleep(1);
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "Recieved urdf from param server, parsing...");
  urdf_ = urdf::parseURDF(urdf_string);
}

void QuadrupedControllerBase::print_state() {
  std::ostringstream oss;
  oss<<std::endl
                         << "------------state:--------------\n"
                         << "update_time" << (state_->update_time_-start_time_).seconds()
                         << "\n"
                         << "quat:\n"
                         << state_->quat_.coeffs().transpose() << "\n"
                         << "pos:\n"
                         << state_->pos_.transpose() << "\n"
                         << "vel:\n"
                         << state_->linear_vel_.transpose() << "\n"
                         << "acc:\n"
                         << state_->accel_.transpose() << "\n"
                         << "omega:\n"
                         << state_->angular_vel_.transpose() << "\n"
                         << "contact:\n"
                         << state_->contact_state_[0] << ","
                         << state_->contact_state_[1] << ","
                         << state_->contact_state_[2] << ","
                         << state_->contact_state_[3] << "\n"
                         << "foot_pos:\n"
                          << state_->foot_pos_[0].transpose() << "\n"
                          << state_->foot_pos_[1].transpose() << "\n"
                          << state_->foot_pos_[2].transpose() << "\n"
                          << state_->foot_pos_[3].transpose() << "\n"
                         << "----------end of state----------";
  RCLCPP_INFO(get_node()->get_logger(),
                    oss.str().c_str() );
}

} // namespace quadruped_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controllers::QuadrupedControllerBase,
                       controller_interface::ControllerInterface)
