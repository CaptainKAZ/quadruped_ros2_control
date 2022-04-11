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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "jpos_initializer_controller/jpos_initializer.hpp"
#include "controller_interface/helpers.hpp"

namespace jpos_initializer_controller
{
  JPosInitializer::JPosInitializer() : controller_interface::ControllerInterface() {}

  controller_interface::return_type JPosInitializer::init(const std::string &controller_name)
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK)
    {
      return ret;
    }
    auto param_change_callback =
        [this](std::vector<rclcpp::Parameter> parameters)
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters)
      {
        auto update_param_by_name =
            [this, &parameter, &result](auto &value, std::string parameter_name)
        {
          if (parameter.get_name() == parameter_name)
          {
            try
            {
              // Fucking Magic!
              value = parameter.get_value<typename std::remove_reference<decltype(value)>::type>();
              RCLCPP_INFO_STREAM(get_node()->get_logger(),"Got new parameter"<<parameter_name<<", value="<<value<<"!");
            }
            catch (std::exception &e)
            {
              result.successful = false;
              result.reason = "Failed to update parameter: " + parameter.get_name() + ". Because:" + e.what();
              RCLCPP_WARN(get_node()->get_logger(), e.what());
            }
          }
        };
        auto reject_param_by_name =
            [&parameter, &result](std::string parameter_name)
        {
          if (parameter.get_name() == parameter_name)
          {
            result.successful = false;
            result.reason = "Reject to update parameter: " + parameter.get_name();
          }
        };
        reject_param_by_name("cheetah_joints");
        reject_param_by_name("mid_jpos");
        reject_param_by_name("target_jpos");
        reject_param_by_name("duration");
        update_param_by_name(kp_, "hybrid_kp");
        update_param_by_name(kd_, "hybrid_kd");
        update_param_by_name(dt_, "dt");
      }
      return result;
    };
    // callback_handler needs to be alive to keep the callback functional
    param_set_callback_handle_ = get_node()->add_on_set_parameters_callback(param_change_callback);

    try
    {
      auto_declare<std::vector<std::string>>("cheetah_joints", {});
      auto_declare<std::vector<double>>("mid_jpos", {});
      auto_declare<std::vector<double>>("target_jpos", {});
      auto_declare<double>("duration", {});
      auto_declare<double>("dt", {});
      auto_declare<double>("hybrid_kp", {});
      auto_declare<double>("hybrid_kd", {});
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::return_type::ERROR;
    }
    state_ = ControllerState::INITIALIZED;

    return controller_interface::return_type::OK;
  }

  CallbackReturn JPosInitializer::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto error_if_empty = [&](const auto &parameter, const char *parameter_name)
    {
      if (parameter.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
        return true;
      }
      return false;
    };
    auto error_if_zero = [&](const auto &parameter, const char *parameter_name)
    {
      if (parameter == 0)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was not specified", parameter_name);
        return true;
      }
      return false;
    };

    auto get_string_array_param_and_error_if_empty =
        [&](std::vector<std::string> &parameter, const char *parameter_name)
    {
      parameter = get_node()->get_parameter(parameter_name).as_string_array();
      return error_if_empty(parameter, parameter_name);
    };

    auto get_double_array_param_and_error_if_empty =
        [&](std::vector<double> &parameter, const char *parameter_name)
    {
      parameter = get_node()->get_parameter(parameter_name).as_double_array();
      return error_if_empty(parameter, parameter_name);
    };
    auto get_double_param_and_error_if_zero =
        [&](double &parameter, const char *parameter_name)
    {
      parameter = get_node()->get_parameter(parameter_name).as_double();
      return error_if_zero(parameter, parameter_name);
    };

    if (
        get_string_array_param_and_error_if_empty(cheetah_joint_names_, "cheetah_joints") ||
        get_double_array_param_and_error_if_empty(mid_jpos_, "mid_jpos") ||
        get_double_array_param_and_error_if_empty(target_jpos_, "target_jpos") ||
        get_double_param_and_error_if_zero(duration_, "duration") ||
        get_double_param_and_error_if_zero(dt_, "dt") ||
        get_double_param_and_error_if_zero(kp_, "hybrid_kp") ||
        get_double_param_and_error_if_zero(kd_, "hybrid_kd"))
    {
      return CallbackReturn::ERROR;
    }
    if (cheetah_joint_names_.size() != mid_jpos_.size() || cheetah_joint_names_.size() != target_jpos_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Size of cheetah_joints, mid_jpos and target_jpos are not equal");
      return CallbackReturn::ERROR;
    }
    init_jpos_.resize(cheetah_joint_names_.size());

    // Create Service
    auto handle_is_calibrated_srv =
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<control_msgs::srv::QueryCalibrationState::Request> request,
               std::shared_ptr<control_msgs::srv::QueryCalibrationState::Response> response) -> void
    {
      (void)request_header;
      (void)request;
      response->is_calibrated = (state_ == ControllerState::CALIBRATED);
    };
    calibrated_jpos_srv_ = get_node()->create_service<CalibrationState>(
        "is_calibrated", handle_is_calibrated_srv);

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration JPosInitializer::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(cheetah_joint_names_.size() * 5);
    size_t i = 0;
    for (const auto &joint : cheetah_joint_names_)
    {
      command_interfaces_config.names.emplace_back(joint + "/hybrid_position");
      interface_map_.emplace(joint + "/hybrid_position", i++);
      command_interfaces_config.names.emplace_back(joint + "/hybrid_velocity");
      interface_map_.emplace(joint + "/hybrid_velocity", i++);
      command_interfaces_config.names.emplace_back(joint + "/hybrid_effort");
      interface_map_.emplace(joint + "/hybrid_effort", i++);
      command_interfaces_config.names.emplace_back(joint + "/hybrid_kp");
      interface_map_.emplace(joint + "/hybrid_kp", i++);
      command_interfaces_config.names.emplace_back(joint + "/hybrid_kd");
      interface_map_.emplace(joint + "/hybrid_kd", i++);
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration JPosInitializer::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(cheetah_joint_names_.size());
    for (const auto &joint : cheetah_joint_names_)
    {
      state_interfaces_config.names.emplace_back(joint + "/" + "position");
    }

    return state_interfaces_config;
  }

  CallbackReturn JPosInitializer::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    state_ = ControllerState::MOVING;
    for (size_t i = 0; i < cheetah_joint_names_.size(); i++)
    {
      init_jpos_[i] = state_interfaces_[i].get_value();
    }
    start_time_ = rclcpp::Clock().now();
    last_update_time_ = start_time_;

    spline_ = tinyspline::BSpline(7, cheetah_joint_names_.size(), 3, tinyspline::BSpline::Type::Clamped);
    std::vector<tinyspline::real> ctrlp = spline_.controlPoints();
    // Constrain start acc/vel/pos
    ctrlp.assign(init_jpos_.begin(), init_jpos_.end());
    ctrlp.insert(ctrlp.end(), init_jpos_.begin(), init_jpos_.end());
    ctrlp.insert(ctrlp.end(), init_jpos_.begin(), init_jpos_.end());
    // Constrain middle point
    ctrlp.insert(ctrlp.end(), mid_jpos_.begin(), mid_jpos_.end());
    // Constrain end acc/vel/pos
    ctrlp.insert(ctrlp.end(), target_jpos_.begin(), target_jpos_.end());
    ctrlp.insert(ctrlp.end(), target_jpos_.begin(), target_jpos_.end());
    ctrlp.insert(ctrlp.end(), target_jpos_.begin(), target_jpos_.end());
    spline_.setControlPoints(ctrlp);

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Activate successful!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn JPosInitializer::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto &command_interface : command_interfaces_)
    {
      command_interface.set_value(0);
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JPosInitializer::update()
  {
    auto now = rclcpp::Clock().now();
    if ((now - last_update_time_).seconds() > dt_)
    {
      last_update_time_ = now;
      // Safety first
      for (auto &command_interfaces_i : command_interfaces_)
      {
        command_interfaces_i.set_value(0);
      }
      if (state_ == ControllerState::MOVING)
      {
        auto time_from_start = (now - start_time_).seconds();
        double u = time_from_start / duration_;
        if (u >= 1.0)
        {
          state_ = ControllerState::CALIBRATED;
          u = 1.0;
        }
        std::vector<tinyspline::real> result = spline_.eval(u).result();
        for (size_t i = 0; i < cheetah_joint_names_.size(); i++)
        {
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_position")].set_value(result[i]);
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_kp")].set_value(kp_);
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_kd")].set_value(kd_);
        }
      }
      else if (state_ == ControllerState::CALIBRATED)
      {
        for (size_t i = 0; i < cheetah_joint_names_.size(); i++)
        {
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_position")].set_value(target_jpos_[i]);
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_kp")].set_value(kp_);
          command_interfaces_[interface_map_.at(cheetah_joint_names_[i] + "/hybrid_kd")].set_value(kd_);
        }
      }
    }

    return controller_interface::return_type::OK;
  }

} // namespace jpos_initializer_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(jpos_initializer_controller::JPosInitializer, controller_interface::ControllerInterface)
