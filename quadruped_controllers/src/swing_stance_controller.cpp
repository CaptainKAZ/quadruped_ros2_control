#include "quadruped_controllers/swing_stance_controller.hpp"
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/foot_swing_trajectory.hpp"
#include <cstddef>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <string>
#include <vector>

namespace quadruped_controllers {
controller_interface::return_type
QuadrupedSwingStanceController::init(const std::string &controller_name) {
  auto ret = QuadrupedControllerBase::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto_declare<std::vector<double>>("kp_stance", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  auto_declare<std::vector<double>>("kd_stance", {0, 0, 0, 0, 0, 0, 0, 0, 0});
  auto_declare<std::vector<double>>("kp_swing", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  auto_declare<std::vector<double>>("kd_swing", {0, 0, 0, 0, 0, 0, 0, 0, 0});

  for (auto &traj : swing_traj_) {
    traj = std::make_unique<FootSwingTrajectory<double>>();
  }
  return controller_interface::return_type::OK;
}

CallbackReturn QuadrupedSwingStanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  // Leg Controller initted in QuadrupedControllerBase::init
  auto ret = QuadrupedControllerBase::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // note that this function is called in non-realtime thread
  auto swing_stance_param_callback =
      [this](std::vector<rclcpp::Parameter> params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto &param : params) {
          auto accept_mat3x3 = [&result, &param, this](std::string name) {
            if (param.get_name() == name) {
              if (param.get_type() !=
                  rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                result.successful = false;
                result.reason = "parameter type error";
                return;
              }
              if (param.get_value<std::vector<double>>().size() != 9) {
                result.successful = false;
                result.reason =
                    "parameter " + name + " should be a 9-element array";
                return;
              }
              need_update_param_=true;
            }
          };
          accept_mat3x3("kp_stance");
          accept_mat3x3("kd_stance");
          accept_mat3x3("kp_swing");
          accept_mat3x3("kd_swing");
        }
        return result;
      };
  // callback_handler needs to be alive to keep the callback functional
  swing_stance_param_callback_handle_ =
      get_node()->add_on_set_parameters_callback(swing_stance_param_callback);

  for (size_t i = 0; i < 4; i++) {
    if (leg_controller_ != nullptr) {
      leg_cmd_[i] = leg_controller_->getLegCommand(i);
      swing_stance_config_[i].cmd_time_ = node_->now();
    } else {
      RCLCPP_FATAL(get_node()->get_logger(), "leg_controller is nullptr");
      return CallbackReturn::ERROR;
    }
  }
  // undate param once at the beginning
  if(updateParam()){
    need_update_param_=false;
    return CallbackReturn::SUCCESS;
  }else{
    return CallbackReturn::ERROR;
  }
}

controller_interface::return_type QuadrupedSwingStanceController::update() {
  if(need_update_param_){
    if(updateParam()){
      need_update_param_=false;
      return controller_interface::return_type::OK;
    }else{
      return controller_interface::return_type::ERROR;
    }
  }
  auto now = node_->now();
  for (size_t i = 0; i < 4; i++) {
    if ((swing_stance_config_[i].swing_duration_ > 1e-3 &&
         (now - swing_stance_config_[i].cmd_time_).seconds() <
             swing_stance_config_[i].swing_duration_) ||
        (now - swing_stance_config_[i].cmd_time_).seconds() < 1e-1) {
      if (swing_stance_config_[i].stance_) {
        // stance leg
        leg_cmd_[i]->foot_pos_des_ = swing_stance_config_[i].stance_foot_pos_;
        leg_cmd_[i]->foot_vel_des_.setZero();
        leg_cmd_[i]->kp_cartesian_ = kp_stance_;
        leg_cmd_[i]->kd_cartesian_ = kd_stance_;
        leg_cmd_[i]->ff_cartesian_ = -swing_stance_config_[i].stance_force_ff_;
        leg_cmd_[i]->stamp_ = now;
      } else if (swing_traj_[i]) {
        // swing leg
        leg_cmd_[i]->kp_cartesian_ = kp_swing_;
        leg_cmd_[i]->kd_cartesian_ = kd_swing_;
        double phase = (now - swing_stance_config_[i].cmd_time_).seconds() /
                       swing_stance_config_[i].swing_duration_;
        if (phase > 1)
          phase = 1;
        swing_traj_[i]->computeSwingTrajectoryBezier(
            phase, swing_stance_config_[i].swing_duration_);
        leg_cmd_[i]->foot_pos_des_ = swing_traj_[i]->getPosition();
        leg_cmd_[i]->foot_vel_des_ = swing_traj_[i]->getVelocity();
        leg_cmd_[i]->ff_cartesian_.setZero();
        leg_cmd_[i]->stamp_ = now;
      }
    } else {
      // set zero all cartisian command
      leg_cmd_[i]->foot_pos_des_.setZero();
      leg_cmd_[i]->foot_vel_des_.setZero();
      leg_cmd_[i]->kp_cartesian_.setZero();
      leg_cmd_[i]->kd_cartesian_.setZero();
      leg_cmd_[i]->ff_cartesian_.setZero();
      leg_cmd_[i]->stamp_ = now;
    }
  }
  return QuadrupedControllerBase::update();
}

void QuadrupedSwingStanceController::setSwing(size_t leg,
                                              Eigen::Vector3d &final_pos,
                                              double height, double duration) {
  rcpputils::check_true(leg < 4, "leg index out of range");
  swing_stance_config_[leg].swing_duration_ = duration;
  swing_stance_config_[leg].stance_ = false;
  swing_stance_config_[leg].cmd_time_ = node_->now();
  swing_traj_[leg]->setHeight(height);
  swing_traj_[leg]->setFinalPosition(final_pos);
  swing_traj_[leg]->setInitialPosition(state_->foot_pos_[leg]);
}

void QuadrupedSwingStanceController::setStance(size_t leg,
                                               Eigen::Vector3d &force_ff) {
  rcpputils::check_true(leg < 4, "leg index out of range");
  swing_stance_config_[leg].stance_ = true;
  swing_stance_config_[leg].cmd_time_ = node_->now();
  swing_stance_config_[leg].stance_force_ff_ = force_ff;
  swing_stance_config_[leg].swing_duration_ = 0;
  swing_stance_config_[leg].stance_foot_pos_ = state_->foot_pos_[leg];
}

bool QuadrupedSwingStanceController::updateParam(){
 for (auto &config : swing_stance_config_) {
    config.stance_force_ff_.setZero();
  }
  auto get_mat3 = [this](std::string name, Mat3<double> &mat) {
    auto param = get_node()->get_parameter(name);
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return false;
    }
    std::vector<double> v;
    v = param.get_value<std::vector<double>>();
    if (v.size() != 9) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Parameter %s should be a 9-element array, setting it to "
                  "default value {1,0,0,0,1,0,0,0,1}",
                  name.c_str());
      get_node()->set_parameter(rclcpp::Parameter(
          name, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1}));
      mat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      return false;
    }
    mat << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return true;
  };
  if (get_mat3("kp_stance", kp_stance_) && get_mat3("kd_stance", kd_stance_) &&
      get_mat3("kp_swing", kp_swing_) && get_mat3("kd_swing", kd_swing_)) {
    return true;
  } else {
    return false;
  }
}

} // namespace quadruped_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controllers::QuadrupedSwingStanceController,
                       controller_interface::ControllerInterface)
