#include "quadruped_controllers/locomotion_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/gait.hpp"
#include "quadruped_controllers/mpc_controller.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/utils.hpp"
#include <controller_interface/controller_interface.hpp>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace quadruped_controllers {

controller_interface::return_type
QuadrupedLocomotionController::init(const std::string &controller_name) {
  auto ret = QuadrupedMpcController::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  std::vector<std::string> gait_names =
      auto_declare<std::vector<std::string>>("locomotion.gait.name", {});
  if (gait_names.size() == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "No gait is specified");
    return controller_interface::return_type::ERROR;
  }
  for (const auto &name : gait_names) {
    double cycle =
        auto_declare<double>("locomotion.gait." + name + ".cycle", {});
    std::vector<double> offsets = auto_declare<std::vector<double>>(
        "locomotion.gait." + name + ".offsets", {});
    std::vector<double> durations = auto_declare<std::vector<double>>(
        "locomotion.gait." + name + ".durations", {});
    double mpc_dt = auto_declare<double>("locomotion.gait." + name + ".mpc_dt",
                                         mpc_config_.dt_);
    int mpc_horizon = auto_declare<int>(
        "locomotion.gait." + name + ".mpc_horizon", mpc_config_.horizon_);
    if (offsets.size() != durations.size() || offsets.size() != 4) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "offsets and durations must have the same size 4");
      return controller_interface::return_type::ERROR;
    }
    Vec4<double> offsetsVec;
    offsetsVec << offsets[0], offsets[1], offsets[2], offsets[3];
    Vec4<double> durationsVec;
    durationsVec << durations[0], durations[1], durations[2], durations[3];
    gaits_.emplace(name,
                   std::make_shared<MpcGait>(cycle, offsetsVec, durationsVec,
                                             mpc_dt, mpc_horizon));
  }
  std::string current =
      auto_declare<std::string>("locomotion.gait.current", {});
  try {
    current_gait_ = gaits_.at(current);
    setMpcHorizonDt(current_gait_->mpc_horizon_, current_gait_->mpc_dt_);
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "current gait %s is not found",
                 current.c_str());
    return controller_interface::return_type::ERROR;
  }
  std::fill(std::begin(first_swing_), std::end(first_swing_), true);
  return controller_interface::return_type::OK;
}

CallbackReturn QuadrupedLocomotionController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  auto ret = QuadrupedMpcController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  p3d_pub_ = std::make_shared<P3dPublisher>(get_node(), state_, command_);
  line_pub_ = std::make_shared<LinePublisher>(get_node(), state_, command_);
  need_update_param_ = false;
  auto locomotion_param_callback_handle =
      [this](std::vector<rclcpp::Parameter> params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (const auto &param : params) {
          if (param.get_name() == "locomotion.gait.current") {
            if (gaits_.find(param.get_value<std::string>()) == gaits_.end()) {
              RCLCPP_ERROR(get_node()->get_logger(),
                           "current gait %s is not found",
                           param.get_value<std::string>().c_str());
              result.successful = false;
              result.reason = "current gait is not found";
              break;
            } else {
              need_update_param_ = true;
            }
          } else if (param.get_name().find("locomotion.gait.") !=
                     std::string::npos) {
            result.successful = false;
            result.reason = "not allow to set gait parameters by now";
          }
        }
        return result;
      };
  locomotion_param_callback_handle_ =
      node_->add_on_set_parameters_callback(locomotion_param_callback_handle);
  des_kine_solver_ = std::make_shared<PinocchioSolver>(urdf_);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedLocomotionController::update() {
  p3d_pub_->clearPoint();
  line_pub_->clearLine();
  if (need_update_param_) {
    need_update_param_ = false;
    try {
      current_gait_ = gaits_.at(node_->get_parameter("locomotion.gait.current")
                                    .get_value<std::string>());
      setMpcHorizonDt(current_gait_->mpc_horizon_, current_gait_->mpc_dt_);
    } catch (const std::out_of_range &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "current gait %s is not found",
                   node_->get_parameter("locomotion.gait.current")
                       .get_value<std::string>()
                       .c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  Vec3<double> v_world_des =
      state_->quat_.toRotationMatrix() * command_->linear_vel_;

  line_pub_->addLine(state_->pos_(0), state_->pos_(1), 1,
                     state_->pos_(0) + v_world_des(0),
                     state_->pos_(1) + v_world_des(1), 1);
  auto state_rpy = quatToRPY(state_->quat_);
  std::cout<<state_->linear_vel_.transpose()<<std::endl;
  Eigen::VectorXd traj;
  // traj order: rpy xyz anglua_vel linear_vel
  traj.resize(12 * mpc_config_.horizon_);
  traj.setZero();
  if (current_gait_ == gaits_.at("stand")) {
    for (int i = 0; i < mpc_config_.horizon_; ++i) {
      traj[12 * i + 5] = 0.26;
    }
  } else {
    for (int i = 0; i < mpc_config_.horizon_; ++i) {
      traj[12 * i + 5] = 0.26;
      traj[12 * i + 3] =
          state_->pos_(0) + v_world_des(0) * mpc_config_.dt_ * (i + 1);
      traj[12 * i + 4] =
          state_->pos_(1) + v_world_des(1) * mpc_config_.dt_ * (i + 1);
      traj[12 * i + 9] = v_world_des(0);
      traj[12 * i + 10] = v_world_des(1);
      traj[12 * i + 8] = command_->angular_vel_(2);
      traj[12 * i + 2] =
          state_rpy(2) + command_->angular_vel_(2) * mpc_config_.dt_ * (i);
    }
  }
  setTraj(traj);
  
  current_gait_->update(node_->now());
  Eigen::VectorXd table = current_gait_->getMpcTable(mpc_solver_->getHorizon());
  setGaitTable(table);
  Vec4<double> swing_time = current_gait_->getSwingTime();
  Vec4<double> stance_time = current_gait_->getStanceTime();

  Eigen::VectorXd q(des_kine_solver_->getNq()), v(des_kine_solver_->getNv());
  q.setZero();
  v.setZero();
  q.head(7) << state_->pos_, state_->quat_.coeffs();
  v.head(6) << v_world_des, 0, 0, command_->angular_vel_(2);
  des_kine_solver_->calcForwardKinematics(q, v);

  // front rare
  // static constexpr double sign_fr[4] = {1.0, 1.0, -1.0, -1.0};
  // left right
  static constexpr double sign_lr[4] = {1.0, -1.0, 1.0, -1.0};
  line_pub_->addLine(state_->pos_[0], state_->pos_[1], 1,state_->pos_[0]+state_->linear_vel_[0],state_->pos_[1]+state_->linear_vel_[1],1+state_->linear_vel_[2]);
  for (int i = 0; i < 4; ++i) {
    // pfoot = phip
    Eigen::Vector3d pos;
    pos << kine_solver_->getHipLocationWorld(i);
    // offset
    pos(0) -= sign_lr[i] * std::sin(state_rpy(2)) * 0.085;
    pos(1) += sign_lr[i] * std::cos(state_rpy(2)) * 0.085;
    // pfoot +=  vhip*tstance/2
    Eigen::Vector3d vel;
    vel << kine_solver_->getHipVelocityWorld(i);
    line_pub_->addLine(pos(0), pos(1), pos(2), pos(0) + vel(0), pos(1) + vel(1),
                       pos(2));
    pos += vel * stance_time(i) / 2;
    // pfoot += kp* (vhip-vhip_des)
    Eigen::Vector3d vhip_des;
    vhip_des = des_kine_solver_->getHipVelocityWorld(i);
    pos += (vhip_des - vel) * 0.03;
    pos(2) = 0.0;
    p3d_pub_->addPoint(pos(0), pos(1), pos(2));
    if (table[i] == 0 && swing_stance_config_[i].stance_ == true) {
      setFirstSwing(i, pos, 0.08, swing_time[i]);
    } else if (table[i] == 0) {
      setSwing(i, pos);
    }
  }
  p3d_pub_->update(node_->now());
  line_pub_->update(node_->now());
  return QuadrupedMpcController::update();
}
} // namespace quadruped_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controllers::QuadrupedLocomotionController,
                       controller_interface::ControllerInterface)