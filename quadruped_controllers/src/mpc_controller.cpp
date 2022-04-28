#include "quadruped_controllers/mpc_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/swing_stance_controller.hpp"
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace quadruped_controllers {
controller_interface::return_type
QuadrupedMpcController::init(const std::string &controller_name) {
  auto ret = QuadrupedSwingStanceController::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  mpc_config_.weight.ori_roll_ =
      auto_declare<double>("mpc.weight.ori_roll", 0.25);
  mpc_config_.weight.ori_pitch_ =
      auto_declare<double>("mpc.weight.ori_pitch", 0.25);
  mpc_config_.weight.ori_yaw_ = auto_declare<double>("mpc.weight.ori_yaw", 10.);
  mpc_config_.weight.pos_x_ = auto_declare<double>("mpc.weight.pos_x", 2.);
  mpc_config_.weight.pos_y_ = auto_declare<double>("mpc.weight.pos_y", 2.);
  mpc_config_.weight.pos_z_ = auto_declare<double>("mpc.weight.pos_z", 10.);
  mpc_config_.weight.vel_x_ = auto_declare<double>("mpc.weight.vel_x", 0.2);
  mpc_config_.weight.vel_y_ = auto_declare<double>("mpc.weight.vel_y", 0.2);
  mpc_config_.weight.vel_z_ = auto_declare<double>("mpc.weight.vel_z", 0.6);
  mpc_config_.weight.rate_roll_ =
      auto_declare<double>("mpc.weight.rate_roll", 0);
  mpc_config_.weight.rate_pitch_ =
      auto_declare<double>("mpc.weight.rate_pitch", 0);
  mpc_config_.weight.rate_yaw_ =
      auto_declare<double>("mpc.weight.rate_yaw", 0.3);
  mpc_config_.weight.alpha_ = auto_declare<double>("mpc.weight.alpha", 1e-6);
  mpc_config_.horizon_ = auto_declare<int>("mpc.horizon", 10);
  mpc_config_.dt_ = auto_declare<double>("mpc.dt", 0.015);

  auto link = urdf_->getLink("base_link");
  Eigen::Matrix3d inertia;
  inertia << link->inertial->ixx*0.9, link->inertial->ixy, link->inertial->ixz,
      link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
      link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
  inertia=inertia*1.2;
  mpc_solver_ = std::make_shared<QpOasesSolver>(link->inertial->mass*2.5, -9.81,
                                                0.6, inertia);
  traj_.resize(12 * mpc_config_.horizon_);
  Eigen::Matrix<double, 13, 1> weight;
  weight << mpc_config_.weight.ori_roll_, mpc_config_.weight.ori_pitch_,
      mpc_config_.weight.ori_yaw_, mpc_config_.weight.pos_x_,
      mpc_config_.weight.pos_y_, mpc_config_.weight.pos_z_,
      mpc_config_.weight.rate_roll_, mpc_config_.weight.rate_pitch_,
      mpc_config_.weight.rate_yaw_, mpc_config_.weight.vel_x_,
      mpc_config_.weight.vel_y_, mpc_config_.weight.vel_z_, 0;
  mpc_solver_->setup(mpc_config_.dt_, mpc_config_.horizon_, 120, weight,
                     mpc_config_.weight.alpha_, 1.0);
  return controller_interface::return_type::OK;
}

CallbackReturn QuadrupedMpcController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  auto ret = QuadrupedSwingStanceController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  auto mpc_param_callback = [this](std::vector<rclcpp::Parameter> params) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (auto &param : params) {
      auto accept_mpc_param = [&param, &result, this](std::string name,
                                                      auto &value) {
        if (param.get_name() == name) {
          try {
            param.get_value<
                typename std::remove_reference<decltype(value)>::type>();
            need_update_param_ = true;
          } catch (const rclcpp::ParameterTypeException &e) {
            result.successful = false;
            result.reason = e.what();
            need_update_param_ = false;
          }
        }
      };
      accept_mpc_param("mpc.weight.ori_yaw", mpc_config_.weight.ori_yaw_);
      accept_mpc_param("mpc.weight.ori_roll", mpc_config_.weight.ori_roll_);
      accept_mpc_param("mpc.weight.ori_pitch", mpc_config_.weight.ori_pitch_);
      accept_mpc_param("mpc.weight.pos_x", mpc_config_.weight.pos_x_);
      accept_mpc_param("mpc.weight.pos_y", mpc_config_.weight.pos_y_);
      accept_mpc_param("mpc.weight.pos_z", mpc_config_.weight.pos_z_);
      accept_mpc_param("mpc.weight.vel_x", mpc_config_.weight.vel_x_);
      accept_mpc_param("mpc.weight.vel_y", mpc_config_.weight.vel_y_);
      accept_mpc_param("mpc.weight.vel_z", mpc_config_.weight.vel_z_);
      accept_mpc_param("mpc.weight.rate_roll", mpc_config_.weight.rate_roll_);
      accept_mpc_param("mpc.weight.rate_pitch", mpc_config_.weight.rate_pitch_);
      accept_mpc_param("mpc.weight.rate_yaw", mpc_config_.weight.rate_yaw_);
      accept_mpc_param("mpc.weight.alpha", mpc_config_.weight.alpha_);
      accept_mpc_param("mpc.horizon", mpc_config_.horizon_);
      accept_mpc_param("mpc.dt", mpc_config_.dt_);
    }
    return result;
  };
  mpc_param_callback_handle_ =
      get_node()->add_on_set_parameters_callback(mpc_param_callback);
  if(updateParam()){
    need_update_param_ = false;
    return CallbackReturn::SUCCESS;
  }else{
    return CallbackReturn::FAILURE;
  }
}

controller_interface::return_type QuadrupedMpcController::update() {
  if(need_update_param_){
    if(updateParam()){
      need_update_param_ = false;
      std::cout<<"update param"<<std::endl;
      return controller_interface::return_type::OK;
    }else{
      return controller_interface::return_type::ERROR;
    }
  }
  mpc_solver_->solve(get_node()->now(), state_, gait_table_, traj_);
  std::vector<Vec3<double>> solution = mpc_solver_->getSolution();
  for (int i = 0; i < 4; ++i)
    if (gait_table_[i] == 1)
      setStance(i, solution[i]);
  return QuadrupedSwingStanceController::update();
}

void QuadrupedMpcController::setTraj(const Eigen::VectorXd &traj) {
  traj_ = traj;
}

void QuadrupedMpcController::setGaitTable(const Eigen::VectorXd &gait_table) {
  gait_table_ = gait_table;
}

CallbackReturn QuadrupedMpcController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto ret = QuadrupedSwingStanceController::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  Eigen::Vector3d force;
  force.setZero();
  for (size_t i = 0; i < 4; ++i)
    setStance(i, force);
  return CallbackReturn::SUCCESS;
}

bool QuadrupedMpcController::updateParam() {
  auto getParam = [this](std::string name, auto &value) {
    try {
      value = node_->get_parameter(name)
                  .get_value<
                      typename std::remove_reference<decltype(value)>::type>();
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(node_->get_logger(), e.what());
      return false;
    }
    return true;
  };
  if (getParam("mpc.weight.ori_yaw", mpc_config_.weight.ori_yaw_) &&
      getParam("mpc.weight.ori_roll", mpc_config_.weight.ori_roll_) &&
      getParam("mpc.weight.ori_pitch", mpc_config_.weight.ori_pitch_) &&
      getParam("mpc.weight.pos_x", mpc_config_.weight.pos_x_) &&
      getParam("mpc.weight.pos_y", mpc_config_.weight.pos_y_) &&
      getParam("mpc.weight.pos_z", mpc_config_.weight.pos_z_) &&
      getParam("mpc.weight.vel_x", mpc_config_.weight.vel_x_) &&
      getParam("mpc.weight.vel_y", mpc_config_.weight.vel_y_) &&
      getParam("mpc.weight.vel_z", mpc_config_.weight.vel_z_) &&
      getParam("mpc.weight.rate_roll", mpc_config_.weight.rate_roll_) &&
      getParam("mpc.weight.rate_pitch", mpc_config_.weight.rate_pitch_) &&
      getParam("mpc.weight.rate_yaw", mpc_config_.weight.rate_yaw_) &&
      getParam("mpc.weight.alpha", mpc_config_.weight.alpha_) &&
      getParam("mpc.horizon", mpc_config_.horizon_) &&
      getParam("mpc.dt", mpc_config_.dt_)) {
    traj_.resize(12 * mpc_config_.horizon_);
    Eigen::Matrix<double, 13, 1> weight;
    weight << mpc_config_.weight.ori_roll_, mpc_config_.weight.ori_pitch_,
        mpc_config_.weight.ori_yaw_, mpc_config_.weight.pos_x_,
        mpc_config_.weight.pos_y_, mpc_config_.weight.pos_z_,
        mpc_config_.weight.rate_roll_, mpc_config_.weight.rate_pitch_,
        mpc_config_.weight.rate_yaw_, mpc_config_.weight.vel_x_,
        mpc_config_.weight.vel_y_, mpc_config_.weight.vel_z_, 0;
    mpc_solver_->setup(mpc_config_.dt_, mpc_config_.horizon_, 120, weight,
                       mpc_config_.weight.alpha_, 1.0);
    return true;
  } else {
    return false;
  }
}

void QuadrupedMpcController::setMpcHorizonDt(double horizon, double dt) {
  mpc_config_.horizon_ = horizon;
  mpc_config_.dt_ = dt;
  traj_.resize(12 * mpc_config_.horizon_);
  Eigen::Matrix<double, 13, 1> weight;
  weight << mpc_config_.weight.ori_roll_, mpc_config_.weight.ori_pitch_,
      mpc_config_.weight.ori_yaw_, mpc_config_.weight.pos_x_,
      mpc_config_.weight.pos_y_, mpc_config_.weight.pos_z_,
      mpc_config_.weight.rate_roll_, mpc_config_.weight.rate_pitch_,
      mpc_config_.weight.rate_yaw_, mpc_config_.weight.vel_x_,
      mpc_config_.weight.vel_y_, mpc_config_.weight.vel_z_, 0;
  mpc_solver_->setup(mpc_config_.dt_, mpc_config_.horizon_, 120, weight,
                     mpc_config_.weight.alpha_, 1.0);
}

} // namespace quadruped_controllers