#include "quadruped_controllers/mpc_controller.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/swing_stance_controller.hpp"
#include <memory>
#include <rclcpp/parameter.hpp>
#include <string>
#include <type_traits>
#include <vector>

namespace quadruped_controllers {
controller_interface::return_type
QuadrupedMpcController::init(const std::string &controller_name) {
  auto ret = QuadrupedControllerBase::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  mpc_config_.weight.ori_roll_ =
      auto_declare<double>("mpc.weight.ori_roll", 0.25);
  mpc_config_.weight.ori_pitch_ =
      auto_declare<double>("mpc.weight.ori_pitch", 0.25);
  mpc_config_.weight.ori_yaw_ = auto_declare<double>("mpc.weight.ori_yaw", 10);
  mpc_config_.weight.pos_x_ = auto_declare<double>("mpc.weight.pos_x", 2);
  mpc_config_.weight.pos_y_ = auto_declare<double>("mpc.weight.pos_y", 2);
  mpc_config_.weight.pos_z_ = auto_declare<double>("mpc.weight.pos_z", 10);
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
  inertia << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz,
      link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
      link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
  mpc_solver_ = std::make_shared<QpOasesSolver>(link->inertial->mass, -9.81,
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
    bool need_update_mpc = false;
    for (auto &param : params) {
      auto update_mpc_param = [&param, &result, &need_update_mpc](
                                  std::string name, auto &value) {
        if (param.get_name() == name) {
          try {
            value = param.get_value<
                typename std::remove_reference<decltype(value)>::type>();
            need_update_mpc |= true;
          } catch (const rclcpp::ParameterTypeException &e) {
            result.successful = false;
            result.reason = e.what();
            need_update_mpc = false;
          }
        }
      };
      update_mpc_param("mpc.weight.ori_yaw", mpc_config_.weight.ori_yaw_);
      update_mpc_param("mpc.weight.ori_roll", mpc_config_.weight.ori_roll_);
      update_mpc_param("mpc.weight.ori_pitch", mpc_config_.weight.ori_pitch_);
      update_mpc_param("mpc.weight.pos_x", mpc_config_.weight.pos_x_);
      update_mpc_param("mpc.weight.pos_y", mpc_config_.weight.pos_y_);
      update_mpc_param("mpc.weight.pos_z", mpc_config_.weight.pos_z_);
      update_mpc_param("mpc.weight.vel_x", mpc_config_.weight.vel_x_);
      update_mpc_param("mpc.weight.vel_y", mpc_config_.weight.vel_y_);
      update_mpc_param("mpc.weight.vel_z", mpc_config_.weight.vel_z_);
      update_mpc_param("mpc.weight.rate_roll", mpc_config_.weight.rate_roll_);
      update_mpc_param("mpc.weight.rate_pitch", mpc_config_.weight.rate_pitch_);
      update_mpc_param("mpc.weight.rate_yaw", mpc_config_.weight.rate_yaw_);
      update_mpc_param("mpc.weight.alpha", mpc_config_.weight.alpha_);
      update_mpc_param("mpc.horizon", mpc_config_.horizon_);
      update_mpc_param("mpc.dt", mpc_config_.dt_);
    }
    if (need_update_mpc == true && result.successful == true) {
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
    return result;
  };
  mpc_param_callback_handle_ =
      get_node()->add_on_set_parameters_callback(mpc_param_callback);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedMpcController::update(){
  mpc_solver_->solve(get_node()->now(), state_, gait_table_, traj_);
  std::vector<Vec3<double>> solution = mpc_solver_->getSolution();
  for (int i = 0; i < 4; ++i)
    if (gait_table_[i] == 1)
      setStance(i, solution[i]);

  return QuadrupedSwingStanceController::update();
}

} // namespace quadruped_controllers