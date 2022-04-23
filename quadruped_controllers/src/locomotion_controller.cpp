#include "quadruped_controllers/locomotion_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/mpc_controller.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
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
    if (offsets.size() != durations.size() || offsets.size() != 4) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "offsets and durations must have the same size 4");
      return controller_interface::return_type::ERROR;
    }
    Vec4<double> offsetsVec;
    offsetsVec << offsets[0], offsets[1], offsets[2], offsets[3];
    Vec4<double> durationsVec;
    durationsVec << durations[0], durations[1], durations[2], durations[3];
    gaits_.emplace(name, std::make_shared<OffsetDurationGait<double>>(
                             cycle, offsetsVec, durationsVec));
  }
  std::string current =
      auto_declare<std::string>("locomotion.gait.current", {});
  try {
    current_gait_ = gaits_.at(current);
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
  p3d_pub_=std::make_shared<P3dPublisher>(get_node(),state_,command_);
  need_update_param_ = false;
  auto locomotion_param_callback_handle =
      [this](std::vector<rclcpp::Parameter> params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (const auto &param : params) {
          if (param.get_name() == "locomotion.gait.current") {
            if(gaits_.find(param.get_value<std::string>()) == gaits_.end()) {
              RCLCPP_ERROR(get_node()->get_logger(),
                           "current gait %s is not found",
                           param.get_value<std::string>().c_str());
              result.successful = false;
              result.reason = "current gait is not found";
              break;
            }else {
              need_update_param_=true;
            }
          } else if (param.get_name().find("locomotion.gait.") !=
                     std::string::npos) {
            result.successful = false;
            result.reason = "not allow to set gait parameters by now";
          }
        }
        return result;
      };
  locomotion_param_callback_handle_=node_->add_on_set_parameters_callback(locomotion_param_callback_handle);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedLocomotionController::update() {
  if (need_update_param_) {
    need_update_param_ = false;
    try{
    current_gait_ = gaits_.at(node_->get_parameter("locomotion.gait.current").get_value<std::string>());
    }catch(const std::out_of_range &e){
      RCLCPP_ERROR(get_node()->get_logger(), "current gait %s is not found",
                   node_->get_parameter("locomotion.gait.current").get_value<std::string>().c_str());
      return controller_interface::return_type::ERROR;
    }
  }
  Eigen::VectorXd traj;
  // setTraj(calcTrajFromCommand(traj));
  traj.resize(12 * mpc_config_.horizon_);
  traj.setZero();
  for (int i = 0; i < mpc_config_.horizon_; ++i)
    traj[12 * i + 5] = 0.26;
  setTraj(traj);

  current_gait_->update(node_->now());
  Eigen::VectorXd table = current_gait_->getMpcTable(mpc_solver_->getHorizon());
  //table.setOnes();
  setGaitTable(table);
  Vec4<double> swing_time = current_gait_->getSwingTime();
  // front rare
  static constexpr double sign_fr[4] = {1.0, 1.0, -1.0, -1.0};
  // left right
  static constexpr double sign_lr[4] = {1.0, -1.0, 1.0, -1.0};
  for (int i = 0; i < 4; ++i) {
    // if (table[i] == 0 && swing_stance_config_[i].stance_ == true) {
    //   Eigen::Vector3d pos;
    //   first_swing_[i]=true;
    //   calcMitCheetahFootPos(i, pos);
    //   setSwing(i, pos, 0.05, swing_time[i]);
    // }else if(table[i] == 0 && swing_stance_config_[i].stance_ == false){
    //   Eigen::Vector3d pos;
    //   first_swing_[i]=false;
    //   calcMitCheetahFootPos(i, pos);
    //   setSwing(i, pos, 0.05, swing_time[i]);
    // }
    if (table[i] == 0 && swing_stance_config_[i].stance_ == true) {
      Eigen::Vector3d pos;
      pos << kine_solver_->getHipLocationWorld(i);
      pos(2)=-0.05;
      pos(1)+=sign_lr[i]*0.082;
      p3d_pub_->setPoint(pos(0),pos(1),0,"world");
      //p3d_pub_->update(node_->now());
      std::cout<<"setting swing " +LEG_CONFIG[i]<<std::endl;
      setSwing(i, pos, 0.07, swing_time[i]);
    }
    //std::cout<<table[0]<<" "<<table[1]<<" "<<table[2]<<" "<<table[3]<<std::endl;
  }
  return QuadrupedMpcController::update();
}

Eigen::VectorXd &
QuadrupedLocomotionController::calcTrajFromCommand(Eigen::VectorXd &traj) {
  Vec3<double> v_des_world =
      state_->quat_.toRotationMatrix() * command_->linear_vel_;
  traj.resize(12 * mpc_config_.horizon_);
  traj.setZero();
  for (int i = 0; i < mpc_config_.horizon_; ++i) {
    if (i == 0) {
      traj.head(12) << command_->rpy_, command_->xyz_, command_->angular_vel_,
          v_des_world;
      // height = 0.29 from mit code
      traj(12 * i + 5) = 0.35;
    } else {
      // integrate rpy and xyz
      traj.segment(12 * i, 3) = traj.segment(12 * (i - 1), 3) +
                                command_->angular_vel_ * mpc_config_.dt_;
      traj.segment(12 * i + 3, 3) = traj.segment(12 * (i - 1) + 3, 3) +
                                    command_->linear_vel_ * mpc_config_.dt_;
      // vel and rate remains the same
      traj.segment(12 * i + 6, 3) = command_->angular_vel_;
      traj.segment(12 * i + 9, 3) = v_des_world;
    }
  }
  return traj;
}

Eigen::Vector3d &
QuadrupedLocomotionController::calcMitCheetahFootPos(size_t leg,
                                                     Eigen::Vector3d &footpos) {
  Vec3<double> v_des_world =
      state_->quat_.toRotationMatrix() * command_->linear_vel_;

  // front rare
  // static constexpr double sign_fr[4] = {1.0, 1.0, -1.0, -1.0};
  // left right
  static constexpr double sign_lr[4] = {1.0, -1.0, 1.0, -1.0};
  footpos.setZero();
  auto pRobotFrame = kine_solver_->getHipLocationRef(leg);
  pRobotFrame[2] = 0;
  Vec3<double> offset(0, sign_lr[leg] * .065, 0);
  pRobotFrame += offset;
  double stance_time = current_gait_->getStanceTime(leg);
  Eigen::AngleAxisd transfrom(-command_->angular_vel_.z() * stance_time / 2,
                              Vector3d(0, 0, 1));
  Vec3<double> pYawCorrected = transfrom.matrix() * pRobotFrame;
  double swingTimeRemaining;
  if (first_swing_[leg]) {
    swingTimeRemaining = current_gait_->getSwingTime(leg);
    start_swing_time_[leg] = node_->now();
  } else {
    swingTimeRemaining = current_gait_->getSwingTime(leg) -
                         (node_->now() - start_swing_time_[leg]).seconds();
  }
  Vec3<double> Pf =
      state_->pos_ +
      state_->quat_.toRotationMatrix().transpose() *
          (pYawCorrected + command_->linear_vel_ * swingTimeRemaining);
  float p_rel_max = 0.3f;

  // Using the estimated velocity is correct
  // Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
  float pfx_rel = state_->linear_vel_[0] * (.5) * stance_time +
                  .03f * (state_->linear_vel_[0] - v_des_world[0]) +
                  (0.5f * state_->pos_[2] / 9.81f) *
                      (state_->linear_vel_[1] * command_->angular_vel_[2]);

  float pfy_rel = state_->linear_vel_[1] * .5 * stance_time +
                  .03f * (state_->linear_vel_[1] - v_des_world[1]) +
                  (0.5f * state_->pos_[2] / 9.81f) *
                      (-state_->linear_vel_[0] * command_->angular_vel_[2]);
  pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
  pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
  Pf[0] += pfx_rel;
  Pf[1] += pfy_rel;
  Pf[2] = -0.003;
  footpos = Pf;

  return footpos;
}

} // namespace quadruped_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controllers::QuadrupedLocomotionController,
                       controller_interface::ControllerInterface)