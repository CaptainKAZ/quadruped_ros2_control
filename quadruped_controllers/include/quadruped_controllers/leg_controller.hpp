#pragma once

#include "quadruped_controllers/pinocchio_solver.hpp"
#include "quadruped_controllers/quadruped_interface.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include <cstddef>
#include <memory>

// Leg controller accepts data form LegCommand and convert it to joint interface
// command

namespace quadruped_controllers {
struct LegCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  rclcpp::Time stamp_;
  Eigen::Vector3d foot_pos_des_, foot_vel_des_, ff_cartesian_;
  Eigen::Vector3d q_des_, qd_des_, tau_ff_;
  Eigen::Vector3d kp_joint_, kd_joint_;
  Eigen::Matrix3d kp_cartesian_, kd_cartesian_;
};

class LegController {
public:
  explicit LegController(std::shared_ptr<QuadrupedInterface> &interface,
                         std::shared_ptr<PinocchioSolver> &solver,
                         std::shared_ptr<QuadrupedState> &state) {
    if (interface == nullptr || solver == nullptr || state == nullptr) {
      throw std::runtime_error(
          "LegController: interface or solver or state is nullptr");
    }
    interface_ = interface;
    solver_ = solver;
    state_ = state;
    for (auto &legCommand : leg_commands_) {
      legCommand = std::allocate_shared<LegCommand>(Eigen::aligned_allocator<LegCommand>());
    }
    zero();
  }
  void zero() {
    for (auto &legCommand : leg_commands_) {
      legCommand->foot_pos_des_ = Eigen::Vector3d::Zero();
      legCommand->foot_vel_des_ = Eigen::Vector3d::Zero();
      legCommand->ff_cartesian_ = Eigen::Vector3d::Zero();
      legCommand->kp_cartesian_ = Eigen::Matrix3d::Zero();
      legCommand->kd_cartesian_ = Eigen::Matrix3d::Zero();
    }
  }
  void zero(size_t leg) {
    leg_commands_[leg]->foot_pos_des_ = Eigen::Vector3d::Zero();
    leg_commands_[leg]->foot_vel_des_ = Eigen::Vector3d::Zero();
    leg_commands_[leg]->ff_cartesian_ = Eigen::Vector3d::Zero();
    leg_commands_[leg]->kp_cartesian_ = Eigen::Matrix3d::Zero();
    leg_commands_[leg]->kd_cartesian_ = Eigen::Matrix3d::Zero();
  }
  void eStop() { zero(); writeInterface();}
  void writeInterface() {
    for (size_t leg = 0; leg < 4; ++leg) {
      Eigen::Vector3d foot_force = leg_commands_[leg]->ff_cartesian_;
      // cartesian PD
      foot_force +=
          leg_commands_[leg]->kp_cartesian_ *
          (leg_commands_[leg]->foot_pos_des_ - state_->foot_pos_[leg]);
      foot_force +=
          leg_commands_[leg]->kd_cartesian_ *
          (leg_commands_[leg]->foot_vel_des_ - state_->foot_vel_[leg]);
      Eigen::Matrix<double, 6, 18> jac;
      solver_->getJacobian(LEG_CONFIG[leg], jac);
      Eigen::Matrix<double, 6, 1> wrench;
      wrench.setZero();
      wrench.head(3) = foot_force;
      Eigen::Matrix<double, 18, 1> tau =
          jac.transpose() * wrench ;
      for (int joint = 0; joint < 3; ++joint) {
        interface_->getLeg(leg)->getJoint(joint)->SetCommand(
            leg_commands_[leg]->q_des_(joint),
            leg_commands_[leg]->qd_des_(joint), tau(6 + leg * 3 + joint)+leg_commands_[leg]->tau_ff_(joint),
            leg_commands_[leg]->kp_joint_(joint),
            leg_commands_[leg]->kd_joint_(joint));
      }
    }
  }
  inline std::shared_ptr<LegCommand>& getLegCommand(size_t leg) {
    return leg_commands_[leg];
  }
  std::shared_ptr<LegCommand> leg_commands_[4];
private:
  std::shared_ptr<PinocchioSolver> solver_;
  std::shared_ptr<QuadrupedInterface> interface_;
  std::shared_ptr<QuadrupedState> state_;
};

} // namespace quadruped_controllers