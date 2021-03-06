#include "quadruped_controllers/pinocchio_solver.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "quadruped_controllers/quadruped_types.hpp"

#include "rcpputils/asserts.hpp"
#include <cstddef>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>

namespace quadruped_controllers {
void PinocchioSolver::parseUrdf(std::shared_ptr<urdf::ModelInterface> &urdf) {
  pin_model_ = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(),
                              *pin_model_);
  pin_data_ = std::allocate_shared<pinocchio::Data>(
      Eigen::aligned_allocator<pinocchio::Data>(), *pin_model_);
}

void PinocchioSolver::calcForwardKinematics(const Eigen::VectorXd &q,
                                            const Eigen::VectorXd &v) {
  pinocchio::forwardKinematics(*pin_model_, *pin_data_, q, v);
}

void PinocchioSolver::getFootPosVel(std::shared_ptr<QuadrupedState> &state) {
  pinocchio::computeJointJacobians(*pin_model_, *pin_data_);
  pinocchio::updateFramePlacements(*pin_model_, *pin_data_);
  for (size_t leg = 0; leg < 4; ++leg) {
    rcpputils::check_true(
        pin_model_->existFrame(LEG_CONFIG[leg] + "_foot_link"),
        LEG_CONFIG[leg] + "_foot_link does not exist");
    pinocchio::FrameIndex frame_id =
        pin_model_->getFrameId(LEG_CONFIG[leg] + "_foot_link");
    state->foot_pos_[leg] = pin_data_->oMf[frame_id].translation();
    state->foot_vel_[leg] = pinocchio::getFrameVelocity(
                                *pin_model_, *pin_data_, frame_id,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
                                .linear();
  }
}

void PinocchioSolver::getJacobian(const std::string &leg,
                                  Eigen::Matrix<double, 6, 18> &jacobian) {
  rcpputils::check_true(pin_model_->existFrame(leg + "_foot_link"),
                        leg + "_foot_link does not exist");
  pinocchio::getFrameJacobian(*pin_model_, *pin_data_,
                              pin_model_->getFrameId(leg + "_foot_link"),
                              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                              std::forward<decltype(jacobian)>(jacobian));
}

size_t PinocchioSolver::getNq() const { return pin_model_->nq; }
size_t PinocchioSolver::getNv() const { return pin_model_->nv; }

const Eigen::Vector3d &PinocchioSolver::getHipLocationRef(size_t leg) {
  rcpputils::check_true(
      pin_model_->existJointName(LEG_CONFIG[leg] + "_hip_joint"),
      LEG_CONFIG[leg] + "_hip_link does not exist");
  pinocchio::JointIndex joint_id =
      pin_model_->getJointId(LEG_CONFIG[leg] + "_hip_joint");
  return (pin_data_->liMi[joint_id]).translation();
}

const Eigen::Vector3d &PinocchioSolver::getHipLocationWorld(size_t leg) {
  rcpputils::check_true(
      pin_model_->existJointName(LEG_CONFIG[leg] + "_hip_joint"),
      LEG_CONFIG[leg] + "_joint_link does not exist");
  pinocchio::JointIndex joint_id =
      pin_model_->getJointId(LEG_CONFIG[leg] + "_hip_joint");
  return (pin_data_->oMi[joint_id]).translation();
}

const Eigen::Vector3d PinocchioSolver::getHipVelocityWorld(size_t leg) {
  pinocchio::updateFramePlacements(*pin_model_, *pin_data_);
  rcpputils::check_true(pin_model_->existFrame(LEG_CONFIG[leg] + "_hip_link"),
                        LEG_CONFIG[leg] + "_hip_link does not exist");
  pinocchio::FrameIndex frame_id =
      pin_model_->getFrameId(LEG_CONFIG[leg] + "_hip_link");
  return pinocchio::getFrameVelocity(
             *pin_model_, *pin_data_, frame_id,
             pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
      .linear();
}

} // namespace quadruped_controllers