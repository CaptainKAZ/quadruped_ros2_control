#include "quadruped_controllers/state_update.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/utils.hpp"
#include <cstddef>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rcpputils/asserts.hpp>

namespace quadruped_controllers {

FromGroundTruthStateUpdate::FromGroundTruthStateUpdate(
    rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state,
    std::shared_ptr<QuadrupedState> ground_truth_state)
    : StateUpdateBase(std::forward<decltype(node)>(node),
                      std::forward<decltype(state)>(state)) {
  this->ground_truth_state_ = ground_truth_state;
  this->ground_truth_state_->update_time_=this->node_->now();
}

void FromGroundTruthStateUpdate::update(const rclcpp::Time &current_time) {
  if ((current_time - ground_truth_state_->update_time_).seconds() < 0.01) {
    state_->update_time_ = ground_truth_state_->update_time_;
    state_->linear_vel_=ground_truth_state_->linear_vel_;
    state_->accel_ = ground_truth_state_->accel_;
    state_->pos_ = ground_truth_state_->pos_;
    state_->angular_vel_ = ground_truth_state_->angular_vel_;
    state_->quat_ = ground_truth_state_->quat_;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "Try to update groundtruth failed: Too old!");
  }
}

InterfaceStateUpdate::InterfaceStateUpdate(
    rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state,
    std::shared_ptr<QuadrupedInterface> quadrupedInterface)
    : StateUpdateBase(std::forward<decltype(node)>(node),
                      std::forward<decltype(state)>(state)) {
  quadruped_interface_ = quadrupedInterface;
}

void InterfaceStateUpdate::update(const rclcpp::Time &time) {
  // TODO: check interface valaid
  state_->update_time_ = time;
  for (size_t leg = 0; leg < 4; leg++) {
    for (size_t joint = 0; joint < 3; joint++) {
      state_->joint_pos_(leg * 3 + joint) =
          quadruped_interface_->getLeg(leg)->getJoint(joint)->getPosition();
      state_->joint_vel_(leg * 3 + joint) =
          quadruped_interface_->getLeg(leg)->getJoint(joint)->getVelocity();
    }
    state_->contact_state_[leg] =
        quadruped_interface_->getLeg(leg)->getContact();
  }
  auto quatArray_ = quadruped_interface_->getIMUSensor()->get_orientation();
  state_->quat_.coeffs() << quatArray_[0], quatArray_[1], quatArray_[2],
      quatArray_[3];
  state_->accel_ = Eigen::Map<decltype(state_->accel_)>(
      quadruped_interface_->getIMUSensor()->get_linear_acceleration().data(),
      3);
  //state_->accel_=state_->quat_.toRotationMatrix()*state_->accel_;
  state_->angular_vel_ = Eigen::Map<decltype(state_->angular_vel_)>(
      quadruped_interface_->getIMUSensor()->get_angular_velocity().data(), 3);
  state_->angular_vel_=state_->quat_.toRotationMatrix()*state_->angular_vel_;
  
  if (initial_yaw_ == 0)
    initial_yaw_ = quatToRPY(state_->quat_)(2);

  Eigen::Quaternion<double> yaw =
      RpyToQuat(Vec3<double>(0., 0., -initial_yaw_));
  state_->quat_ *= yaw;
}

KinematicSolverStateUpdate::KinematicSolverStateUpdate(
    rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state,
    std::shared_ptr<PinocchioSolver> &kinematicSolver)
    : StateUpdateBase(std::forward<decltype(node)>(node),
                      std::forward<decltype(state)>(state)) {
  rcpputils::check_true(kinematicSolver != nullptr,
                        "kinematicSolver is nullptr");
  pinocchio_solver_ = kinematicSolver;
}

void KinematicSolverStateUpdate::update(const rclcpp::Time &time) {
  Eigen::VectorXd q(pinocchio_solver_->getNq()), v(pinocchio_solver_->getNv());
  q.tail(12) = state_->joint_pos_;
  v.tail(12) = state_->joint_vel_;
  q.head(7) << state_->pos_, state_->quat_.coeffs();
  v.head(6) << state_->linear_vel_, state_->angular_vel_;
  pinocchio_solver_->calcForwardKinematics(q, v);
  pinocchio_solver_->getFootPosVel(state_);
  state_->update_time_ = time;
}

LinearKFPosVelEstimateUpdate::LinearKFPosVelEstimateUpdate(
    rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state)
    : StateUpdateBase(std::forward<decltype(node)>(node),
                      std::forward<decltype(state)>(state)) {
  double dt = 0.001;
  x_hat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  a_.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
  b_.setZero();
  // b_.block(0, 0, 3, 3) =
  //     0.5 * dt * dt * Eigen::Matrix<double, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<double, 3, 3>::Identity(),
      Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<double, 3, 3>::Zero(),
      Eigen::Matrix<double, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(6, 0, 3, 6) = c1;
  c_.block(9, 0, 3, 6) = c1;
  c_.block(0, 6, 12, 12) = -1.0* Eigen::Matrix<double, 12, 12>::Identity();
  c_.block(12, 0, 3, 6) = c2;
  c_.block(15, 0, 3, 6) = c2;
  c_.block(18, 0, 3, 6) = c2;
  c_.block(21, 0, 3, 6) = c2;
  c_(27, 17) = 1.0;
  c_(26, 14) = 1.0;
  c_(25, 11) = 1.0;
  c_(24, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) =
      (dt * 9.81f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<double, 12, 12>::Identity();
  r_.setIdentity();
}

void LinearKFPosVelEstimateUpdate::update(const rclcpp::Time &time) {
  constexpr double imu_process_noise_position = 0.02;
  constexpr double imu_process_noise_velocity = 0.02;
  constexpr double foot_process_noise_position = 0.002;
  constexpr double foot_sensor_noise_position = 0.001;
  constexpr double foot_sensor_noise_velocity = 0.1;
  constexpr double foot_height_sensor_noise = 0.001;
  Eigen::Matrix<double, 18, 18> q = Eigen::Matrix<double, 18, 18>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity;
  q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * foot_process_noise_position;

  Eigen::Matrix<double, 28, 28> r = Eigen::Matrix<double, 28, 28>::Identity();
  r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * foot_sensor_noise_position;
  r.block(12, 12, 12, 12) =
      r_.block(12, 12, 12, 12) * foot_sensor_noise_velocity;
  r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * foot_height_sensor_noise;

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;

    int q_index = 6 + i1;
    int r_index1 = i1;
    int r_index2 = 12 + i1;
    int r_index3 = 24 + i;

    double high_suspect_number(100);
    q.block(q_index, q_index, 3, 3) =
        (state_->contact_state_[i] ? 1. : high_suspect_number) *
        q.block(q_index, q_index, 3, 3);
    r.block(r_index1, r_index1, 3, 3) = 1. * r.block(r_index1, r_index1, 3, 3);
    r.block(r_index2, r_index2, 3, 3) =
        (state_->contact_state_[i] ? 1. : high_suspect_number) *
        r.block(r_index2, r_index2, 3, 3);
    r(r_index3, r_index3) =
        (state_->contact_state_[i] ? 1. : high_suspect_number) *
        r(r_index3, r_index3);

    ps_.segment(3 * i, 3) = state_->pos_ - state_->foot_pos_[i];
    vs_.segment(3 * i, 3) = state_->linear_vel_ - state_->foot_vel_[i];
  }

  Vec3<double> g(0, 0, -9.81);
  Vec3<double> accel = state_->quat_.toRotationMatrix() * state_->accel_ + g;
  Vec4<double> pzs = Vec4<double>::Zero();

  Eigen::Matrix<double, 28, 1> y;
  y << ps_, vs_, pzs;
  x_hat_ = a_ * x_hat_ + b_ * accel;
  Eigen::Matrix<double, 18, 18> at = a_.transpose();
  Eigen::Matrix<double, 18, 18> pm = a_ * p_ * at + q;
  Eigen::Matrix<double, 18, 28> ct = c_.transpose();
  Eigen::Matrix<double, 28, 1> y_model = c_ * x_hat_;
  Eigen::Matrix<double, 28, 1> ey = y - y_model;
  Eigen::Matrix<double, 28, 28> s = c_ * pm * ct + r;

  Eigen::Matrix<double, 28, 1> s_ey = s.lu().solve(ey);
  x_hat_ += pm * ct * s_ey;

  Eigen::Matrix<double, 28, 18> s_c = s.lu().solve(c_);
  p_ = (Eigen::Matrix<double, 18, 18>::Identity() - pm * ct * s_c) * pm;

  Eigen::Matrix<double, 18, 18> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
    p_.block(0, 2, 2, 16).setZero();
    p_.block(2, 0, 16, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }

  state_->pos_ = x_hat_.block(0, 0, 3, 1);
  state_->linear_vel_ = x_hat_.block(3, 0, 3, 1);
  state_->update_time_ = time;
}

} // namespace quadruped_controllers