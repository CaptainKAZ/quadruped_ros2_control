#pragma once

#include "quadruped_controllers/pinocchio_solver.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <semantic_components/imu_sensor.hpp>
#include <unordered_map>
#include <vector>
#include <quadruped_controllers/quadruped_interface.hpp>

// Get infomation form joint interfaces, imu interface or directly form ground
// truth
namespace quadruped_controllers {

class StateUpdateBase {
public:
  StateUpdateBase() = delete;
  StateUpdateBase(const StateUpdateBase &) = delete;
  StateUpdateBase(rclcpp::Node::SharedPtr &&node,
                  std::shared_ptr<QuadrupedState> &state) {
    node_ = node;
    state_ = state;
  };
  virtual ~StateUpdateBase(){};
  virtual void update(const rclcpp::Time& current_time)=0;

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<QuadrupedState> state_;
};

class FromGroundTruthStateUpdate : public StateUpdateBase {
public:
  explicit FromGroundTruthStateUpdate(
      rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state,
      std::shared_ptr<QuadrupedState> ground_truth_state);
  ~FromGroundTruthStateUpdate() override{};

  void update(const rclcpp::Time& current_time) override;

private:
  std::shared_ptr<QuadrupedState> ground_truth_state_;
};

class LinearKFPosVelEstimateUpdate : public StateUpdateBase {
public:
  LinearKFPosVelEstimateUpdate(rclcpp::Node::SharedPtr &&node,
                               std::shared_ptr<QuadrupedState> &state);
  void update(const rclcpp::Time& time) override;

private:
  Eigen::Matrix<double, 18, 1> x_hat_;
  Eigen::Matrix<double, 12, 1> ps_;
  Eigen::Matrix<double, 12, 1> vs_;
  Eigen::Matrix<double, 18, 18> a_;
  Eigen::Matrix<double, 18, 18> q_;
  Eigen::Matrix<double, 18, 18> p_;
  Eigen::Matrix<double, 28, 28> r_;
  Eigen::Matrix<double, 18, 3> b_;
  Eigen::Matrix<double, 28, 18> c_;
};

class InterfaceStateUpdate : public StateUpdateBase {
public:
  InterfaceStateUpdate(rclcpp::Node::SharedPtr &&node,
                       std::shared_ptr<QuadrupedState> &state,
                       std::shared_ptr<QuadrupedInterface> quadrupedInterface);
  void update(const rclcpp::Time& time) override;

private:
  std::shared_ptr<QuadrupedInterface> quadruped_interface_;
  double initial_yaw_;
};

class KinematicSolverStateUpdate : public StateUpdateBase {
public:
  KinematicSolverStateUpdate(
      rclcpp::Node::SharedPtr &&node, std::shared_ptr<QuadrupedState> &state,
      std::shared_ptr<PinocchioSolver> &pinocchio_solver);
  void update(const rclcpp::Time& time) override;

protected:
  std::shared_ptr<PinocchioSolver> pinocchio_solver_;
};

} // namespace quadruped_controllers