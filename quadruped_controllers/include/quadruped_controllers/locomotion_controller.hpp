#pragma once

#include "quadruped_controllers/gait.hpp"
#include "quadruped_controllers/mpc_solver.hpp"
#include "quadruped_controllers/mpc_controller.hpp"
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

namespace quadruped_controllers {
class QuadrupedLocomotionController : public QuadrupedMpcController {
public:
  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type
  init(const std::string &controller_name) override;
  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type update() override;
  void setGait(const std::string name, std::shared_ptr<OffsetDurationGait<double>> gait);

protected:
  inline Eigen::VectorXd& calcTrajFromCommand(Eigen::VectorXd& traj);
  inline Eigen::Vector3d& calcMitCheetahFootPos(size_t leg,Eigen::Vector3d& footpos);
  std::unordered_map<std::string, std::shared_ptr<OffsetDurationGait<double>>> gaits_;
  std::shared_ptr<OffsetDurationGait<double>> current_gait_;
  bool first_swing_[4];
  rclcpp::Time start_swing_time_[4];
};
}