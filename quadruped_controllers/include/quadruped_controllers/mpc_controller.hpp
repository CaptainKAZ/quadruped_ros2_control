#pragma once
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/gait.hpp"
#include "quadruped_controllers/mpc_solver.hpp"
#include "quadruped_controllers/swing_stance_controller.hpp"
#include <atomic>

namespace quadruped_controllers {
class QuadrupedMpcController : public QuadrupedSwingStanceController {
public:
  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type
  init(const std::string &controller_name) override;
  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type update() override;
  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  void setTraj(const Eigen::VectorXd &traj);
  void setGaitTable(const Eigen::VectorXd &table);

  Eigen::VectorXd gait_table_;
  Eigen::VectorXd traj_;
  std::shared_ptr<MpcSolverBase> mpc_solver_;
  struct SolverConfig {
    struct SolverWeight {
      double ori_roll_;
      double ori_pitch_;
      double ori_yaw_;
      double pos_x_;
      double pos_y_;
      double pos_z_;
      double vel_x_;
      double vel_y_;
      double vel_z_;
      double rate_roll_;
      double rate_pitch_;
      double rate_yaw_;
      double alpha_;
    } weight;
    int horizon_;
    double dt_;
  } mpc_config_;

private:
  std::atomic<bool> need_update_param_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      mpc_param_callback_handle_;
  bool updateParam(void);
};
} // namespace quadruped_controllers