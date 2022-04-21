#pragma once

#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/foot_swing_trajectory.hpp"
#include "quadruped_controllers/leg_controller.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/visibility_control.h"
#include <cstddef>
#include <memory>
#include <rclcpp/time.hpp>

namespace quadruped_controllers {
class QuadrupedSwingStanceController : public QuadrupedControllerBase {
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
  void setSwing(size_t leg, Eigen::Vector3d &final_pos, double height,
                double duration);
  QUADRUPED_CONTROLLERS_PUBLIC
  void setStance(size_t leg, Eigen::Vector3d &force_ff);

protected:
  struct SwingStanceConfig {
    bool stance_;
    rclcpp::Time cmd_time_;
    double swing_duration_;
    Eigen::Vector3d stance_force_ff_;
    Eigen::Vector3d stance_foot_pos_;
  } swing_stance_config_[4];

  std::unique_ptr<FootSwingTrajectory<double>> swing_traj_[4];
  std::shared_ptr<LegCommand> leg_cmd_[4];
  Mat3<double> kp_stance_, kd_stance_, kp_swing_, kd_swing_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      swing_stance_param_callback_handle_;
};

} // namespace quadruped_controllers