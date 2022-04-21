#pragma once
#include "quadruped_controllers/controller_base.hpp"
#include "swing_stance_controller.hpp"
#include <memory>
#include <rclcpp/time.hpp>

namespace quadruped_controllers {
class QuadrupedStandUpController : public QuadrupedControllerBase {
public:
  //override on_active
  QUADRUPED_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  //override update
  QUADRUPED_CONTROLLERS_PUBLIC
  controller_interface::return_type update() override;
  //TODO Recover Stand

protected:
Eigen::Vector3d init_foot_pos_[4];
std::shared_ptr<LegCommand> leg_command_[4];
};
}