#include "quadruped_controllers/stand_up_controller.hpp"
#include "quadruped_controllers/controller_base.hpp"
#include "quadruped_controllers/state_update.hpp"
#include <cstddef>
#include <rclcpp/logging.hpp>

namespace quadruped_controllers {

CallbackReturn QuadrupedStandUpController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto ret = QuadrupedControllerBase::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // update once
  for (auto &state_updater : state_updater_queue_) {
    state_updater->update(node_->now());
  }
  for (size_t i = 0; i < 4; ++i) {
    init_foot_pos_[i] = state_->foot_pos_[i];
    leg_command_[i] = leg_controller_->getLegCommand(i);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedStandUpController::update() {
  constexpr double target_height = 0.2;
  double time_from_start = (node_->now() - start_time_).seconds();
  if (time_from_start > 1) {
    time_from_start = 1;
  }
  for (size_t i = 0; i < 4; ++i) {
    leg_command_[i]->kp_cartesian_ = Vec3<double>(500, 500, 500).asDiagonal();
    leg_command_[i]->kd_cartesian_ = Vec3<double>(8, 8, 8).asDiagonal();
    leg_command_[i]->foot_pos_des_ = init_foot_pos_[i];
    leg_command_[i]->foot_pos_des_(2) =
        -target_height * time_from_start +
        (1 - time_from_start) * init_foot_pos_[i](2);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "foot_pos_des_: " << leg_command_[i]->foot_pos_des_(2));
  }
  return QuadrupedControllerBase::update();
}

} // namespace quadruped_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(quadruped_controllers::QuadrupedStandUpController,
                       controller_interface::ControllerInterface)