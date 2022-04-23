#pragma once

#include "quadruped_controllers/gait.hpp"
#include "quadruped_controllers/mpc_controller.hpp"
#include "quadruped_controllers/mpc_solver.hpp"
#include "quadruped_controllers/ros2_node_interface.hpp"
#include <atomic>
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
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  void setGait(const std::string name,
               std::shared_ptr<OffsetDurationGait<double>> gait);

protected:
  std::unordered_map<std::string, std::shared_ptr<OffsetDurationGait<double>>>
      gaits_;
  std::shared_ptr<OffsetDurationGait<double>> current_gait_;

private:
  inline Eigen::VectorXd &calcTrajFromCommand(Eigen::VectorXd &traj);
  inline Eigen::Vector3d &calcMitCheetahFootPos(size_t leg,
                                                Eigen::Vector3d &footpos);
  std::atomic<bool> need_update_param_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      locomotion_param_callback_handle_;
  bool first_swing_[4];
  rclcpp::Time start_swing_time_[4];
  std::shared_ptr<P3dPublisher> p3d_pub_;
};
} // namespace quadruped_controllers