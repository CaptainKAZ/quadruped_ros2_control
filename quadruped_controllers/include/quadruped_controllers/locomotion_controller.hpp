#pragma once

#include "quadruped_controllers/gait.hpp"
#include "quadruped_controllers/mpc_controller.hpp"
#include "quadruped_controllers/mpc_solver.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
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

  std::unordered_map<std::string, std::shared_ptr<MpcGait>>
      gaits_;
  std::shared_ptr<MpcGait> current_gait_;
private:
  inline Eigen::VectorXd &calcTrajFromCommand(Eigen::VectorXd &traj);
  std::atomic<bool> need_update_param_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      locomotion_param_callback_handle_;
  bool first_swing_[4];
  rclcpp::Time start_swing_time_[4];
  std::shared_ptr<P3dPublisher> p3d_pub_;
  std::shared_ptr<PinocchioSolver> des_kine_solver_;
};
} // namespace quadruped_controllers