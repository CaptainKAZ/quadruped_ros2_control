#include "quadruped_controllers/mpc_solver.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include <memory>
#include <unistd.h>

using namespace std;
using namespace chrono;

using namespace quadruped_controllers;
using namespace Eigen;

int main()
{
  int horizon = 20;

  double mass = 11.041;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  std::shared_ptr<MpcSolverBase> mpc_solver = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);
  Matrix<double, 13, 1> weight;
  weight << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0.;

  mpc_solver->setup(0.01, horizon, 100., weight, 1e-6,1.0);

  // State space
  QuadrupedState state;
  state.pos_ << 0, 0, 0.25;
  state.quat_.setIdentity();
  state.foot_pos_[0] << 0.25, 0.2, 0;
  state.foot_pos_[1] << 0.25, -0.2, 0;
  state.foot_pos_[2] << -0.25, 0.2, 0;
  state.foot_pos_[3] << -0.25, -0.2, 0;

  Eigen::VectorXd gait_table;
  gait_table.resize(horizon * 4);
  for (int i = 0; i < horizon /2; i++){
    gait_table(i*4) = 1.0;
    gait_table(i*4+1) = 0.0;
    gait_table(i*4+2) = 0.0;
    gait_table(i*4+3) = 1.0;
  }
  for (int i = horizon /2; i < horizon; i++){
    gait_table(i*4) = 0.0;
    gait_table(i*4+1) = 1.0;
    gait_table(i*4+2) = 1.0;
    gait_table(i*4+3) = 0.0;
  }
  //gait_table.setOnes();
  Eigen::VectorXd traj;
  traj.resize(12 * horizon);
  traj.setZero();
  for (int i = 0; i < horizon; ++i)
  {
    traj[12 * i + 3] = 0.2;
    traj[12 * i + 5] = 0.2;
  }
  std::shared_ptr<QuadrupedState> state_ptr = std::make_shared<QuadrupedState>(state);
  mpc_solver->solve(rclcpp::Time(1e9), state_ptr, gait_table, traj);
  sleep(1);
  for (const auto& force : mpc_solver->getSolution())
    std::cout << force << "\n" << std::endl;
  return 0;
}