// copied form qiayuan

#pragma once
#include "qpOASES.hpp"
#include "quadruped_controllers/mpc_formulation.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/utils.hpp"
#include "rcutils/logging_macros.h"
#include <cstdio>
#include <iostream>
#include <mutex>
#include <rclcpp/time.hpp>
#include <thread>

namespace quadruped_controllers {
class MpcSolverBase {
public:
  virtual ~MpcSolverBase(){};
  MpcSolverBase(double mass, double gravity, double mu, const Matrix3d &inertia)
      : mass_(mass), gravity_(gravity), mu_(mu), inertia_(inertia) {
    solution_.resize(4);
    for (auto &solution : solution_)
      solution.setZero();
  }

  void setup(double dt, int horizon, double f_max,
             const Matrix<double, 13, 1> &weight, double alpha,
             double final_cost_scale) {
    dt_ = dt;
    f_max_ = f_max;
    weight_ = weight;
    alpha_ = alpha;
    horizon_ = horizon;
    final_cost_scale_ = final_cost_scale;
    mpc_formulation_.setup(horizon, weight, alpha, final_cost_scale);
  }

  void setHorizon(int horizon, double dt, double final_cost_scale) {
    if (horizon_ != horizon || dt_ != dt ||
        final_cost_scale_ != final_cost_scale)
      horizon_changed_ = true;
    horizon_ = horizon;
    dt_ = dt;
    final_cost_scale_ = final_cost_scale;
  }

  void solve(rclcpp::Time time, const std::shared_ptr<QuadrupedState> &state,
             const VectorXd &gait_table,
             const Matrix<double, Dynamic, 1> &traj) {
    double dt ;
    try{
    dt= (time - last_update_).seconds();
    }catch (std::exception &e){
      std::cout << "exception caught: " << e.what() << std::endl;
      dt=-1.0;
    }
    if (dt < 0) // Simulation reset
      last_update_ = time;
    if (dt > dt_) {
      std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
      if (guard.owns_lock()) {
        if (horizon_changed_) {
          horizon_changed_ = false;
          setup(dt_, horizon_, f_max_, weight_, alpha_, final_cost_scale_);
          //          ROS_INFO_STREAM("horizon: " << horizon_ << " dt: " <<
          //          dt_);
        }
        last_update_ = time;
        state_ = *state.get();
        gait_table_ = gait_table;
        traj_ = traj;
        //TODO: use thread pooling
        thread_ = std::make_shared<std::thread>(
            std::thread(&MpcSolverBase::solvingThread, this));
        thread_->detach();
      } else
        RCUTILS_LOG_WARN_NAMED("MPC Solver", "Solve timeout.");
    }
  }

  const std::vector<Vec3<double>> &getSolution() { return solution_; }

  int getHorizon() { return horizon_; }

  double getDt() { return dt_; };

protected:
  void solvingThread() {
    std::lock_guard<std::mutex> guard(mutex_);
    formulate();
    solving();
  };

  virtual void solving() = 0;

  MpcFormulation mpc_formulation_;
  std::vector<Vec3<double>> solution_;

  std::mutex mutex_;
  std::shared_ptr<std::thread> thread_;
  int horizon_;
  double final_cost_scale_;

private:
  void formulate() {
    //Timer timer;
    mpc_formulation_.buildStateSpace(mass_, inertia_, state_);
    // std::cout<<"cost "<<timer.toc()*1000<<"ms to build state space"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildQp(dt_);
    //std::cout<<"cost "<<timer.toc()*1000<<"ms to build Qp"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildHessianMat();
    // std::cout<<"cost "<<timer.toc()*1000<<"ms to build HessianMat"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildGVec(gravity_, state_, traj_);
    // std::cout<<"cost "<<timer.toc()*1000<<"ms to build GVec"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildConstrainMat(mu_);
    // std::cout<<"cost "<<timer.toc()*1000<<"ms to build ConstrainMat"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildConstrainUpperBound(f_max_, gait_table_);
    // std::cout<<"cost "<<timer.toc()*1000<<"ms to build ConstrainUpperBound"<<std::endl;
    // timer.tic();
    mpc_formulation_.buildConstrainLowerBound();
    //std::cout<<"cost "<<timer.toc()*1000<<"ms to build lower bound"<<std::endl;
  }

  rclcpp::Time last_update_;

  double dt_, mass_, gravity_, mu_, f_max_;
  Matrix3d inertia_;
  QuadrupedState state_;
  Matrix<double, Dynamic, 1> traj_;
  VectorXd gait_table_;

  // Only for setHorizon()
  Matrix<double, 13, 1> weight_;
  double alpha_;

  bool horizon_changed_;
};

class QpOasesSolver : public MpcSolverBase {
public:
  using MpcSolverBase::MpcSolverBase;

protected:
  void solving() override {
    //Timer timer;
    auto qp_problem = qpOASES::QProblem(
        12 * mpc_formulation_.horizon_,
        20 * mpc_formulation_.horizon_); // TODO: Test SQProblem
    qpOASES::Options options;
    options.setToMPC();
    //    options.enableEqualities = qpOASES::BT_TRUE;
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);
    int n_wsr = 200;
    qpOASES::returnValue rvalue = qp_problem.init(
        mpc_formulation_.h_.data(), mpc_formulation_.g_.data(),
        mpc_formulation_.a_.data(), nullptr, nullptr,
        mpc_formulation_.lb_a_.data(), mpc_formulation_.ub_a_.data(), n_wsr);
    printFailedInit(rvalue);

    if (rvalue != qpOASES::SUCCESSFUL_RETURN) {
      for (auto &solution : solution_)
        solution.setZero();
      return;
    }

    std::vector<qpOASES::real_t> qp_sol(12 * mpc_formulation_.horizon_, 0);

    if (qp_problem.getPrimalSolution(qp_sol.data()) !=
        qpOASES::SUCCESSFUL_RETURN)
      RCUTILS_LOG_WARN_NAMED("MPC Solver", "Failed to solve mpc!\n");

    for (int leg = 0; leg < 4; ++leg) {
      solution_[leg].x() = qp_sol[3 * leg];
      solution_[leg].y() = qp_sol[3 * leg + 1];
      solution_[leg].z() = qp_sol[3 * leg + 2];
      if (solution_[leg].norm() > 1e3) {
        RCUTILS_LOG_ERROR_NAMED("MPC Solver", "MPC solve failed, result:");
        std::cerr << solution_[leg];
      }
    }
    //std::cout<<"cost "<<timer.toc()*1000<<"ms to solve"<<std::endl;
  }

  void printFailedInit(qpOASES::returnValue rvalue) {
    switch (rvalue) {
    case qpOASES::RET_INIT_FAILED:
      RCUTILS_LOG_WARN_NAMED("MPC Solver", "MPC init failed");
      break;
    case qpOASES::RET_INIT_FAILED_CHOLESKY:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed with: RET_INIT_FAILED_CHOLESKY");
      break;
    case qpOASES::RET_INIT_FAILED_TQ:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed with: RET_INIT_FAILED_CHOLESKY");
      break;
    case qpOASES::RET_INIT_FAILED_HOTSTART:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed: RET_INIT_FAILED_HOTSTART");
      break;
    case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed: RET_INIT_FAILED_INFEASIBILITY");
      break;
    case qpOASES::RET_INIT_FAILED_UNBOUNDEDNESS:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed: RET_INIT_FAILED_UNBOUNDEDNESS");
      break;
    case qpOASES::RET_MAX_NWSR_REACHED:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed: RET_MAX_NWSR_REACHED");
      break;
    case qpOASES::RET_INVALID_ARGUMENTS:
      RCUTILS_LOG_WARN_NAMED("MPC Solver",
                             "MPC init failed: RET_INVALID_ARGUMENTS");
      break;
    default:
      break;
    }
  }
};

} // namespace quadruped_controllers
