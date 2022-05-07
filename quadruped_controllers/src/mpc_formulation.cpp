// copied form qiayuan
#include "quadruped_controllers/mpc_formulation.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "quadruped_controllers/quadruped_types.hpp"
#include "quadruped_controllers/utils.hpp"

#include <cstddef>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "rcutils/logging_macros.h"

namespace quadruped_controllers {
void MpcFormulation::setup(int horizon,
                           const Matrix<double, STATE_DIM, 1> &weight,
                           double alpha, double final_cost_scale) {
  horizon_ = horizon;
  final_cost_scale_ = final_cost_scale;
  // Resize
  a_c_.resize(STATE_DIM, STATE_DIM);
  b_c_.resize(STATE_DIM, ACTION_DIM);
  a_qp_.resize(STATE_DIM * horizon, Eigen::NoChange);
  b_qp_.resize(STATE_DIM * horizon, ACTION_DIM * horizon);
  l_.resize(STATE_DIM * horizon);
  alpha_.resize(12 * horizon_, 12 * horizon_);
  h_.resize(ACTION_DIM * horizon, ACTION_DIM * horizon);
  g_.resize(ACTION_DIM * horizon, Eigen::NoChange);
  a_.resize(5 * 4 * horizon, ACTION_DIM * horizon);
  ub_a_.resize(5 * 4 * horizon, Eigen::NoChange);
  lb_a_.resize(5 * 4 * horizon, Eigen::NoChange);
  // Set Zero
  a_c_.setZero();
  b_c_.setZero();
  a_qp_.setZero();
  b_qp_.setZero();
  l_.setZero();
  l_.diagonal() = weight.replicate(horizon, 1);
  l_.diagonal().block((horizon - 1) * STATE_DIM, 0, STATE_DIM, 1) *=
      final_cost_scale;
  alpha_.setIdentity();
  alpha_ = alpha * alpha_;
}

// Converts a vector to the skew symmetric matrix form. For an input vector
// [a, b, c], the output matrix would be:
//   [ 0, -c,  b]
//   [ c,  0, -a]
//   [-b,  a,  0]
Matrix3d convertToSkewSymmetric(const Vector3d &vec) {
  Matrix3d skew_sym_mat;
  skew_sym_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_sym_mat;
}

void MpcFormulation::buildStateSpace(double mass, const Matrix3d &inertia,
                                     const QuadrupedState &state) {
  Matrix3d angular_velocity_to_rpy_rate;
  Vector3d rpy = quatToRPY(state.quat_);
  double yaw_cos = std::cos(rpy(2));
  double yaw_sin = std::sin(rpy(2));
  angular_velocity_to_rpy_rate << yaw_cos, yaw_sin, 0, -yaw_sin, yaw_cos, 0, 0,
      0, 1;

  Matrix<double, 3, 4> r_feet;
  for (int i = 0; i < 4; ++i)
    r_feet.col(i) = state.foot_pos_[i] - state.pos_;

  a_c_.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;

  a_c_(3, 9) = 1.;
  a_c_(4, 10) = 1.;
  a_c_(5, 11) = 1.;
  a_c_(11, 12) = 1.;

  Matrix3d inertia_world = angular_velocity_to_rpy_rate.transpose() * inertia *
                           angular_velocity_to_rpy_rate;
  //  b contains non_zero elements only in row 6 : 12.
  for (int i = 0; i < 4; ++i) {
    // inv can be place outside
    b_c_.block<3, 3>(6, i * 3) =
        inertia_world.inverse() * convertToSkewSymmetric(r_feet.col(i));
    b_c_.block<3, 3>(9, i * 3) = Matrix<double, 3, 3>::Identity() / mass;
  }
  // std::cout<<"a_c_: \n"<<a_c_<<std::endl;
  // std::cout<<"b_c_: \n"<<b_c_<<std::endl;
}

void MpcFormulation::buildQp(double dt) {
  // Convert model from continuous to discrete time
  Matrix<double, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> ab_c;
  ab_c.setZero();
  ab_c.block(0, 0, STATE_DIM, STATE_DIM) = a_c_;
  ab_c.block(0, STATE_DIM, STATE_DIM, ACTION_DIM) = b_c_;
  ab_c = dt * ab_c;
  Matrix<double, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> exp =
      ab_c.exp();
  Matrix<double, STATE_DIM, STATE_DIM> a_dt =
      exp.block(0, 0, STATE_DIM, STATE_DIM);
  Matrix<double, STATE_DIM, ACTION_DIM> b_dt =
      exp.block(0, STATE_DIM, STATE_DIM, ACTION_DIM);

  std::vector<Matrix<double, STATE_DIM, STATE_DIM>> power_mats;
  power_mats.resize(horizon_ + 1);
  for (auto &power_mat : power_mats)
    power_mat.setZero();
  power_mats[0].setIdentity();
  for (int i = 1; i < horizon_ + 1; i++)
    power_mats[i] = a_dt * power_mats[i - 1];

  for (int r = 0; r < horizon_; r++) {
    a_qp_.block(STATE_DIM * r, 0, STATE_DIM, STATE_DIM) =
        power_mats[r + 1]; // Adt.pow(r+1);

    for (int c = 0; c < horizon_; c++) {
      if (r >= c) {
        int a_num = r - c;
        b_qp_.block(STATE_DIM * r, ACTION_DIM * c, STATE_DIM, ACTION_DIM) =
            power_mats[a_num] * b_dt;
      }
    }
  }
  // std::cout<<"a_qp_: \n"<<a_qp_<<std::endl;
  // std::cout<<"b_qp_: \n"<<b_qp_<<std::endl;
}

const Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> &
MpcFormulation::buildHessianMat() {
  h_ = 2. * (b_qp_.transpose() * l_ * b_qp_ + alpha_);
  // std::cout<<"h_: \n"<<h_<<std::endl;
  return h_;
}

const VectorXd &
MpcFormulation::buildGVec(double gravity, const QuadrupedState &state,
                          const Matrix<double, Dynamic, 1> &traj) {
  // Update x_0 and x_ref
  Matrix<double, STATE_DIM, 1> x_0;
  VectorXd x_ref(STATE_DIM * horizon_);

  Vector3d rpy = quatToRPY(state.quat_);
  x_0 << rpy(0), rpy(1), rpy(2), state.pos_, state.angular_vel_,
      state.linear_vel_, gravity;
  for (int i = 0; i < horizon_; i++)
    for (int j = 0; j < STATE_DIM - 1; j++)
      x_ref(STATE_DIM * i + j, 0) = traj[12 * i + j];
  g_ = 2. *  b_qp_.transpose() * l_ * (a_qp_ * x_0 - x_ref);
  // std::cout<<"g_: \n"<<g_<<std::endl;
  return g_;
}

const Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> &
MpcFormulation::buildConstrainMat(double mu) {
  a_.setZero();
  double mu_inv = 1.f / mu;
  Matrix<double, 5, 3> a_block;
  a_block << mu_inv, 0, 1., -mu_inv, 0, 1., 0, mu_inv, 1., 0, -mu_inv, 1., 0, 0,
      1.;
  for (int i = 0; i < horizon_ * 4; i++)
    a_.block(i * 5, i * 3, 5, 3) = a_block;
  // std::cout<<"a_: \n"<<a_<<std::endl;
  return a_;
}

const VectorXd &
MpcFormulation::buildConstrainUpperBound(double f_max,
                                         const VectorXd &gait_table) {
  for (int i = 0; i < horizon_; ++i) {
    for (int j = 0; j < 4; ++j) {
      const int row = (i * 4 + j) * 5;
      ub_a_(row) = BIG_VALUE;
      ub_a_(row + 1) = BIG_VALUE;
      ub_a_(row + 2) = BIG_VALUE;
      ub_a_(row + 3) = BIG_VALUE;
      ub_a_(row + 4) = f_max * gait_table(i * 4 + j);
    }
  }
  // std::cout<<"ub_a_: \n"<<ub_a_<<std::endl;
  return ub_a_;
}

const VectorXd &MpcFormulation::buildConstrainLowerBound() {
  lb_a_.setZero();
  // std::cout<<"lb_a_: \n"<<lb_a_<<std::endl;
  return lb_a_;
}

void MpcFormulation::reduceQpProblem(){
  size_t num_var =12*horizon_;
  size_t num_constraints = 20 * horizon_;
  size_t num_var_reduced=num_var;
  size_t num_contraints_reduced=num_constraints;
  std::vector<bool> var_need_reduced(num_var,false);
  std::vector<bool> constraint_need_reduced(num_constraints,false);
  auto near_zero = [](double x) { return std::abs(x) < 1e-2; };
  auto near_one = [](double x) { return std::abs(x - 1) < 1e-2; };
  for (size_t i=0;i<num_constraints;i++){
    if (!(near_zero(lb_a_[i]) && near_zero(ub_a_[i]))){
      continue;
    }
    for (size_t j=0;j<num_var;j++){
      if(near_one(a_(i,j))){
        num_var_reduced-=3;
        num_contraints_reduced-=5;
        var_need_reduced[j-2]=true;
        var_need_reduced[j-1]=true;
        var_need_reduced[j]=true;
        size_t constraint_index=(j*5)/3-3;
        constraint_need_reduced[constraint_index]=true;
        constraint_need_reduced[constraint_index+1]=true;
        constraint_need_reduced[constraint_index+2]=true;
        constraint_need_reduced[constraint_index+3]=true;
        constraint_need_reduced[constraint_index+4]=true;
      }
    }
  }

  g_reduced_.resize(num_var_reduced);
  h_reduced_.resize(num_var_reduced,num_var_reduced);
  a_reduced_.resize(num_contraints_reduced,num_var_reduced);
  lb_a_reduced_.resize(num_contraints_reduced);
  ub_a_reduced_.resize(num_contraints_reduced);
  g_reduced_.setZero();
  h_reduced_.setZero();
  a_reduced_.setZero();
  lb_a_reduced_.setZero();
  ub_a_reduced_.setZero();
  var_index_.resize(num_var_reduced);
  std::vector<size_t> constraint_index(num_contraints_reduced);
  size_t index_count=0;
  for (size_t i=0;i<num_var;i++){
    if(!var_need_reduced[i]){
      if(index_count>=num_var_reduced){
        RCUTILS_LOG_WARN_NAMED(
            "mpc_formulation",
            "too many var reduced");
      }
      var_index_[index_count]=i;
      index_count++;
    }
  }
  index_count=0;
  for (size_t i=0;i<num_constraints;i++){
    if(!constraint_need_reduced[i]){
      if(index_count>=num_contraints_reduced){
        RCUTILS_LOG_WARN_NAMED(
            "mpc_formulation",
            "too many constraint reduced");
      }
      constraint_index[index_count]=i;
      index_count++;
    }
  }
  for (size_t i=0;i<num_var_reduced;i++){
    g_reduced_[i]=g_[var_index_[i]];
    for (size_t j=0;j<num_var_reduced;j++){
      h_reduced_(i,j)=h_(var_index_[i],var_index_[j]);
    }
  }
  for (size_t con=0;con<num_contraints_reduced;con++){
    lb_a_reduced_[con]=lb_a_[constraint_index[con]];
    ub_a_reduced_[con]=ub_a_[constraint_index[con]];
    for (size_t var=0;var<num_var_reduced;var++){
      a_reduced_(con,var)=a_(constraint_index[con],var_index_[var]);
    }
  }
  num_var_reduced_=num_var_reduced;
  num_constrain_reduced_=num_contraints_reduced;
}

void MpcFormulation::recoverQpSolution(std::vector<double> &solution){
  std::vector<double> solution_recoverd(12*horizon_,0);
  for (size_t i=0;i<num_var_reduced_;i++){
    solution_recoverd[var_index_[i]]=solution[i];
  }
  solution.swap(solution_recoverd);
}

} // namespace quadruped_controllers
