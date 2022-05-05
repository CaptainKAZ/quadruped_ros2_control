#include <iostream>
#include "quadruped_controllers/state_update.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
int main(){
  Eigen::Matrix<double,13,13> A;
  A.setZero();
  A(3,9) = 1.f;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A=A.exp();
  std::cout<<A<<std::endl;
}