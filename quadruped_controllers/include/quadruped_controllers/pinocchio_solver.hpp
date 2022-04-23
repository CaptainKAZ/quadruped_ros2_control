#pragma once

// Pinocchio warpper
#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <quadruped_controllers/quadruped_types.hpp>
#include <tuple>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>


namespace quadruped_controllers {
class PinocchioSolver{
  public:
    explicit PinocchioSolver(std::shared_ptr<urdf::ModelInterface>& urdf){
      parseUrdf(std::forward<decltype(urdf)>(urdf));
    };
    ~PinocchioSolver()=default;
    void parseUrdf(std::shared_ptr<urdf::ModelInterface>& urdf);
    void calcForwardKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    void getFootPosVel(std::shared_ptr<QuadrupedState>& state);
    void getJacobian(const std::string& leg,Eigen::Matrix<double, 6, 18>& jacobian);
    size_t getNq() const;
    size_t getNv() const;
    const Eigen::Vector3d& getHipLocationRef(size_t leg);
    const Eigen::Vector3d& getHipLocationWorld(size_t leg);
  protected:
    std::shared_ptr<pinocchio::Model> pin_model_;
    std::shared_ptr<pinocchio::Data> pin_data_;
};
}// namespace quadruped_controllers