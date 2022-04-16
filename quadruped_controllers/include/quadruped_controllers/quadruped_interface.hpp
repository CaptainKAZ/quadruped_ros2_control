#pragma once

#include "quadruped_controllers/quadruped_types.hpp"
#include "rcpputils/asserts.hpp"
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <semantic_components/imu_sensor.hpp>
#include <utility>
#include <vector>

// only for debug
#include "rcutils/logging_macros.h"

namespace quadruped_controllers {

class QuadrupedInterfaceBase {
public:
  virtual ~QuadrupedInterfaceBase() = default;
  inline virtual void assignLoanedInterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
      std::vector<hardware_interface::LoanedCommandInterface>
          &command_interfaces) = 0;
  virtual void releaseInterfaces() = 0;
};

class CheetahJointInterface : public QuadrupedInterfaceBase {
public:
  inline explicit CheetahJointInterface(const std::string &name) {
    cmd_interface_names_.reserve(5);
    state_interface_names_.reserve(3);
    cmd_interface_names_.emplace_back(name + "/hybrid_position");
    cmd_interface_names_.emplace_back(name + "/hybrid_velocity");
    cmd_interface_names_.emplace_back(name + "/hybrid_effort");
    cmd_interface_names_.emplace_back(name + "/hybrid_kp");
    cmd_interface_names_.emplace_back(name + "/hybrid_kd");
    state_interface_names_.emplace_back(name + "/position");
    state_interface_names_.emplace_back(name + "/velocity");
    state_interface_names_.emplace_back(name + "/effort");
    state_interfaces_.reserve(3);
    command_interfaces_.reserve(5);
  }
  ~CheetahJointInterface() = default;
  inline void assignLoanedInterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
      std::vector<hardware_interface::LoanedCommandInterface>
          &command_interfaces) override {
    controller_interface::get_ordered_interfaces(
        std::forward<decltype(state_interfaces)>(state_interfaces),
        state_interface_names_, "", state_interfaces_);
    controller_interface::get_ordered_interfaces(
        std::forward<decltype(command_interfaces)>(command_interfaces),
        cmd_interface_names_, "", command_interfaces_);
    if (state_interfaces_.size() != 3 || command_interfaces_.size() != 5) {
      throw std::runtime_error("Failed to get interfaces");
    }
  }
  inline void releaseInterfaces() override {
    state_interfaces_.clear();
    command_interfaces_.clear();
  }
  inline void SetCommand(double hybrid_position, double hybrid_velocity,
                         double hybrid_effort, double kp, double kd) {
    rcpputils::check_true(command_interfaces_.size() == 5,
                          "Failed to get all interfaces, expected 5");
    command_interfaces_[0].get().set_value(hybrid_position);
    command_interfaces_[1].get().set_value(hybrid_velocity);
    command_interfaces_[2].get().set_value(hybrid_effort);
    command_interfaces_[3].get().set_value(kp);
    command_interfaces_[4].get().set_value(kd);
  }
  inline double getPosition() {
    rcpputils::check_true(state_interfaces_.size() == 3,
                          "Failed to get all interfaces, expected 3");
    return state_interfaces_[0].get().get_value();
  }
  inline double getVelocity() {
    rcpputils::check_true(state_interfaces_.size() == 3,
                          "Failed to get all interfaces, expected 3");
    return state_interfaces_[1].get().get_value();
  }
  inline double getEffort() {
    rcpputils::check_true(state_interfaces_.size() == 3,
                          "Failed to get all interfaces, expected 3");
    return state_interfaces_[2].get().get_value();
  }

private:
  std::vector<std::string> cmd_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      state_interfaces_;
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      command_interfaces_;
};

class LegInterface : public QuadrupedInterfaceBase {
public:
  inline explicit LegInterface(const std::string &name) {
    name_ = name;
    joint_interfaces_[0] =
        std::make_shared<CheetahJointInterface>(name + "_hip_joint");
    joint_interfaces_[1] =
        std::make_shared<CheetahJointInterface>(name + "_thigh_joint");
    joint_interfaces_[2] =
        std::make_shared<CheetahJointInterface>(name + "_calf_joint");
  }
  inline void assignLoanedInterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
      std::vector<hardware_interface::LoanedCommandInterface>
          &command_interfaces) override {
    for (auto &joint_interface : joint_interfaces_) {
      joint_interface->assignLoanedInterfaces(
          std::forward<decltype(state_interfaces)>(state_interfaces),
          std::forward<decltype(command_interfaces)>(command_interfaces));
    }
    for (auto &state_interface : state_interfaces) {
      // RCUTILS_LOG_DEBUG_NAMED(
      //     "quadruped_interface",
      //     "Attempting to get contact state interface desired name: %s, now
      //     matching %s",(name_ + "_contact_sensor/contact").c_str(),
      //     state_interface.get_full_name().c_str());
      if (state_interface.get_full_name() ==
          name_ + "_contact_sensor/contact") {
        contact_interface_ = std::make_shared<
            std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
            std::ref(state_interface));
        break;
      }
    }
    rcpputils::check_true(contact_interface_ != nullptr,
                          "Failed to get contact interface");
    RCUTILS_LOG_INFO_NAMED("quadruped_interface", "Got contact interface");
  }

  inline void releaseInterfaces() override {
    for (auto &joint_interface : joint_interfaces_) {
      joint_interface->releaseInterfaces();
    }
    contact_interface_ = nullptr;
  }

  inline std::shared_ptr<CheetahJointInterface> getHip() {
    rcpputils::check_true(joint_interfaces_[0] != nullptr,
                          "Hip joint interface is null");
    return joint_interfaces_[0];
  }
  inline std::shared_ptr<CheetahJointInterface> getThigh() {
    rcpputils::check_true(joint_interfaces_[1] != nullptr,
                          "Thigh joint interface is null");
    return joint_interfaces_[1];
  }
  inline std::shared_ptr<CheetahJointInterface> getCalf() {
    rcpputils::check_true(joint_interfaces_[2] != nullptr,
                          "Calf joint interface is null");
    return joint_interfaces_[2];
  }
  inline bool getContact() {
    rcpputils::check_true(contact_interface_ != nullptr,
                          "Failed to get contact interface");
    return (contact_interface_->get().get_value() == 1);
  }

  inline std::shared_ptr<CheetahJointInterface>
  getJoint(const unsigned int joint_id) {
    rcpputils::check_true(joint_id < 3, "Invalid joint id");
    return joint_interfaces_[joint_id];
  }

  inline std::shared_ptr<CheetahJointInterface>
  operator[](std::string joint_name) {
    if (joint_name == "hip") {
      return getHip();
    } else if (joint_name == "thigh") {
      return getThigh();
    } else if (joint_name == "calf") {
      return getCalf();
    } else {
      throw std::out_of_range("Unknown joint name");
    }
  }

private:
  std::string name_;
  std::shared_ptr<CheetahJointInterface> joint_interfaces_[3]{nullptr};
  std::shared_ptr<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      contact_interface_{nullptr};
};

class QuadrupedInterface : public QuadrupedInterfaceBase {
public:
  inline explicit QuadrupedInterface() {
    leg_interfaces_[0] = std::make_shared<LegInterface>(LEG_CONFIG[0]);
    leg_interfaces_[1] = std::make_shared<LegInterface>(LEG_CONFIG[1]);
    leg_interfaces_[2] = std::make_shared<LegInterface>(LEG_CONFIG[2]);
    leg_interfaces_[3] = std::make_shared<LegInterface>(LEG_CONFIG[3]);
    imu_sensor_ = std::make_shared<semantic_components::IMUSensor>("base_imu");
  }
  inline void assignLoanedInterfaces(
      std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
      std::vector<hardware_interface::LoanedCommandInterface>
          &command_interfaces) override {
    for (auto &leg_interface : leg_interfaces_) {
      leg_interface->assignLoanedInterfaces(
          std::forward<decltype(state_interfaces)>(state_interfaces),
          std::forward<decltype(command_interfaces)>(command_interfaces));
    }
    imu_sensor_->assign_loaned_state_interfaces(
        std::forward<decltype(state_interfaces)>(state_interfaces));
  }
  inline void releaseInterfaces() override {
    for (auto &leg_interface : leg_interfaces_) {
      leg_interface->releaseInterfaces();
    }
    imu_sensor_->release_interfaces();
  }
  inline std::shared_ptr<LegInterface> getLeg(const unsigned int leg_id) {
    rcpputils::check_true(leg_id < 4, "Leg id must be between 0 and 3");
    rcpputils::check_true(leg_interfaces_[leg_id] != nullptr,
                          "Leg interface is not assigned");
    return leg_interfaces_[leg_id];
  }
  std::shared_ptr<semantic_components::IMUSensor> getIMUSensor() {
    return imu_sensor_;
  }

protected:
  std::shared_ptr<LegInterface> leg_interfaces_[4]{nullptr};
  std::shared_ptr<semantic_components::IMUSensor> imu_sensor_;
};
} // namespace quadruped_controllers