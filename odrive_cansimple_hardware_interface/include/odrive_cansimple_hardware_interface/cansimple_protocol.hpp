#pragma once
#include <linux/can.h>
#include <sys/types.h>
#include "odrive_cansimple_hardware_interface/socketcan.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include "rcutils/logging_macros.h"

namespace odrive_cansimple_hardware_interface
{
class CanSimpleAxis
{
public:
  enum OdriveCommand
  {
    MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR,  // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_TORQUE,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_INERTIA,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_SET_GAINS,
    MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
  };
  void configure(double gear, double kt)
  {
    reduction_ratio_ = gear;
    motor_kt_ = kt;
  }
  void setHome(double home_ref)
  {
    joint_offset_ = home_ref - joint_state_pos_;
  }
  void attach(std::shared_ptr<SocketCan> can_device, int can_id)
  {
    can_device_ = can_device;
    can_id_ = can_id;
    can_device_->bindRxCallback(std::bind(&CanSimpleAxis::canRxCallback, this, std::placeholders::_1));
  }
  void eStop()
  {
    can_frame frame{};
    frame.can_id = (can_id_ & 0x3F) << 5 | MSG_ODRIVE_ESTOP;
    frame.can_dlc = 0;
    *can_device_ << frame;
  }
  void clearErrors()
  {
    can_frame frame{};
    frame.can_id = (can_id_ & 0x3F) << 5 | MSG_CLEAR_ERRORS;
    frame.can_dlc = 0;
    *can_device_ << frame;
  }
  // https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
  void setAxisRequestedState(uint16_t state)
  {
    can_frame frame{};
    frame.can_id = (can_id_ & 0x3F) << 5 | MSG_SET_AXIS_REQUESTED_STATE;
    frame.can_dlc = 2;
    std::memcpy(frame.data, &state, 2);
    *can_device_ << frame;
  }
  // https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
  void setControllerMode(uint32_t control_mode,uint32_t input_mode)
  {
    can_frame frame{};
    frame.can_id = (can_id_ & 0x3F) << 5 | MSG_SET_CONTROLLER_MODES;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &control_mode, 4);
    std::memcpy(frame.data+4, &input_mode, 4);
    *can_device_ << frame;
  }

  void setInputPos(double pos, double vel,double tau_ff){
    can_frame frame{};
    float actuator_command_pos = (pos - joint_offset_) * reduction_ratio_ * RAD_TO_TURN;
    float actuator_command_vel = vel * reduction_ratio_ * RAD_TO_TURN;
    float actuator_command_eff = tau_ff / reduction_ratio_;
    // use Set Input Pos command
    frame.can_id = (can_id_ & 0x3F) << 5 | MSG_SET_INPUT_POS;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &actuator_command_pos, sizeof(float));
    int16_t temp = (int16_t)(actuator_command_vel * 1000);
    std::memcpy(frame.data + 4, &temp, sizeof(int16_t));
    temp = (int16_t)(actuator_command_eff * 1000);
    std::memcpy(frame.data + 6, &temp, sizeof(int16_t));
    *can_device_ << frame;
  }

  void setGain(double kp,double kd){
    can_frame frame{};
    frame.can_id=(can_id_ & 0x3F) << 5 | MSG_SET_GAINS;
    float fp32kp=kp;
    float fp32kd=kd;
    std::memcpy(frame.data, &fp32kp,sizeof(float));
    std::memcpy(frame.data+4, &fp32kd,sizeof(float));
    *can_device_ << frame;
  }

  void writeCommand()
  {
    setInputPos(joint_command_pos_,joint_command_vel_,joint_command_eff_);
    if (last_kp != joint_command_kp_)
    {
      last_kp = joint_command_kp_;
      can_frame kp_frame{};
      kp_frame.can_id = (can_id_ & 0x3F) << 5 | 0x01A;
      double actuator_command_kp = joint_command_kp_ / reduction_ratio_;
      kp_frame.can_dlc = 4;
      std::memcpy(kp_frame.data, &actuator_command_kp, sizeof(float));
      *can_device_ << kp_frame;
    }
    if (last_kd != joint_command_kd_)
    {
      last_kd = joint_command_kd_;
      can_frame kd_frame{};
      kd_frame.can_id = (can_id_ & 0x3F) << 5 | 0x01B;
      double actuator_command_kd = joint_command_kd_ / reduction_ratio_;
      kd_frame.can_dlc = 8;
      std::memcpy(kd_frame.data, &actuator_command_kd, sizeof(float));
      std::memset(kd_frame.data + 4, 0, 4);
      *can_device_ << kd_frame;
    }
    if (joint_command_calibration_pos_ != std::numeric_limits<double>::quiet_NaN())
    {
      setHome(joint_command_calibration_pos_);
      joint_command_calibration_pos_ = std::numeric_limits<double>::quiet_NaN();
    }
    if (joint_command_request_state_ != std::numeric_limits<uint32_t>::quiet_NaN())
    {
      setAxisRequestedState(joint_command_request_state_);
      joint_command_request_state_ = std::numeric_limits<uint32_t>::quiet_NaN();
    }
    if (joint_command_clear_error_ != std::numeric_limits<uint32_t>::quiet_NaN())
    {
      clearErrors();
      joint_command_clear_error_ = std::numeric_limits<uint32_t>::quiet_NaN();
    }
  }
  inline double* getCommandPosPtr()
  {
    return &joint_command_pos_;
  }
  inline double* getCommandVelPtr()
  {
    return &joint_command_vel_;
  }
  inline double* getCommandEffPtr()
  {
    return &joint_command_eff_;
  }
  inline double* getCommandKpPtr()
  {
    return &joint_command_kp_;
  }
  inline double* getCommandKdPtr()
  {
    return &joint_command_kd_;
  }
  inline double* getStatePosPtr()
  {
    return &joint_state_pos_;
  }
  inline double* getStateVelPtr()
  {
    return &joint_state_vel_;
  }
  inline double* getStateEffPtr()
  {
    return &joint_state_eff_;
  }
  inline double* getCalibrationPosPtr()
  {
    return &joint_command_calibration_pos_;
  }
  inline double* getRequestStatePtr()
  {
    return &joint_command_request_state_;
  }
  inline double* getClearErrorPtr()
  {
    return &joint_command_clear_error_;
  }
  static constexpr double TURN_TO_RAD = 2 * M_PI;
  static constexpr double RAD_TO_TURN = 1 / TURN_TO_RAD;

private:
  static bool canRxCallback(CanSimpleAxis* self, struct can_frame& frame)
  {
    // printf("callback!!\n");
    if (frame.can_id >> 5 != self->can_id_)
    {
      // printf("%x self:%x\n",frame.can_id>>5,self->can_id_);
      return false;
    }
    switch (frame.can_id & 0x1F)
    {
      float actuator_state_pos;
      float actuator_state_vel;
      uint32_t axis_error;
      uint8_t axis_current_state;
      uint8_t controller_status;
      float iq_setpoint;
      float iq_measured;
      case 0x01:  // ODrive Heartbeat Message
        std::memcpy(&axis_error, frame.data, sizeof(uint32_t));
        std::memcpy(&axis_current_state, frame.data + 4, sizeof(uint8_t));
        std::memcpy(&controller_status, frame.data + 7, sizeof(uint8_t));
        // RCUTILS_LOG_INFO_NAMED("Odrive",
        //                        "Received ODrive Heartbeat Message:\n"
        //                        "axis_error: %d, axis_current_state: %d, "
        //                        "controller_status: %d",
        //                        axis_error, axis_current_state, controller_status);
        break;
      case 0x09:  // Get Encoder Estimates
        // printf("estimates!\n");
        actuator_state_pos = 0;
        actuator_state_vel = 0;
        std::memcpy(&actuator_state_pos, frame.data, sizeof(float));
        // odrive axis unit is in turns and turns/sec, convert it to rad and rad/s
        self->joint_state_pos_ = actuator_state_pos * TURN_TO_RAD / self->reduction_ratio_ + self->joint_offset_;
        std::memcpy(&actuator_state_vel, frame.data + 4, sizeof(float));
        self->joint_state_vel_ = actuator_state_vel * TURN_TO_RAD / self->reduction_ratio_;
        break;
      case 0x14:
        std::memcpy(&iq_setpoint, frame.data, sizeof(float));
        std::memcpy(&iq_measured, frame.data + 4, sizeof(float));
        self->joint_state_eff_ = iq_measured / self->motor_kt_ * self->reduction_ratio_;
        break;
      default:
        break;
    }
    return true;
  }

  double joint_offset_ = 0;
  double reduction_ratio_ = 1;
  double motor_kt_ = 1;
  double joint_command_kp_ = 0;
  double joint_command_kd_ = 0;
  double joint_command_pos_ = 0;
  double joint_command_vel_ = 0;
  double joint_command_eff_ = 0;
  double joint_command_calibration_pos_ = std::numeric_limits<double>::quiet_NaN();
  double joint_command_request_state_ = std::numeric_limits<double>::quiet_NaN();
  double joint_command_clear_error_ = std::numeric_limits<double>::quiet_NaN();
  double joint_state_pos_ = std::numeric_limits<double>::quiet_NaN();
  double joint_state_vel_ = std::numeric_limits<double>::quiet_NaN();
  double joint_state_eff_ = std::numeric_limits<double>::quiet_NaN();
  // to detect kp kd change
  double last_kp = 0;
  double last_kd = 0;

  std::shared_ptr<SocketCan> can_device_{ nullptr };
  unsigned short can_id_ = -1;
};
}  // namespace odrive_cansimple_hardware_interface