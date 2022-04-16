#pragma once

#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"
#include "gazebo/sensors/SensorManager.hh"
#include <gazebo_ros2_control/gazebo_system.hpp>
class gazebo_ros2_control::GazeboSystemPrivate
{
public:
    GazeboSystemPrivate() = default;

    ~GazeboSystemPrivate() = default;

    /// \brief Degrees od freedom.
    size_t n_dof_;

    /// \brief Number of sensors.
    size_t n_sensors_;

    /// \brief Gazebo Model Ptr.
    gazebo::physics::ModelPtr parent_model_;

    /// \brief last time the write method was called.
    rclcpp::Time last_update_sim_time_ros_;

    /// \brief vector with the joint's names.
    std::vector<std::string> joint_names_;

    /// \brief vector with the control method defined in the URDF for each
    //joint. 
    std::vector<GazeboSystemInterface::ControlMethod>
    joint_control_methods_;

    /// \brief handles to the joints from within Gazebo
    std::vector<gazebo::physics::JointPtr> sim_joints_;

    /// \brief vector with the current joint position
    std::vector<double> joint_position_;

    /// \brief vector with the current joint velocity
    std::vector<double> joint_velocity_;

    /// \brief vector with the current joint effort
    std::vector<double> joint_effort_;

    /// \brief vector with the current cmd joint position
    std::vector<double> joint_position_cmd_;

    /// \brief vector with the current cmd joint velocity
    std::vector<double> joint_velocity_cmd_;

    /// \brief vector with the current cmd joint effort
    std::vector<double> joint_effort_cmd_;

    /// \brief handles to the imus from within Gazebo
    std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

    /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3
    //linear acceleration 
    std::vector<std::array<double, 10>> imu_sensor_data_;

    /// \brief handles to the FT sensors from within Gazebo
    std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

    /// \brief An array per FT sensor for 3D force and torquee
    std::vector<std::array<double, 6>> ft_sensor_data_;

    /// \brief state interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::StateInterface> state_interfaces_;

    /// \brief command interfaces that will be exported to the Resource
    //Manager 
    std::vector<hardware_interface::CommandInterface>
    command_interfaces_;
};
