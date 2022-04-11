#include "quadruped_gazebo/gazebo_cheetah_system.hpp"
#include "angles/angles.h"
#include "gazebo/common/Time.hh"
#include "rcutils/logging.h"

// Stupid redefinition of `GazeboSystemPrivate'
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"
#include "gazebo/sensors/SensorManager.hh"
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

    /// \brief vector with the control method defined in the URDF for each joint.
    std::vector<GazeboSystemInterface::ControlMethod> joint_control_methods_;

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

    /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
    std::vector<std::array<double, 10>> imu_sensor_data_;

    /// \brief handles to the FT sensors from within Gazebo
    std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

    /// \brief An array per FT sensor for 3D force and torquee
    std::vector<std::array<double, 6>> ft_sensor_data_;

    /// \brief state interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::StateInterface> state_interfaces_;

    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> cheetah_command_interfaces_;
};

class quadruped_gazebo::GazeboCheetahSystemPrivate
{
public:
    GazeboCheetahSystemPrivate() = default;
    ~GazeboCheetahSystemPrivate() = default;
    /// \brief DOF equals to number of joints.
    size_t n_cheetah_joint;
    /// \brief Gazebo Model Ptr.
    gazebo::physics::ModelPtr parent_model_;
    /// \brief vector with the joint's names.
    std::vector<std::string> joint_names_;
    /// \brief vector with the current cmd joint position
    std::vector<double> joint_position_cmd_;
    /// \brief vector with the current cmd joint velocity
    std::vector<double> joint_velocity_cmd_;
    /// \brief vector with the current cmd joint effort
    std::vector<double> joint_effort_cmd_;
    /// \brief vector with the current cmd joint kp
    std::vector<double> joint_kp_cmd_;
    /// \brief vector with the current cmd joint kd
    std::vector<double> joint_kd_cmd_;
    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> cheetah_command_interfaces_;
    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> all_command_interfaces_;
    /// \brief handles to the joints from within Gazebo
    std::vector<gazebo::physics::JointPtr> sim_joints_;
};

namespace quadruped_gazebo
{
    bool GazeboCheetahSystem::initSim(
        rclcpp::Node::SharedPtr &model_nh,
        gazebo::physics::ModelPtr parent_model,
        const hardware_interface::HardwareInfo &hardware_info,
        sdf::ElementPtr sdf)
    {
        rcutils_logging_set_logger_level("pluginlib.ClassLoader",RCUTILS_LOG_SEVERITY_DEBUG);
        
        RCLCPP_INFO(model_nh->get_logger(), "Initializing GazeboCheetahSystem...");
        this->dataPtr = std::make_unique<GazeboCheetahSystemPrivate>();
        gazebo_ros2_control::GazeboSystem::initSim(model_nh, parent_model, hardware_info, sdf);
        registerCheetahJoints(hardware_info, parent_model);
        return true;
    }

    void GazeboCheetahSystem::registerCheetahJoints(
        const hardware_interface::HardwareInfo &hardware_info,
        gazebo::physics::ModelPtr parent_model)
    {
        auto n_dof = hardware_info.joints.size();
        // Find if there are cheetah joints
        for (unsigned int j = 0; j < n_dof; j++)
        {
            auto search = hardware_info.joints[j].parameters.find("joint_type");
            if (search != hardware_info.joints[j].parameters.end() && search->second == "cheetah")
            {

                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Parsed Cheetah Actuator Joint: " << hardware_info.joints[j].name);
                this->dataPtr->n_cheetah_joint++;
                this->dataPtr->joint_names_.push_back(hardware_info.joints[j].name);
            }
        }

        // allocate memory for the command interfaces
        this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_cheetah_joint);
        this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_cheetah_joint);
        this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_cheetah_joint);
        this->dataPtr->joint_kp_cmd_.resize(this->dataPtr->n_cheetah_joint);
        this->dataPtr->joint_kd_cmd_.resize(this->dataPtr->n_cheetah_joint);

        for (unsigned int j = 0; j < this->dataPtr->n_cheetah_joint; j++)
        {
            std::string joint_name = this->dataPtr->joint_names_[j];
            gazebo::physics::JointPtr simjoint = parent_model->GetJoint(this->dataPtr->joint_names_[j]);
            if (!simjoint)
            {
                RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "Cheetah Actuator Joint " << hardware_info.joints[j].name << " not found in gazebo model");
                return;
            }
            this->dataPtr->sim_joints_.push_back(simjoint);
            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);
            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");
            for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++)
            {
                if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_position")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_position");
                    this->dataPtr->cheetah_command_interfaces_.emplace_back(
                        joint_name,
                        "hybrid_position",
                        &this->dataPtr->joint_position_cmd_[j]);
                }
                if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_velocity")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_velocity");

                    this->dataPtr->cheetah_command_interfaces_.emplace_back(
                        joint_name,
                        "hybrid_velocity",
                        &this->dataPtr->joint_velocity_cmd_[j]);
                }
                if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_effort")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_effort");
                    this->dataPtr->cheetah_command_interfaces_.emplace_back(
                        joint_name,
                        "hybrid_effort",
                        &this->dataPtr->joint_effort_cmd_[j]);
                }
                if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_kp")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_kp");
                    this->dataPtr->cheetah_command_interfaces_.emplace_back(
                        joint_name,
                        "hybrid_kp",
                        &this->dataPtr->joint_kp_cmd_[j]);
                }
                if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_kd")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_kd");
                    this->dataPtr->cheetah_command_interfaces_.emplace_back(
                        joint_name,
                        "hybrid_kd",
                        &this->dataPtr->joint_kd_cmd_[j]);
                }
            }
        }
        this->dataPtr->parent_model_ = parent_model;
    }

    std::vector<hardware_interface::CommandInterface>
    GazeboCheetahSystem::export_command_interfaces()
    {
        // RCLCPP_WARN(this->nh_->get_logger(),"Called export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces = GazeboSystem::export_command_interfaces();
        for (auto &command_interface : this->dataPtr->cheetah_command_interfaces_)
        {
            command_interfaces.emplace_back(std::move(command_interface));
        }
        this->dataPtr->all_command_interfaces_.swap(command_interfaces);
        return std::move(this->dataPtr->all_command_interfaces_);
    }

    hardware_interface::return_type GazeboCheetahSystem::write()
    {
        // count avarge frequency
        /*
        gazebo::common::Time gz_time_now = this->dataPtr->parent_model_->GetWorld()->SimTime();
        static gazebo::common::Time gz_time_last{};
        static unsigned int gz_count{};
        if(gz_time_now - gz_time_last > gazebo::common::Time(1,0))
        {
            gz_time_last = gz_time_now;
            RCLCPP_INFO(this->nh_->get_logger(), "Avarge Control Frequency: %d",gz_count);
            gz_count = 0;
        }
        gz_count++;
        */
        auto ret = GazeboSystem::write();
        if (ret != hardware_interface::return_type::OK)
        {
            return ret;
        }
        for (unsigned int j = 0; j < this->dataPtr->n_cheetah_joint; j++)
        {
            const double ePos = angles::shortest_angular_distance(this->dataPtr->sim_joints_[j]->Position(0), this->dataPtr->joint_position_cmd_[j]);
            // effort=effort_des + kp*(position_des-position_cur) + kd*(velocity_des-velocity_cur);
            const double effort_cmd =
                this->dataPtr->joint_effort_cmd_[j] + this->dataPtr->joint_kp_cmd_[j] * (ePos) + this->dataPtr->joint_kd_cmd_[j] * (this->dataPtr->joint_velocity_cmd_[j] - this->dataPtr->sim_joints_[j]->GetVelocity(0));
            this->dataPtr->sim_joints_[j]->SetForce(0, effort_cmd);
        }
        return hardware_interface::return_type::OK;
    }
} // namespace quadruped_gazebo

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(
    quadruped_gazebo::GazeboCheetahSystem, gazebo_ros2_control::GazeboSystemInterface)