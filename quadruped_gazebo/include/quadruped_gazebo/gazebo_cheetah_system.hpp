#pragma once

#include <gazebo_ros2_control/gazebo_system.hpp>

namespace quadruped_gazebo
{
    class GazeboCheetahSystemPrivate;
    class GazeboCheetahSystem : public gazebo_ros2_control::GazeboSystem
    {
    public:
        // Documentation Inherited
        // hardware_interface::return_type configure(const hardware_interface::HardwareInfo &system_info)
        //     override;

        // Documentation Inherited
        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Documentation Inherited
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Documentation Inherited
        // hardware_interface::return_type start() override;

        // Documentation Inherited
        // hardware_interface::return_type stop() override;

        // Documentation Inherited
        // hardware_interface::return_type read() override;

        // Documentation Inherited
        hardware_interface::return_type write() override;

        // Documentation Inherited
        bool initSim(
            rclcpp::Node::SharedPtr &model_nh,
            gazebo::physics::ModelPtr parent_model,
            const hardware_interface::HardwareInfo &hardware_info,
            sdf::ElementPtr sdf) override;

    private:
        void registerCheetahJoints(
            const hardware_interface::HardwareInfo &hardware_info,
            gazebo::physics::ModelPtr parent_model);

        /// \brief Private data class
        std::unique_ptr<GazeboCheetahSystemPrivate> dataPtr;
    };

}

