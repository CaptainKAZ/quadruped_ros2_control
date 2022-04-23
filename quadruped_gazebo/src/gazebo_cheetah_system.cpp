#include "quadruped_gazebo/gazebo_cheetah_system.hpp"
#include "angles/angles.h"
#include "gazebo/common/Time.hh"
#include "rcutils/logging.h"
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "quadruped_gazebo/gazebo_system_private.hpp"

class quadruped_gazebo::GazeboCheetahSystemPrivate {
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
  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> cheetah_state_interfaces_;
  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> all_state_interfaces_;
  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  /// \brief vector with the contact sensors' names.
  std::vector<std::string> contact_sensor_names_;
  /// \brief vector with the current contact state
  std::vector<double> contact_state_;
  /// \brief contact manager
  gazebo::physics::ContactManager *contact_manager_;
  /// \brief contact link to be suspected
  std::vector<std::string> contact_link_;
};

namespace quadruped_gazebo {
bool GazeboCheetahSystem::initSim(
    rclcpp::Node::SharedPtr &model_nh, gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo &hardware_info,
    sdf::ElementPtr sdf) {
   (void)rcutils_logging_set_logger_level("pluginlib.ClassLoader",RCUTILS_LOG_SEVERITY_DEBUG);

  RCLCPP_INFO(model_nh->get_logger(), "Initializing GazeboCheetahSystem...");
  this->dataPtr = std::make_unique<GazeboCheetahSystemPrivate>();
  gazebo_ros2_control::GazeboSystem::initSim(model_nh, parent_model,
                                             hardware_info, sdf);
  registerCheetahJoints(hardware_info, parent_model);
  registerCheetahContacts(hardware_info, parent_model);
  return true;
}

void GazeboCheetahSystem::registerCheetahContacts(
    const hardware_interface::HardwareInfo &hardware_info,
    gazebo::physics::ModelPtr parent_model) {
  // Find contact sensor
  size_t n_contact_sensors = 0;
  for (auto &sensor : hardware_info.sensors) {
    if (sensor.name.find("contact") != std::string::npos) {
      RCLCPP_INFO(this->nh_->get_logger(), "Found contact sensor: %s",
                  sensor.name.c_str());
      n_contact_sensors++;
      this->dataPtr->contact_sensor_names_.emplace_back(sensor.name);
      this->dataPtr->contact_link_.emplace_back(sensor.parameters.at("link"));
    }
  }
  if (n_contact_sensors == 0) {
    RCLCPP_WARN(this->nh_->get_logger(),
                "No contact sensor found, not registering contact interface");
    return;
  }
  this->dataPtr->contact_state_.resize(n_contact_sensors);
  this->dataPtr->contact_manager_ =
      parent_model->GetWorld()->Physics()->GetContactManager();
  this->dataPtr->contact_manager_->SetNeverDropContacts(
      true); // NOTE: If false, we need to select view->contacts in gazebo GUI
             // to avoid returning nothing when calling
             // ContactManager::GetContacts()
  for (size_t i = 0; i < n_contact_sensors; i++) {
    this->dataPtr->cheetah_state_interfaces_.emplace_back(
        this->dataPtr->contact_sensor_names_[i], "contact",
        &this->dataPtr->contact_state_[i]);
  }
}

void GazeboCheetahSystem::registerCheetahJoints(
    const hardware_interface::HardwareInfo &hardware_info,
    gazebo::physics::ModelPtr parent_model) {
  auto n_dof = hardware_info.joints.size();
  // Find if there are cheetah joints
  for (unsigned int j = 0; j < n_dof; j++) {
    auto search = hardware_info.joints[j].parameters.find("joint_type");
    if (search != hardware_info.joints[j].parameters.end() &&
        search->second == "cheetah") {

      RCLCPP_INFO_STREAM(
          this->nh_->get_logger(),
          "Parsed Cheetah Actuator Joint: " << hardware_info.joints[j].name);
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

  for (unsigned int j = 0; j < this->dataPtr->n_cheetah_joint; j++) {
    std::string joint_name = this->dataPtr->joint_names_[j];
    gazebo::physics::JointPtr simjoint =
        parent_model->GetJoint(this->dataPtr->joint_names_[j]);
    if (!simjoint) {
      RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                          "Cheetah Actuator Joint "
                              << hardware_info.joints[j].name
                              << " not found in gazebo model");
      return;
    }
    this->dataPtr->sim_joints_.push_back(simjoint);
    RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                       "Loading joint: " << joint_name);
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");
    for (unsigned int i = 0;
         i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name ==
          "hybrid_position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_position");
        this->dataPtr->cheetah_command_interfaces_.emplace_back(
            joint_name, "hybrid_position",
            &this->dataPtr->joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
          "hybrid_velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_velocity");

        this->dataPtr->cheetah_command_interfaces_.emplace_back(
            joint_name, "hybrid_velocity",
            &this->dataPtr->joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name ==
          "hybrid_effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_effort");
        this->dataPtr->cheetah_command_interfaces_.emplace_back(
            joint_name, "hybrid_effort", &this->dataPtr->joint_effort_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_kp") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_kp");
        this->dataPtr->cheetah_command_interfaces_.emplace_back(
            joint_name, "hybrid_kp", &this->dataPtr->joint_kp_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "hybrid_kd") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t hybrid_kd");
        this->dataPtr->cheetah_command_interfaces_.emplace_back(
            joint_name, "hybrid_kd", &this->dataPtr->joint_kd_cmd_[j]);
      }
    }
  }
  this->dataPtr->parent_model_ = parent_model;
}

std::vector<hardware_interface::CommandInterface>
GazeboCheetahSystem::export_command_interfaces() {
  // RCLCPP_WARN(this->nh_->get_logger(),"Called export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      GazeboSystem::export_command_interfaces();
  for (auto &command_interface : this->dataPtr->cheetah_command_interfaces_) {
    command_interfaces.emplace_back(std::move(command_interface));
  }
  this->dataPtr->all_command_interfaces_.swap(command_interfaces);
  return std::move(this->dataPtr->all_command_interfaces_);
}

std::vector<hardware_interface::StateInterface>
GazeboCheetahSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces =
      GazeboSystem::export_state_interfaces();
  for (auto &state_interface : this->dataPtr->cheetah_state_interfaces_) {
    state_interfaces.emplace_back(std::move(state_interface));
  }
  this->dataPtr->all_state_interfaces_.swap(state_interfaces);
  return std::move(this->dataPtr->all_state_interfaces_);
}

hardware_interface::return_type GazeboCheetahSystem::write() {
  // count avarge frequency
  /*
  gazebo::common::Time gz_time_now =
  this->dataPtr->parent_model_->GetWorld()->SimTime(); static
  gazebo::common::Time gz_time_last{}; static unsigned int gz_count{};
  if(gz_time_now - gz_time_last > gazebo::common::Time(1,0))
  {
      gz_time_last = gz_time_now;
      RCLCPP_INFO(this->nh_->get_logger(), "Avarge Control Frequency:
  %d",gz_count); gz_count = 0;
  }
  gz_count++;
  */
  auto ret = GazeboSystem::write();
  if (ret != hardware_interface::return_type::OK) {
    return ret;
  }
  for (unsigned int j = 0; j < this->dataPtr->n_cheetah_joint; j++) {
    const double ePos = angles::shortest_angular_distance(
        this->dataPtr->sim_joints_[j]->Position(0),
        this->dataPtr->joint_position_cmd_[j]);
    // effort=effort_des + kp*(position_des-position_cur) +
    // kd*(velocity_des-velocity_cur);
    double effort_cmd =
        this->dataPtr->joint_effort_cmd_[j] +
        this->dataPtr->joint_kp_cmd_[j] * (ePos) +
        this->dataPtr->joint_kd_cmd_[j] *
            (this->dataPtr->joint_velocity_cmd_[j] -
             this->dataPtr->sim_joints_[j]->GetVelocity(0));
    //TODO CHEETAH read from urdf!!!
    if(effort_cmd>17){
      effort_cmd=17;
    }
    if(effort_cmd<-17){
      effort_cmd=-17;
    }
    this->dataPtr->sim_joints_[j]->SetForce(0, effort_cmd);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboCheetahSystem::read() {
  auto ret = GazeboSystem::read();
  if (ret != hardware_interface::return_type::OK) {
    return ret;
  }
  for (auto &contact_state : this->dataPtr->contact_state_) {
    contact_state = 0;
  }
  auto contacts = this->dataPtr->contact_manager_->GetContacts();
  for (size_t i = 0; i < this->dataPtr->contact_manager_->GetContactCount();
       i++) {
    auto now = this->dataPtr->parent_model_->GetWorld()->SimTime();
    if (now - contacts[i]->time > gazebo::common::Time(0, 2000000)) {
      RCLCPP_WARN(this->nh_->get_logger(), "Contact with %s is too old",
                  contacts[i]->collision1->GetName().c_str());
      continue;
    }
    for (size_t j = 0; j < this->dataPtr->contact_state_.size(); j++) {

      if (contacts[i]->collision1->GetLink()->GetName().find(
              this->dataPtr->contact_link_[j]) != std::string::npos ||
          contacts[i]->collision2->GetLink()->GetName().find(
              this->dataPtr->contact_link_[j]) != std::string::npos) {
        this->dataPtr->contact_state_[j] = 1;
        break;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

} // namespace quadruped_gazebo

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(quadruped_gazebo::GazeboCheetahSystem,
                       gazebo_ros2_control::GazeboSystemInterface)