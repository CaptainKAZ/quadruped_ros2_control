#include "quadruped_controllers/ros2_node_interface.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "quadruped_controllers/quadruped_types.hpp"
#include "rclcpp/time.hpp"
#include <memory>
#include <rclcpp/qos.hpp>
#include <utility>

namespace quadruped_controllers {
void OdomTfPublisher::init(rclcpp::Node::SharedPtr &&node,
                           std::shared_ptr<QuadrupedState> &state,
                           std::shared_ptr<QuadrupedCommand> &command) {
  Ros2NodeInterfaceBase::init(std::forward<decltype(node)>(node), state,
                              command);
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SystemDefaultsQoS());
  rt_odom_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
  odom_tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
  rt_odom_tf_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odom_tf_pub_);
  rt_odom_pub_->lock();
  auto &odometry_message = rt_odom_pub_->msg_;
  odometry_message.header.frame_id = "root";
  odometry_message.child_frame_id = "world";
  rt_odom_pub_->unlock();
  rt_odom_tf_pub_->lock();
  auto &tf_message = rt_odom_tf_pub_->msg_;
  tf_message.transforms.resize(1);
  tf_message.transforms.front().header.frame_id = "world";
  tf_message.transforms.front().child_frame_id = "root";
  rt_odom_tf_pub_->unlock();
}

void OdomTfPublisher::update(const rclcpp::Time &current_time) {
  if (rt_odom_pub_->trylock()) {
    rt_odom_pub_->msg_.header.stamp = current_time;
    rt_odom_pub_->msg_.pose.pose.position.x = state_->pos_(0);
    rt_odom_pub_->msg_.pose.pose.position.y = state_->pos_(1);
    rt_odom_pub_->msg_.pose.pose.position.z = state_->pos_(2);
    rt_odom_pub_->msg_.pose.pose.orientation.x = state_->quat_.x();
    rt_odom_pub_->msg_.pose.pose.orientation.y = state_->quat_.y();
    rt_odom_pub_->msg_.pose.pose.orientation.z = state_->quat_.z();
    rt_odom_pub_->msg_.pose.pose.orientation.w = state_->quat_.w();
    rt_odom_pub_->msg_.twist.twist.angular.x = state_->angular_vel_[0];
    rt_odom_pub_->msg_.twist.twist.angular.y = state_->angular_vel_[1];
    rt_odom_pub_->msg_.twist.twist.angular.z = state_->angular_vel_[2];
    rt_odom_pub_->msg_.twist.twist.linear.x = state_->linear_vel_[0];
    rt_odom_pub_->msg_.twist.twist.linear.y = state_->linear_vel_[1];
    rt_odom_pub_->msg_.twist.twist.linear.z = state_->linear_vel_[2];
    rt_odom_pub_->unlockAndPublish();
  }
  if (rt_odom_tf_pub_->trylock()) {
    auto &transform = rt_odom_tf_pub_->msg_.transforms.front();
    transform.header.stamp = current_time;
    transform.transform.translation.x = state_->pos_(0);
    transform.transform.translation.y = state_->pos_(1);
    transform.transform.translation.z = state_->pos_(2);
    transform.transform.rotation.x = state_->quat_.x();
    transform.transform.rotation.y = state_->quat_.y();
    transform.transform.rotation.z = state_->quat_.z();
    transform.transform.rotation.w = state_->quat_.w();
    rt_odom_tf_pub_->unlockAndPublish();
  }
}

void TwistSubscriber::init(rclcpp::Node::SharedPtr &&node,
                           std::shared_ptr<QuadrupedState> &state,
                           std::shared_ptr<QuadrupedCommand> &command) {
  Ros2NodeInterfaceBase::init(std::forward<decltype(node)>(node), state,
                              command);
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        rx_msg_ptr.set(std::move(msg));
        last_time_ = node_->now();
      });
  last_time_ = node->get_clock()->now();
  rx_msg_ptr.set(std::make_shared<geometry_msgs::msg::Twist>());
}

void TwistSubscriber::update(const rclcpp::Time &current_time) {
  std::shared_ptr<geometry_msgs::msg::Twist> last_msg;
  rx_msg_ptr.get(last_msg);

  if (last_msg == nullptr) {
    return;
  }
  if (current_time - last_time_ >
      rclcpp::Duration::from_seconds(0.5)) {
    rx_msg_ptr.set(std::make_shared<geometry_msgs::msg::Twist>());
  }
  command_->linear_vel_(0) = last_msg->linear.x;
  command_->linear_vel_(1) = last_msg->linear.y;
  // cant set z velocity by twist message
  command_->linear_vel_(2) = 0;
  command_->angular_vel_(0) = 0;
  command_->angular_vel_(1) = 0;
  command_->angular_vel_(2) = last_msg->angular.z;
}

void GroundTruthSubscriber::init(rclcpp::Node::SharedPtr &&node,
                                 std::shared_ptr<QuadrupedState> &state,
                                 std::shared_ptr<QuadrupedCommand> &command) {
  Ros2NodeInterfaceBase::init(std::forward<decltype(node)>(node), state,
                              command);
  cmd_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/state", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) -> void {
        rx_msg_ptr.set(std::move(msg));
      });
  rx_msg_ptr.set(std::make_shared<nav_msgs::msg::Odometry>());
}

void GroundTruthSubscriber::update(const rclcpp::Time &current_time) {
  std::shared_ptr<nav_msgs::msg::Odometry> last_msg;
  rx_msg_ptr.get(last_msg);

  if (last_msg == nullptr) {
    return;
  }
  if (current_time - last_msg->header.stamp >
      rclcpp::Duration::from_seconds(0.1)) {
    rx_msg_ptr.set(std::make_shared<nav_msgs::msg::Odometry>());
    return;
  }
  truth_state_->pos_(0) = last_msg->pose.pose.position.x;
  truth_state_->pos_(1) = last_msg->pose.pose.position.y;
  truth_state_->pos_(2) = last_msg->pose.pose.position.z;
  truth_state_->quat_.x() = last_msg->pose.pose.orientation.x;
  truth_state_->quat_.y() = last_msg->pose.pose.orientation.y;
  truth_state_->quat_.z() = last_msg->pose.pose.orientation.z;
  truth_state_->quat_.w() = last_msg->pose.pose.orientation.w;
  truth_state_->linear_vel_(0) = last_msg->twist.twist.linear.x;
  truth_state_->linear_vel_(1) = last_msg->twist.twist.linear.y;
  truth_state_->linear_vel_(2) = last_msg->twist.twist.linear.z;
  truth_state_->angular_vel_(0) = last_msg->twist.twist.angular.x;
  truth_state_->angular_vel_(1) = last_msg->twist.twist.angular.y;
  truth_state_->angular_vel_(2) = last_msg->twist.twist.angular.z;
  //auto rot=truth_state_->quat_.toRotationMatrix();
  //truth_state_->linear_vel_=rot*truth_state_->linear_vel_;
  //truth_state_->angular_vel_=rot*truth_state_->angular_vel_;
  truth_state_->update_time_ = current_time;
}

std::shared_ptr<QuadrupedState> GroundTruthSubscriber::getTruthState() {
  return truth_state_;
}

void P3dPublisher::init(rclcpp::Node::SharedPtr &&node,
                        std::shared_ptr<QuadrupedState> &state,
                        std::shared_ptr<QuadrupedCommand> &command) {
  Ros2NodeInterfaceBase::init(std::forward<decltype(node)>(node), state,
                              command);
  point_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      topic_name_, rclcpp::SystemDefaultsQoS());
  rt_point_pub = std::make_shared<
      realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>(
      point_pub);
  rt_point_pub->lock();
  auto &point_marker_message = rt_point_pub->msg_;
  point_marker_message.header.frame_id = "world";
  rt_point_pub->unlock();
}

void P3dPublisher::update(const rclcpp::Time &current_time) {
  if (rt_point_pub->trylock()) {
    rt_point_pub->msg_.header.stamp = current_time;
  }
  rt_point_pub->unlockAndPublish();
}

void P3dPublisher::addPoint(double x, double y, double z) {
  if (rt_point_pub->trylock()) {
    auto &point_marker_message = rt_point_pub->msg_;
    point_marker_message.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    point_marker_message.action = visualization_msgs::msg::Marker::ADD;
    point_marker_message.lifetime = rclcpp::Duration(1e9);
    point_marker_message.color.a = 1.0;
    point_marker_message.color.g = 1.0;
    point_marker_message.scale.x = 0.01;
    point_marker_message.scale.y = 0.01;
    point_marker_message.scale.z = 0.01;
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    point_marker_message.points.push_back(p);
  }
  rt_point_pub->unlock();
}

void P3dPublisher::clearPoint() {
  rt_point_pub->lock();
  auto &point_marker_message = rt_point_pub->msg_;
  point_marker_message.points.clear();
  rt_point_pub->unlock();
}

void LinePublisher::init(rclcpp::Node::SharedPtr &&node,
                         std::shared_ptr<QuadrupedState> &state,
                         std::shared_ptr<QuadrupedCommand> &command) {
  Ros2NodeInterfaceBase::init(std::forward<decltype(node)>(node), state,
                              command);
  line_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      "/debug_line", rclcpp::SystemDefaultsQoS());
  rt_line_pub = std::make_shared<
      realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>(
      line_pub);
  rt_line_pub->lock();
  auto &point_marker_message = rt_line_pub->msg_;
  point_marker_message.header.frame_id = "world";
  rt_line_pub->unlock();
}

void LinePublisher::update(const rclcpp::Time &current_time) {
  if (rt_line_pub->trylock()) {
    rt_line_pub->msg_.header.stamp = current_time;
  }
  rt_line_pub->unlockAndPublish();
}

void LinePublisher::addLine(double x1, double y1, double z1, double x2,
                            double y2, double z2) {
  if (rt_line_pub->trylock()) {
    auto &point_marker_message = rt_line_pub->msg_;
    point_marker_message.type = visualization_msgs::msg::Marker::LINE_LIST;
    point_marker_message.action = visualization_msgs::msg::Marker::ADD;
    point_marker_message.lifetime = rclcpp::Duration(1e9);
    point_marker_message.color.a = 1.0;
    point_marker_message.color.r = 1.0;
    point_marker_message.scale.x = 0.01;
    point_marker_message.scale.y = 0.01;
    point_marker_message.scale.z = 0.01;
    geometry_msgs::msg::Point p1;
    p1.x = x1;
    p1.y = y1;
    p1.z = z1;
    point_marker_message.points.push_back(p1);
    geometry_msgs::msg::Point p2;
    p2.x = x2;
    p2.y = y2;
    p2.z = z2;
    point_marker_message.points.push_back(p2);
  }
  rt_line_pub->unlock();
}

void LinePublisher::clearLine() {
  rt_line_pub->lock();
  auto &point_marker_message = rt_line_pub->msg_;
  point_marker_message.points.clear();
  rt_line_pub->unlock();
}

} // namespace quadruped_controllers