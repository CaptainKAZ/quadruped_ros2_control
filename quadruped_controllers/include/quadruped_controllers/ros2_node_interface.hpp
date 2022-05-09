#pragma once

// The module handles ROS non-realtime node comunication stuff

#include "quadruped_controllers/quadruped_types.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace quadruped_controllers {
class Ros2NodeInterfaceBase {
public:
  Ros2NodeInterfaceBase() = default;
  virtual ~Ros2NodeInterfaceBase() = default;
  virtual void init(rclcpp::Node::SharedPtr &&node,
                    std::shared_ptr<QuadrupedState> &state,
                    std::shared_ptr<QuadrupedCommand> &command) {
    node_ = node;
    state_ = state;
    command_ = command;
  };
  virtual void update(const rclcpp::Time &current_time) = 0;

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<QuadrupedState> state_;
  std::shared_ptr<QuadrupedCommand> command_;
};

// Publish estimated state to odom and tf
class OdomTfPublisher : public Ros2NodeInterfaceBase {
public:
  explicit OdomTfPublisher(rclcpp::Node::SharedPtr &&node,
                           std::shared_ptr<QuadrupedState> &state,
                           std::shared_ptr<QuadrupedCommand> &command) {
    init(std::forward<decltype(node)>(node),
         std::forward<decltype(state)>(state),
         std::forward<decltype(command)>(command));
  }
  ~OdomTfPublisher() = default;
  void init(rclcpp::Node::SharedPtr &&node,
            std::shared_ptr<QuadrupedState> &state,
            std::shared_ptr<QuadrupedCommand> &command) override;
  void update(const rclcpp::Time &current_time) override;

protected:
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_ =
      nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
      rt_odom_pub_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odom_tf_pub_ =
      nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
      rt_odom_tf_pub_ = nullptr;
};

// Subscribe to geometry_msgs/Twist command
class TwistSubscriber : public Ros2NodeInterfaceBase {
public:
  explicit TwistSubscriber(rclcpp::Node::SharedPtr &&node,
                           std::shared_ptr<QuadrupedState> &state,
                           std::shared_ptr<QuadrupedCommand> &command) {
    init(std::forward<decltype(node)>(node),
         std::forward<decltype(state)>(state),
         std::forward<decltype(command)>(command));
  }
  ~TwistSubscriber() = default;
  void init(rclcpp::Node::SharedPtr &&node,
            std::shared_ptr<QuadrupedState> &state,
            std::shared_ptr<QuadrupedCommand> &command) override;
  void update(const rclcpp::Time &current_time) override;

protected:
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_sub_ =
      nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::Twist>>
      rx_msg_ptr{nullptr};
  rclcpp::Time last_time_;
};

// Subscribe to groundtruth odom
class GroundTruthSubscriber : public Ros2NodeInterfaceBase {
public:
  explicit GroundTruthSubscriber(rclcpp::Node::SharedPtr &&node,
                                 std::shared_ptr<QuadrupedState> &state,
                                 std::shared_ptr<QuadrupedCommand> &command) {
    init(std::forward<decltype(node)>(node),
         std::forward<decltype(state)>(state),
         std::forward<decltype(command)>(command));
  }
  ~GroundTruthSubscriber() = default;
  void init(rclcpp::Node::SharedPtr &&node,
            std::shared_ptr<QuadrupedState> &state,
            std::shared_ptr<QuadrupedCommand> &command) override;
  void update(const rclcpp::Time &current_time) override;
  std::shared_ptr<QuadrupedState> getTruthState();

protected:
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> cmd_sub_ =
      nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<nav_msgs::msg::Odometry>>
      rx_msg_ptr{nullptr};
  // must reserve before init
  std::shared_ptr<QuadrupedState> truth_state_{
      std::make_shared<QuadrupedState>()};
};

class P3dPublisher : public Ros2NodeInterfaceBase {
public:
  explicit P3dPublisher(rclcpp::Node::SharedPtr &&node,
                        std::shared_ptr<QuadrupedState> &state,
                        std::shared_ptr<QuadrupedCommand> &command,
                        std::string topic_name)
      : topic_name_(topic_name) {
    init(std::forward<decltype(node)>(node),
         std::forward<decltype(state)>(state),
         std::forward<decltype(command)>(command));
  }
  ~P3dPublisher() = default;
  void init(rclcpp::Node::SharedPtr &&node,
            std::shared_ptr<QuadrupedState> &state,
            std::shared_ptr<QuadrupedCommand> &command) override;
  void update(const rclcpp::Time &current_time) override;
  void addPoint(double x, double y, double z);
  void clearPoint();

protected:
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>>
      point_pub = nullptr;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>
      rt_point_pub = nullptr;
  std::string topic_name_;
};

class LinePublisher : public Ros2NodeInterfaceBase {
public:
  explicit LinePublisher(rclcpp::Node::SharedPtr &&node,
                         std::shared_ptr<QuadrupedState> &state,
                         std::shared_ptr<QuadrupedCommand> &command,
                         std::string topic_name)
      : topic_name_(topic_name) {
    init(std::forward<decltype(node)>(node),
         std::forward<decltype(state)>(state),
         std::forward<decltype(command)>(command));
  }
  ~LinePublisher() = default;
  void init(rclcpp::Node::SharedPtr &&node,
            std::shared_ptr<QuadrupedState> &state,
            std::shared_ptr<QuadrupedCommand> &command) override;
  void update(const rclcpp::Time &current_time) override;
  void addLine(double x1, double y1, double z1, double x2, double y2,
               double z2);
  void clearLine();

protected:
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> line_pub =
      nullptr;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>>
      rt_line_pub = nullptr;
  std::string topic_name_;
};

} // namespace quadruped_controllers