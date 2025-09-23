#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "vesc_driver/vesc_interface.h"

namespace vesc_driver {

class VescDriverNode : public rclcpp::Node {
public:
  VescDriverNode();
  ~VescDriverNode() = default;

private:
  // Core functionality
  void timerCallback();
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void sendDriveCommands();
  void updateOdometry(double rpm);

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // VESC interface
  std::unique_ptr<VescInterface> vesc_;
  
  // State variables
  std::atomic<double> target_velocity_{0.0};
  std::atomic<double> t_last_command_{0.0};
  bool driver_initialized_{false};
};

} // namespace vesc_driver