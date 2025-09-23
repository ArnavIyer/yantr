#include "vesc_driver/vesc_driver_node.h"
#include "vesc_driver/vesc_packet.h"
#include <iostream>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

namespace vesc_driver {

// Hardcoded constants (no config files needed) - CALCULATED FOR YOUR ROBOT
static constexpr double SPEED_TO_ERPM_GAIN = 219.45;     // 14-pole motor, 25:1 gear, 10" wheels
static constexpr double SPEED_TO_ERPM_OFFSET = 0.0;
static constexpr double ERPM_SPEED_LIMIT = 439.0;        // 2 m/s max safe speed
static constexpr double MAX_ACCELERATION = 5.0;
static constexpr double MAX_DECELERATION = 10.0;
static constexpr double COMMAND_TIMEOUT = 0.5;
static constexpr const char* SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00";
static constexpr double COMMAND_RATE = 20.0; // Hz

VescDriverNode::VescDriverNode() : Node("vesc_driver_node") {
  
  // Initialize VESC connection
  try {
    // Use boost::bind for callback since VescInterface expects boost::function
    vesc_ = std::make_unique<VescInterface>(
      std::string(),  // Empty string, will call connect() later
      boost::bind(&VescDriverNode::vescPacketCallback, this, _1)
    );
    
    if (!vesc_->connect(SERIAL_PORT)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to VESC at %s", SERIAL_PORT);
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "VESC initialization failed: %s", e.what());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Connected to VESC at %s", SERIAL_PORT);
  
  // Create publishers
  state_pub_ = this->create_publisher<std_msgs::msg::Float64>("vesc/rpm", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  
  // Create subscriber
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&VescDriverNode::velocityCallback, this, std::placeholders::_1));
  
  // Create timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / COMMAND_RATE)),
      std::bind(&VescDriverNode::timerCallback, this));
  
  driver_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "VESC driver initialized");
}

void VescDriverNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  target_velocity_ = msg->linear.x;  // Only use linear velocity
  t_last_command_ = this->get_clock()->now().seconds();
}

void VescDriverNode::timerCallback() {
  if (!driver_initialized_ || !vesc_->isConnected()) {
    return;
  }
  
  // Check for command timeout
  double current_time = this->get_clock()->now().seconds();
  if (current_time - t_last_command_ > COMMAND_TIMEOUT) {
    target_velocity_ = 0.0;  // Stop motor on timeout
  }
  
  // Send commands and request state
  sendDriveCommands();
  vesc_->requestState();
}

void VescDriverNode::sendDriveCommands() {
  // Convert velocity to ERPM
  double erpm = SPEED_TO_ERPM_GAIN * target_velocity_ + SPEED_TO_ERPM_OFFSET;
  
  // Apply limits
  erpm = std::clamp(erpm, -ERPM_SPEED_LIMIT, ERPM_SPEED_LIMIT);
  
  // Send to VESC
  vesc_->setSpeed(erpm);
}

void VescDriverNode::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet) {
  if (packet->name() == "Values") {
    // Cast to VescPacketValues to access RPM data
    auto values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    if (!values) return;
    
    // Publish RPM state
    auto rpm_msg = std_msgs::msg::Float64();
    rpm_msg.data = values->rpm();
    state_pub_->publish(rpm_msg);
    
    // Update odometry
    updateOdometry(values->rpm());
  }
}

void VescDriverNode::updateOdometry(double rpm) {
  // Simple linear odometry (no steering)
  double linear_velocity = (rpm - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN;
  
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.angular.z = 0.0;  // No turning
  
  odom_pub_->publish(odom_msg);
}

} // namespace vesc_driver

// Main function - replace glog/gflags version
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_driver::VescDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
