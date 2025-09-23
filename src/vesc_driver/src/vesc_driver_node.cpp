#include "vesc_driver/vesc_driver_node.h"
#include "vesc_driver/vesc_packet.h"
#include <iostream>
#include <cmath>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

namespace vesc_driver {

// Hardcoded constants (no config files needed) - CALCULATED FOR YOUR ROBOT
static constexpr double SPEED_TO_ERPM_GAIN = 13160.0;    // 14-pole motor, 25:1 gear, 10" wheels (corrected)
static constexpr double SPEED_TO_ERPM_OFFSET = 0.0;
static constexpr double ERPM_SPEED_LIMIT = 26320.0;      // 2 m/s max safe speed (corrected)
static constexpr double MAX_ACCELERATION = 5.0;
static constexpr double MAX_DECELERATION = 10.0;
static constexpr double COMMAND_TIMEOUT = 0.5;
static constexpr const char* LEFT_SERIAL_PORT = "/dev/ttyACM0";
static constexpr const char* RIGHT_SERIAL_PORT = "/dev/ttyACM1";
static constexpr double COMMAND_RATE = 20.0; // Hz
static constexpr double WHEEL_TRACK = 0.3175;   // Distance between wheels (m) - MEASURE YOUR ROBOT!

VescDriverNode::VescDriverNode() : Node("vesc_driver_node"), 
                                   wheel_track_(WHEEL_TRACK),
                                   position_x_(0.0),
                                   position_y_(0.0),
                                   orientation_yaw_(0.0),
                                   last_odom_time_(this->get_clock()->now()) {
  
  // Initialize LEFT VESC connection
  try {
    left_vesc_ = std::make_unique<VescInterface>(
      std::string(),  // Empty string, will call connect() later
      boost::bind(&VescDriverNode::leftVescPacketCallback, this, _1)
    );
    
    if (!left_vesc_->connect(LEFT_SERIAL_PORT)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to LEFT VESC at %s", LEFT_SERIAL_PORT);
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "LEFT VESC initialization failed: %s", e.what());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Connected to LEFT VESC at %s", LEFT_SERIAL_PORT);
  
  // Initialize RIGHT VESC connection
  try {
    right_vesc_ = std::make_unique<VescInterface>(
      std::string(),  // Empty string, will call connect() later
      boost::bind(&VescDriverNode::rightVescPacketCallback, this, _1)
    );
    
    if (!right_vesc_->connect(RIGHT_SERIAL_PORT)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to RIGHT VESC at %s", RIGHT_SERIAL_PORT);
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "RIGHT VESC initialization failed: %s", e.what());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Connected to RIGHT VESC at %s", RIGHT_SERIAL_PORT);
  
  // Verify both connections before proceeding
  if (!left_vesc_->isConnected() || !right_vesc_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "One or both VESC connections failed!");
    RCLCPP_ERROR(this->get_logger(), "Left VESC connected: %s", left_vesc_->isConnected() ? "YES" : "NO");
    RCLCPP_ERROR(this->get_logger(), "Right VESC connected: %s", right_vesc_->isConnected() ? "YES" : "NO");
    return;
  }
  
  // Create publishers
  left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("vesc/left/rpm", 10);
  right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("vesc/right/rpm", 10);
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
  target_linear_velocity_ = msg->linear.x;
  target_angular_velocity_ = msg->angular.z;  // Use angular velocity for turning
  t_last_command_ = this->get_clock()->now().seconds();
}

void VescDriverNode::timerCallback() {
  if (!driver_initialized_) {
    return;
  }
  
  // Check individual VESC connections
  bool left_connected = left_vesc_->isConnected();
  bool right_connected = right_vesc_->isConnected();
  
  if (!left_connected || !right_connected) {
    RCLCPP_WARN(this->get_logger(), "VESC connection lost! Left: %s, Right: %s", 
                left_connected ? "OK" : "DISCONNECTED", 
                right_connected ? "OK" : "DISCONNECTED");
    
    // Emergency stop - set both targets to zero
    target_linear_velocity_ = 0.0;
    target_angular_velocity_ = 0.0;
    return;
  }
  
  // Check for command timeout
  double current_time = this->get_clock()->now().seconds();
  if (current_time - t_last_command_ > COMMAND_TIMEOUT) {
    target_linear_velocity_ = 0.0;
    target_angular_velocity_ = 0.0;  // Stop both motors on timeout
  }
  
  // Send commands and request state from both VESCs
  try {
    sendDriveCommands();
    left_vesc_->requestState();
    right_vesc_->requestState();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error sending commands to VESCs: %s", e.what());
    // Set emergency stop
    target_linear_velocity_ = 0.0;
    target_angular_velocity_ = 0.0;
  }
}

void VescDriverNode::calculateWheelVelocities(double linear_vel, double angular_vel, double& left_vel, double& right_vel) {
  // Differential drive kinematics
  // left_vel = linear_vel - (angular_vel * wheel_track / 2)
  // right_vel = linear_vel + (angular_vel * wheel_track / 2)
  
  double half_track = wheel_track_ / 2.0;
  left_vel = linear_vel - (angular_vel * half_track);
  right_vel = linear_vel + (angular_vel * half_track);
}

void VescDriverNode::sendDriveCommands() {
  // Calculate individual wheel velocities using differential drive
  double left_velocity, right_velocity;
  calculateWheelVelocities(target_linear_velocity_, target_angular_velocity_, left_velocity, right_velocity);
  
  // Convert velocities to ERPM for each motor
  double left_erpm = SPEED_TO_ERPM_GAIN * left_velocity + SPEED_TO_ERPM_OFFSET;
  double right_erpm = SPEED_TO_ERPM_GAIN * right_velocity + SPEED_TO_ERPM_OFFSET;
  
  // Apply limits to both motors
  left_erpm = std::clamp(left_erpm, -ERPM_SPEED_LIMIT, ERPM_SPEED_LIMIT);
  right_erpm = std::clamp(right_erpm, -ERPM_SPEED_LIMIT, ERPM_SPEED_LIMIT);
  
  // Log commands for debugging (at reduced frequency)
  static int debug_counter = 0;
  if (++debug_counter % 40 == 0) {  // Log every 2 seconds at 20Hz
    RCLCPP_DEBUG(this->get_logger(), "Commands - Left: %.0f ERPM (%.2f m/s), Right: %.0f ERPM (%.2f m/s)",
                 left_erpm, left_velocity, right_erpm, right_velocity);
  }
  
  // Send to both VESCs with error handling
  try {
    if (left_vesc_->isConnected()) {
      left_vesc_->setSpeed(left_erpm);
    }
    if (right_vesc_->isConnected()) {
      right_vesc_->setSpeed(right_erpm);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send speed commands: %s", e.what());
  }
}

void VescDriverNode::leftVescPacketCallback(const boost::shared_ptr<VescPacket const>& packet) {
  if (packet->name() == "Values") {
    // Cast to VescPacketValues to access RPM data
    auto values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    if (!values) return;
    
    // Update left motor state
    left_rpm_ = values->rpm();
    
    // Publish left RPM state
    auto rpm_msg = std_msgs::msg::Float64();
    rpm_msg.data = values->rpm();
    left_rpm_pub_->publish(rpm_msg);
    
    // Update odometry (called from one callback to avoid duplication)
    updateOdometry();
  }
}

void VescDriverNode::rightVescPacketCallback(const boost::shared_ptr<VescPacket const>& packet) {
  if (packet->name() == "Values") {
    // Cast to VescPacketValues to access RPM data
    auto values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    if (!values) return;
    
    // Update right motor state
    right_rpm_ = values->rpm();
    
    // Publish right RPM state
    auto rpm_msg = std_msgs::msg::Float64();
    rpm_msg.data = values->rpm();
    right_rpm_pub_->publish(rpm_msg);
  }
}

void VescDriverNode::updateOdometry() {
  // Calculate linear velocities from ERPM
  double left_velocity = (left_rpm_ - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN;
  double right_velocity = (right_rpm_ - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN;
  
  // Calculate robot velocities from wheel velocities
  double linear_velocity = (left_velocity + right_velocity) / 2.0;
  double angular_velocity = (right_velocity - left_velocity) / wheel_track_;
  
  // Update pose integration
  auto current_time = this->get_clock()->now();
  double dt = (current_time - last_odom_time_).seconds();
  last_odom_time_ = current_time;
  
  // Simple pose integration (odometry)
  double delta_x = linear_velocity * cos(orientation_yaw_) * dt;
  double delta_y = linear_velocity * sin(orientation_yaw_) * dt;
  double delta_yaw = angular_velocity * dt;
  
  position_x_ += delta_x;
  position_y_ += delta_y;
  orientation_yaw_ += delta_yaw;
  
  // Publish odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  
  // Position
  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  
  // Orientation (convert yaw to quaternion)
  odom_msg.pose.pose.orientation.z = sin(orientation_yaw_ / 2.0);
  odom_msg.pose.pose.orientation.w = cos(orientation_yaw_ / 2.0);
  
  // Velocity
  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.angular.z = angular_velocity;
  
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
