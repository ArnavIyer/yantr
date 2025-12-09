// Copyright 2017 - 2018 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include "joystick/joystick_node.hpp"

#include <cmath>

namespace joystick
{

JoystickNode::JoystickNode()
: Node("joystick_node"), safety_enabled_(false)
{
  // Declare and get parameters
  this->declare_parameter("joystick_device", "/dev/input/js0");
  this->declare_parameter("wheel_track", 0.3175);
  this->declare_parameter("max_linear_velocity", 1.0);
  this->declare_parameter("max_angular_velocity", 2.0);
  this->declare_parameter("deadband_threshold", 0.05);
  this->declare_parameter("publish_debug", false);
  this->declare_parameter("safety_timeout_ms", 500);

  joystick_device_ = this->get_parameter("joystick_device").as_string();
  wheel_track_ = this->get_parameter("wheel_track").as_double();
  max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  deadband_threshold_ = this->get_parameter("deadband_threshold").as_double();
  publish_debug_ = this->get_parameter("publish_debug").as_bool();
  safety_timeout_ms_ = this->get_parameter("safety_timeout_ms").as_int();

  // Validate parameters
  if (wheel_track_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "wheel_track must be positive, got: %f", wheel_track_);
    throw std::invalid_argument("Invalid wheel_track parameter");
  }
  if (max_linear_velocity_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "max_linear_velocity must be positive, got: %f", max_linear_velocity_);
    throw std::invalid_argument("Invalid max_linear_velocity parameter");
  }
  if (max_angular_velocity_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "max_angular_velocity must be positive, got: %f", max_angular_velocity_);
    throw std::invalid_argument("Invalid max_angular_velocity parameter");
  }

  RCLCPP_INFO(this->get_logger(), "Joystick node initialized with:");
  RCLCPP_INFO(this->get_logger(), "  Device: %s", joystick_device_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Wheel track: %f m", wheel_track_);
  RCLCPP_INFO(this->get_logger(), "  Max linear velocity: %f m/s", max_linear_velocity_);
  RCLCPP_INFO(this->get_logger(), "  Max angular velocity: %f rad/s", max_angular_velocity_);
  RCLCPP_INFO(this->get_logger(), "  Deadband threshold: %f", deadband_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Safety timeout: %d ms", safety_timeout_ms_);

  // Initialize joystick interface
  joystick_ = std::make_unique<JoystickInterface>(joystick_device_);
  if (!joystick_->open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s", joystick_device_.c_str());
    throw std::runtime_error("Failed to open joystick device");
  }

  // Create publishers
  left_vesc_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_vesc/target_velocity", 1);
  right_vesc_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_vesc/target_velocity", 1);
  
  if (publish_debug_) {
    debug_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joystick/status", 1);
  }

  // Create timer (50Hz as recommended in the plan)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&JoystickNode::timer_callback, this));

  last_input_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "Joystick node ready. Press button 7 to enable control.");
}

void JoystickNode::timer_callback()
{
  // Process joystick events
  int events = joystick_->process_events(2);
  if (events < 0) {
    RCLCPP_ERROR(this->get_logger(), "Joystick error occurred");
    return;
  }

  // Check if joystick is still connected
  if (!joystick_->is_open()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Joystick disconnected, sending zero velocities");
    
    // Send zero velocities
    auto left_msg = std_msgs::msg::Float64();
    auto right_msg = std_msgs::msg::Float64();
    left_msg.data = 0.0;
    right_msg.data = 0.0;
    left_vesc_pub_->publish(left_msg);
    right_vesc_pub_->publish(right_msg);
    return;
  }

  // Check safety button (button 7)
  bool safety_button_pressed = (joystick_->get_num_buttons() > 7) && 
                               (joystick_->get_button(7) > 0);
  
  // Update safety state and last input time
  if (safety_button_pressed || events > 0) {
    last_input_time_ = std::chrono::steady_clock::now();
  }
  
  safety_enabled_ = safety_button_pressed;
  
  // Check for safety timeout
  if (!check_safety_timeout()) {
    safety_enabled_ = false;
  }

  // Prepare velocity commands
  double left_velocity = 0.0;
  double right_velocity = 0.0;

  if (safety_enabled_) {
    // Get joystick axis values (normalized to [-1.0, 1.0])
    double raw_linear = (joystick_->get_num_axes() > 3) ? -joystick_->get_axis(3) : 0.0;  // Flip axis 3
    double raw_angular = (joystick_->get_num_axes() > 0) ? -joystick_->get_axis(0) : 0.0;  // Flip axis 0 for intuitive steering

    // Apply deadband filtering
    apply_deadband(raw_linear, deadband_threshold_);
    apply_deadband(raw_angular, deadband_threshold_);

    // Scale to maximum velocities
    double linear_vel = raw_linear * max_linear_velocity_;
    double angular_vel = raw_angular * max_angular_velocity_;

    // Apply differential drive kinematics
    double half_track = wheel_track_ / 2.0;
    left_velocity = linear_vel - (angular_vel * half_track);
    right_velocity = linear_vel + (angular_vel * half_track);

    if (std::abs(raw_linear) > deadband_threshold_ || std::abs(raw_angular) > deadband_threshold_) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Joy input: linear=%.3f, angular=%.3f -> left_vel=%.3f, right_vel=%.3f",
                   linear_vel, angular_vel, left_velocity, right_velocity);
    }
  }

  // Publish velocity commands
  auto left_msg = std_msgs::msg::Float64();
  auto right_msg = std_msgs::msg::Float64();
  left_msg.data = left_velocity;
  right_msg.data = right_velocity;
  left_vesc_pub_->publish(left_msg);
  right_vesc_pub_->publish(right_msg);

  // Publish debug joy message if enabled
  if (publish_debug_ && debug_joy_pub_) {
    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "joystick";
    
    std::vector<float> axes;
    std::vector<int> buttons;
    joystick_->get_all_axes(axes);
    joystick_->get_all_buttons(buttons);
    
    joy_msg.axes = axes;
    joy_msg.buttons = buttons;
    debug_joy_pub_->publish(joy_msg);
  }

  // Log safety status changes
  static bool last_safety_enabled = false;
  if (safety_enabled_ != last_safety_enabled) {
    if (safety_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Safety enabled - joystick control active");
    } else {
      RCLCPP_INFO(this->get_logger(), "Safety disabled - motors stopped");
    }
    last_safety_enabled = safety_enabled_;
  }
}

void JoystickNode::apply_deadband(double & value, double threshold) const
{
  if (std::abs(value) < threshold) {
    value = 0.0;
  }
}

bool JoystickNode::check_safety_timeout() const
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_time_);
  return elapsed.count() < safety_timeout_ms_;
}

}  // namespace joystick