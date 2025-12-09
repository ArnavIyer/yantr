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

#ifndef JOYSTICK__JOYSTICK_NODE_HPP_
#define JOYSTICK__JOYSTICK_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "joystick/joystick_interface.hpp"

namespace joystick
{

class JoystickNode : public rclcpp::Node
{
public:
  JoystickNode();
  
private:
  void timer_callback();
  void apply_deadband(double & value, double threshold) const;
  bool check_safety_timeout() const;
  
  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_vesc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_vesc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr debug_joy_pub_;
  
  // Joystick interface
  std::unique_ptr<JoystickInterface> joystick_;
  
  // Parameters
  std::string joystick_device_;
  double wheel_track_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double deadband_threshold_;
  bool publish_debug_;
  int safety_timeout_ms_;
  
  // Safety and state tracking
  std::chrono::steady_clock::time_point last_input_time_;
  bool safety_enabled_;
};

}  // namespace joystick

#endif  // JOYSTICK__JOYSTICK_NODE_HPP_