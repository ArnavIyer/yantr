#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include <iostream>
#include <cmath>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

namespace vesc_driver {

// Single VESC driver for left or right motor
class SingleVescDriverNode : public rclcpp::Node {
public:
  SingleVescDriverNode() : Node("single_vesc_driver") {
    // Get parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("motor_side", "left");  // "left" or "right"
    
    serial_port_ = this->get_parameter("serial_port").as_string();
    motor_side_ = this->get_parameter("motor_side").as_string();
    
    // Initialize VESC connection
    try {
      vesc_ = std::make_unique<VescInterface>(
        std::string(),
        boost::bind(&SingleVescDriverNode::vescPacketCallback, this, _1)
      );
      
      if (!vesc_->connect(serial_port_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s VESC at %s", 
                     motor_side_.c_str(), serial_port_.c_str());
        return;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "%s VESC initialization failed: %s", 
                   motor_side_.c_str(), e.what());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Connected to %s VESC at %s", 
                motor_side_.c_str(), serial_port_.c_str());
    
    // Create publishers and subscribers
    rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
    
    velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "target_velocity", 10,
        std::bind(&SingleVescDriverNode::velocityCallback, this, std::placeholders::_1));
    
    // Create timer for state requests
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&SingleVescDriverNode::timerCallback, this));
    
    driver_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "%s VESC driver initialized", motor_side_.c_str());
  }

private:
  // Constants - same as original
  static constexpr double SPEED_TO_ERPM_GAIN = 13160.0;
  static constexpr double SPEED_TO_ERPM_OFFSET = 0.0;
  static constexpr double ERPM_SPEED_LIMIT = 26320.0;
  static constexpr double COMMAND_TIMEOUT = 0.5;

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_velocity_ = msg->data;
    t_last_command_ = this->get_clock()->now().seconds();
  }

  void timerCallback() {
    if (!driver_initialized_ || !vesc_->isConnected()) {
      return;
    }
    
    // Check for command timeout
    double current_time = this->get_clock()->now().seconds();
    if (current_time - t_last_command_ > COMMAND_TIMEOUT) {
      target_velocity_ = 0.0;
    }
    
    // Send command
    sendDriveCommand();
    vesc_->requestState();
  }

  void sendDriveCommand() {
    // Convert velocity to ERPM
    double erpm = SPEED_TO_ERPM_GAIN * target_velocity_ + SPEED_TO_ERPM_OFFSET;
    
    // Apply limits
    erpm = std::clamp(erpm, -ERPM_SPEED_LIMIT, ERPM_SPEED_LIMIT);
    
    // Send to VESC
    try {
      vesc_->setSpeed(erpm);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", e.what());
    }
  }

  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet) {
    if (packet->name() == "Values") {
      auto values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);
      if (!values) return;
      
      // Publish RPM
      auto rpm_msg = std_msgs::msg::Float64();
      rpm_msg.data = values->rpm();
      rpm_pub_->publish(rpm_msg);
    }
  }

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpm_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // VESC interface
  std::unique_ptr<VescInterface> vesc_;
  
  // Parameters
  std::string serial_port_;
  std::string motor_side_;
  
  // State
  std::atomic<double> target_velocity_{0.0};
  std::atomic<double> t_last_command_{0.0};
  bool driver_initialized_{false};
};

} // namespace vesc_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_driver::SingleVescDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}