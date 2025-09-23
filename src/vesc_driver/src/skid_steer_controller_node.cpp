#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

namespace vesc_driver {

class SkidSteerControllerNode : public rclcpp::Node {
public:
  SkidSteerControllerNode() : Node("skid_steer_controller"),
                              position_x_(0.0),
                              position_y_(0.0),
                              orientation_yaw_(0.0),
                              last_odom_time_(this->get_clock()->now()) {
    
    // Robot parameters
    this->declare_parameter("wheel_track", 0.3175);  // Distance between wheels on same axle
    wheel_track_ = this->get_parameter("wheel_track").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Skid-steer controller initialized with wheel_track=%.2f m", wheel_track_);
    
    // Publishers - send target velocities to individual VESC nodes
    left_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_vesc/target_velocity", 10);
    right_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_vesc/target_velocity", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Subscribers - receive cmd_vel and individual motor RPMs
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SkidSteerControllerNode::cmdVelCallback, this, std::placeholders::_1));
    
    left_rpm_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "left_vesc/rpm", 10,
        std::bind(&SkidSteerControllerNode::leftRpmCallback, this, std::placeholders::_1));
        
    right_rpm_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "right_vesc/rpm", 10,
        std::bind(&SkidSteerControllerNode::rightRpmCallback, this, std::placeholders::_1));
    
    // Timer for odometry updates
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&SkidSteerControllerNode::updateOdometry, this));
  }

private:
  // Robot parameters
  static constexpr double SPEED_TO_ERPM_GAIN = 13160.0;
  static constexpr double SPEED_TO_ERPM_OFFSET = 0.0;
  
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Calculate differential drive wheel velocities
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    
    double half_track = wheel_track_ / 2.0;
    double left_velocity = linear_vel - (angular_vel * half_track);
    double right_velocity = linear_vel + (angular_vel * half_track);
    
    // Publish target velocities to individual VESC nodes
    auto left_msg = std_msgs::msg::Float64();
    auto right_msg = std_msgs::msg::Float64();
    left_msg.data = left_velocity;
    right_msg.data = right_velocity;
    
    left_velocity_pub_->publish(left_msg);
    right_velocity_pub_->publish(right_msg);
    
    // Debug output
    RCLCPP_DEBUG(this->get_logger(), "Cmd: linear=%.2f, angular=%.2f -> left=%.2f, right=%.2f",
                 linear_vel, angular_vel, left_velocity, right_velocity);
  }
  
  void leftRpmCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    left_rpm_ = msg->data;
  }
  
  void rightRpmCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    right_rpm_ = msg->data;
  }
  
  void updateOdometry() {
    // Convert RPM to velocities
    double left_velocity = (left_rpm_ - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN;
    double right_velocity = (right_rpm_ - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN;
    
    // Calculate robot velocities from wheel velocities
    double linear_velocity = (left_velocity + right_velocity) / 2.0;
    double angular_velocity = (right_velocity - left_velocity) / wheel_track_;
    
    // Update pose integration
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;
    
    if (dt > 0.0) {
      // Simple pose integration
      double delta_x = linear_velocity * cos(orientation_yaw_) * dt;
      double delta_y = linear_velocity * sin(orientation_yaw_) * dt;
      double delta_yaw = angular_velocity * dt;
      
      position_x_ += delta_x;
      position_y_ += delta_y;
      orientation_yaw_ += delta_yaw;
    }
    
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

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_rpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_rpm_sub_;
  
  rclcpp::TimerBase::SharedPtr odom_timer_;
  
  // State
  double wheel_track_;
  std::atomic<double> left_rpm_{0.0};
  std::atomic<double> right_rpm_{0.0};
  
  // Odometry
  double position_x_;
  double position_y_;  
  double orientation_yaw_;
  rclcpp::Time last_odom_time_;
};

} // namespace vesc_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_driver::SkidSteerControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}