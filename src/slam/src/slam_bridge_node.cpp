#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "slam/utils.hpp"

namespace slam
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class SlamBridgeNode : public rclcpp::Node
{
public:
  SlamBridgeNode()
  : Node("slam_bridge")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose/sample");
    pose_frame_ = declare_parameter<std::string>("pose_frame", "");
    scan_in_topic_ = declare_parameter<std::string>("scan_in_topic", "/scan");
    scan_out_topic_ = declare_parameter<std::string>("scan_out_topic", "/scan_filtered");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto publish_laser_tf = declare_parameter<bool>("publish_laser_tf", false);

    occlude_below_rad_ = declare_parameter<double>("occlude_below_rad", -85.0 * kPi / 180.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (publish_laser_tf) {
      RCLCPP_WARN(
        get_logger(),
        "publish_laser_tf is deprecated; publish base_link -> base_laser from the URDF instead.");
    }

    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_out_topic_, scan_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_in_topic_, scan_qos,
      std::bind(&SlamBridgeNode::scan_callback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, pose_qos,
      std::bind(&SlamBridgeNode::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "slam_bridge started. pose_topic=%s scan_in=%s scan_out=%s",
      pose_topic_.c_str(), scan_in_topic_.c_str(), scan_out_topic_.c_str());
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan filtered;
    filtered.header = msg->header;
    filtered.angle_min = msg->angle_min;
    filtered.angle_max = msg->angle_max;
    filtered.angle_increment = msg->angle_increment;
    filtered.time_increment = msg->time_increment;
    filtered.scan_time = msg->scan_time;
    filtered.range_min = msg->range_min;
    filtered.range_max = msg->range_max;

    filtered.ranges = msg->ranges;

    double angle = msg->angle_min;
    for (auto & range : filtered.ranges) {
      if (angle_wrap(angle) < occlude_below_rad_) {
        range = std::numeric_limits<float>::infinity();
      }
      angle += msg->angle_increment;
    }

    filtered.intensities = msg->intensities;
    scan_pub_->publish(filtered);
  }

  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto odom_frame = odom_frame_.empty() ? msg->header.frame_id : odom_frame_;
    const auto source_pose_frame = pose_frame_.empty() ? msg->child_frame_id : pose_frame_;
    if (odom_frame.empty() || source_pose_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot publish odom -> base_link without odom and T265 pose frame names.");
      return;
    }

    geometry_msgs::msg::TransformStamped base_to_pose_msg;
    try {
      base_to_pose_msg = tf_buffer_->lookupTransform(
        base_frame_, source_pose_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for static transform %s -> %s: %s",
        base_frame_.c_str(), source_pose_frame.c_str(), ex.what());
      return;
    }

    tf2::Transform odom_to_pose;
    tf2::Transform base_to_pose;
    tf2::fromMsg(msg->pose.pose, odom_to_pose);
    tf2::fromMsg(base_to_pose_msg.transform, base_to_pose);
    const tf2::Transform odom_to_base = odom_to_pose * base_to_pose.inverse();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_frame;
    t.child_frame_id = base_frame_;
    t.transform = tf2::toMsg(odom_to_base);

    tf_broadcaster_->sendTransform(t);
  }

  std::string pose_topic_;
  std::string pose_frame_;
  std::string scan_in_topic_;
  std::string scan_out_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double occlude_below_rad_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam::SlamBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
