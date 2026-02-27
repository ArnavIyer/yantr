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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

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
  : Node("slam_bridge"),
    first_seen_(false)
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose/sample");
    scan_in_topic_ = declare_parameter<std::string>("scan_in_topic", "/scan");
    scan_out_topic_ = declare_parameter<std::string>("scan_out_topic", "/scan_filtered");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "base_laser");

    lidar_x_ = declare_parameter<double>("lidar_x", 0.30);
    lidar_y_ = declare_parameter<double>("lidar_y", -0.28);
    lidar_z_ = declare_parameter<double>("lidar_z", 0.18);
    lidar_yaw_ = declare_parameter<double>("lidar_yaw", -kPi / 2.0);

    t265_x_ = declare_parameter<double>("t265_x", 0.34);
    t265_y_ = declare_parameter<double>("t265_y", -0.13);

    occlude_below_rad_ = declare_parameter<double>("occlude_below_rad", -85.0 * kPi / 180.0);
    first_velocity_threshold_ = declare_parameter<double>("first_velocity_threshold", 0.02);
    strict_first_velocity_ = declare_parameter<bool>("strict_first_velocity", false);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transform();

    const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    t265_to_robot_ = pose_to_matrix(t265_x_, t265_y_, 0.0);

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
  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = base_frame_;
    t.child_frame_id = laser_frame_;

    t.transform.translation.x = lidar_x_;
    t.transform.translation.y = lidar_y_;
    t.transform.translation.z = lidar_z_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, lidar_yaw_);
    q.normalize();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(t);
  }

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
    const auto & pos = msg->pose.pose.position;
    const auto & ori = msg->pose.pose.orientation;
    const auto & vel = msg->twist.twist.linear;
    const double wz = msg->twist.twist.angular.z;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternion_to_euler(ori, roll, pitch, yaw);

    const auto pose_mat = pose_to_matrix(pos.x, pos.y, yaw);
    const auto odom_to_current_robot = mat_mul(t265_to_robot_, mat_inv(pose_mat));

    if (!first_seen_) {
      first_seen_ = true;
      first_robot_to_odom_ = mat_inv(odom_to_current_robot);

      const bool velocity_ok =
        std::abs(vel.x) < first_velocity_threshold_ &&
        std::abs(vel.y) < first_velocity_threshold_ &&
        std::abs(wz) < first_velocity_threshold_;

      if (!velocity_ok) {
        const auto message =
          "First odom message has nonzero velocity; continuing anyway. "
          "vel: (" + std::to_string(vel.x) + ", " + std::to_string(vel.y) + ", " +
          std::to_string(wz) + ")";

        if (strict_first_velocity_) {
          RCLCPP_FATAL(get_logger(), "%s", message.c_str());
          rclcpp::shutdown();
          return;
        }

        RCLCPP_WARN(get_logger(), "%s", message.c_str());
      }
      return;
    }

    const auto first_robot_to_current_robot = mat_inv(mat_mul(odom_to_current_robot, first_robot_to_odom_));
    const auto robot_pose = matrix_to_pose(first_robot_to_current_robot);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = robot_pose.x;
    t.transform.translation.y = robot_pose.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    q.normalize();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  bool first_seen_;
  Mat3 t265_to_robot_;
  Mat3 first_robot_to_odom_;

  std::string pose_topic_;
  std::string scan_in_topic_;
  std::string scan_out_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;

  double lidar_x_;
  double lidar_y_;
  double lidar_z_;
  double lidar_yaw_;
  double t265_x_;
  double t265_y_;
  double occlude_below_rad_;
  double first_velocity_threshold_;
  bool strict_first_velocity_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
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
