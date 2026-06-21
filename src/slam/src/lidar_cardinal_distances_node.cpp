#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "slam/utils.hpp"

namespace slam
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Direction
{
  const char * name;
  double angle_rad;
};

constexpr std::array<Direction, 3> kDirections{{
  {"front", kPi / 2.0},
  {"left", kPi},
  {"back", -kPi / 2.0},
}};

bool valid_range(const sensor_msgs::msg::LaserScan & scan, float range)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

std::string format_distance(float distance)
{
  if (!std::isfinite(distance)) {
    return "nan";
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << distance;
  return out.str();
}
}  // namespace

class LidarCardinalDistancesNode : public rclcpp::Node
{
public:
  LidarCardinalDistancesNode()
  : Node("lidar_cardinal_distances")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar_cardinal_distances");
    angle_window_rad_ =
      declare_parameter<double>("angle_window_deg", 2.0) * kPi / 180.0;
    publish_period_s_ = declare_parameter<double>("publish_period_s", 3.0);

    if (angle_window_rad_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "angle_window_deg must be positive; using 2 degrees");
      angle_window_rad_ = 2.0 * kPi / 180.0;
    }

    if (publish_period_s_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_period_s must be positive; using 3 seconds");
      publish_period_s_ = 3.0;
    }

    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
      });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_period_s_)),
      std::bind(&LidarCardinalDistancesNode::publish_distances, this));

    RCLCPP_INFO(
      get_logger(),
      "lidar_cardinal_distances started. scan_topic=%s output_topic=%s period=%.3fs window=%.3fdeg",
      scan_topic_.c_str(), output_topic_.c_str(), publish_period_s_,
      angle_window_rad_ * 180.0 / kPi);
  }

private:
  float distance_near_angle(const sensor_msgs::msg::LaserScan & scan, double target_angle) const
  {
    if (scan.ranges.empty() || scan.angle_increment == 0.0f) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    float closest = std::numeric_limits<float>::infinity();
    double angle = scan.angle_min;

    for (const auto range : scan.ranges) {
      const double delta = std::abs(angle_wrap(angle - target_angle));
      if (delta <= angle_window_rad_ && valid_range(scan, range) && range < closest) {
        closest = range;
      }
      angle += scan.angle_increment;
    }

    if (std::isfinite(closest)) {
      return closest;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  void publish_distances()
  {
    if (!latest_scan_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Waiting for LaserScan messages on %s", scan_topic_.c_str());
      return;
    }

    std::array<float, kDirections.size()> distances{};
    for (std::size_t i = 0; i < kDirections.size(); ++i) {
      distances[i] = distance_near_angle(*latest_scan_, kDirections[i].angle_rad);
    }

    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "front_m=" << format_distance(distances[0])
        << " left_m=" << format_distance(distances[1])
        << " back_m=" << format_distance(distances[2])
        << " frame=" << latest_scan_->header.frame_id;
    msg.data = out.str();

    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
  }

  std::string scan_topic_;
  std::string output_topic_;
  double angle_window_rad_;
  double publish_period_s_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam::LidarCardinalDistancesNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
