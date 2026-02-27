#pragma once

#include <array>

#include "geometry_msgs/msg/quaternion.hpp"

namespace slam
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

using Mat3 = std::array<std::array<double, 3>, 3>;

void quaternion_to_euler(
  const geometry_msgs::msg::Quaternion & q,
  double & roll,
  double & pitch,
  double & yaw);

Mat3 pose_to_matrix(double x, double y, double yaw);
Pose2D matrix_to_pose(const Mat3 & mat);
Mat3 mat_mul(const Mat3 & a, const Mat3 & b);
Mat3 mat_inv(const Mat3 & mat);
double angle_wrap(double rad);

}  // namespace slam
