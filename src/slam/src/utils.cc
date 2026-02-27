#include "slam/utils.hpp"

#include <cmath>
#include <stdexcept>

namespace slam
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

void quaternion_to_euler(
  const geometry_msgs::msg::Quaternion & q,
  double & roll,
  double & pitch,
  double & yaw)
{
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  const double w = q.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

Mat3 pose_to_matrix(double x, double y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Mat3{{
    {{c, -s, x}},
    {{s, c, y}},
    {{0.0, 0.0, 1.0}}
  }};
}

Pose2D matrix_to_pose(const Mat3 & mat)
{
  // Matches experimental/utils.py behavior exactly.
  const double heading = (mat[0][1] < 0.0 ? 1.0 : -1.0) * std::acos(mat[0][0]);
  return Pose2D{mat[0][2], mat[1][2], heading};
}

Mat3 mat_mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 out{{
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}}
  }};

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        out[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return out;
}

Mat3 mat_inv(const Mat3 & mat)
{
  const double a = mat[0][0];
  const double b = mat[0][1];
  const double c = mat[1][0];
  const double d = mat[1][1];
  const double tx = mat[0][2];
  const double ty = mat[1][2];

  const double det = a * d - b * c;
  if (std::abs(det) < 1e-12) {
    throw std::runtime_error("matrix is not invertible");
  }

  const double inv_det = 1.0 / det;
  const double r00 = d * inv_det;
  const double r01 = -b * inv_det;
  const double r10 = -c * inv_det;
  const double r11 = a * inv_det;

  const double itx = -(r00 * tx + r01 * ty);
  const double ity = -(r10 * tx + r11 * ty);

  return Mat3{{
    {{r00, r01, itx}},
    {{r10, r11, ity}},
    {{0.0, 0.0, 1.0}}
  }};
}

double angle_wrap(double rad)
{
  double wrapped = std::fmod(rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

}  // namespace slam
