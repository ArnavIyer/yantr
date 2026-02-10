import math
import numpy as np
from scipy.spatial import KDTree
import bisect


def quaternion_to_euler(q):
    """Extract (roll, pitch, yaw) from a quaternion (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def rot2d(radians):
    """
    Generates a 2D counter-clockwise rotation matrix for a given angle in degrees.
    """
    c, s = np.cos(radians), np.sin(radians)
    R = np.array(((c, -s), 
                  (s, c)))
    return R


def pose_to_matrix(x, y, yaw):
    """Build a 3x3 homogeneous transform from (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x],
                     [s,  c, y],
                     [0,  0, 1]])

def matrix_to_pose(mat):
    """Takes a 3x3 homogeneous transform and converts it to (x, y, yaw)"""
    # if sin theta is positive, then, 0 < theta < 180. otherwise, 0 > theta > -180
    heading = (1 if mat[0][1] < 0 else -1) * math.acos(mat[0][0])
    return np.array([mat[0][2], mat[1][2], heading])


def angle_wrap(rad):
    return (rad + np.pi) % (2* np.pi) - np.pi


def interpolate_pose(buffer, t):
    """Interpolate (x, y, yaw, roll, pitch, vx, vy, wz) from the pose buffer at time t.

    Uses binary search on the sorted buffer. Linearly interpolates position
    and velocities and angle-interpolates yaw/roll/pitch. Falls back to nearest
    entry if t is outside the buffer range. Returns None if buffer is empty.
    """
    if len(buffer) == 0:
        return None

    times = [entry[0] for entry in buffer]
    idx = bisect.bisect_left(times, t)

    if idx == 0:
        e = buffer[0]
        return (e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8])
    if idx >= len(buffer):
        e = buffer[-1]
        return (e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8])

    e0 = buffer[idx - 1]
    e1 = buffer[idx]
    dt = e1[0] - e0[0]
    if dt < 1e-9:
        return (e0[1], e0[2], e0[3], e0[4], e0[5], e0[6], e0[7], e0[8])

    alpha = (t - e0[0]) / dt
    x = e0[1] + alpha * (e1[1] - e0[1])
    y = e0[2] + alpha * (e1[2] - e0[2])

    # Angle interpolation for yaw
    dyaw = e1[3] - e0[3]
    # Wrap to [-pi, pi]
    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
    yaw = e0[3] + alpha * dyaw

    # Interpolate roll/pitch similarly
    droll = (e1[4] - e0[4] + math.pi) % (2 * math.pi) - math.pi
    roll = e0[4] + alpha * droll
    dpitch = (e1[5] - e0[5] + math.pi) % (2 * math.pi) - math.pi
    pitch = e0[5] + alpha * dpitch

    vx = e0[6] + alpha * (e1[6] - e0[6])
    vy = e0[7] + alpha * (e1[7] - e0[7])
    wz = e0[8] + alpha * (e1[8] - e0[8])

    return (x, y, yaw, roll, pitch, vx, vy, wz)
