import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import subprocess
import math
import numpy as np
from collections import deque
import pickle
from utils import quaternion_to_euler, interpolate_pose, pose_to_matrix, rot2d, angle_wrap, matrix_to_pose

# --- Tunable constants ---
LIDAR_TIME_OFFSET_SEC = 0.0      # Shift lidar timestamps: corrected = stamp + offset
POSE_TO_LIDAR_YAW_OFFSET = 0.0   # Rotation from T265 frame to lidar frame (radians)

# Sensor positions relative to base_link (meters)
LIDAR_X = 0.30           # Lidar x in base_link frame
LIDAR_Y = -0.28          # Lidar y in base_link frame
T265_X  = 0.34           # T265 x in base_link frame
T265_Y  = -0.13          # T265 y in base_link frame
LIDAR_FRAME = pose_to_matrix(LIDAR_X, LIDAR_Y, -math.pi/2)
T265_TO_ROBOT = pose_to_matrix(T265_X, T265_Y, 0)

POSE_BUFFER_SIZE = 1000           # ~5 seconds at 200 Hz

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.data = {
            'raw_clouds': [],
            'scan_stamps': [],
            't265_pose': [],
            'pose_count': 0,
        }
        
        self.first_seen = False
        self.first_robot_to_odom = None

        # Pose buffer: deque of (t, x, y, yaw, roll, pitch, vx, vy, wz)
        self.pose_buffer = deque(maxlen=POSE_BUFFER_SIZE)
        self.pose_count = 0

        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_sub = self.create_subscription(
            Odometry, '/pose/sample', self.pose_callback, pose_qos)

    def pose_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(ori)
        vel = msg.twist.twist.linear
        wz = msg.twist.twist.angular.z

        if not self.first_seen:
            self.first_seen = True
            self.first_robot_to_odom = np.linalg.inv(T265_TO_ROBOT @ np.linalg.inv(pose_to_matrix(pos.x, pos.y, yaw)))
            assert abs(vel.x) < 0.02 and abs(vel.y) < 0.02 and abs(wz) < 0.02, f"First odom message has nonzero velocity, cannot establish initial pose\nvel: ({vel.x}, {vel.y}, {wz})"
            self.pose_buffer.append((t, 0.0, 0.0, 0.0, roll, pitch, vel.x, vel.y, wz))
            return
        
        odom_to_current_robot = T265_TO_ROBOT @ np.linalg.inv(pose_to_matrix(pos.x, pos.y, yaw))
        first_robot_to_current_robot = np.linalg.inv(odom_to_current_robot @ self.first_robot_to_odom)
        x, y, theta = matrix_to_pose(first_robot_to_current_robot)

        # this velocity should be from the perspective of the robot currently, not the frame of the robot start.
        # TODO confirm that the T265 gives velocity in globalodom frame instead of robot frame
        rot_vel = rot2d(-yaw) @ np.array([vel.x, vel.y])

        self.pose_buffer.append((t, x, y, theta, roll, pitch, rot_vel[0], rot_vel[1], wz))

        self.pose_count += 1
        self.data['pose_count'] = self.pose_count
        if self.pose_count % 200 == 0:
            print(f"[T265] Received {self.pose_count} pose messages")


    def scan_callback(self, msg):
        print(f"Received LaserScan message with {len(msg.ranges)} ranges")
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamp_sec += LIDAR_TIME_OFFSET_SEC
        self.data['scan_stamps'].append(stamp_sec)

        # 1) Convert polar -> Cartesian and filter robot body
        raw_cloud = []
        curr_angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > msg.range_min and r < msg.range_max:

                # the robot's body is in the third quadrant of the lidar, and there's a wire poking out
                if angle_wrap(curr_angle) < np.deg2rad(-85):
                    continue

                x = math.cos(curr_angle) * r
                y = math.sin(curr_angle) * r
                
                # translate points in cloud from lidar frame to robot frame
                # relative to base link, the lidar's frame is a 90 CW rotation and a (LIDAR_X, LIDAR_Y) translation
                raw_cloud.append((LIDAR_FRAME @ np.array([x, y, 1]))[:2])

            curr_angle += msg.angle_increment
        self.data['raw_clouds'].append(raw_cloud)

        # fetch the realsense pose that is said to have happened at stamp_sec
        self.data['t265_pose'].append(interpolate_pose(self.pose_buffer, stamp_sec))

        print("done processing scan")


def main(args=None):
    print("Starting Scan Subscriber Node...")
    rclpy.init(args=args)
    scan_sub = ScanSubscriber()

    cmd = "sleep 2 && ros2 bag play bags/rosbag2_2026_01_23-21_02_43 --rate 1"
    proc = subprocess.Popen(cmd, shell=True)

    print("Spinning node to listen for LaserScan messages...")
    try:
        rclpy.spin(scan_sub)
    except KeyboardInterrupt:
        pass
    finally:
        output_path = '/app/output/scan_data.pkl'
        data = scan_sub.data
        with open(output_path, 'wb') as f:
            pickle.dump(data, f)
        print(f"Saved {len(scan_sub.data['raw_clouds'])} clouds to {output_path}")
        print(f"Received {scan_sub.data['pose_count']} total pose messages")

        scan_sub.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        proc.wait()

    print("Scan Subscriber Node and bag process has been shut down.")

if __name__ == '__main__':
    main()
