import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import subprocess
import math
import numpy as np
from utils import quaternion_to_euler, pose_to_matrix, matrix_to_pose, angle_wrap
from sensor_msgs.msg import LaserScan
import time

# Sensor positions relative to base_link (meters)
LIDAR_X = 0.30           # Lidar x in base_link frame
LIDAR_Y = -0.28          # Lidar y in base_link frame
T265_X  = 0.34           # T265 x in base_link frame
T265_Y  = -0.13          # T265 y in base_link frame

T265_TO_ROBOT = pose_to_matrix(T265_X, T265_Y, 0)

OCCLUDE_BELOW_RAD = math.radians(-85)   # mirrors ros.py exactly


class SlamBridgeNode(Node):
    def __init__(self, bag_proc):
        super().__init__('slam_bridge')
        self._bag_proc = bag_proc
        self._bag_done = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self.first_seen = False
        self.first_robot_to_odom = None

        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_sub = self.create_subscription(
            Odometry, '/pose/sample', self.pose_callback, pose_qos)

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', scan_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)

        self.create_timer(1.0, self._check_bag_done)

    def _publish_static_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'base_laser'      # matches frame_id in LaserScan msgs from bag
        t.transform.translation.x = LIDAR_X
        t.transform.translation.y = LIDAR_Y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(-math.pi / 4)   # -pi/2 around Z
        t.transform.rotation.w = math.cos(-math.pi / 4)
        self.static_tf_broadcaster.sendTransform(t)

    def _check_bag_done(self):
        if self._bag_done:
            return
        if self._bag_proc.poll() is not None:
            self._bag_done = True
            self.get_logger().info('Bag finished. Waiting 3s for slam_toolbox to flush...')
            time.sleep(3)
            self.get_logger().info('Saving map to /app/output/map ...')
            subprocess.run([
                'ros2', 'service', 'call',
                '/slam_toolbox/save_map',
                'slam_toolbox/srv/SaveMap',
                '{"name": {"data": "/app/output/map"}}'
            ])
            self.get_logger().info('Map saved!')
            rclpy.shutdown()

    def scan_callback(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)

        ranges = list(msg.ranges)
        n_filtered = 0
        angle = msg.angle_min
        for i in range(len(ranges)):
            if angle_wrap(angle) < OCCLUDE_BELOW_RAD:
                ranges[i] = float('inf')
                n_filtered += 1
            angle += msg.angle_increment
        filtered.ranges = ranges

        if not hasattr(self, '_scan_logged'):
            self._scan_logged = True
            self.get_logger().info(
                f'scan_callback first call: angle_min={msg.angle_min:.3f} '
                f'angle_max={msg.angle_max:.3f} num_ranges={len(ranges)} '
                f'n_filtered={n_filtered} OCCLUDE_BELOW_RAD={OCCLUDE_BELOW_RAD:.3f}'
            )

        self.scan_pub.publish(filtered)

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(ori)
        vel = msg.twist.twist.linear
        wz = msg.twist.twist.angular.z

        if not self.first_seen:
            self.first_seen = True
            self.first_robot_to_odom = np.linalg.inv(
                T265_TO_ROBOT @ np.linalg.inv(pose_to_matrix(pos.x, pos.y, yaw))
            )
            assert abs(vel.x) < 0.02 and abs(vel.y) < 0.02 and abs(wz) < 0.02, \
                f"First odom message has nonzero velocity, cannot establish initial pose\nvel: ({vel.x}, {vel.y}, {wz})"
            return

        odom_to_current_robot = T265_TO_ROBOT @ np.linalg.inv(pose_to_matrix(pos.x, pos.y, yaw))
        first_robot_to_current_robot = np.linalg.inv(odom_to_current_robot @ self.first_robot_to_odom)
        x, y, theta = matrix_to_pose(first_robot_to_current_robot)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp   # critical: use sensor timestamp, not get_clock().now()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(theta / 2)
        t.transform.rotation.w = math.cos(theta / 2)
        self.tf_broadcaster.sendTransform(t)


SLAM_PARAMS_YAML = """\
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_filtered
    use_map_saver: true
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    restamp_tf: false
    min_laser_range: 0.0
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 14400.
    stack_size_to_use: 40000000
    enable_interactive_mode: false

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.3
    check_min_dist_and_heading_precisely: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
"""


def main(args=None):
    print("Starting SLAM Bridge Node...")

    with open('/tmp/slam_params.yaml', 'w') as f:
        f.write(SLAM_PARAMS_YAML)
    print("Wrote /tmp/slam_params.yaml")

    slam_proc = subprocess.Popen([
        'ros2', 'run', 'slam_toolbox', 'sync_slam_toolbox_node',
        '--ros-args', '--params-file', '/tmp/slam_params.yaml'
    ])
    print("Launched slam_toolbox (sync_slam_toolbox_node)")

    rclpy.init(args=args)

    cmd = "sleep 5 && ros2 bag play bags/rosbag2_2026_01_23-21_02_43 --rate 1"
    bag_proc = subprocess.Popen(cmd, shell=True)

    node = SlamBridgeNode(bag_proc)

    print("Spinning node, broadcasting TF for SLAM Toolbox...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        bag_proc.terminate()
        bag_proc.wait()
        slam_proc.terminate()
        slam_proc.wait()

    print("SLAM Bridge Node and bag process have been shut down.")


if __name__ == '__main__':
    main()
