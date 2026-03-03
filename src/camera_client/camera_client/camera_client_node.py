import math

import cv2
import rclpy
import requests
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


class CameraClientNode(Node):
    def __init__(self):
        super().__init__("camera_client_node")

        self.declare_parameter("api_url", "http://localhost:8000")
        self.declare_parameter("send_distance_m", 0.25)

        self.api_url = self.get_parameter("api_url").get_parameter_value().string_value
        self.send_distance_m = (
            self.get_parameter("send_distance_m").get_parameter_value().double_value
        )

        self.bridge = CvBridge()

        # Latest messages
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_info = None
        self.latest_depth_info = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Track last position where we sent an image
        self.last_send_x = None
        self.last_send_y = None

        # QoS for camera topics
        camera_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.create_subscription(
            Image, "/camera/color/image_raw", self._color_cb, camera_qos
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self._depth_cb, camera_qos
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._color_info_cb, camera_qos
        )
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._depth_info_cb, camera_qos
        )

        # 5 Hz timer to check distance
        self.create_timer(0.2, self._check_distance)

        self.get_logger().info(
            f"Camera client started — api_url={self.api_url} "
            f"send_distance_m={self.send_distance_m}"
        )

    # ── Subscription callbacks ──────────────────────────────────────────

    def _color_cb(self, msg: Image):
        self.latest_color = msg

    def _depth_cb(self, msg: Image):
        self.latest_depth = msg

    def _color_info_cb(self, msg: CameraInfo):
        self.latest_color_info = msg

    def _depth_info_cb(self, msg: CameraInfo):
        self.latest_depth_info = msg

    # ── Distance-based trigger ──────────────────────────────────────────

    def _check_distance(self):
        # Look up odom -> base_link
        try:
            t = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            self.get_logger().debug(
                "TF lookup odom->base_link failed (slam_toolbox not running yet?)"
            )
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        # First position — initialise and skip
        if self.last_send_x is None:
            self.last_send_x = x
            self.last_send_y = y
            return

        dx = x - self.last_send_x
        dy = y - self.last_send_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.send_distance_m:
            return

        # Distance threshold reached — send image
        if self.latest_color is None:
            self.get_logger().debug("No color frame received yet, skipping send")
            return

        self._send_image()
        self.last_send_x = x
        self.last_send_y = y

    # ── Image upload ────────────────────────────────────────────────────

    def _send_image(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_color, "bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge conversion failed: {e}")
            return

        success, jpeg_buf = cv2.imencode(".jpg", cv_image)
        if not success:
            self.get_logger().warning("JPEG encoding failed")
            return

        try:
            resp = requests.post(
                f"{self.api_url}/upload-image",
                files={"file": ("frame.jpg", jpeg_buf.tobytes(), "image/jpeg")},
                timeout=5,
            )
            self.get_logger().info(
                f"Uploaded image — status={resp.status_code} "
                f"size={len(jpeg_buf.tobytes())} bytes"
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().warning(f"API request failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
