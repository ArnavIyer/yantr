import os
import time
import subprocess

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

BAG_PATH = "/app/bags/slam_map_bag_20260226_211739"
OUTPUT_DIR = "/app/output/map_vis"
SAVE_PERIOD_SEC = 2.0


class MapVisualizer(Node):
    def __init__(self, bag_proc):
        super().__init__("map_visualizer")
        self._bag_proc = bag_proc
        self._latest_map = None
        self._saved_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos)
        self.create_timer(SAVE_PERIOD_SEC, self.save_map_image)
        self.create_timer(0.5, self.check_bag_done)

    def map_callback(self, msg):
        self._latest_map = msg

    def save_map_image(self):
        if self._latest_map is None:
            return

        msg = self._latest_map
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        image = np.full((height, width), 127, dtype=np.uint8)
        known = data >= 0
        image[known] = np.clip(255 - (data[known] * 255 // 100), 0, 255)

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        ts = int(time.time() * 1000)
        out_path = os.path.join(OUTPUT_DIR, f"map_{ts}.png")

        plt.imsave(out_path, np.flipud(image), cmap="gray", vmin=0, vmax=255)

        self._saved_count += 1
        self.get_logger().info(f"Saved map image #{self._saved_count}: {out_path}")

    def check_bag_done(self):
        if self._bag_proc.poll() is None:
            return
        self.get_logger().info("Bag playback finished. Waiting 2s before shutdown...")
        time.sleep(2)
        rclpy.shutdown()


def main(args=None):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    bag_proc = subprocess.Popen([
        "ros2",
        "bag",
        "play",
        BAG_PATH,
    ])

    rclpy.init(args=args)
    node = MapVisualizer(bag_proc)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if bag_proc.poll() is None:
            bag_proc.terminate()
            bag_proc.wait()


if __name__ == "__main__":
    main()
