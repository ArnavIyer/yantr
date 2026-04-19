"""Read-only Piper joint state publisher.

Opens a passive C_PiperInterface_V2 handle alongside the active gello teleop
process and republishes joint positions as sensor_msgs/JointState for rosbag
recording. Never sends control frames — safe to run concurrently with gello.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper_sdk import C_PiperInterface_V2


DEG_TO_RAD = 0.001 * math.pi / 180.0  # Piper SDK returns degrees * 1000


class PiperStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("piper_state_publisher")

        self.declare_parameter("can_port", "can0")
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("include_gripper", True)
        self.declare_parameter("topic", "/piper/joint_states")

        can_port = self.get_parameter("can_port").value
        hz = float(self.get_parameter("publish_hz").value)
        self._include_gripper = bool(self.get_parameter("include_gripper").value)
        topic = self.get_parameter("topic").value

        self.get_logger().info(f"Connecting to Piper on {can_port} (read-only)")
        self._piper = C_PiperInterface_V2(can_name=can_port)
        self._piper.ConnectPort()

        self._joint_names = [f"joint_{i + 1}" for i in range(6)]
        if self._include_gripper:
            self._joint_names.append("gripper")

        self._pub = self.create_publisher(JointState, topic, 50)
        self._timer = self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(f"Publishing {topic} at {hz} Hz")

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        joint_data = self._piper.GetArmJointMsgs().joint_state

        positions = [
            joint_data.joint_1 * DEG_TO_RAD,
            joint_data.joint_2 * DEG_TO_RAD,
            joint_data.joint_3 * DEG_TO_RAD,
            joint_data.joint_4 * DEG_TO_RAD,
            joint_data.joint_5 * DEG_TO_RAD,
            joint_data.joint_6 * DEG_TO_RAD,
        ]

        if self._include_gripper:
            gripper_um = self._piper.GetArmGripperMsgs().gripper_state.grippers_angle
            positions.append(gripper_um / 1_000_000.0)

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self._joint_names
        msg.position = positions
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PiperStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
