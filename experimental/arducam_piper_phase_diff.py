from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image


# HSV bounds tuned from observed green-cube pixels in
# rosbag2_arducam_piper_phase_diff (H ~50-83, S ~30-244, V ~62-226).
GREEN_HSV_LOWER = np.array([45, 40, 50], dtype=np.uint8)
GREEN_HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)
MIN_BLOB_AREA = 100  # px; below this treat as off-screen / occluded


class APSubscriber(Node):
    def __init__(self):
        super().__init__('ardu_piper_subscriber')
        self.ardu_sub = self.create_subscription(
            Image,
            '/arducam/image_raw',
            self.ardu_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/piper/joint_states',
            self.joint_callback,
            10
        )
        self.data = {
            "ardu": [],
            "piper": [],
        }
        self._ardu_frames_seen = 0  # includes frames where cube wasn't detected

    @staticmethod
    def _stamp_to_sec(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def _calculate_green_pos(msg):
        """Return normalized x-position [0, 1] of the green cube, or None if not detected."""
        # Arducam publishes rgb8, 640x480. Convert to BGR for cv2.
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = img[..., ::-1].copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_HSV_LOWER, GREEN_HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n_labels <= 1:
            return None
        # stats[0] is background; pick largest of the remaining blobs
        areas = stats[1:, cv2.CC_STAT_AREA]
        i = 1 + int(np.argmax(areas))
        if stats[i, cv2.CC_STAT_AREA] < MIN_BLOB_AREA:
            return None
        ys, xs = np.where(labels == i)
        return float(xs.mean()) / msg.width

    @staticmethod
    def _get_j1_pos(msg) -> float:
        return float(msg.position[0])

    def ardu_callback(self, msg):
        self._ardu_frames_seen += 1
        x = self._calculate_green_pos(msg)
        if x is not None:
            self.data["ardu"].append((self._stamp_to_sec(msg), x))
        if self._ardu_frames_seen == 1 or self._ardu_frames_seen % 20 == 0:
            self.get_logger().info(
                f"arducam: {self._ardu_frames_seen} frames received "
                f"({len(self.data['ardu'])} with cube detected)"
            )

    def joint_callback(self, msg):
        self.data["piper"].append((self._stamp_to_sec(msg), self._get_j1_pos(msg)))
        n = len(self.data["piper"])
        if n == 1 or n % 200 == 0:
            self.get_logger().info(f"piper: {n} samples received")

    def calculate_phase_diff(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        n_ardu_detected = len(self.data["ardu"])
        n_piper = len(self.data["piper"])
        print(
            f"\n[calc] arducam: {self._ardu_frames_seen} frames seen, "
            f"{n_ardu_detected} with cube "
            f"(detection rate {100.0 * n_ardu_detected / max(1, self._ardu_frames_seen):.1f}%)"
        )
        print(f"[calc] piper:   {n_piper} samples")

        if n_ardu_detected < 10 or n_piper < 10:
            self.get_logger().warn("Not enough data to compute phase diff")
            return

        ardu = np.array(self.data["ardu"])
        piper = np.array(self.data["piper"])

        t0 = max(ardu[0, 0], piper[0, 0])
        t1 = min(ardu[-1, 0], piper[-1, 0])
        if t1 <= t0:
            self.get_logger().warn("No time overlap between streams")
            return

        ardu_t = ardu[:, 0] - t0
        ardu_x = ardu[:, 1]
        piper_t = piper[:, 0] - t0
        piper_j1 = piper[:, 1]
        duration = t1 - t0
        print(
            f"[calc] overlap window: {duration:.2f} s  "
            f"(arducam ~{n_ardu_detected / duration:.1f} Hz detected, "
            f"piper ~{n_piper / duration:.1f} Hz)"
        )

        # Keep only ardu samples within the piper-covered interval so np.interp
        # can always give a valid value. τ-sweep below further restricts this.
        in_window = (ardu_t >= piper_t[0]) & (ardu_t <= piper_t[-1])
        ardu_t = ardu_t[in_window]
        ardu_x = ardu_x[in_window]

        def normalize(x):
            return (x - x.mean()) / (x.std() + 1e-9)

        # Sign check at τ=0. If the two signals are anti-correlated (j1 direction
        # vs cube-x are inverted), flip arducam so MSE finds the in-phase match.
        piper_at_ardu = np.interp(ardu_t, piper_t, piper_j1)
        corr = np.corrcoef(normalize(ardu_x), normalize(piper_at_ardu))[0, 1]
        if corr < 0:
            print(f"[calc] τ=0 correlation = {corr:.3f} (negative) → flipping arducam sign")
            ardu_x = -ardu_x
        else:
            print(f"[calc] τ=0 correlation = {corr:.3f}")

        dt = 0.01
        tau_max = 1.0
        taus = np.arange(-tau_max, tau_max + dt, dt)
        print(f"[calc] sweeping τ over ±{tau_max * 1000:.0f} ms in {dt * 1000:.0f} ms steps...")

        errors = np.full_like(taus, np.nan)
        for i, tau in enumerate(taus):
            t_lookup = ardu_t - tau
            valid = (t_lookup >= piper_t[0]) & (t_lookup <= piper_t[-1])
            if valid.sum() < 10:
                continue
            piper_shifted = np.interp(t_lookup[valid], piper_t, piper_j1)
            a = normalize(ardu_x[valid])
            b = normalize(piper_shifted)
            errors[i] = np.mean((a - b) ** 2)

        best_i = int(np.nanargmin(errors))
        optimal_tau = float(taus[best_i])

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot both normalized signals on a common time axis.
        piper_n = normalize(piper_j1)
        ardu_n = normalize(ardu_x)
        ax1.plot(piper_t, piper_n, label="piper j1 (normalized)", linewidth=1.0, alpha=0.8)
        ax1.plot(ardu_t, ardu_n, "o-", label="arducam cube x (normalized)",
                 markersize=2, linewidth=0.8, alpha=0.9)
        ax1.set_xlabel("time [s]")
        ax1.set_ylabel("normalized signal")
        ax1.set_title("j1 readback vs green cube x-position")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(taus * 1000.0, errors)
        ax2.axvline(
            optimal_tau * 1000.0,
            color="red",
            linestyle="--",
            label=f"optimal τ = {optimal_tau * 1000.0:.1f} ms",
        )
        ax2.set_xlabel("phase shift τ [ms]  (positive: arducam lags piper readback)")
        ax2.set_ylabel("MSE after shifting")
        ax2.set_title("Phase-shift error sweep (±1000 ms)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()
        out_path = Path("~/yantr/experimental/ardu_piper_phase_diff.png").expanduser()
        fig.savefig(out_path, dpi=120)
        self.get_logger().info(f"Saved phase plot to {out_path}")
        self.get_logger().info(f"Optimal phase shift: {optimal_tau * 1000.0:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    ap_subscriber = APSubscriber()

    print("Subscribed to /arducam/image_raw and /piper/joint_states")
    print("Play the rosbag, then Ctrl+C to compute the phase diff.\n")

    try:
        rclpy.spin(ap_subscriber)
    except KeyboardInterrupt:
        print("\nShutting down, computing phase diff...")
    finally:
        ap_subscriber.calculate_phase_diff()

        ap_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
