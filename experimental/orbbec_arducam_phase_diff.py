from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CamSubscriber(Node):
    def __init__(self):
        super().__init__('cam_subscriber')
        self.orbbec_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.orbbec_callback,
            10
        )
        self.ardu_sub = self.create_subscription(
            Image,
            '/arducam/image_raw',
            self.ardu_callback,
            10
        )
        self.data = {
            "ardu": [],
            "orbbec": []
        }

    @staticmethod
    def _stamp_to_sec(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def _brightness(msg) -> float:
        # Mean of raw bytes — works for BGR8/RGB8/Mono8 uncompressed Image.
        # Tracks scene-wide light level, which is what the light-toggling signal modulates.
        return float(np.frombuffer(msg.data, dtype=np.uint8).mean())

    def orbbec_callback(self, msg):
        self.data["orbbec"].append((self._stamp_to_sec(msg), self._brightness(msg)))
        n = len(self.data["orbbec"])
        if n == 1 or n % 20 == 0:
            self.get_logger().info(f"orbbec: {n} frames received")

    def ardu_callback(self, msg):
        self.data["ardu"].append((self._stamp_to_sec(msg), self._brightness(msg)))
        n = len(self.data["ardu"])
        if n == 1 or n % 20 == 0:
            self.get_logger().info(f"arducam: {n} frames received")

    def calculate_phase_diff(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        print(
            f"\n[calc] orbbec frames: {len(self.data['orbbec'])}, "
            f"arducam frames: {len(self.data['ardu'])}"
        )

        if not self.data["orbbec"] or not self.data["ardu"]:
            self.get_logger().warn("No data captured on one or both topics")
            return

        orb = np.array(self.data["orbbec"])
        ard = np.array(self.data["ardu"])

        # Restrict to overlapping time window.
        t0 = max(orb[0, 0], ard[0, 0])
        t1 = min(orb[-1, 0], ard[-1, 0])
        if t1 <= t0:
            self.get_logger().warn("No time overlap between streams")
            return

        orb_t = orb[:, 0] - t0
        ard_t = ard[:, 0] - t0
        duration = t1 - t0
        print(
            f"[calc] overlapping window: {duration:.2f} s  "
            f"(orbbec ~{len(orb) / duration:.1f} Hz, arducam ~{len(ard) / duration:.1f} Hz)"
        )

        # Resample both streams onto a common 100 Hz grid.
        dt = 0.01
        grid = np.arange(0.0, duration, dt)
        orb_b = np.interp(grid, orb_t, orb[:, 1])
        ard_b = np.interp(grid, ard_t, ard[:, 1])

        # Zero-mean + unit-std so the MSE compares *shape*, not absolute brightness
        # (the two cameras have different exposures/gains).
        def normalize(x):
            return (x - x.mean()) / (x.std() + 1e-9)

        orb_n = normalize(orb_b)
        ard_n = normalize(ard_b)

        # Sweep phase shift τ. Positive τ means arducam lags orbbec by τ seconds,
        # so we shift the orbbec signal forward and compare to arducam.
        tau_max = min(1.0, duration / 4.0)
        taus = np.arange(-tau_max, tau_max + dt, dt)
        print(f"[calc] sweeping τ over ±{tau_max * 1000:.0f} ms in {dt * 1000:.0f} ms steps...")
        errors = np.empty_like(taus)
        for i, tau in enumerate(taus):
            shift = int(round(tau / dt))
            if shift >= 0:
                a = orb_n[shift:]
                b = ard_n[: len(a)]
            else:
                b = ard_n[-shift:]
                a = orb_n[: len(b)]
            errors[i] = np.mean((a - b) ** 2)

        optimal_tau = float(taus[int(np.argmin(errors))])

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        ax1.plot(grid, orb_b, label="orbbec", linewidth=1.2)
        ax1.plot(grid, ard_b, label="arducam", linewidth=1.2)
        ax1.set_xlabel("time [s]")
        ax1.set_ylabel("mean pixel brightness")
        ax1.set_title("Camera brightness signals (overlapping window)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(taus * 1000.0, errors)
        ax2.axvline(
            optimal_tau * 1000.0,
            color="red",
            linestyle="--",
            label=f"optimal τ = {optimal_tau * 1000.0:.1f} ms",
        )
        ax2.set_xlabel("phase shift τ [ms]  (positive: arducam lags orbbec)")
        ax2.set_ylabel("MSE after shifting")
        ax2.set_title("Phase-shift error sweep")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()
        out_path = Path("~/yantr/experimental/cam_phase_diff.png").expanduser()
        fig.savefig(out_path, dpi=120)
        self.get_logger().info(f"Saved phase plot to {out_path}")
        self.get_logger().info(f"Optimal phase shift: {optimal_tau * 1000.0:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    cam_subscriber = CamSubscriber()

    print("Subscribed to /camera/color/image_raw and /arducam/image_raw")
    print("Play the rosbag, then Ctrl+C to compute the phase diff.\n")

    try:
        rclpy.spin(cam_subscriber)
    except KeyboardInterrupt:
        print("\nShutting down, computing phase diff...")
    finally:
        cam_subscriber.calculate_phase_diff()

        cam_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
