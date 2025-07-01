#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_avoider")
        self.declare_parameter("forward_speed", 0.20)          # m/s
        self.declare_parameter("turn_speed", 0.60)             # rad/s
        self.declare_parameter("safe_distance", 0.60)          # m
        self.declare_parameter("sector_half_width_deg", 15)    # ±deg around sector centre
        self.declare_parameter("debug", False)                 # print min ranges

        # Fetch once for speed‑critical loop
        self.forward_speed: float = self.get_parameter("forward_speed").value
        self.turn_speed: float = self.get_parameter("turn_speed").value
        self.safe_distance: float = self.get_parameter("safe_distance").value
        self.sector_half_width_deg: float = self.get_parameter("sector_half_width_deg").value
        self.debug: bool = self.get_parameter("debug").value

        # ── ROS 2 I/O ──────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("ObstacleAvoider ready")

    def scan_callback(self, msg: LaserScan) -> None:
        """Main control loop executed for every incoming LaserScan."""
        n_beams = len(msg.ranges)
        if n_beams == 0:
            return

        deg_per_beam = 360.0 / n_beams
        half_width_beams = max(1, int(self.sector_half_width_deg / deg_per_beam))

        # Front sector centred at 0 deg
        front_indices = [(i % n_beams) for i in range(-half_width_beams, half_width_beams + 1)]

        # Right sector centred at 90 deg → index n/4 for a full 360 deg scan
        right_centre = n_beams // 4
        right_indices = [
            (right_centre + i) % n_beams for i in range(-half_width_beams, half_width_beams + 1)
        ]

        front_ranges = self._valid_ranges(msg.ranges, front_indices)
        right_ranges = self._valid_ranges(msg.ranges, right_indices)

        min_front = min(front_ranges) if front_ranges else float("inf")
        min_right = min(right_ranges) if right_ranges else float("inf")

        if self.debug:
            self.get_logger().debug(f"min_front={min_front:.2f} m  min_right={min_right:.2f} m")

        cmd = Twist()

        # ── Simple behaviour tree ─────────────────────────────────────────────
        if min_front < self.safe_distance:
            # Obstacle directly ahead → rotate left in place
            cmd.angular.z = self.turn_speed
        else:
            cmd.linear.x = self.forward_speed

            # Rudimentary right‑wall following (optional)
            if min_right > self.safe_distance * 1.5:
                cmd.angular.z = -0.3 * self.turn_speed   # drift right
            elif min_right < self.safe_distance * 0.8:
                cmd.angular.z = +0.3 * self.turn_speed   # drift left

        self.cmd_pub.publish(cmd)

    # --------------------------------------------------------------------- #

    @staticmethod
    def _valid_ranges(ranges, indices):
        """Return a list of finite, non‑NaN ranges for the requested indices."""
        return [
            ranges[i]
            for i in indices
            if not math.isinf(ranges[i]) and not math.isnan(ranges[i])
        ]


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
