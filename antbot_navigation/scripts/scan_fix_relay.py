#!/usr/bin/env python3
"""Relay node that normalizes LaserScan to a fixed number of ranges.

The COIN D4 driver publishes variable-length scans (399~403 points per frame)
with metadata that doesn't match the actual ranges count. This causes
slam_toolbox to warn about mismatched range counts every frame.

This node resamples every incoming scan into a fixed number of equally-spaced
angular bins using nearest-neighbor interpolation.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFixRelay(Node):

    def __init__(self):
        super().__init__('scan_fix_relay')

        self.declare_parameter('input_topic', '/scan_0')
        self.declare_parameter('output_topic', '/scan_0_fixed')
        self.declare_parameter('num_ranges', 400)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.num_ranges_ = self.get_parameter('num_ranges').value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_ = self.create_publisher(LaserScan, output_topic, qos)
        self.sub_ = self.create_subscription(
            LaserScan, input_topic, self.callback, qos)

        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} '
            f'(fixed {self.num_ranges_} points)')

    def callback(self, msg: LaserScan):
        n_in = len(msg.ranges)
        if n_in < 2:
            return

        n_out = self.num_ranges_
        if n_out < 2:
            self.get_logger().warn('num_ranges must be at least 2')
            return

        # Input angles from the raw scan
        angles_in = msg.angle_min + np.arange(n_in) * msg.angle_increment

        # Fixed output angles
        angle_inc_out = (msg.angle_max - msg.angle_min) / (n_out - 1)
        angles_out = msg.angle_min + np.arange(n_out) * angle_inc_out

        # Nearest-neighbor resampling
        ranges_in = np.array(msg.ranges, dtype=np.float32)
        indices = np.searchsorted(angles_in, angles_out, side='left')
        indices = np.clip(indices, 1, n_in - 1)
        left_indices = indices - 1
        dist_left = np.abs(angles_in[left_indices] - angles_out)
        dist_right = np.abs(angles_in[indices] - angles_out)
        indices = np.where(dist_left < dist_right, left_indices, indices)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_min + angle_inc_out * (n_out - 1)
        out.angle_increment = angle_inc_out
        out.scan_time = msg.scan_time
        out.time_increment = msg.scan_time / (n_out - 1)
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges_in[indices].tolist()

        if len(msg.intensities) == n_in:
            intensities_in = np.array(msg.intensities, dtype=np.float32)
            out.intensities = intensities_in[indices].tolist()

        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFixRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
