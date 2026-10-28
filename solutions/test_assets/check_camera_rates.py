#!/usr/bin/env python3
"""Count image_raw and camera_info messages over a fixed window, with explicit
best-effort QoS, to check whether camera_info is a one-shot "latched" publish or a
per-frame stream (diagnosing why image_transport's synchronizer reports 0 pairs)."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class Checker(Node):
    def __init__(self):
        super().__init__("camera_rate_checker")
        self.image_count = 0
        self.info_count = 0
        self.info_stamps = []
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, "/top_camera_color/image_raw", self.on_image, qos)
        self.create_subscription(CameraInfo, "/top_camera_color/camera_info", self.on_info, qos)

    def on_image(self, msg):
        self.image_count += 1

    def on_info(self, msg):
        self.info_count += 1
        self.info_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)


def main():
    rclpy.init()
    node = Checker()
    deadline = time.time() + 20.0
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"image_raw messages: {node.image_count}")
    print(f"camera_info messages: {node.info_count}")
    print(f"camera_info stamps (first 5): {node.info_stamps[:5]}")
    print(f"camera_info stamps (last 5): {node.info_stamps[-5:]}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
