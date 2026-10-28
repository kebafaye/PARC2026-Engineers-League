#!/usr/bin/env python3
"""One-shot: save a frame from /top_camera_color/image_raw to a PNG. Throwaway test
helper for visually checking the apriltag36h11_00000 test fixture."""
import sys

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, out_path):
        super().__init__("frame_grabber")
        self.out_path = out_path
        self.bridge = CvBridge()
        self.got_it = False
        self.create_subscription(Image, "/top_camera_color/image_raw", self.on_image, 1)

    def on_image(self, msg):
        import cv2

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.out_path, frame)
        self.get_logger().info(f"Saved {self.out_path}")
        self.got_it = True


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "frame.png"
    rclpy.init()
    node = Grabber(out_path)
    deadline = node.get_clock().now().nanoseconds + int(30e9)
    while rclpy.ok() and not node.got_it and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
