#!/usr/bin/env python3
"""Publish a synthetic camera image of a tag36h11 marker at an EXACTLY known pose.

Gazebo's renderer stopped applying the test fixture's texture partway through this
session (the panel renders solid black regardless of texture path, model name, or a
clean restart - an Ogre2 material-cache problem in the fixture, not in any of our
nodes). That blocks live end-to-end validation, but it does not have to block
validating the thing that actually matters and that live rendering would only tell us
approximately anyway: whether apriltag_ros's reported DEPTH matches the `size`
declared in apriltag_tags.yaml.

Here the ground truth is exact rather than measured. Given the camera intrinsics from
camera_info, a marker whose black square is physically `size` metres at distance
`depth` metres projects to exactly size*f/depth pixels. So we draw it at precisely
that many pixels: if apriltag_ros then reports `depth`, the size/intrinsics/detector
chain is correct; if it reports something else, the ratio is the calibration error.

Publishes on /test_apriltag/* so it never competes with the robot's real camera.
"""
import argparse
import math
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

TAG_ID = 0


class SyntheticTagPublisher(Node):
    def __init__(self, tag_size_m, depth_m):
        super().__init__("synthetic_tag_publisher")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.tag_size_m = tag_size_m
        self.depth_m = depth_m
        self.bridge = CvBridge()
        self.frame = None
        self.camera_info = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.image_pub = self.create_publisher(Image, "/test_apriltag/image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "/test_apriltag/camera_info", qos)
        self.create_subscription(
            CameraInfo, "/top_camera_color/camera_info", self.on_info, qos
        )
        self.create_timer(0.1, self.on_timer)

    def on_info(self, msg):
        if self.camera_info is not None:
            return
        self.camera_info = msg
        fx = msg.k[0]
        width, height = msg.width, msg.height
        # Exact projected edge length of the tag's black square at self.depth_m.
        quad_px = self.tag_size_m * fx / self.depth_m
        quad_px_int = int(round(quad_px))

        canvas = np.full((height, width), 255, dtype=np.uint8)
        marker = cv2.aruco.drawMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            TAG_ID,
            quad_px_int,
        )
        y0 = (height - quad_px_int) // 2
        x0 = (width - quad_px_int) // 2
        canvas[y0 : y0 + quad_px_int, x0 : x0 + quad_px_int] = marker
        self.frame = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

        self.get_logger().info(
            f"fx={fx:.2f} px | tag {self.tag_size_m} m at {self.depth_m} m "
            f"=> drawing quad of {quad_px:.2f} px (rounded {quad_px_int})"
        )
        self.get_logger().info(
            f"Expect apriltag_ros to report depth ~"
            f"{self.tag_size_m * fx / quad_px_int:.3f} m"
        )

    def on_timer(self):
        if self.frame is None or self.camera_info is None:
            return
        stamp = self.get_clock().now().to_msg()
        img = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        img.header.stamp = stamp
        img.header.frame_id = self.camera_info.header.frame_id
        self.image_pub.publish(img)

        info = self.camera_info
        info.header.stamp = stamp
        self.info_pub.publish(info)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag-size", type=float, default=0.6, help="metres")
    parser.add_argument("--depth", type=float, default=3.0, help="metres")
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticTagPublisher(args.tag_size, args.depth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
