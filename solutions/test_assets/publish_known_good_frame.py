#!/usr/bin/env python3
"""Republish a saved frame (confirmed to decode correctly via direct cv2 testing)
under /test_apriltag/*, paired with a one-time copy of the real camera_info, so the
apriltag_ros node graph can be validated independently of Gazebo's live rendering -
which has been flaky under this machine's CPU/GPU load (see solutions/README.md's own
notes on RTF collapse under load). Publishes on a separate namespace, not the robot's
real camera topics, so it never competes with the live image bridge."""
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class Republisher(Node):
    def __init__(self, image_path):
        super().__init__("known_good_frame_publisher")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.bridge = CvBridge()
        self.frame = cv2.imread(image_path)
        self.camera_info = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.image_pub = self.create_publisher(Image, "/test_apriltag/image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "/test_apriltag/camera_info", qos)
        self.info_sub = self.create_subscription(
            CameraInfo, "/top_camera_color/camera_info", self.on_info, qos
        )
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_info(self, msg):
        if self.camera_info is None:
            self.get_logger().info("Got a real camera_info - now republishing both topics.")
        self.camera_info = msg

    def on_timer(self):
        if self.camera_info is None:
            return
        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.camera_info.header.frame_id
        self.image_pub.publish(img_msg)

        info_msg = self.camera_info
        info_msg.header.stamp = stamp
        self.info_pub.publish(info_msg)


def main():
    image_path = sys.argv[1]
    rclpy.init()
    node = Republisher(image_path)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
