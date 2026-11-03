#!/usr/bin/env python3
"""Does apriltag_ros report a usable tag ORIENTATION, or only a usable position?

The rules require the robot to stop facing the tag within +/-12 degrees of its
surface. tag_goal_bridge.py currently refuses to use the tag's reported orientation
at all, on the grounds that the PnP solution for a planar marker flips near
fronto-parallel views - a decision taken when the declared tag `size` was also wrong
by 2x, which corrupted the pose in a way that could easily have been mistaken for the
flip. That needs settling with a measurement rather than a memory.

So: render the marker at a known yaw by projecting its four corners through the real
camera matrix and warping the image onto them. Ground truth is then exact rather than
measured - the only error in the loop is the estimator's. Sweep the yaw, and compare
the angle apriltag_ros reports against the angle actually rendered.

Run with the detection pipeline pointed at /test_apriltag/* (see test_assets/README).
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

TAG_ID = 0
TAG_SIZE_M = 0.165      # the organisers' real tag
DISTANCE_M = 1.0        # inside the 0.5 m visit radius is too close to frame; 1 m reads well
ANGLES_DEG = [0, 5, 10, 20, 30, 40, 50]
CAMERA_FRAME = "top_camera_color_optical_frame"
TAG_FRAME = "tag36h11:0"
SETTLE_S = 6.0


def render(angle_deg, fx, fy, cx, cy, width, height):
    """A view of the tag yawed by angle_deg, built by projecting its corners.

    Optical frame convention: x right, y down, z forward. Yawing about the camera's
    y axis turns the tag the way a tag on a wall turns as the robot moves around it.
    """
    half = TAG_SIZE_M / 2.0
    a = math.radians(angle_deg)
    rot_y = np.array(
        [
            [math.cos(a), 0.0, math.sin(a)],
            [0.0, 1.0, 0.0],
            [-math.sin(a), 0.0, math.cos(a)],
        ]
    )

    # Tag-frame corners in image order: TL, TR, BR, BL.
    corners = [(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)]
    projected = []
    for p in corners:
        x, y, z = rot_y @ np.array(p)
        z += DISTANCE_M
        projected.append([fx * x / z + cx, fy * y / z + cy])
    projected = np.array(projected, dtype=np.float32)

    marker_px = 400
    marker = cv2.aruco.drawMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), TAG_ID, marker_px
    )
    src = np.array(
        [[0, 0], [marker_px, 0], [marker_px, marker_px], [0, marker_px]], dtype=np.float32
    )
    homography = cv2.getPerspectiveTransform(src, projected)
    canvas = cv2.warpPerspective(
        marker, homography, (width, height),
        borderMode=cv2.BORDER_CONSTANT, borderValue=255,
    )
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def synthetic_camera_info():
    """The Phase-1 D435i colour stream, from d435.xacro: 640x480, hfov 1.57 rad, no
    distortion. Built here rather than subscribed so this test needs no simulator -
    it is a question about the pose estimator, not about Gazebo."""
    info = CameraInfo()
    info.width, info.height = 640, 480
    fx = (info.width / 2.0) / math.tan(1.57 / 2.0)
    cx, cy = info.width / 2.0, info.height / 2.0
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.header.frame_id = CAMERA_FRAME
    return info


class Sweep(Node):
    def __init__(self, sim_time=False):
        super().__init__("tag_angle_sweep")
        # Off by default so the sweep runs standalone with no simulator. It must be
        # ON when feeding a stack that uses sim time, or the images carry wall-clock
        # stamps, the tag TF is stamped in a different epoch from `map`, and
        # `map -> tag` silently never resolves.
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=sim_time)])
        self.bridge = CvBridge()
        self.info = None
        self.frame = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.image_pub = self.create_publisher(Image, "/test_apriltag/image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "/test_apriltag/camera_info", qos)
        self.info = synthetic_camera_info()
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        if self.frame is None or self.info is None:
            return
        stamp = self.get_clock().now().to_msg()
        img = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        img.header.stamp = stamp
        img.header.frame_id = self.info.header.frame_id
        self.image_pub.publish(img)
        info = self.info
        info.header.stamp = stamp
        self.info_pub.publish(info)

    def reported_angle(self):
        """Yaw of the tag's surface about the camera's vertical axis, in degrees.

        Taken from the tag frame's z axis expressed in the camera frame: the tag's
        normal. Comparing normals rather than raw quaternions keeps this independent
        of which way round apriltag_ros defines the tag's x and y.
        """
        if not self.buf.can_transform(CAMERA_FRAME, TAG_FRAME, rclpy.time.Time()):
            return None
        q = self.buf.lookup_transform(CAMERA_FRAME, TAG_FRAME, rclpy.time.Time()).transform.rotation
        # third column of the rotation matrix = tag z axis in camera coordinates
        nx = 2 * (q.x * q.z + q.w * q.y)
        nz = 1 - 2 * (q.x * q.x + q.y * q.y)
        return math.degrees(math.atan2(nx, nz))


def main():
    # --angle N holds one view instead of sweeping, so the rest of the stack can be
    # driven against a tag whose face points somewhere other than at the robot.
    # --sim-time is required whenever that stack is running on the Gazebo clock.
    hold = None
    if "--angle" in sys.argv:
        hold = float(sys.argv[sys.argv.index("--angle") + 1])

    rclpy.init()
    node = Sweep(sim_time="--sim-time" in sys.argv)

    k = node.info.k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    print(f"fx={fx:.2f} tag={TAG_SIZE_M} m distance={DISTANCE_M} m\n")

    if hold is not None:
        node.frame = render(hold, fx, fy, cx, cy, node.info.width, node.info.height)
        print(f"maintien d'une vue a {hold:.0f}° - Ctrl-C pour arreter")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print(f"{'rendu':>8}{'rapporte':>12}{'erreur':>10}   verdict")
    print("-" * 46)

    worst = 0.0
    for angle in ANGLES_DEG:
        node.frame = render(angle, fx, fy, cx, cy, node.info.width, node.info.height)
        end = time.time() + SETTLE_S
        got = None
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            got = node.reported_angle()
        if got is None:
            print(f"{angle:7.0f}°{'non detecte':>12}")
            continue
        # apriltag_ros points the tag's z axis INTO the surface, away from the camera,
        # so a tag square-on to the lens reads as 180 deg rather than 0. Undo that
        # convention before comparing, and wrap into (-180, 180].
        corrected = (-(got + 180.0) + 180.0) % 360.0 - 180.0
        corrected = abs(corrected)
        err = abs(corrected - angle)
        worst = max(worst, err)
        verdict = "OK" if err <= 6 else "limite" if err <= 12 else "INEXPLOITABLE"
        print(f"{angle:7.0f}°{corrected:11.1f}°{err:9.1f}°   {verdict}")

    print(f"\nerreur maximale : {worst:.1f}°  (budget du reglement : 12°)")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
