#!/usr/bin/env python3
"""Measure the `size` that apriltag_tags.yaml should declare for a tag.

Why this exists: apriltag_ros's PnP pose is only as good as the declared tag edge
length. The depth it reports scales LINEARLY with that number, so a `size` that is
2x too large puts the tag 2x too far away - and nothing in the detection itself looks
wrong when that happens (hamming 0, decision_margin 112, corners crisp). Measured
here for real: `size: 0.6` against a tag whose detected square is 0.296 m reported
4.78 m for a tag that was actually 2.36 m away, and the resulting Nav2 goal aimed at
empty floor well beyond it.

Usage: put the robot a known distance from the tag, then

    python3 calibrate_tag_size.py --true-distance <metres> [--declared-size <m>]

It reads the live camera->tag TF, compares the reported depth against the distance
you measured, and prints the corrected `size` to put in apriltag_tags.yaml.

The same procedure applies to the competition's real tags on the Jetson: measure one
known distance with a tape, run this, write the answer into the config.
"""
import argparse
import math
import sys

import rclpy
import tf2_ros
from rclpy.node import Node

CAMERA_FRAME = "top_camera_color_optical_frame"
TAG_FRAME_DEFAULT = "tag36h11:0"


class Calibrator(Node):
    def __init__(self, tag_frame):
        super().__init__("tag_size_calibrator")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.tag_frame = tag_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def measure_depth(self, timeout_s=60.0):
        """Reported straight-line camera->tag distance, averaged over a few reads."""
        samples = []
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while rclpy.ok() and self.get_clock().now() < deadline and len(samples) < 5:
            if self.tf_buffer.can_transform(CAMERA_FRAME, self.tag_frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(
                    CAMERA_FRAME, self.tag_frame, rclpy.time.Time()
                ).transform.translation
                samples.append(math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z))
            rclpy.spin_once(self, timeout_sec=0.3)
        return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--true-distance",
        type=float,
        required=True,
        help="Actual camera-to-tag distance in metres, measured independently.",
    )
    parser.add_argument(
        "--declared-size",
        type=float,
        required=True,
        help="The `size` currently set in apriltag_tags.yaml.",
    )
    parser.add_argument("--tag-frame", default=TAG_FRAME_DEFAULT)
    args = parser.parse_args()

    rclpy.init()
    node = Calibrator(args.tag_frame)
    samples = node.measure_depth()
    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        print(f"No {args.tag_frame} TF seen - is the tag in view and detection running?")
        return 1

    reported = sum(samples) / len(samples)
    corrected = args.declared_size * args.true_distance / reported

    print(f"samples              : {len(samples)}  ({', '.join(f'{s:.3f}' for s in samples)})")
    print(f"reported distance    : {reported:.3f} m")
    print(f"true distance        : {args.true_distance:.3f} m")
    print(f"error factor         : {reported / args.true_distance:.3f}x")
    print(f"declared size        : {args.declared_size:.3f} m")
    print(f"CORRECTED size       : {corrected:.3f} m   <- put this in apriltag_tags.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
