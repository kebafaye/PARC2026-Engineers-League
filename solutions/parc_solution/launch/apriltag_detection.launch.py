"""Rectify + AprilTag detection on the robot's top_camera.

image_proc's rectify_node is kept in the pipeline even though this sim's camera has
no modelled distortion (d435.xacro declares no <distortion> block, so camera_info's D
is all zeros and image_raw is already an ideal pinhole image). The real D435i on the
Jetson will have real distortion, and apriltag_ros's AprilTagNode expects a rectified
image either way - better to have the real pipeline shape validated in sim than to
special-case it away and discover the gap on hardware.

AprilTagNode publishes one TF transform per detected tag (frame `tag36h11:<id>`,
parented on the camera optical frame) rather than a pose topic - see
parc_solution/tag_goal_bridge.py for what consumes that.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare(package="parc_solution").find("parc_solution")
    tags_yaml = os.path.join(pkg_path, "config/apriltag_tags.yaml")

    # use_sim_time matters here specifically because tag_goal_bridge.py composes TF
    # across this pipeline's output (camera -> tag) with world_anchor's and
    # robot_state_publisher's (map -> ... -> camera), which both stamp in sim time.
    # Without it, AprilTagNode stamps its TF in wall-clock time, and `map -> tag...`
    # lookups silently fail to resolve despite both halves of the chain existing -
    # measured: `map -> base_footprint` and `camera -> tag36h11:0` both resolved on
    # their own, but `map -> tag36h11:0` did not, until this was added.
    rectify_top_camera = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_top_camera",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("image", "/top_camera_color/image_raw"),
            ("camera_info", "/top_camera_color/camera_info"),
            ("image_rect", "/top_camera_color/image_rect"),
        ],
    )

    apriltag_top_camera = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_top_camera",
        parameters=[tags_yaml, {"use_sim_time": True}],
        remappings=[
            ("image_rect", "/top_camera_color/image_rect"),
            ("camera_info", "/top_camera_color/camera_info"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(rectify_top_camera)
    ld.add_action(apriltag_top_camera)
    return ld
