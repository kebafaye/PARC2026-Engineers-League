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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare(package="parc_solution").find("parc_solution")
    tags_yaml = os.path.join(pkg_path, "config/apriltag_tags.yaml")

    # Overridable without rebuilding, because this is the number most likely to be
    # wrong on the day and the one that scales every reported distance. The default
    # in apriltag_tags.yaml is measured from the organisers' arena generator; the
    # printed tags on the physical robot may differ. See that file's comment.
    declare_tag_size = DeclareLaunchArgument(
        "tag_size",
        default_value="0.12",
        description="Edge of the tag's BLACK SQUARE in metres, not the card.",
    )
    tag_size = LaunchConfiguration("tag_size")

    # use_sim_time matters here specifically because tag_goal_bridge.py composes TF
    # across this pipeline's output (camera -> tag) with world_anchor's and
    # robot_state_publisher's (map -> ... -> camera), which both stamp in sim time.
    # Without it, AprilTagNode stamps its TF in wall-clock time, and `map -> tag...`
    # lookups silently fail to resolve despite both halves of the chain existing -
    # measured: `map -> base_footprint` and `camera -> tag36h11:0` both resolved on
    # their own, but `map -> tag36h11:0` did not, until this was added.
    # The rectified image MUST be published next to the camera_info it belongs to -
    # hence /camera1/camera1/color/, the robot's own namespace, and not a tidier
    # /camera1/color/ of our own. AprilTagNode subscribes through
    # image_transport::CameraSubscriber, which DERIVES the camera_info topic from the
    # image topic by swapping the last path component. It does not use a camera_info
    # remapping; passing one is silently ignored.
    #
    # With image_rect on /camera1/color/, the detector therefore looked for
    # /camera1/color/camera_info, which nothing publishes, and reported:
    #
    #   Topics '/camera1/color/image_rect' and '/camera1/color/camera_info' do not
    #   appear to be synchronized. In the last 10s:
    #     Image messages received:      8
    #     CameraInfo messages received: 0
    #     Synchronized pairs:           0
    #
    # Zero synchronized pairs means zero detections, ever: without intrinsics paired to
    # each frame there is no pose to solve. The robot can park squarely in front of a
    # tag and see nothing, and the only symptom upstream is "tag introuvable" - which
    # looks like an exploration or camera-coverage problem and is neither. It arrived
    # with the switch to the official simulator's double-namespaced camera topics: the
    # inputs above were updated, this output was not.
    RECT_IMAGE = "/camera1/camera1/color/image_rect"

    rectify_top_camera = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_top_camera",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("image", "/camera1/camera1/color/image_raw"),
            ("camera_info", "/camera1/camera1/color/camera_info"),
            ("image_rect", RECT_IMAGE),
        ],
    )

    apriltag_top_camera = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_top_camera",
        parameters=[tags_yaml, {"use_sim_time": True, "size": tag_size}],
        # No camera_info remapping: see above, it would do nothing. The pairing is
        # established by where image_rect lives.
        remappings=[("image_rect", RECT_IMAGE)],
    )

    ld = LaunchDescription()
    ld.add_action(declare_tag_size)
    ld.add_action(rectify_top_camera)
    ld.add_action(apriltag_top_camera)
    return ld
