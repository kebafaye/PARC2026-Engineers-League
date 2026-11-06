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
    # wrong on the day and the one that scales every reported distance.
    #
    # Defaults to the PHYSICAL arena, not the simulator, for the same reason
    # use_sim_time does below: the real run is the one that scores.
    #
    # 0.120 is measured from the organisers' world_generator (TAG_SIZE = 0.15 m face,
    # black square 0.800 of it) and is correct ONLY in simulation. Photographs of the
    # real arena show something different: each tag is printed on a sheet and taped to
    # a plywood board, and the black square spans roughly 76-80% of the sheet's width.
    # On A4 portrait (210 mm) that is 160-168 mm - consistent with the 16.5 cm the
    # phase-réelle guide quotes, and NOT with 0.120.
    #
    # Getting this wrong is silent and scales every reported distance: at 0.120 against
    # a real 0.165 tag the robot believes each one is 1.37x closer than it is, parks
    # short, and fails the 0.5 m rule while detection looks flawless.
    #
    # Still worth measuring on the day rather than trusting either number - the print
    # need not match the guide. outils/calibrate_tag_size.py, at a tape-measured
    # distance, through the real lens:
    #   ros2 launch parc_solution apriltag_detection.launch.py tag_size:=<mesure>
    declare_tag_size = DeclareLaunchArgument(
        "tag_size",
        default_value="0.165",
        description="Edge of the tag's BLACK SQUARE in metres, not the sheet. "
                    "0.165 on the physical arena; pass 0.12 in simulation.",
    )
    tag_size = LaunchConfiguration("tag_size")

    # Defaults FALSE, because the real robot is the run that scores and it has no
    # /clock. This was hardcoded True for both nodes below, which is correct in
    # simulation and silently fatal on hardware: with use_sim_time true and nothing
    # publishing /clock, the nodes hold at time zero, every TF they broadcast is
    # stamped 0, and `map -> tag36h11:<id>` never resolves - while detection itself
    # looks perfectly healthy. The rehearsal commands in DEPLOY.md therefore pass
    # use_sim_time:=true explicitly; a wrong value in sim costs a confusing rehearsal,
    # a wrong value on the robot costs the race.
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="true only in simulation, where Gazebo publishes /clock.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

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
    # WHICH CAMERA. Settled by the robot's own /tf_static, read from the
    # 2026-11-06 rosbag:
    #
    #   base_link -> camera1_link  xyz=(+0.140, 0, 0.530)  rpy=(0, +67deg, 0)
    #   base_link -> camera2_link  xyz=(-0.060, 0, 0.920)  rpy=(0,   0deg, 0)
    #
    # camera1 sits 0.53 m up and is pitched 67 degrees DOWN: its optical axis meets
    # the floor 0.22 m ahead, and even the top of its frame only reaches ~0.65 m
    # ahead. A tag on a post at ~0.9 m is outside its field of view at EVERY
    # distance - it is an obstacle camera, not a tag camera. camera2 is horizontal
    # at 0.92 m, the height the arena photographs show the tag boards at, and sees
    # them from 0.5 m out to 4 m.
    #
    # This is the likeliest reason /detections ran for the whole 406 s recording and
    # produced 260 messages with not one non-empty detection.
    #
    # Caveat, and it needs checking before the run: in that same recording
    # /camera2/camera2/color/image_raw and its camera_info both carry ZERO messages.
    # camera2 was streaming only IMU - its colour stream was disabled. If it is
    # still off on the day the organisers have to enable it, so verify early:
    #   ros2 topic hz /camera2/camera2/color/image_raw
    # and fall back with camera:=camera1 only if there is no other option, knowing
    # that geometry says it will see nothing.
    declare_camera = DeclareLaunchArgument(
        "camera",
        default_value="camera2",
        description="camera2 (horizontal, 0.92 m - sees the tags) or camera1 "
                    "(pitched 67deg down, floor only).",
    )
    camera = LaunchConfiguration("camera")

    RAW_IMAGE = ["/", camera, "/", camera, "/color/image_raw"]
    RAW_INFO = ["/", camera, "/", camera, "/color/camera_info"]
    RECT_IMAGE = ["/", camera, "/", camera, "/color/image_rect"]

    rectify_top_camera = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_top_camera",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("image", RAW_IMAGE),
            ("camera_info", RAW_INFO),
            ("image_rect", RECT_IMAGE),
        ],
    )

    apriltag_top_camera = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_top_camera",
        parameters=[tags_yaml, {"use_sim_time": use_sim_time, "size": tag_size}],
        # No camera_info remapping: see above, it would do nothing. The pairing is
        # established by where image_rect lives.
        remappings=[("image_rect", RECT_IMAGE)],
    )

    ld = LaunchDescription()
    ld.add_action(declare_tag_size)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_camera)
    ld.add_action(rectify_top_camera)
    ld.add_action(apriltag_top_camera)
    return ld
