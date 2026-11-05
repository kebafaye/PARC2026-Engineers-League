"""Navigation stack with real SLAM - the bring-up that can run on the real robot.

navigation.launch.py localises with world_anchor.py, which reads Gazebo's ground-truth
pose topic and publishes `map -> odom` from it. That makes `map` exactly the Gazebo
world frame, which is why the Phase 1 goal could be used verbatim and never re-aimed.
It is also the reason that bring-up is simulation-only: **the real robot publishes no
ground-truth pose**, so world_anchor there would wait forever on a topic that never
arrives, Nav2 would never get `map -> odom`, and nothing would plan.

So this file swaps the one piece that cannot transfer. slam_toolbox fills both jobs
world_anchor and evidence_map were doing between them:

  map -> odom   scan matching against its own map, instead of ground truth
  /map          an occupancy grid the global costmap's static layer consumes, on the
                same topic evidence_map.py publishes in the other bring-up

Everything else is unchanged and shared: speed_governor still raises the ceiling on
open floor, and Nav2 runs from the same tuned nav2_params.yaml. Nothing here reads a
topic the real robot does not have, and nothing publishes one it already provides.

What this costs, and it is worth saying plainly: `map` is no longer the world frame.
It is wherever the robot happened to start, and it drifts and jumps as the graph is
optimised. Any goal expressed in world coordinates - the Phase 1 goal, locations.yaml -
is meaningless under this bring-up. Goals have to come from something the robot
observes for itself, which is exactly what the real task asks for: find the tags.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_solution = get_package_share_directory("parc_solution")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Gazebo /clock in simulation; set false on the real robot.",
    )

    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_solution, "config", "nav2_params.yaml"),
        description="Nav2 parameter file - the same one the sim bring-up uses.",
    )

    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(pkg_solution, "config", "slam_params.yaml"),
        description="slam_toolbox parameter file.",
    )

    # Threaded through because a wrong value here is the single most expensive silent
    # failure in this stack: every node comes up, Nav2 plans, and the robot simply
    # never moves. Overridable on the robot without a rebuild:
    #   ros2 launch parc_solution slam_navigation.launch.py cmd_vel_out_topic:=/cmd_vel
    declare_cmd_vel_out = DeclareLaunchArgument(
        "cmd_vel_out_topic",
        default_value="/cmd_vel",
        description="Topic the robot drives from. Confirm with `ros2 topic info "
                    "<topic>` that it has a subscriber before trusting a run.",
    )

    # No scan_filter of ours. The robot's own bringup publishes /scan_filtered now -
    # confirmed on the real simulator's topic list - and a second publisher on the
    # same topic would have slam_toolbox matching against two interleaved streams.
    # slam_params.yaml still consumes /scan_filtered; it simply comes from them.

    # Async, not sync: the sync node blocks the scan callback while it optimises, and
    # on a machine already running Gazebo that shows up as dropped scans. The async
    # node keeps consuming while the back end works.
    start_slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # slam_toolbox is a LIFECYCLE node on Jazzy, and starting it is not the same as
    # running it. Launched on its own it sits in `unconfigured` forever: it creates
    # the /map topic, publishes nothing on it, and never broadcasts `map -> odom`.
    # Nav2 then stalls mid-bringup with planner_server stuck `inactive`, logging
    # "Invalid frame ID map ... frame does not exist" - which reads like a SLAM
    # failure and is really nobody having pressed start. Confirmed by hand: a
    # configure + activate made `map -> odom` appear and planner_server went active
    # on its own. This manager is what presses start.
    start_slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["slam_toolbox"],
                # Left at the default. The companion fix is use_lifecycle_manager in
                # slam_params.yaml - without it slam_toolbox never creates a bond at
                # all, and no timeout here can help. Confirmed the hard way: raising
                # this to 20 s only changed the number in the error message.
                #
                #   Server slam_toolbox was unable to be reached after 20.00s by bond
                #   Failed to bring up all requested nodes. Aborting bringup.
                #
                # That abort does not deactivate anything, which is the trap: on the
                # seed 12 run slam_toolbox stayed `active` and kept publishing /map and
                # `map -> odom` throughout, so the stack looked healthy. But the manager
                # had stopped watching, so had SLAM died mid-race nothing would have
                # noticed and Nav2 would have planned on against a frozen map.
            }
        ],
    )

    start_speed_governor = Node(
        package="parc_solution",
        executable="speed_governor.py",
        name="speed_governor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_solution, "launch", "nav2_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": "true",
            "use_composition": "False",
            "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_cmd_vel_out)
    ld.add_action(start_slam)
    ld.add_action(start_slam_lifecycle_manager)
    ld.add_action(start_speed_governor)
    ld.add_action(start_navigation)
    return ld
