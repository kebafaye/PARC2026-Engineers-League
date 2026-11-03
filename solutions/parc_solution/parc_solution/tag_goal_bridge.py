#!/usr/bin/env python3
"""Drive the robot to an AprilTag's location, on request.

Listens on /parc/go_to_tag_id (std_msgs/Int32) for a tag ID. When one arrives, it
looks up that tag's last-known pose in `map` from its own cache (built continuously
from /detections + TF, not queried fresh - see the class docstring for why) and sends
Nav2 a goal a short standoff distance in front of it, facing it.

Prerequisites, both already true for this robot:
  * apriltag_ros running (parc_solution/launch/apriltag_detection.launch.py) and
    publishing TF for the tags it sees.
  * world_anchor.py running, so `map` already *is* the Gazebo world frame - a tag's
    pose needs no further conversion, exactly like the main task goal in
    task_solution.py.

Approach along the tag's own normal, because the rules score alignment. A tag counts
as visited only when the robot stops with its centre within 0.5 m AND facing the tag
within +/-12 degrees of its surface, so parking wherever the robot happened to come
from does not qualify - the standoff point has to sit on the line the tag faces along.

An earlier version here used the tag's position only and derived the heading from the
robot's own position, on the grounds that the planar-marker PnP solution flips near
fronto-parallel views. That was wrong, and wrong for an embarrassing reason: it was
decided while the declared tag `size` was 2x too large, which corrupts the whole pose
and looks exactly like a flip. Re-measured with the size correct, against a marker
rendered at known yaw by projecting its corners through the real camera matrix
(test_assets/tag_angle_sweep.py), the reported angle tracks truth to **5.0 deg worst
case** across 0-50 deg - comfortably inside the 12 deg budget. The residual is largest
square-on, which is the ambiguity still showing, and it biases towards reporting
"square-on", so it costs alignment rather than inventing it.

apriltag_ros points the tag's z axis OUT of the printed face, so the direction the
robot must approach from is the tag frame's +z, flattened to the floor.
"""
import math
import sys
import threading
import time

import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

# Contact detection and the backoff that follows it are shared with
# task_solution.py; see contact_reflex.py for why each value is what it is.
from parc_solution.contact_reflex import (
    BACKOFF_TIME_ALLOWANCE_S,
    COLLISION_TOPICS,
    Contacts,
    build_backup_goal,
    is_new_contact,
)

TAG_FAMILY = "36h11"

# The rules give 0.5 m from the robot's centre to the tag. Park at 0.35 m: with
# nav2_params.yaml's xy_goal_tolerance tightened to 0.10 m, every stop then lands
# inside 0.45 m with room to spare. 0.40 m was tried first and measured 0.49 m on
# the day - inside the limit, but only just, and only by luck.
VISIT_RADIUS_M = 0.5
STANDOFF_M = 0.35

# Alignment the rules demand, and what the estimator costs us of it: measured at 5.0
# deg worst case (test_assets/tag_angle_sweep.py), leaving about 7 deg for everything
# downstream - Nav2's yaw_goal_tolerance, wheel slip, and the SLAM pose itself.
VISIT_ALIGN_DEG = 12.0

ROBOT_LOOKUP_TIMEOUT_S = 2.0
GOAL_SEND_ATTEMPTS = 5
GOAL_SEND_TIMEOUT_S = 20.0

# How many times a single request may be re-planned after a contact before giving up.
# Matches task_solution.py's MAX_GOAL_RETRIES: a robot that has hit something four
# times on the way to one tag is wedged, and continuing to shove at it is how the
# tipping-over failure in contact_reflex.py happened in the first place.
MAX_CONTACT_RETRIES = 4


class TagGoalBridge(Node):
    """Remembers where every tag it has ever seen is, in `map`, so it can be sent
    there long after the robot has driven away and lost sight of it.

    tf2's own Buffer only keeps a short rolling window (default ~10 s) - fine for "is
    this tag in front of me right now", useless for "go to the tag I saw across the
    room two minutes ago". /detections tells us which tag IDs are visible on every
    frame; for each one, the TF lookup happens immediately (well inside the buffer's
    window) and the result is cached in self.tag_cache for as long as this node runs.
    """

    def __init__(self):
        super().__init__("tag_goal_bridge")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.backup_client = ActionClient(self, BackUp, "backup")

        self.tag_cache = {}  # tag id -> (x, y, last_seen_clock_seconds)
        self.pending_request = None
        self.goal_handle = None
        self.result_future = None

        # Contact state, same shape as task_solution.py's RunMonitor.
        self.contacts = 0
        self._contact_last_seen_s = {}

        self.create_subscription(
            AprilTagDetectionArray, "/detections", self.on_detections, 10
        )
        self.create_subscription(Int32, "/parc/go_to_tag_id", self.on_tag_request, 10)

        if Contacts is not None:
            sensor_qos = QoSProfile(depth=10)
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            for topic in COLLISION_TOPICS:
                self.create_subscription(
                    Contacts, topic, self.make_contact_callback(topic), sensor_qos
                )

        self.get_logger().info("Ready. Publish a tag ID on /parc/go_to_tag_id to drive to it.")

    def make_contact_callback(self, topic):
        def callback(msg):
            now_s = self.get_clock().now().nanoseconds / 1e9
            for contact in msg.contacts:
                if not is_new_contact(self._contact_last_seen_s, topic, contact, now_s):
                    continue
                self.contacts += 1
                self.get_logger().warn(f"Contact #{self.contacts}: {contact.collision2.name}")

        return callback

    def on_detections(self, msg):
        for detection in msg.detections:
            tag_frame = f"tag{TAG_FAMILY}:{detection.id}"
            if not self.tf_buffer.can_transform("map", tag_frame, rclpy.time.Time()):
                continue
            t = self.tf_buffer.lookup_transform("map", tag_frame, rclpy.time.Time())
            normal = self.outward_normal(t.transform.rotation)
            if normal is None:
                # The tag is lying flat enough that its face has no direction along
                # the floor. Nothing sensible to approach along, so keep the previous
                # sighting rather than overwrite it with a heading we cannot use.
                continue
            now_s = self.get_clock().now().nanoseconds / 1e9
            was_new = detection.id not in self.tag_cache
            self.tag_cache[detection.id] = (
                t.transform.translation.x,
                t.transform.translation.y,
                normal[0],
                normal[1],
                now_s,
            )
            if was_new:
                self.get_logger().info(
                    f"First sighting of {tag_frame} at "
                    f"({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}), "
                    f"face pointing {math.degrees(math.atan2(normal[1], normal[0])):.0f} deg"
                )

    @staticmethod
    def outward_normal(q):
        """Unit vector along the floor in the direction the tag's face looks.

        apriltag_ros points the tag's z axis OUT of the printed face, back towards
        whoever is reading it, so the outward normal is +z and not -z. Measured both
        ways rather than assumed: with -z the approach pose for a tag 0.95 m ahead
        landed at (1.27, -0.23), i.e. on the far side of the tag, which would have
        the robot drive through it to get there. Returns None when the tag is nearly
        floor-parallel and the direction collapses - there is no approach line then.
        """
        # Third column of the rotation matrix: the tag frame's z axis in `map`.
        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        norm = math.hypot(nx, ny)
        if norm < 0.3:  # face tilted more than ~72 deg out of vertical
            return None
        return (nx / norm, ny / norm)

    def on_tag_request(self, msg):
        """Record the request only. The actual work blocks (TF polling, waiting on
        Nav2's goal acknowledgement) and must NOT run inside an executor callback.

        Doing it here directly deadlocked for real: this callback runs ON the
        executor thread, so a nested spin_until_future_complete() for the goal
        response could never be serviced - the executor was busy running this very
        callback. Nav2 had accepted the goal and was already logging "Passing new
        path to controller" while all five send attempts timed out at exactly 20 s
        each, which is what that failure looks like from the outside.
        """
        self.pending_request = msg.data

    def handle_request(self, tag_id):
        tag_frame = f"tag{TAG_FAMILY}:{tag_id}"
        self.get_logger().info(f"Requested: go to {tag_frame}")

        cached = self.tag_cache.get(tag_id)
        if cached is None:
            self.get_logger().error(
                f"{tag_frame} has never been seen since this node started - nothing "
                "to navigate to."
            )
            return
        tag_xy = (cached[0], cached[1])
        normal = (cached[2], cached[3])
        age_s = self.get_clock().now().nanoseconds / 1e9 - cached[4]
        self.get_logger().info(f"Using cached sighting, {age_s:.1f} s old.")

        goal_pose = self.make_approach_goal(tag_xy, normal)
        self.get_logger().info(
            f"{tag_frame} at ({tag_xy[0]:.2f}, {tag_xy[1]:.2f}); approaching along its "
            f"face to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )

        if not self.send_goal(goal_pose):
            self.get_logger().error("Nav2 never acknowledged the goal - aborting.")
            return

        self.monitor_navigation(tag_xy, normal)
        self.report_visit(tag_frame, tag_xy, normal)

    def report_visit(self, tag_frame, tag_xy, normal):
        """State plainly whether this counts as a visit under the published rules.

        Both halves are scored and both can fail independently: the robot can be
        beautifully aligned a metre too far away, or close enough but square to the
        wrong wall. Reporting one number for each is what makes a failed visit
        diagnosable from the log instead of a shrug.
        """
        robot = self.lookup_pose("base_footprint", ROBOT_LOOKUP_TIMEOUT_S)
        if robot is None:
            self.get_logger().warn("No robot pose - cannot score the visit.")
            return
        (rx, ry, ryaw) = robot

        distance = math.hypot(rx - tag_xy[0], ry - tag_xy[1])
        # The robot should be looking back along the tag's face, i.e. opposite the
        # outward normal. Wrap the difference into (-180, 180] before judging it.
        want = math.atan2(-normal[1], -normal[0])
        misalign = abs((math.degrees(ryaw - want) + 180.0) % 360.0 - 180.0)

        ok_dist = distance <= VISIT_RADIUS_M
        ok_align = misalign <= VISIT_ALIGN_DEG
        verdict = "VISITE VALIDE" if (ok_dist and ok_align) else "NE COMPTE PAS"
        self.get_logger().info(
            f"{verdict} - {tag_frame}: {distance:.2f} m "
            f"(limite {VISIT_RADIUS_M:.2f}), desalignement {misalign:.1f} deg "
            f"(limite {VISIT_ALIGN_DEG:.0f})"
        )

    def monitor_navigation(self, tag_xy, normal):
        """Watch the run and apply the contact reflex until Nav2 is done.

        Without this the node was fire-and-forget: it sent one goal and never looked
        again. Watched that fail for real - the robot drove into cafe_table_6, sat
        there grinding against it while the controller logged "Failed to make
        progress", and nothing ever backed it off. task_solution.py has had this
        reflex since the same table tipped the robot over; the tag path needs it just
        as much, because a tag is every bit as likely to sit behind furniture.

        Blocking here is safe: handle_request() runs on the main loop, not on an
        executor callback (see on_tag_request).
        """
        contacts_handled = self.contacts
        retries = 0

        while rclpy.ok():
            if self.result_future is not None and self.result_future.done():
                self.get_logger().info("Navigation finished.")
                break

            if self.contacts > contacts_handled:
                contacts_handled = self.contacts
                if retries >= MAX_CONTACT_RETRIES:
                    self.get_logger().error(
                        f"Still hitting things after {retries} backoffs - giving up on "
                        "this tag rather than keeping on shoving."
                    )
                    self.cancel_current_goal()
                    break
                retries += 1
                self.get_logger().warn(
                    f"Contact - stopping and backing off before replanning "
                    f"(attempt {retries}/{MAX_CONTACT_RETRIES})."
                )
                self.cancel_current_goal()
                self.send_backup()

                # The approach pose itself does not move - it is fixed to the tag's
                # face, not to wherever the robot ended up. Re-send the same one and
                # let Nav2 plan a fresh route to it around whatever was just hit.
                if not self.send_goal(self.make_approach_goal(tag_xy, normal)):
                    self.get_logger().error("Nav2 never acknowledged the replanned goal.")
                    break

            time.sleep(0.1)

        self.goal_handle = None
        self.result_future = None

    def cancel_current_goal(self):
        if self.goal_handle is None:
            return
        future = self.goal_handle.cancel_goal_async()
        deadline = time.time() + 5.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.1)

    def send_backup(self, attempts=3, timeout_s=10.0):
        """Nav2's collision-checked BackUp, with the same lost-response tolerance as
        send_goal(). Polled rather than spun on, for the reason in on_tag_request()."""
        if not self.backup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("No backup action server - skipping the backoff.")
            return

        goal = build_backup_goal()
        for attempt in range(1, attempts + 1):
            future = self.backup_client.send_goal_async(goal)
            deadline = time.time() + timeout_s
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.1)
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                result_future = handle.get_result_async()
                deadline = time.time() + BACKOFF_TIME_ALLOWANCE_S + 5.0
                while rclpy.ok() and not result_future.done() and time.time() < deadline:
                    time.sleep(0.1)
                return
            self.get_logger().warn(
                f"No answer to the backup request (attempt {attempt}/{attempts})."
            )
            if attempt < attempts:
                time.sleep(1.0)
        self.get_logger().warn("Backup never acknowledged - replanning from here anyway.")

    def make_approach_goal(self, tag_xy, normal):
        """Stand STANDOFF_M out along the tag's face, looking straight back at it.

        Unlike the earlier version this does not depend on where the robot is: the
        pose is a property of the tag alone, which is what the rules score and what
        makes a replan after a contact send the robot back to the same qualifying
        spot rather than to a new one that happens to suit its latest position.
        """
        nx, ny = normal
        goal_x = tag_xy[0] + STANDOFF_M * nx
        goal_y = tag_xy[1] + STANDOFF_M * ny
        yaw = math.atan2(-ny, -nx)  # look back down the normal, into the tag face

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def lookup_pose(self, frame, timeout_s):
        """(x, y, yaw) of `frame` in `map`, or None. Polls rather than spins."""
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            if self.tf_buffer.can_transform("map", frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
                q = t.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                return (t.transform.translation.x, t.transform.translation.y, yaw)
            time.sleep(0.1)
        return None

    def send_goal(self, pose, attempts=GOAL_SEND_ATTEMPTS, timeout_s=GOAL_SEND_TIMEOUT_S):
        """Same lost-response-safe retry pattern as task_solution.py's send_goal():
        a response that never arrives must not hang this node forever, and bt_navigator
        can reject a goal for a few hundred ms right after activation.

        The future is polled rather than waited on with spin_until_future_complete():
        the background executor in main() is what completes it, and spinning this node
        from here would be the same re-entrancy that on_tag_request documents.
        """
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ""

        for attempt in range(1, attempts + 1):
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                continue
            future = self.nav_to_pose_client.send_goal_async(goal)
            deadline = time.time() + timeout_s
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.1)
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                # Kept so the contact reflex can cancel this goal, and so
                # monitor_navigation() can tell when Nav2 is finished with it.
                self.goal_handle = handle
                self.result_future = handle.get_result_async()
                self.get_logger().info("Goal sent.")
                return True
            self.get_logger().warn(
                f"No answer to the goal request (attempt {attempt}/{attempts}) - retrying."
            )
            if attempt < attempts:
                time.sleep(1.0)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TagGoalBridge()

    # The executor runs on its own thread so that callbacks (detections, TF, the
    # action client's responses) keep being serviced while the main loop below
    # blocks waiting on Nav2. See on_tag_request() for what happens without this.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if node.pending_request is not None:
                tag_id = node.pending_request
                node.pending_request = None
                node.handle_request(tag_id)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
