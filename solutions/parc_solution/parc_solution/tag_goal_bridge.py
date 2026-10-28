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

Position only, not orientation. apriltag_ros's "pnp" pose estimator has a classic
planar-marker ambiguity near fronto-parallel views: tested against a tag viewed
almost head-on at ~2.3 m, it reported a translation roughly twice the true depth and
a rotation ~180 deg from the tag's actual mounted orientation - the textbook flipped
PnP solution, not a bug in this node or in apriltag_ros's detector (the 2D detection
itself was clean: hamming 0, decision_margin 112). Trusting that rotation would aim
the robot at nonsense. The translation's (x, y) is far more robust - it only needs
the detected corners' pixel centroid and the camera intrinsics - so this node uses
only that, and derives the approach heading itself from the robot's own position
rather than from the tag's reported orientation.
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
STANDOFF_M = 0.6
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
            now_s = self.get_clock().now().nanoseconds / 1e9
            was_new = detection.id not in self.tag_cache
            self.tag_cache[detection.id] = (
                t.transform.translation.x,
                t.transform.translation.y,
                now_s,
            )
            if was_new:
                self.get_logger().info(
                    f"First sighting of {tag_frame} at "
                    f"({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f})"
                )

    def lookup_xy(self, frame, timeout_s):
        """Poll the TF buffer. Never spins the node: the background executor owns
        that (see main()), and spinning the same node from here would re-enter it."""
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            if self.tf_buffer.can_transform("map", frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            time.sleep(0.1)
        return None

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
        age_s = self.get_clock().now().nanoseconds / 1e9 - cached[2]
        self.get_logger().info(f"Using cached sighting, {age_s:.1f} s old.")

        robot_xy = self.lookup_xy("base_footprint", ROBOT_LOOKUP_TIMEOUT_S)
        if robot_xy is None:
            self.get_logger().error("No current robot pose in 'map' - is world_anchor running?")
            return

        goal_pose = self.make_standoff_goal(tag_xy, robot_xy)
        self.get_logger().info(
            f"{tag_frame} at ({tag_xy[0]:.2f}, {tag_xy[1]:.2f}); "
            f"standing off to ({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f})"
        )

        if not self.send_goal(goal_pose):
            self.get_logger().error("Nav2 never acknowledged the goal - aborting.")
            return

        self.monitor_navigation(tag_xy)

    def monitor_navigation(self, tag_xy):
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

                # Re-aim from where the robot actually ended up: backing off moves it,
                # so the approach line to the tag is not the one we planned from.
                robot_xy = self.lookup_xy("base_footprint", ROBOT_LOOKUP_TIMEOUT_S)
                if robot_xy is None:
                    self.get_logger().error("Lost the robot's pose - abandoning the run.")
                    break
                if not self.send_goal(self.make_standoff_goal(tag_xy, robot_xy)):
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

    def make_standoff_goal(self, tag_xy, robot_xy):
        """A point STANDOFF_M in front of the tag, on the line from the tag to the
        robot's current position, facing the tag.

        Deliberately ignores the tag's own reported orientation - see module
        docstring. Approaching from "wherever the robot already is" is not as
        elegant as approaching along the tag's true facing direction, but it is
        robust to the exact failure mode measured here, and it is what the robot can
        act on without that risk.
        """
        dx = robot_xy[0] - tag_xy[0]
        dy = robot_xy[1] - tag_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            dx, dy, dist = 1.0, 0.0, 1.0
        ux, uy = dx / dist, dy / dist

        goal_x = tag_xy[0] + STANDOFF_M * ux
        goal_y = tag_xy[1] + STANDOFF_M * uy
        yaw = math.atan2(-uy, -ux)  # face the tag, i.e. the opposite of (ux, uy)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

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
