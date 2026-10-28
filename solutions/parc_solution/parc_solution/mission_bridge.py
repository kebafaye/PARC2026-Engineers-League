#!/usr/bin/env python3
"""Turn named destinations into Nav2 missions.

Publish one or more place names on /parc/go_to_locations and the robot drives to
them in order:

    ros2 topic pub --once /parc/go_to_locations std_msgs/msg/String "data: 'table_6'"
    ros2 topic pub --once /parc/go_to_locations std_msgs/msg/String \\
        "data: 'comptoir, table_6, base'"

This is the layer a voice/LLM front-end plugs into. The split matters: the speech
layer decides WHICH named place was meant, this node owns WHERE that place is. An
LLM asked for coordinates can invent plausible ones and nothing downstream can tell;
a name either matches config/locations.yaml or it does not, and an unmatched name
comes back on /parc/mission_status as something the robot can say out loud. Nothing
here needs an LLM to be exercised - a person typing names on the command line drives
exactly the same path, which is why this half can be validated in simulation while
the Jetson half cannot yet.

Multi-stop missions go through NavigateThroughPoses so Nav2 plans the whole tour at
once rather than stopping dead at each waypoint; a single stop uses NavigateToPose.

Contact handling is shared with task_solution.py and tag_goal_bridge.py via
contact_reflex.py - same bumper topics, same floor filter, same cooldown, same
collision-checked BackUp - because a reflex that drifts between nodes is one that
silently stops protecting a robot that has already tipped over once.
"""
import math
import os
import sys
import threading
import time
import unicodedata

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from parc_solution.contact_reflex import (
    BACKOFF_TIME_ALLOWANCE_S,
    COLLISION_TOPICS,
    Contacts,
    build_backup_goal,
    is_new_contact,
)

LOCATIONS_RELPATH = os.path.join("config", "locations.yaml")
REQUEST_TOPIC = "/parc/go_to_locations"
STATUS_TOPIC = "/parc/mission_status"
ROBOT_LOOKUP_TIMEOUT_S = 2.0
GOAL_SEND_ATTEMPTS = 5
GOAL_SEND_TIMEOUT_S = 20.0
ARRIVAL_RADIUS_M = 0.45

# Same cap, and the same reason, as tag_goal_bridge: a robot that has hit something
# four times on one mission is wedged, and shoving harder is how it ends up on its side.
MAX_CONTACT_RETRIES = 4


def normalise(name):
    """Fold a spoken or typed place name to its lookup key.

    Speech and keyboards both produce "Table 6", "table six", "TABLE_6" and - via a
    French ASR - "entrée" where the config says "entree". Accents are stripped and
    separators unified so none of those need a separate alias entry.
    """
    text = unicodedata.normalize("NFKD", str(name))
    text = "".join(c for c in text if not unicodedata.combining(c))
    text = text.lower().strip()
    for ch in ("_", "-"):
        text = text.replace(ch, " ")
    return " ".join(text.split())


class MissionBridge(Node):
    def __init__(self):
        super().__init__("mission_bridge")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.locations, self.lookup = self.load_locations()
        self.get_logger().info(
            f"{len(self.locations)} places known: {', '.join(sorted(self.locations))}"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_through_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self.backup_client = ActionClient(self, BackUp, "backup")

        self.pending_request = None
        self.goal_handle = None
        self.result_future = None

        self.contacts = 0
        self._contact_last_seen_s = {}

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, REQUEST_TOPIC, self.on_request, 10)

        if Contacts is not None:
            sensor_qos = QoSProfile(depth=10)
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            for topic in COLLISION_TOPICS:
                self.create_subscription(
                    Contacts, topic, self.make_contact_callback(topic), sensor_qos
                )

        self.get_logger().info(f"Ready. Publish place names on {REQUEST_TOPIC}.")

    # ---------------------------------------------------------------- locations

    def load_locations(self):
        path = os.path.join(
            get_package_share_directory("parc_solution"), LOCATIONS_RELPATH
        )
        with open(path, "r") as handle:
            data = yaml.safe_load(handle)["locations"]

        lookup = {}
        for name, entry in data.items():
            lookup[normalise(name)] = name
            for alias in entry.get("aliases") or []:
                key = normalise(alias)
                if key in lookup and lookup[key] != name:
                    # Two places answering to one phrase would make the robot pick one
                    # arbitrarily, which is far worse than refusing to load.
                    raise ValueError(
                        f"alias '{alias}' maps to both '{lookup[key]}' and '{name}'"
                    )
                lookup[key] = name
        return data, lookup

    def resolve(self, spoken):
        """Place name for a spoken phrase, or None."""
        return self.lookup.get(normalise(spoken))

    def pose_for(self, name):
        entry = self.locations[name]
        yaw = math.radians(float(entry["yaw_deg"]))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(entry["x"])
        pose.pose.position.y = float(entry["y"])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ---------------------------------------------------------------- contacts

    def make_contact_callback(self, topic):
        def callback(msg):
            now_s = self.get_clock().now().nanoseconds / 1e9
            for contact in msg.contacts:
                if not is_new_contact(self._contact_last_seen_s, topic, contact, now_s):
                    continue
                self.contacts += 1
                self.get_logger().warn(f"Contact #{self.contacts}: {contact.collision2.name}")

        return callback

    # ---------------------------------------------------------------- requests

    def on_request(self, msg):
        """Record only - the work blocks, and must not run on the executor thread.
        tag_goal_bridge.py documents what happens when it does."""
        self.pending_request = msg.data

    def say(self, text):
        """Report outcome in one line, for a TTS layer and for the log."""
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def handle_request(self, request):
        names, unknown = [], []
        for part in str(request).split(","):
            if not part.strip():
                continue
            resolved = self.resolve(part)
            (names if resolved else unknown).append(resolved or part.strip())

        if unknown:
            self.say(
                f"Je ne connais pas : {', '.join(unknown)}. "
                f"Je connais : {', '.join(sorted(self.locations))}."
            )
            if not names:
                return

        self.say(f"En route vers : {' puis '.join(names)}.")
        self.run_mission(names)

    # ---------------------------------------------------------------- driving

    def run_mission(self, names):
        remaining = list(names)
        retries = 0
        contacts_handled = self.contacts

        if not self.send_waypoints(remaining):
            self.say("Nav2 n'a pas accepté la mission.")
            return

        while rclpy.ok():
            if self.result_future is not None and self.result_future.done():
                break

            # Drop waypoints already reached, so a replan after a contact does not
            # send the robot back to a stop it has already served.
            remaining = self.drop_reached(remaining)
            if not remaining:
                break

            if self.contacts > contacts_handled:
                contacts_handled = self.contacts
                if retries >= MAX_CONTACT_RETRIES:
                    self.cancel_current_goal()
                    self.say(
                        f"Je bute encore après {retries} reculs ; j'abandonne "
                        f"{' puis '.join(remaining)}."
                    )
                    break
                retries += 1
                self.get_logger().warn(
                    f"Contact - backing off before replanning "
                    f"(attempt {retries}/{MAX_CONTACT_RETRIES})."
                )
                self.cancel_current_goal()
                self.send_backup()
                if not self.send_waypoints(remaining):
                    self.say("Nav2 n'a pas accepté la mission replanifiée.")
                    break

            time.sleep(0.1)

        self.goal_handle = None
        self.result_future = None
        self.report_arrival(names)

    def drop_reached(self, remaining):
        robot_xy = self.robot_xy()
        if robot_xy is None or not remaining:
            return remaining
        entry = self.locations[remaining[0]]
        if math.hypot(robot_xy[0] - entry["x"], robot_xy[1] - entry["y"]) <= ARRIVAL_RADIUS_M:
            self.say(f"Arrivé à {remaining[0]}.")
            return remaining[1:]
        return remaining

    def report_arrival(self, names):
        robot_xy = self.robot_xy()
        if robot_xy is None:
            self.say("Mission terminée, position inconnue.")
            return
        last = self.locations[names[-1]]
        gap = math.hypot(robot_xy[0] - last["x"], robot_xy[1] - last["y"])
        if gap <= ARRIVAL_RADIUS_M:
            self.say(f"Mission terminée, je suis à {names[-1]}.")
        else:
            self.say(f"Mission terminée à {gap:.2f} m de {names[-1]}.")

    def robot_xy(self):
        if not self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time()):
            return None
        t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        return (t.transform.translation.x, t.transform.translation.y)

    def send_waypoints(self, names):
        """One stop goes through NavigateToPose; several through
        NavigateThroughPoses, so Nav2 plans the whole tour instead of stopping dead
        at each waypoint."""
        poses = [self.pose_for(n) for n in names]
        if len(poses) == 1:
            goal = NavigateToPose.Goal()
            goal.pose = poses[0]
            goal.behavior_tree = ""
            return self.send_goal(self.nav_to_pose_client, goal)

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ""
        return self.send_goal(self.nav_through_client, goal)

    def send_goal(self, client, goal, attempts=GOAL_SEND_ATTEMPTS, timeout_s=GOAL_SEND_TIMEOUT_S):
        """Retrying send, tolerant of both a lost acceptance response and of
        bt_navigator rejecting goals for a moment right after activation. The future
        is polled, not spun on - see on_request()."""
        for attempt in range(1, attempts + 1):
            if not client.wait_for_server(timeout_sec=5.0):
                continue
            future = client.send_goal_async(goal)
            deadline = time.time() + timeout_s
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.1)
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                self.goal_handle = handle
                self.result_future = handle.get_result_async()
                return True
            self.get_logger().warn(
                f"No answer to the goal request (attempt {attempt}/{attempts})."
            )
            if attempt < attempts:
                time.sleep(1.0)
        return False

    def cancel_current_goal(self):
        if self.goal_handle is None:
            return
        future = self.goal_handle.cancel_goal_async()
        deadline = time.time() + 5.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.1)

    def send_backup(self, attempts=3, timeout_s=10.0):
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
            if attempt < attempts:
                time.sleep(1.0)
        self.get_logger().warn("Backup never acknowledged - replanning from here anyway.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionBridge()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if node.pending_request is not None:
                request = node.pending_request
                node.pending_request = None
                node.handle_request(request)
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
