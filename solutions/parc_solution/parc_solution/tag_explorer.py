#!/usr/bin/env python3
"""Drive the arena until a wanted AprilTag has been seen.

    ros2 topic pub --once /parc/explore_for_tag std_msgs/msg/Int32 "data: 4"

The real task gives no tag positions and no map, so nothing can be approached before
it has been found. This node does the finding; tag_goal_bridge.py, listening to the
same /detections, caches the pose and does the approaching.

Why it drives to a point and then spins, rather than following frontiers
-----------------------------------------------------------------------
Frontier exploration grows the *map*, and the map comes from the LiDAR - which sits
4.8 cm off the floor and will never see a tag mounted at 1 m. It maps table legs.
The tags are found by the cameras, which see about 90 deg at a time and, at the
published 16.5 cm tag size, only decode reliably within roughly 1.3 m.

So the thing worth maximising is not mapped area but *camera time pointed at new
places*. A spin on the spot sweeps the full 360 deg from one position, which is far
cheaper than translating. Covering a 5 x 5 m arena with 1.3 m circles takes on the
order of six to nine vantage points - small enough to do deliberately inside a
10-minute budget, with the spacing below leaving the circles overlapping.

Vantage points are chosen from slam_toolbox's live map rather than planned up front,
because at the start of a run that map is nearly empty: the reachable free space is
itself a thing being discovered.
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

REQUEST_TOPIC = "/parc/explore_for_tag"
STATUS_TOPIC = "/parc/explore_status"

# Spacing between vantage points. Detection reaches ~1.3 m, so 1.6 m apart leaves the
# swept circles overlapping rather than leaving unlooked-at gaps between them.
VANTAGE_SPACING_M = 1.6

# A vantage point must have this much clear floor around it. The robot's circumscribed
# radius is 0.354 m (nav2_params footprint), plus margin for the map being coarse and
# slightly stale.
CLEARANCE_M = 0.45

# A frontier run has to be worth the drive. Below this the robot just chases the
# map's own ragged edge, which slam_toolbox regenerates a few centimetres ahead of
# it however far it goes.
MIN_FRONTIER_TRAVEL_M = 1.2

# Don't spin again without having moved at least this far since the last spin: the
# view is the same and the seconds are not refundable.
RESWEEP_DISTANCE_M = 0.8

# How many times one search may forget where it has already swept. Uncapped, this
# loops forever on a map that genuinely holds a single reachable spot.
MAX_FORGETS = 2

SPIN_TIME_ALLOWANCE_S = 30.0
GOTO_TIMEOUT_S = 120.0
GOAL_SEND_ATTEMPTS = 3
GOAL_SEND_TIMEOUT_S = 20.0
DEFAULT_BUDGET_S = 300.0


class TagExplorer(Node):
    def __init__(self):
        super().__init__("tag_explorer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")

        self.map = None
        self.seen_tags = set()
        self.visited = []          # vantage points already swept, in `map`
        self.swept_from = None     # where the last spin happened
        self.pending_request = None

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(
            AprilTagDetectionArray, "/detections", self.on_detections, 10
        )
        self.create_subscription(Int32, REQUEST_TOPIC, self.on_request, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self.get_logger().info(f"Ready. Publish a tag ID on {REQUEST_TOPIC}.")

    # ------------------------------------------------------------------ inputs

    def on_map(self, msg):
        self.map = msg

    def on_detections(self, msg):
        for detection in msg.detections:
            if detection.id not in self.seen_tags:
                self.seen_tags.add(detection.id)
                self.say(f"Tag {detection.id} aperçu.")

    def on_request(self, msg):
        """Record only - exploring blocks, and must not run on the executor thread."""
        self.pending_request = msg.data

    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    # ------------------------------------------------------------------ geometry

    def robot_xy(self):
        if not self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time()):
            return None
        t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        return (t.transform.translation.x, t.transform.translation.y)

    def free_mask(self, grid, unknown_blocks=True):
        """Cells with CLEARANCE_M of clear floor around them.

        `unknown_blocks` decides what unmeasured space counts as, and the two callers
        want opposite answers. A vantage point is somewhere the robot must arrive and
        then rotate on the spot, so there unknown counts as blocked - committing to a
        pirouette on cells nobody has measured is how it ends up wedged against
        something the map never showed. A frontier target is the opposite: it is
        chosen *because* it touches unknown space, so there only real obstacles
        disqualify it.
        """
        data = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width
        )
        if unknown_blocks:
            free = (data >= 0) & (data < 50)
        else:
            free = data < 50  # -1 included

        # Erode by the clearance radius: a cell survives only if the whole disc
        # around it is free. Done as two 1-D minimum filters over a square window,
        # which is a slight over-estimate of the disc and errs towards caution.
        radius = max(1, int(round(CLEARANCE_M / grid.info.resolution)))
        eroded = free.copy()
        for shift in range(1, radius + 1):
            eroded[shift:, :] &= free[:-shift, :]
            eroded[:-shift, :] &= free[shift:, :]
        safe = eroded.copy()
        for shift in range(1, radius + 1):
            safe[:, shift:] &= eroded[:, :-shift]
            safe[:, :-shift] &= eroded[:, shift:]

        # Anything within the clearance radius of the map's own edge is rejected.
        # Those cells keep passing the erosion above for the wrong reason - there is
        # simply no neighbour on the outward side to disqualify them - and the map
        # edge is exactly where the unmeasured world starts, which on a map that
        # grows as slam_toolbox explores is the last place to go rotate blind.
        # Caught by the offline check: cell (0, 0) of a free grid came back "safe".
        safe[:radius, :] = False
        safe[-radius:, :] = False
        safe[:, :radius] = False
        safe[:, -radius:] = False
        return safe

    def pick_vantage(self):
        """The safe free cell furthest from everywhere already swept.

        Furthest-first rather than nearest-first: the budget is spent on sweeps, and
        a sweep next door to the last one re-photographs ground already covered. The
        robot's own position counts as swept, so the first choice walks away from
        wherever it started rather than spinning twice on the spot.
        """
        grid = self.map
        if grid is None:
            return None
        safe = self.free_mask(grid)
        if not safe.any():
            return None

        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        rows, cols = np.nonzero(safe)
        xs = ox + (cols + 0.5) * res
        ys = oy + (rows + 0.5) * res

        anchors = list(self.visited)
        here = self.robot_xy()
        if here is not None:
            anchors.append(here)
        if not anchors:
            return (float(xs[0]), float(ys[0]))

        nearest = np.full(xs.shape, np.inf)
        for ax, ay in anchors:
            nearest = np.minimum(nearest, np.hypot(xs - ax, ys - ay))

        best = int(np.argmax(nearest))
        if nearest[best] < VANTAGE_SPACING_M:
            return None  # everything reachable is already within a sweep's reach
        return (float(xs[best]), float(ys[best]))

    def pick_frontier(self):
        """Nearest known-free cell that touches unknown space, with clearance.

        Needed because vantage points can only be chosen inside the mapped area, and
        at the start of a run there is barely any: measured on the first live test,
        the explorer swept once on the spot and then declared every reachable cell
        already covered, having gone nowhere. Driving to a frontier grows the map so
        that there is somewhere to pick a vantage point from next time round.

        Nearest rather than furthest, unlike vantage points: the aim here is to buy
        new map as cheaply as possible, not to cover ground with the camera.
        """
        grid = self.map
        if grid is None:
            return None
        data = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width
        )
        unknown = data < 0
        # 4-neighbour dilation of the unknown region, intersected with clear floor.
        touches = np.zeros_like(unknown)
        touches[1:, :] |= unknown[:-1, :]
        touches[:-1, :] |= unknown[1:, :]
        touches[:, 1:] |= unknown[:, :-1]
        touches[:, :-1] |= unknown[:, 1:]
        frontier = touches & (data >= 0) & (data < 50) & self.free_mask(
            grid, unknown_blocks=False
        )
        if not frontier.any():
            return None

        res = grid.info.resolution
        rows, cols = np.nonzero(frontier)
        xs = grid.info.origin.position.x + (cols + 0.5) * res
        ys = grid.info.origin.position.y + (rows + 0.5) * res

        here = self.robot_xy()
        if here is None:
            return (float(xs[0]), float(ys[0]))
        distance = np.hypot(xs - here[0], ys - here[1])

        # Two filters, both learned from watching this shuffle on the spot. Taking
        # simply the nearest frontier had the robot step 0.2-0.8 m at a time between
        # (0.63, 0.49), (0.78, -0.26) and (0.88, -0.11) and go nowhere: slam_toolbox
        # keeps regenerating a ragged edge a few centimetres away, so "nearest" is
        # always right here. Requiring a worthwhile step, and refusing ground already
        # driven to, forces each move to actually buy map.
        eligible = distance > MIN_FRONTIER_TRAVEL_M
        for vx, vy in self.visited:
            eligible &= np.hypot(xs - vx, ys - vy) > VANTAGE_SPACING_M
        if not eligible.any():
            return None

        best = int(np.argmin(np.where(eligible, distance, np.inf)))
        return (float(xs[best]), float(ys[best]))

    # ------------------------------------------------------------------ actions

    def explore(self, target_id, budget_s=DEFAULT_BUDGET_S):
        deadline = time.time() + budget_s
        self.say(f"Recherche du tag {target_id}.")

        # It may already be in view, in which case none of this is needed.
        if target_id in self.seen_tags:
            self.say(f"Tag {target_id} déjà connu.")
            return True

        # Sweep where we stand before driving anywhere: the cheapest look available
        # - unless we already swept from here. Searching for the next tag starts
        # wherever the last one ended, so without this check every tag after the
        # first pays 45 s to re-photograph the identical view.
        here = self.robot_xy()
        if here is None or self.swept_from is None or math.hypot(
            here[0] - self.swept_from[0], here[1] - self.swept_from[1]
        ) > RESWEEP_DISTANCE_M:
            self.sweep(deadline - time.time())
        if target_id in self.seen_tags:
            self.say(f"Tag {target_id} trouvé sans se déplacer.")
            return True

        forgets = 0
        while rclpy.ok() and time.time() < deadline:
            # Prefer a vantage point: a sweep from new ground is what actually finds
            # tags. Fall back to a frontier only when the mapped area is too small to
            # hold one, which is the normal state of affairs at the start of a run.
            target = self.pick_vantage()
            kind = "poste d'observation"
            if target is None:
                target = self.pick_frontier()
                kind = "frontière"
            if target is None and self.visited and forgets < MAX_FORGETS:
                # Everything is filtered out as already-swept - but `visited`
                # accumulates across searches and is never cleared, so by the second
                # tag of a race every candidate looks stale and the search gives up
                # without moving. Measured on seed 12: tag 1 used its whole 135 s,
                # then tags 9, 4 and 7 each died in 45 s - the initial spin and then
                # "nothing to explore", three budgets thrown away.
                #
                # Forgetting is capped, because forgetting without a cap is worse
                # than not forgetting at all: the first version of this looped on the
                # single reachable cell, re-offering (-0.19, -1.19) for the whole
                # budget. If the known map really does hold one spot, the answer is
                # to stop and let the next tag have the time.
                forgets += 1
                self.say(
                    f"Postes épuisés - on oublie les anciens ({forgets}/{MAX_FORGETS})."
                )
                self.visited.clear()
                target = self.pick_vantage() or self.pick_frontier()
                kind = "poste d'observation"
            if target is None:
                self.say("Plus rien à explorer dans la carte connue.")
                return target_id in self.seen_tags

            self.say(f"Cap sur {kind} ({target[0]:.2f}, {target[1]:.2f}).")
            reached = self.goto(target, deadline - time.time())
            # Record it either way: unreachable targets must not be chosen again, or
            # the search picks the same impossible cell for the rest of the budget.
            self.visited.append(target)
            if not reached:
                continue

            self.sweep(deadline - time.time())
            if target_id in self.seen_tags:
                self.say(f"Tag {target_id} trouvé.")
                return True

        self.say(f"Tag {target_id} introuvable dans le temps imparti.")
        return False

    def goto(self, xy, budget_s):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ""
        handle = self.send(self.nav_client, goal)
        if handle is None:
            return False
        return self.await_result(handle, min(GOTO_TIMEOUT_S, budget_s))

    def sweep(self, budget_s=None):
        """One full turn on the spot, so the camera sees every bearing from here.

        Nav2's Spin is collision-checked against the local costmap, so a turn that
        would clip furniture stops rather than pushing through it - which matters,
        because contacts are scored.
        """
        allowance = SPIN_TIME_ALLOWANCE_S
        if budget_s is not None:
            allowance = min(allowance, max(0.0, budget_s))
        if allowance <= 1.0:
            return
        self.swept_from = self.robot_xy()
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = rclpy.duration.Duration(seconds=allowance).to_msg()
        handle = self.send(self.spin_client, goal)
        if handle is not None:
            self.await_result(handle, allowance + 15.0)

    def send(self, client, goal, attempts=GOAL_SEND_ATTEMPTS):
        """Retrying send, polled rather than spun on - the executor thread in main()
        is what completes these futures, and spinning this node here would deadlock
        exactly the way tag_goal_bridge.on_tag_request documents."""
        for attempt in range(1, attempts + 1):
            if not client.wait_for_server(timeout_sec=5.0):
                continue
            future = client.send_goal_async(goal)
            end = time.time() + GOAL_SEND_TIMEOUT_S
            while rclpy.ok() and not future.done() and time.time() < end:
                time.sleep(0.1)
            handle = future.result() if future.done() else None
            if handle is not None and handle.accepted:
                return handle
            if attempt < attempts:
                time.sleep(1.0)
        return None

    def await_result(self, handle, timeout_s):
        """Wait, and CANCEL on timeout rather than walking away from a live goal.

        Leaving it running was measured costing real time: a 90 s search budget took
        165 s, because explore() only checks the clock between targets while a drive
        underneath it can run for two minutes. In a race that overrun comes straight
        out of the tags still ahead.
        """
        if timeout_s <= 0:
            self.cancel(handle)
            return False
        future = handle.get_result_async()
        end = time.time() + timeout_s
        while rclpy.ok() and not future.done() and time.time() < end:
            time.sleep(0.1)
        if not future.done():
            self.cancel(handle)
            return False
        return True

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        end = time.time() + 5.0
        while rclpy.ok() and not future.done() and time.time() < end:
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = TagExplorer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        while rclpy.ok():
            if node.pending_request is not None:
                target = node.pending_request
                node.pending_request = None
                node.explore(target)
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
