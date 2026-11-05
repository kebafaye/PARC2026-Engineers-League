#!/usr/bin/env python3
"""Shared contact detection and backoff, used by task_solution.py and tag_goal_bridge.py.

These values are measured, not chosen, and the reasoning behind each one is recorded
below. They live here rather than in either node because both need exactly the same
behaviour and a divergence between them would be silent: a contact reflex that is
subtly wrong still looks like it is working right up until the robot is on its side.
That is not hypothetical - an earlier version of the dedup logic remembered every
(sensor, object) pair forever, so a *second* collision with the same table was
silently ignored, and the robot pushed against it and tipped over with its own report
still reading "contacts: 1".
"""
try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - the bumper topics are simulation-only
    Contacts = None

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp

# Bumper topics. The organisers moved these under /sim/ and said plainly that
# /sim/ topics are for testing and must not be used in a solution - so nothing in
# the race path subscribes to them any more. They stay named here because
# task_solution.py (Phase 1, simulation only) still reads them, and because
# StallDetector below exists precisely to replace them.
COLLISION_TOPICS = (
    "/sim/base_collisions",
    "/sim/top_chassis_collisions",
    "/sim/left_wheel_collisions",
    "/sim/right_wheel_collisions",
)

# The bumper sensors report permanent contact with the restaurant floor, which is not
# a collision in the sense the competition scores. Without this filter the contact
# count is inflated from the first second.
GROUND_CONTACT_NAMES = ("main_floor", "ground_plane")

# How long a (sensor, object) pair must go quiet before touching it again counts as a
# fresh contact. Below this it is the same sustained push, reported at sensor rate.
# 2 s is comfortably longer than the gap between messages during one continuous push,
# and comfortably shorter than a backoff-and-replan cycle, so a genuine second
# incident is never mistaken for the first.
CONTACT_COOLDOWN_S = 2.0

# On contact: stop, back off, then let the planner try again with the obstacle now on
# the map. Without this reflex the robot keeps executing the path it was on - pushing
# against whatever it just hit - until a new plan arrives seconds later. It did exactly
# that against a cafe_table and TIPPED OVER: 77 s into the run, 6 m from the goal, on
# its side with no way back. A robot whose centre of mass sits 0.30 m up over a 0.17 m
# half-track cannot afford to lean on anything.
#
# The backoff itself must not be blind. An earlier version published raw Twist commands
# straight backward for a fixed duration, with no idea what was behind the robot - and
# it backed straight into a WoodenChair it never checked for, turning one contact into
# two. Nav2's own BackUp behavior (behavior_server already loads it) walks the same
# path nav2_regulated_pure_pursuit_controller does for collision checking: it queries
# the local costmap along the reverse path and stops early if that path is blocked,
# rather than trusting the caller to have picked a safe distance.
BACKOFF_DIST_M = 0.35
BACKOFF_SPEED_MS = 0.12
BACKOFF_TIME_ALLOWANCE_S = 8.0


def is_new_contact(last_seen_s, topic, contact, now_s):
    """True if this contact is a fresh incident rather than an ongoing push.

    `last_seen_s` is a caller-owned dict used as the dedup state; it is updated in
    place on every call, including the ones that return False, so that a sustained
    push keeps refreshing its own cooldown instead of expiring mid-contact.
    """
    names = f"{contact.collision1.name} {contact.collision2.name}"
    if any(ground in names for ground in GROUND_CONTACT_NAMES):
        return False
    pair = (topic, contact.collision1.name, contact.collision2.name)
    last = last_seen_s.get(pair)
    last_seen_s[pair] = now_s
    return last is None or now_s - last >= CONTACT_COOLDOWN_S


def build_backup_goal():
    """The BackUp goal both nodes send after a contact."""
    goal = BackUp.Goal()
    goal.target = Point(x=BACKOFF_DIST_M)
    goal.speed = BACKOFF_SPEED_MS
    goal.time_allowance = Duration(sec=int(BACKOFF_TIME_ALLOWANCE_S))
    return goal


# Below this the robot is not meaningfully moving. Wheel odometry on a stalled
# differential drive still reports small nonzero values as the wheels creep and slip.
STALL_SPEED = 0.03          # m/s and rad/s
STALL_PATIENCE_S = 3.0      # how long that must persist while Nav2 is driving
STALL_COOLDOWN_S = 8.0      # one shove is one event, not one per cycle


class StallDetector:
    """Contact detection for a robot with no bumpers.

    COLLISION_TOPICS above are Gazebo contact sensors. The real Sito-É publishes
    nothing of the kind - checked against the organisers' own topic dump - so on the
    robot the reflex those topics feed would simply never fire, and the run would
    drive into furniture with no protection at all. Each contact costs 2 to 5 of the
    20 safe-navigation points.

    What every robot does have is odometry. If Nav2 is driving and the wheels report
    nothing moving, something is stopping the robot, and on this course that is
    almost always furniture. That is exactly the state the tip-over happened in:
    the controller logged "Failed to make progress" while the robot leaned on
    cafe_table_6 for a minute and a half.

    Deliberately biased towards firing. A false positive costs one unnecessary
    backoff, a second or two; a false negative costs points and possibly the robot.
    Nav2's own `wait` recovery will trip it occasionally - backing off from a wait is
    harmless.

    Pure and side-effect free so it can be exercised without a robot; the node feeds
    it odometry and whether a goal is currently active.
    """

    def __init__(self, speed_floor=STALL_SPEED, patience_s=STALL_PATIENCE_S,
                 cooldown_s=STALL_COOLDOWN_S):
        self.speed_floor = speed_floor
        self.patience_s = patience_s
        self.cooldown_s = cooldown_s
        self.stalled_since = None
        self.last_event_s = None

    def update(self, now_s, linear, angular, navigating):
        """True exactly once per stall. `linear`/`angular` are odometry twists."""
        if not navigating:
            self.stalled_since = None
            return False

        moving = abs(linear) > self.speed_floor or abs(angular) > self.speed_floor
        if moving:
            self.stalled_since = None
            return False

        if self.stalled_since is None:
            self.stalled_since = now_s
            return False
        if now_s - self.stalled_since < self.patience_s:
            return False

        if self.last_event_s is not None and now_s - self.last_event_s < self.cooldown_s:
            return False

        self.last_event_s = now_s
        self.stalled_since = None
        return True
