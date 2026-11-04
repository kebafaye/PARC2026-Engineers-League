#!/usr/bin/env python3
"""A stand-in for the organisers' referee, so the interface can be rehearsed.

    python3 fake_referee.py --order 7,1,9,4

Speaks the five /parc/ topics exactly as the Referee Topics document describes them,
including the transient-local QoS on the two it publishes - which is the part most
likely to be got wrong, because getting it wrong fails silently rather than loudly.

It also enforces the rules a real referee enforces, so a rehearsal can actually fail:
the clock starts when StartSignal arrives, reporting the last tag of the order ends
the run, Finish ends it early, and the ten-minute limit times it out.

Test fixture. Never deployed - the real referee runs on the robot.
"""
import sys
import time

import rclpy
from rclpy.node import Node

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

sys.path.insert(0, __file__.rsplit("/", 1)[0])


def latched():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def events():
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


class FakeReferee(Node):
    def __init__(self, order, time_limit_s):
        super().__init__("fake_referee")
        self.order = order
        self.time_limit_s = time_limit_s
        self.state = RunStatus.ARMED      # as if the judges had just armed the run
        self.started_at = None
        self.reported = []
        self.start_method = StartSignal.METHOD_COMMAND
        self.visits_with_image = 0
        self.visits_without_image = 0

        self.order_pub = self.create_publisher(TagOrder, "/parc/tag_order", latched())
        self.status_pub = self.create_publisher(RunStatus, "/parc/run_status", latched())
        self.create_subscription(StartSignal, "/parc/start", self.on_start, events())
        self.create_subscription(TagVisit, "/parc/tag_visit", self.on_visit, events())
        self.create_subscription(Finish, "/parc/finish", self.on_finish, events())

        msg = TagOrder()
        msg.stamp = self.get_clock().now().to_msg()
        msg.tag_ids = [int(t) for t in order]
        self.order_pub.publish(msg)
        self.get_logger().info(f"ARMED - ordre publié {order}")

        self.create_timer(0.5, self.publish_status)

    def elapsed(self):
        return 0.0 if self.started_at is None else time.time() - self.started_at

    def on_start(self, msg):
        if self.started_at is not None:
            self.get_logger().warn("StartSignal en double - ignoré.")
            return
        self.started_at = time.time()
        self.start_method = msg.method
        self.state = RunStatus.RUNNING
        how = "VOIX" if msg.method == StartSignal.METHOD_VOICE else "COMMANDE"
        self.get_logger().info(f"RUNNING - départ par {how} (entendu {msg.heard!r})")

    def on_visit(self, msg):
        if self.state != RunStatus.RUNNING:
            self.get_logger().warn(f"Visite hors course (état {self.state}) - ignorée.")
            return
        has_image = len(msg.image.data) > 0
        self.visits_with_image += has_image
        self.visits_without_image += not has_image
        self.reported.append(int(msg.tag_id))
        self.get_logger().info(
            f"VISITE tag {msg.tag_id} à {self.elapsed():.0f} s - "
            f"{'image ' + str(len(msg.image.data)) + ' octets' if has_image else 'SANS IMAGE'}"
        )
        if self.order and int(msg.tag_id) == int(self.order[-1]):
            self.state = RunStatus.FINISHED
            self.get_logger().info("FINISHED - dernier tag de l'ordre signalé.")

    def on_finish(self, msg):
        if self.state == RunStatus.RUNNING:
            self.state = RunStatus.FINISHED
            self.get_logger().info(
                f"FINISHED - Finish reçu (raison {msg.reason}) à {self.elapsed():.0f} s"
            )

    def publish_status(self):
        if self.state == RunStatus.RUNNING and self.elapsed() > self.time_limit_s:
            self.state = RunStatus.TIMED_OUT
            self.get_logger().warn("TIMED_OUT - limite atteinte.")
        msg = RunStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.elapsed_s = float(self.elapsed())
        msg.time_limit_s = float(self.time_limit_s)
        msg.tag_order = [int(t) for t in self.order]
        msg.tags_reported = [int(t) for t in self.reported]
        msg.start_method = self.start_method
        msg.false_start = False
        self.status_pub.publish(msg)


def main():
    order = [7, 1, 9, 4]
    if "--order" in sys.argv:
        order = [int(x) for x in sys.argv[sys.argv.index("--order") + 1].split(",")]
    limit = 600.0
    if "--limit" in sys.argv:
        limit = float(sys.argv[sys.argv.index("--limit") + 1])

    rclpy.init()
    node = FakeReferee(order, limit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"Bilan : {node.reported} signalés, "
            f"{node.visits_with_image} avec image, "
            f"{node.visits_without_image} sans."
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
