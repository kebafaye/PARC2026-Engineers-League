#!/usr/bin/env python3
"""Everything the solution says to, and hears from, the referee.

Five topics under /parc/, per the PARC 2026 Referee Topics document. Gathered in one
place because the QoS is not uniform and getting it wrong fails silently: the two
topics the referee publishes are **transient local**, so a subscriber using the
default volatile profile simply never receives the order that was latched before it
started - and the run would sit there waiting for something already sent.

Obligations in the document that are easy to miss and cost the run:

  * While the state is ARMED the robot must not move. Any movement before the start
    signal is flagged as a false start.
  * A TagVisit must carry a JPEG in which the tag is clearly visible, and the robot
    must then hold still for at least three seconds. A visit reported while rolling
    past is not a visit.
  * Reporting the last tag in the order ends the run by itself. Finish is only for
    ending early - when tags were skipped and the last one will never be reported.
  * When run_status reaches FINISHED, TIMED_OUT or TERMINATED the robot stops,
    whatever it was doing.

The clock belongs to the referee: it starts when the StartSignal is *received*, and
run_status carries elapsed_s. Our own timers are for pacing, never for deciding
whether time is left.
"""
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

TOPIC_ORDER = "/parc/tag_order"
TOPIC_STATUS = "/parc/run_status"
TOPIC_START = "/parc/start"
TOPIC_VISIT = "/parc/tag_visit"
TOPIC_FINISH = "/parc/finish"

# The document's own words: "hold still for at least 3 seconds". Held a little longer
# so that a late-arriving status sample still sees the robot stopped.
VISIT_HOLD_S = 3.5

# States in which the robot must not be driving.
TERMINAL_STATES = (RunStatus.FINISHED, RunStatus.TIMED_OUT, RunStatus.TERMINATED)

# How long to wait for the referee to be visible on a topic before publishing to it.
# Reliable QoS guarantees delivery to subscribers that are already *discovered*; a
# message published before discovery completes is simply dropped. Caught in
# rehearsal: the start signal went out, the log said "DÉPART envoyé", and the referee
# never left ARMED - the clock never started and the run would have scored nothing.
# The organisers' own example commands carry `-w 1` for the same reason.
DISCOVERY_WAIT_S = 5.0


def latched_qos():
    """For the two topics the referee publishes. Transient local, or the order that
    was latched before we subscribed never arrives."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def event_qos():
    """For the three topics we publish."""
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


class Referee:
    """The referee interface, attached to a node that owns the spinning."""

    def __init__(self, node, camera_topic="/top_camera_color/image_raw/compressed"):
        self.node = node
        self.log = node.get_logger()

        self.order = None
        self.status = None
        self.latest_image = None
        self._reported = set()
        self._started = False

        node.create_subscription(TagOrder, TOPIC_ORDER, self._on_order, latched_qos())
        node.create_subscription(RunStatus, TOPIC_STATUS, self._on_status, latched_qos())

        # The JPEG that goes with a visit. Subscribing to the camera's own compressed
        # stream avoids re-encoding: the referee wants a JPEG, and the driver already
        # publishes one.
        node.create_subscription(
            CompressedImage, camera_topic, self._on_image, event_qos()
        )

        self.start_pub = node.create_publisher(StartSignal, TOPIC_START, event_qos())
        self.visit_pub = node.create_publisher(TagVisit, TOPIC_VISIT, event_qos())
        self.finish_pub = node.create_publisher(Finish, TOPIC_FINISH, event_qos())

    # ------------------------------------------------------------------ inbound

    def _on_order(self, msg):
        if self.order is None:
            self.order = list(msg.tag_ids)
            self.log.info(f"Arbitre : ordre des tags {self.order}")

    def _on_status(self, msg):
        previous = self.status.state if self.status else None
        self.status = msg
        if msg.state != previous:
            self.log.info(
                f"Arbitre : état {self.state_name()} "
                f"({msg.elapsed_s:.0f}/{msg.time_limit_s:.0f} s, "
                f"reçus {list(msg.tags_reported)})"
            )
        if msg.false_start:
            self.log.error("Arbitre : FAUX DÉPART signalé.")

    def _on_image(self, msg):
        self.latest_image = msg

    # ------------------------------------------------------------------ queries

    def state_name(self):
        if self.status is None:
            return "inconnu"
        names = {
            RunStatus.IDLE: "IDLE",
            RunStatus.ARMED: "ARMED",
            RunStatus.RUNNING: "RUNNING",
            RunStatus.FINISHED: "FINISHED",
            RunStatus.TIMED_OUT: "TIMED_OUT",
            RunStatus.TERMINATED: "TERMINATED",
        }
        return names.get(self.status.state, str(self.status.state))

    def armed(self):
        return self.status is not None and self.status.state == RunStatus.ARMED

    def running(self):
        return self.status is not None and self.status.state == RunStatus.RUNNING

    def over(self):
        """True once the referee says to stop, whatever we were doing."""
        return self.status is not None and self.status.state in TERMINAL_STATES

    def remaining_s(self, fallback):
        """Time left on the REFEREE's clock, which is the only one that counts."""
        if self.status is None or self.status.time_limit_s <= 0:
            return fallback
        return max(0.0, self.status.time_limit_s - self.status.elapsed_s)

    # ------------------------------------------------------------------ outbound

    def _await_referee(self, publisher, what):
        """Block until the referee is subscribed, or give up and send anyway.

        See DISCOVERY_WAIT_S: publishing into a topic nobody has discovered yet is
        a silent loss, and for the start signal that loss costs the entire run.
        """
        deadline = time.time() + DISCOVERY_WAIT_S
        while time.time() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        self.log.warn(
            f"Aucun abonné sur {what} après {DISCOVERY_WAIT_S:.0f} s - "
            "envoi quand même. L'arbitre tourne-t-il ?"
        )
        return False

    def send_start(self, heard=None):
        """One StartSignal. The clock starts when the referee receives it."""
        if self._started:
            return
        self._started = True
        self._await_referee(self.start_pub, TOPIC_START)
        msg = StartSignal()
        msg.stamp = self.node.get_clock().now().to_msg()
        if heard:
            msg.method = StartSignal.METHOD_VOICE
            msg.heard = heard
        else:
            msg.method = StartSignal.METHOD_COMMAND
        self.start_pub.publish(msg)
        self.log.info(
            f"Arbitre : DÉPART envoyé ({'voix: ' + heard if heard else 'commande'})."
        )

    def report_visit(self, tag_id, hold_s=VISIT_HOLD_S):
        """Report one tag, then hold still. Returns False if no image was available.

        The caller must already be stopped in the visit pose: the hold below is the
        three seconds the rules ask for, and it is only still if nothing is driving.
        """
        if tag_id in self._reported:
            self.log.warn(f"Tag {tag_id} déjà signalé - on ne le renvoie pas.")
            return True

        if self.latest_image is None:
            # Reported anyway: a visit without a picture may still be judged, while
            # not reporting it certainly scores nothing.
            self.log.error(
                "Aucune image caméra disponible - visite signalée sans photo. "
                "Vérifier le topic caméra."
            )

        self._await_referee(self.visit_pub, TOPIC_VISIT)
        msg = TagVisit()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.tag_id = int(tag_id)
        if self.latest_image is not None:
            msg.image = self.latest_image
            msg.header.frame_id = self.latest_image.header.frame_id
        self.visit_pub.publish(msg)
        self._reported.add(tag_id)
        self.log.info(f"Arbitre : VISITE du tag {tag_id} signalée, immobile {hold_s} s.")

        time.sleep(hold_s)
        return self.latest_image is not None

    def send_finish(self, completed):
        """End the run early. Not needed when the last tag in the order was reported -
        that ends it by itself."""
        self._await_referee(self.finish_pub, TOPIC_FINISH)
        msg = Finish()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.reason = Finish.REASON_COMPLETED if completed else Finish.REASON_GAVE_UP
        self.finish_pub.publish(msg)
        self.log.info(
            f"Arbitre : FIN envoyée ({'terminé' if completed else 'abandon'})."
        )
