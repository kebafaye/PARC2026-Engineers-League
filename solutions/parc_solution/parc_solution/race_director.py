#!/usr/bin/env python3
"""Run the whole scored race: read the tag order, find each tag, visit it, report.

    ros2 run parc_solution race_director.py

Drives tag_explorer.py and tag_goal_bridge.py directly rather than talking to them
over topics. Both are plain Nodes, so this process owns one executor and calls their
methods, which means one thing to launch under a 10-minute clock, no inter-process
handshake to debug, and real return values instead of parsed log lines.

The ordering rule is what shapes the loop
-----------------------------------------
Tags must be visited in the published order. A tag that cannot be found may be
skipped, scoring nothing, and **cannot be returned to**; later tags still score as
long as they keep the given order. With the order 7, 1, 9, 4 that means 7-1-9-4
scores all four, 7-9-4 scores three, and visiting 9 before 7 throws 7 away entirely.

So the position in the order is a pointer that only ever moves forward. Giving up on
a tag costs 20 points; going back for it costs every point after it. When the budget
runs short the right move is to abandon the current tag, not to keep hunting.

Talking to the referee
----------------------
All five /parc/ topics live in referee.py. Three of its obligations shape this file
rather than that one:

  * While the referee says ARMED, the robot must not move - any motion before the
    start signal is a false start. So nothing is commanded until the signal is sent.
  * Reporting the last tag in the order ends the run by itself. Finish is published
    only when tags were skipped and that last tag will never be reported.
  * The referee's clock is the only one that counts. Our own budget paces the search;
    run_status.elapsed_s decides whether time is left.

`--order` and `--budget` still allow a dry run with no referee present.
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from parc_solution.referee import Referee
from parc_solution.tag_explorer import TagExplorer
from parc_solution.tag_goal_bridge import TagGoalBridge

# Rules: 10 minutes from the start command or the spoken word.
RACE_BUDGET_S = 600.0

# Stop hunting with this much left and spend it on the tags still ahead. A tag found
# with no time to drive to it scores nothing, so the last stretch belongs to whatever
# is already in the cache.
ENDGAME_RESERVE_S = 60.0

# Never spend more than this share of what remains on any single tag, so one
# stubborn tag cannot eat the tags behind it.
MAX_SHARE_PER_TAG = 0.5

# Below this there is no point setting off for a tag already in the cache: Nav2
# needs time to plan, drive and settle inside the 0.5 m / 12 deg window, and a stop
# that does not qualify scores exactly the same as not going at all.
MIN_APPROACH_S = 25.0

# Our own voice node publishes the word it heard here; the referee is told about it
# through StartSignal.heard, which is what earns the bonus.
DEFAULT_VOICE_TOPIC = "/parc/start_race"
DEFAULT_STATUS_TOPIC = "/parc/race_status"

# The camera the visit photo comes from. camera1 is the upper one, and the
# robot publishes raw frames only - referee.py encodes the JPEG itself.
DEFAULT_CAMERA_TOPIC = "/camera1/camera1/color/image_raw"


class RaceDirector:
    def __init__(self, explorer, bridge, node):
        self.explorer = explorer
        self.bridge = bridge
        self.node = node
        self.log = node.get_logger()

        node.declare_parameter("voice_topic", DEFAULT_VOICE_TOPIC)
        node.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        node.declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC)

        self.referee = Referee(node, node.get_parameter("camera_topic").value)
        self.order = None
        self.heard = None

        node.create_subscription(
            String, node.get_parameter("voice_topic").value, self.on_voice, 10
        )
        self.status_pub = node.create_publisher(
            String, node.get_parameter("status_topic").value, 10
        )

    # ------------------------------------------------------------------ inputs

    def on_voice(self, msg):
        """The start word, from voice_start.py. Recorded rather than acted on: the
        referee wants it inside StartSignal, and the bonus depends on it arriving
        there rather than on the robot simply moving."""
        if self.heard is None:
            self.heard = msg.data
            self.log.info(f"Mot de départ entendu : {self.heard!r}")

    def report(self, text):
        """Everything the judges and the team need to follow the run, in one place.

        The organisers' own status package replaces this topic once its format is
        published; until then a human reading one topic can referee the run.
        """
        self.log.info(text)
        self.status_pub.publish(String(data=text))

    # ------------------------------------------------------------------ the race

    def wait_for(self, predicate, what, timeout_s=None):
        deadline = None if timeout_s is None else time.time() + timeout_s
        announced = False
        while rclpy.ok() and not predicate():
            if not announced:
                self.log.info(f"En attente : {what}")
                announced = True
            if deadline is not None and time.time() > deadline:
                return False
            time.sleep(0.2)
        return rclpy.ok()

    def run(self, order=None, wait_start=True, budget_s=RACE_BUDGET_S, voice=False):
        # The order comes from the referee, latched when the judges arm the run.
        if order is not None:
            self.order = order
            self.log.info(f"Ordre fourni en ligne de commande : {self.order}")
        else:
            if not self.wait_for(
                lambda: self.referee.order is not None, "l'ordre de l'arbitre"
            ):
                return 1
            self.order = self.referee.order

        # ARMED means the judges are ready and the robot must stay perfectly still:
        # moving now is a false start. Nothing below commands anything until the
        # start signal has gone out.
        if wait_start:
            what = "le mot de départ" if voice else "le signal de départ"
            if not self.wait_for(
                lambda: (self.heard is not None) if voice else self.referee.armed(),
                what,
            ):
                return 1

        self.referee.send_start(heard=self.heard if voice else None)

        started_at = time.time()
        self.report(f"DEPART - ordre {self.order}, budget {budget_s:.0f} s")

        visited, skipped = [], []
        for position, tag_id in enumerate(self.order):
            if self.referee.over():
                self.report(f"Arbitre : {self.referee.state_name()} - on s'arrête.")
                skipped.extend(self.order[position:])
                break

            # The referee's clock is authoritative; ours only paces the search when
            # no referee is present.
            remaining = self.referee.remaining_s(budget_s - (time.time() - started_at))
            ahead = len(self.order) - position
            if remaining <= 0:
                self.report("Temps écoulé.")
                skipped.extend(self.order[position:])
                break

            budget = self.budget_for(remaining, ahead)
            self.report(
                f"Tag {tag_id} ({position + 1}/{len(self.order)}) - "
                f"{remaining:.0f} s restantes, {budget:.0f} s allouées."
            )

            reached, qualified = self.visit(tag_id, budget, remaining)

            if reached:
                # Report whatever we reached, qualifying by our own reckoning or not.
                # The document is explicit that "whether a visit scores is decided by
                # the judges" - so our verdict is advice, not a gate. Staying silent
                # about a stop we measured at 13 deg guarantees zero; reporting it
                # costs nothing and might be judged inside the limit. Only a tag we
                # never drove to goes unreported.
                self.referee.report_visit(tag_id)
                visited.append(tag_id)
                self.report(
                    f"TAG {tag_id} SIGNALE"
                    + ("." if qualified else " (hors tolérance selon nous - aux juges de trancher).")
                )
            else:
                # Forward only. Coming back for this one would void every tag after
                # it, which costs far more than the 20 points being left here.
                skipped.append(tag_id)
                self.report(f"TAG {tag_id} ABANDONNE - on passe au suivant.")

        # Reporting the last tag of the order ends the run on its own. Finish is for
        # the other case: tags were skipped, so that last report will never come and
        # the clock would otherwise run to the full ten minutes.
        if self.order and self.order[-1] not in visited and not self.referee.over():
            self.referee.send_finish(completed=False)

        elapsed = time.time() - started_at
        self.report(
            f"FIN - visités {visited}, abandonnés {skipped}, {elapsed:.0f} s, "
            f"{len(visited) * 25} points de tags et navigation sûre (hors pénalités)."
        )
        return 0

    def budget_for(self, remaining, tags_ahead):
        """Time this tag may take before it starts costing the ones behind it."""
        fair_share = max(0.0, remaining - ENDGAME_RESERVE_S) / max(1, tags_ahead)
        return min(fair_share, remaining * MAX_SHARE_PER_TAG)

    def visit(self, tag_id, budget_s, remaining_s):
        """Find the tag if needed and drive to it.

        Returns (reached, qualified): whether an approach was actually carried out,
        and whether our own measurement puts the stop inside the rules. The caller
        reports on `reached` and merely logs `qualified` - the judges score it.

        The budget gates *searching*, not *going*. A tag already in the cache is
        worth driving to with whatever is left on the clock - the offline check
        caught this costing a free 20 points, refusing a known tag with 40 s in hand
        because the endgame reserve had taken the computed budget to zero. Hunting
        late is what wastes time; collecting late is just collecting.
        """
        if tag_id not in self.bridge.tag_cache:
            if budget_s <= 0:
                self.log.warn(f"Pas de temps pour chercher le tag {tag_id}.")
                return False, False
            if not self.explorer.explore(tag_id, budget_s=budget_s):
                return False, False

            # explore() returns on the *detection*; the pose reaches the bridge's
            # cache through its own /detections callback, a beat behind.
            if not self.wait_for(
                lambda: tag_id in self.bridge.tag_cache, f"la pose du tag {tag_id}", 10.0
            ):
                self.log.warn(f"Tag {tag_id} vu mais sans pose exploitable.")
                return False, False

        if remaining_s < MIN_APPROACH_S:
            self.log.warn(
                f"Tag {tag_id} connu mais il reste {remaining_s:.0f} s - trop court "
                "pour l'atteindre."
            )
            return False, False

        # Leave the endgame reserve intact for the tags behind this one. The
        # approach always ends with the robot stopped at wherever it got to, so
        # `reached` is True regardless of the verdict.
        qualified = self.bridge.handle_request(
            tag_id, budget_s=max(MIN_APPROACH_S, remaining_s - ENDGAME_RESERVE_S)
        )
        return True, qualified


def main(args=None):
    rclpy.init(args=args)

    explorer = TagExplorer()
    bridge = TagGoalBridge()
    director_node = rclpy.create_node("race_director")
    director_node.set_parameters(
        [rclpy.parameter.Parameter("use_sim_time", value=True)]
    )
    director = RaceDirector(explorer, bridge, director_node)

    # One executor for all three: the workers' callbacks - detections, TF, action
    # responses - must keep running while run() blocks waiting on them.
    executor = SingleThreadedExecutor()
    for node in (explorer, bridge, director_node):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    budget = RACE_BUDGET_S
    if "--budget" in sys.argv:
        # Rehearsal only. Test slots are 15 minutes, so replaying a full 10-minute
        # race inside one leaves no room to fix what it shows.
        budget = float(sys.argv[sys.argv.index("--budget") + 1])

    order = None
    if "--order" in sys.argv:
        order = [int(x) for x in sys.argv[sys.argv.index("--order") + 1].split(",")]
    wait_start = "--no-wait-start" not in sys.argv
    voice = "--voice" in sys.argv

    code = 1
    try:
        code = director.run(order=order, wait_start=wait_start,
                            budget_s=budget, voice=voice)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in (explorer, bridge, director_node):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
