#!/usr/bin/env python3
"""Start the race on a spoken word. Worth +5 points, and worth losing a race over.

    ros2 run parc_solution voice_start.py --word allez --model ~/vosk-model-small-fr

Listens on the robot's ReSpeaker array and publishes once on /parc/start_race when it
hears the start word. race_director.py is already waiting on that topic, so nothing
here commands the robot - which matters more than it sounds, see below.

What the rules actually punish
------------------------------
The bonus is 5 points. Starting *early* - "par exemple déclenché par le bruit du
public" - restarts the race and forfeits the bonus entirely. So a false positive
costs far more than the feature is worth, and every choice below trades sensitivity
away for silence:

  * Recognition is **grammar-restricted** to the start word plus [unk]. Vosk can then
    only ever emit that word or nothing, so a room full of chatter decodes to [unk]
    instead of finding the nearest match in a 100k-word vocabulary. This is the
    single biggest difference between usable and hopeless.
  * Only **finalised utterances** count. Partial results flicker through plausible
    hypotheses on the way to a decision and are exactly where a stray match comes
    from.
  * The word must be the **whole utterance**, not a fragment of a longer one. "Allez
    les bleus" from the crowd is not a start command.
  * A **confidence floor** on the word itself.
  * A **settling period** after startup, while gain control is still converging.

The structural guarantee is separate from all of that: this node only publishes a
topic, and the robot moves only because race_director is blocked waiting for it. The
"must not move before the word" rule is satisfied by that blocking, not by this node
being clever - which is why it is safe for the team to simply not run this node and
publish the start by hand, losing the bonus and nothing else.

Offline, on the robot, as required. Vosk with a small model is the recommended route
and is imported lazily, so this file is importable - and its decision logic testable -
on a machine with neither vosk nor a microphone.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

START_TOPIC = "/parc/start_race"

# Accepted start words, one per language. The rules say handling more than one gains
# nothing, so pick the language the operator will actually speak and pass --word.
KNOWN_WORDS = ("allez", "go", "vai")

# Vosk reports per-word confidence on a 0-1 scale. 0.80 was chosen to sit well clear
# of the band where ambient speech produces matches; tune it during a test slot by
# reading the "candidat rejeté" lines this node logs for everything it hears.
MIN_CONFIDENCE = 0.80

# Ignore anything in the first moments: microphone gain control is still settling and
# the startup transient itself can decode as speech.
SETTLE_S = 2.0

SAMPLE_RATE = 16000
BLOCK_SIZE = 4000


def accept_result(payload, word, min_confidence=MIN_CONFIDENCE):
    """Decide whether a finalised Vosk result is the start command.

    Pure, and separated from all audio handling, because this is the function whose
    bugs cost a race rather than a log line - it can be exercised exhaustively with
    no microphone, no model and no robot.

    Returns (accepted, reason).
    """
    try:
        data = json.loads(payload) if isinstance(payload, str) else dict(payload)
    except (ValueError, TypeError):
        return False, "résultat illisible"

    text = (data.get("text") or "").strip().lower()
    if not text:
        return False, "silence"

    # Whole utterance only. A match buried in a longer phrase is crowd noise, not a
    # command addressed to the robot.
    if text != word:
        return False, f"pas le mot seul ({text!r})"

    words = data.get("result") or []
    if not words:
        # Grammar-restricted decoding matched, but without per-word detail there is
        # nothing to threshold. Refuse rather than guess: silence is the safe failure.
        return False, "pas de confiance rapportée"

    confidence = min(float(w.get("conf", 0.0)) for w in words)
    if confidence < min_confidence:
        return False, f"confiance {confidence:.2f} < {min_confidence:.2f}"

    return True, f"confiance {confidence:.2f}"


class VoiceStart(Node):
    def __init__(self, word, model_path, device=None):
        super().__init__("voice_start")
        self.word = word
        self.model_path = model_path
        self.device = device
        self.published = False
        self.publisher = self.create_publisher(String, START_TOPIC, 10)
        self.get_logger().info(
            f"Mot de départ : {word!r} — publication sur {START_TOPIC} une seule fois."
        )

    def fire(self, reason):
        if self.published:
            return
        self.published = True
        self.get_logger().info(f"DÉPART ENTENDU ({reason}).")
        self.publisher.publish(String(data=self.word))

    def listen(self):
        """Capture from the microphone and feed Vosk until the word is heard."""
        try:
            import sounddevice
            import vosk
        except ImportError as exc:
            self.get_logger().error(
                f"{exc.name} absent. Installer sur le robot pendant un créneau "
                "d'essai : pip install vosk sounddevice, plus un petit modèle Vosk. "
                "Sans lui, lancer la course à la main - on perd 5 points, rien d'autre."
            )
            return 1

        model = vosk.Model(self.model_path)
        # The grammar is the whole defence against crowd noise: the decoder is given
        # the start word and an explicit unknown token, and can output nothing else.
        grammar = json.dumps([self.word, "[unk]"])
        recogniser = vosk.KaldiRecognizer(model, SAMPLE_RATE, grammar)
        recogniser.SetWords(True)

        started_at = time.time()
        self.get_logger().info("À l'écoute.")

        with sounddevice.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            dtype="int16",
            channels=1,
            device=self.device,
        ) as stream:
            while rclpy.ok() and not self.published:
                block, overflowed = stream.read(BLOCK_SIZE)
                if overflowed:
                    self.get_logger().warn("Débordement audio - bloc perdu.")
                # Finalised utterances only; partials are where false starts live.
                if not recogniser.AcceptWaveform(bytes(block)):
                    continue
                if time.time() - started_at < SETTLE_S:
                    continue
                accepted, reason = accept_result(recogniser.Result(), self.word)
                if accepted:
                    self.fire(reason)
                else:
                    # Logged rather than dropped: this is the record a test slot needs
                    # to tune MIN_CONFIDENCE against the real room.
                    self.get_logger().info(f"candidat rejeté - {reason}")
        return 0


def main(args=None):
    rclpy.init(args=args)

    word = "allez"
    if "--word" in sys.argv:
        word = sys.argv[sys.argv.index("--word") + 1].lower()
    if word not in KNOWN_WORDS:
        print(f"Mot inconnu {word!r} - attendus : {', '.join(KNOWN_WORDS)}")
        return 2

    model_path = None
    if "--model" in sys.argv:
        model_path = sys.argv[sys.argv.index("--model") + 1]

    device = None
    if "--device" in sys.argv:
        device = sys.argv[sys.argv.index("--device") + 1]

    node = VoiceStart(word, model_path, device)
    code = 1
    try:
        if "--fire-now" in sys.argv:
            # Rehearsal without a microphone: proves the wiring into race_director
            # independently of whether anything can hear.
            node.fire("déclenchement manuel")
            code = 0
        elif model_path is None:
            node.get_logger().error("--model <chemin vers un modèle Vosk> est requis.")
            code = 2
        else:
            code = node.listen()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
