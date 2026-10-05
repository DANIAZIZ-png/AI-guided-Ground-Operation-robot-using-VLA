#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  voice_command.py  —  SPEECH INPUT FOR THE VLA AGENT   (v1)
#
#  WHAT IT IS
#    A microphone front-end. It listens, transcribes speech to text with
#    Whisper, and publishes that text on /vla/command — the exact same topic
#    the GUI uses and the exact same string the keyboard produces.
#
#  WHY IT IS A SEPARATE NODE (the viva answer)
#    The agent must not know how a command was produced. Speech recognition
#    is a NOISY, FALLIBLE front-end: it mishears, it truncates, it fires on
#    a cough. Keeping it outside the agent means
#      (a) every safeguard already in the agent (typo tolerance #25, the
#          bare-yes/no guard #26, the verb guardrail in llm_brain) applies to
#          spoken commands for free, with no new code paths to defend;
#      (b) ASR can be swapped, disabled or run on another machine without
#          touching a single line of robot control code;
#      (c) if the microphone node crashes, the robot is unaffected — it just
#          stops receiving spoken commands.
#    This is the same "modular pipeline" argument as the rest of the system:
#    the LLM proposes, deterministic code disposes. Here, ASR proposes.
#
#  THREE WAYS TO TALK TO IT
#    --mode ptt     PUSH TO TALK (default, and the one to demo). Press ENTER
#                   to start recording, ENTER again to stop. Zero false
#                   triggers, which is what you want in a room full of people.
#    --mode auto    HANDS-FREE. Continuously listens; a phrase is only sent
#                   if it starts with the WAKE WORD (default "robot").
#                   Endpointing is by energy: speech starts when the level
#                   rises above a floor calibrated from YOUR room, and ends
#                   after SILENCE_END seconds of quiet.
#    (always on)    REMOTE TRIGGER. /vla/voice/trigger (std_msgs/Bool):
#                   True = start recording, False = stop and transcribe.
#                   This is how the GUI's press-and-hold mic button works —
#                   the model stays loaded, so there is no reload latency.
#
#  TOPICS
#    OUT  /vla/command            std_msgs/String   the recognised command
#    OUT  /vla/voice/transcript   std_msgs/String   every transcript, incl.
#                                                   ones that were NOT sent
#    OUT  /vla/voice/state        std_msgs/String   idle|listening|recording|
#                                                   transcribing|error
#    IN   /vla/voice/trigger      std_msgs/Bool     GUI push-to-talk
#    IN   /vla/reply              std_msgs/String   (only if --speak)
#
#  INSTALL  (inside the ubuntu22-gpu distrobox)
#    sudo apt install -y portaudio19-dev
#    pip install sounddevice faster-whisper
#    # optional spoken replies:
#    sudo apt install -y espeak-ng
#
#  RUN
#    python3 voice_command.py                 # push-to-talk
#    python3 voice_command.py --mode auto     # wake word "robot"
#    python3 voice_command.py --speak         # also read replies aloud
#    python3 voice_command.py --list-devices  # find your mic index
#
#  SAFETY RULES BAKED IN
#    * "quit"/"exit" spoken aloud is NEVER forwarded. Shutting the agent down
#      is unrecoverable and a microphone must not be able to do it. (The
#      agent refuses it too — belt and braces.)
#    * "cancel"/"stop" IS forwarded, and is forwarded FIRST, before any
#      other filtering, because a stop that is filtered is not a stop.
#    * Transcripts shorter than MIN_CHARS, or below the model's own
#      confidence floor, are printed but not sent.
# ─────────────────────────────────────────────────────────────────

import argparse
import collections
import math
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

VOICE_VERSION = "voice_command v1"

# ── topics ──────────────────────────────────────────────────────────
CMD_TOPIC        = "/vla/command"
TRANSCRIPT_TOPIC = "/vla/voice/transcript"
STATE_TOPIC      = "/vla/voice/state"
TRIGGER_TOPIC    = "/vla/voice/trigger"
REPLY_TOPIC      = "/vla/reply"

# ── audio ───────────────────────────────────────────────────────────
SAMPLE_RATE   = 16000        # Whisper's native rate — resampling costs quality
BLOCK_MS      = 30           # audio callback granularity
MAX_UTTERANCE = 15.0         # s — hard cap so a stuck mic can't eat all RAM
MIN_UTTERANCE = 0.35         # s — anything shorter is a click, not a command

# ── hands-free endpointing (--mode auto) ────────────────────────────
CALIBRATE_S   = 1.5          # measure the room's noise floor for this long
SPEECH_MULT   = 3.0          # speech = this many times the noise floor RMS
SPEECH_FLOOR  = 0.008        # ...but never trigger below this absolute RMS
SILENCE_END   = 0.8          # s of quiet that ends a phrase
PREROLL_MS    = 300          # keep this much audio from BEFORE the trigger,
                             #   so the first syllable is never clipped

# ── lazy microphone (HARDWARE fix, 24 Sep 2026) ─────────────────────
# The PC's Wi-Fi and Bluetooth share one antenna (Intel AX201). A Bluetooth
# HEADSET link (needed for the mic) starves the robot's hotspot: measured
# 100 % packet loss / 6.5 Mbit/s with the earbuds connected vs 0 % / 72 Mbit/s
# without. This node normally holds the mic open for its whole life, so the
# headset link never sleeps. With VLA_VOICE_LAZY_MIC=1 the audio stream is
# stopped while idle and started only while push-to-talk is held, so the
# radio is shared for the few seconds you are actually speaking.
# Default "0" = the old behaviour exactly (simulation unaffected).
# Cost: no pre-roll, so press the button, wait for the warm-up, then speak.
LAZY_MIC      = os.environ.get("VLA_VOICE_LAZY_MIC", "0") == "1"
LAZY_WARMUP   = float(os.environ.get("VLA_VOICE_LAZY_WARMUP", "0.35"))   # s for the mic to settle

# ── transcript filtering ────────────────────────────────────────────
MIN_CHARS     = 3
# Whisper emits these for silence/noise. Sending them would make the robot
# "think" about nothing, waking Qwen for no reason.
JUNK = {"", "you", "thank you", "thanks", "bye", "uh", "um", "hmm", "mm",
        "okay", ".", "..", "...", "[blank_audio]", "[silence]", "(silence)",
        "thanks for watching", "thank you for watching", "subtitles by",
        "please subscribe"}
# never forwarded, whatever the operator says
BLOCKED = ("quit", "exit", "shutdown", "shut down")
# forwarded immediately, before any other check
PRIORITY = ("cancel", "stop", "halt", "abort")

WAKE_DEFAULT = "robot"
# 14 Sep (hardware): an optional vocabulary hint for Whisper. faster-whisper's
# initial_prompt biases decoding toward the words it contains, which is the
# standard remedy when a rare phrase is misheard as a common one ("go to the
# furthest person" -> "welcome to our dressing room" through clipped earbud
# audio). Empty (the default) = no prompt = exactly the previous behaviour;
# the hardware launcher sets it, the simulation never does.
VOICE_PROMPT = os.environ.get("VLA_VOICE_PROMPT", "").strip()


# ─────────────────────────────────────────────────────────────────
#  #v2  SAMPLE-RATE INDEPENDENCE
#
#  Whisper wants 16 kHz mono. Almost no sound card offers 16 kHz: consumer
#  hardware runs at 44100 or 48000, and Bluetooth headsets are presented by
#  PulseAudio at 48000 whatever the radio link is doing underneath. Demanding
#  16 kHz from the device is why the stream refused to open with
#  "Invalid sample rate [PaErrorCode -9997]".
#
#  So: open the device at a rate it actually supports, and convert in
#  software. Two stages, because skipping either one costs accuracy:
#    1. LOW-PASS. Throwing away samples without first removing the high
#       frequencies folds them back down as ALIASING — a hiss that Whisper
#       hears as consonants that were never spoken. A moving average of
#       length (rate_in / rate_out) is a crude but effective anti-alias
#       filter and costs nothing.
#    2. LINEAR INTERPOLATION down to exactly 16 kHz.
#  Speech lives below 4 kHz, far under the 8 kHz Nyquist limit of a 16 kHz
#  signal, so this is transparent for our purpose.
# ─────────────────────────────────────────────────────────────────
def resample_to_16k(audio, rate_in, rate_out=SAMPLE_RATE):
    """Mono float32 at rate_in -> mono float32 at rate_out."""
    if rate_in == rate_out or audio.size == 0:
        return audio.astype(np.float32)
    ratio = float(rate_in) / float(rate_out)
    if ratio > 1.0:
        k = int(round(ratio))
        if k >= 2:
            pad = (-len(audio)) % k
            if pad:
                audio = np.concatenate([audio, np.zeros(pad, dtype=np.float32)])
            # moving average = simple anti-alias low-pass before decimating
            audio = audio.reshape(-1, k).mean(axis=1).astype(np.float32)
            rate_in = rate_in / float(k)
    n_out = int(round(len(audio) * rate_out / float(rate_in)))
    if n_out <= 1:
        return np.zeros(0, dtype=np.float32)
    x_in = np.linspace(0.0, 1.0, num=len(audio), endpoint=False)
    x_out = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
    return np.interp(x_out, x_in, audio).astype(np.float32)


def pick_input_rate(sd, device):
    """Find a sample rate this microphone will actually accept.

    Tries 16 kHz first (free — no conversion needed), then the device's own
    reported default, then the usual suspects. Returns the first one that
    opens, so the node works on built-in cards, USB headsets and Bluetooth
    without the operator needing to know or care."""
    candidates = [SAMPLE_RATE]
    try:
        info = sd.query_devices(device if device is not None else None, "input")
        native = int(round(float(info.get("default_samplerate") or 0)))
        if native:
            candidates.append(native)
    except Exception:
        pass
    candidates += [48000, 44100, 32000, 22050, 16000, 8000]
    seen, out = set(), []
    for c in candidates:
        if c and c not in seen:
            seen.add(c); out.append(c)
    for rate in out:
        try:
            sd.check_input_settings(device=device, channels=1,
                                    dtype="float32", samplerate=rate)
            return rate
        except Exception:
            continue
    return None


def resolve_device(sd, spec):
    """--mic accepts an index (4) or a name fragment ('pulse', 'usb',
    'headset'). Names matter for Bluetooth, whose index changes every time it
    reconnects — 'pulse' is a stable handle where a number is not."""
    if spec is None:
        return None
    try:
        return int(spec)
    except (TypeError, ValueError):
        pass
    want = str(spec).lower()
    for i, d in enumerate(sd.query_devices()):
        if d.get("max_input_channels", 0) > 0 and want in d.get("name", "").lower():
            return i
    raise SystemExit(f"[voice] no INPUT device whose name contains {spec!r}. "
                     f"Run --list-devices to see what is available.")


def rms(x):
    """Root-mean-square level of a float32 block — a cheap loudness measure."""
    if x.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64))))


def clean(text):
    """Normalise a raw Whisper transcript into something the agent can parse.
    Whisper punctuates and capitalises; the agent's control-word matching
    (#25) works on bare lowercase words."""
    t = (text or "").strip().lower()
    t = re.sub(r"[^\w\s'-]", " ", t)       # drop punctuation, keep don't / re-scan
    t = re.sub(r"\s+", " ", t).strip()
    return t


# ─────────────────────────────────────────────────────────────────
#  ASR back end
# ─────────────────────────────────────────────────────────────────
class Transcriber:
    """Wraps whichever Whisper implementation is installed.

    faster-whisper is preferred: it is a CTranslate2 re-implementation, about
    4x quicker and lighter on VRAM than the reference package, which matters
    because Qwen2.5-7B is already resident on the same GPU. If it is missing
    we fall back to openai-whisper, and if that is missing too we say so
    plainly instead of failing halfway through a demo."""

    def __init__(self, model_size="small.en", device="auto", compute="auto"):
        self.backend = None
        self.model = None
        self.model_size = model_size
        if device == "auto":
            device = "cuda" if self._cuda_available() else "cpu"
        self.device = device
        if compute == "auto":
            compute = "float16" if device == "cuda" else "int8"
        self.compute = compute
        self._load()

    @staticmethod
    def _cuda_available():
        try:
            import torch
            return bool(torch.cuda.is_available())
        except Exception:
            # torch may not be installed at all; faster-whisper does not need
            # it. nvidia-smi existing is a decent second-best signal.
            return shutil.which("nvidia-smi") is not None

    def _load(self):
        t0 = time.time()
        try:
            from faster_whisper import WhisperModel
            print(f"[voice] loading faster-whisper '{self.model_size}' on "
                  f"{self.device} ({self.compute}) ...")
            try:
                self.model = WhisperModel(self.model_size, device=self.device,
                                          compute_type=self.compute)
            except Exception as e:
                # typical cause: no cuDNN/CUDA libs inside the container
                print(f"[voice] GPU load failed ({e}); falling back to CPU int8")
                self.device, self.compute = "cpu", "int8"
                self.model = WhisperModel(self.model_size, device="cpu",
                                          compute_type="int8")
            self.backend = "faster-whisper"
        except ImportError:
            try:
                import whisper
                name = self.model_size.replace(".en", "") if "large" in self.model_size \
                    else self.model_size
                print(f"[voice] faster-whisper not found; loading openai-whisper "
                      f"'{name}' ...")
                self.model = whisper.load_model(name)
                self.backend = "openai-whisper"
            except ImportError:
                print("[voice] No speech recogniser installed.\n"
                      "        pip install faster-whisper       (recommended)\n"
                      "        pip install openai-whisper       (alternative)")
                sys.exit(2)
        print(f"[voice] {self.backend} ready in {time.time() - t0:.1f}s")

    def transcribe(self, audio_f32):
        """audio_f32: mono float32 in [-1, 1] at SAMPLE_RATE.
        Returns (text, mean_logprob or None)."""
        if self.backend == "faster-whisper":
            segments, _info = self.model.transcribe(
                audio_f32,
                language="en",
                beam_size=5,
                vad_filter=True,                       # drop leading/trailing silence
                condition_on_previous_text=False,      # each command stands alone —
                                                       # stops one misheard phrase
                                                       # from biasing the next
                initial_prompt=VOICE_PROMPT or None,   # vocabulary hint (see top)
            )
            parts, lps = [], []
            for s in segments:
                parts.append(s.text)
                lp = getattr(s, "avg_logprob", None)
                if lp is not None:
                    lps.append(lp)
            return " ".join(parts).strip(), (sum(lps) / len(lps) if lps else None)
        # openai-whisper
        r = self.model.transcribe(audio_f32, language="en",
                                  condition_on_previous_text=False, fp16=False)
        segs = r.get("segments") or []
        lps = [s.get("avg_logprob") for s in segs if s.get("avg_logprob") is not None]
        return (r.get("text") or "").strip(), (sum(lps) / len(lps) if lps else None)


# ─────────────────────────────────────────────────────────────────
#  ROS node
# ─────────────────────────────────────────────────────────────────
class VoiceCommandNode(Node):
    def __init__(self, args):
        super().__init__("vla_voice_command")
        self.args = args
        self.cmd_pub   = self.create_publisher(String, CMD_TOPIC, 10)
        self.trans_pub = self.create_publisher(String, TRANSCRIPT_TOPIC, 10)
        self.state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self.create_subscription(Bool, TRIGGER_TOPIC, self.on_trigger, 10)
        if args.speak:
            self.create_subscription(String, REPLY_TOPIC, self.on_reply, 10)
            self.tts_q = queue.Queue()
            threading.Thread(target=self.tts_worker, daemon=True).start()

        self.asr = Transcriber(args.model, args.device, args.compute)

        self.recording = False
        self.frames = []
        self.capture_rate = SAMPLE_RATE      # set by main() once the mic opens
        self.record_start = 0.0
        self.lock = threading.Lock()
        self.noise_floor = SPEECH_FLOOR
        self.level = 0.0                     # live level, for the meter
        self.stream = None                       # set by main(); used by LAZY_MIC
        self.preroll = collections.deque(
            maxlen=max(1, int(PREROLL_MS / BLOCK_MS)))
        self.shutdown = False
        self.work = queue.Queue()            # completed utterances -> ASR thread
        threading.Thread(target=self.asr_worker, daemon=True).start()
        self.set_state("idle")

    # ── state / output ──────────────────────────────────────────────
    def set_state(self, s):
        self.state = s
        m = String(); m.data = s
        try:
            self.state_pub.publish(m)
        except Exception:
            pass

    def publish_command(self, text):
        m = String(); m.data = text
        self.cmd_pub.publish(m)
        print(f"[voice] --> SENT: {text!r}")

    def publish_transcript(self, text, sent):
        m = String(); m.data = f"{'SENT' if sent else 'HELD'}: {text}"
        try:
            self.trans_pub.publish(m)
        except Exception:
            pass

    # ── remote push-to-talk from the GUI ────────────────────────────
    def on_trigger(self, msg):
        if msg.data:
            self.start_recording("gui")
        else:
            self.stop_recording()

    # ── optional spoken replies ─────────────────────────────────────
    def on_reply(self, msg):
        txt = (msg.data or "").strip()
        # only speak the robot's own sentences, not debug chatter
        if not txt or txt.startswith("(") or txt.startswith("["):
            return
        self.tts_q.put(txt)

    def tts_worker(self):
        engine = shutil.which("espeak-ng") or shutil.which("espeak")
        if not engine:
            print("[voice] --speak requested but espeak-ng is not installed "
                  "(sudo apt install espeak-ng)")
            return
        while not self.shutdown:
            try:
                txt = self.tts_q.get(timeout=0.3)
            except queue.Empty:
                continue
            # never speak while recording — the robot would hear itself and
            # transcribe its own voice as a command
            while self.recording and not self.shutdown:
                time.sleep(0.05)
            try:
                subprocess.run([engine, "-s", "150", txt],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=25)
            except Exception:
                pass

    # ── audio capture ───────────────────────────────────────────────
    def audio_cb(self, indata, frames_n, time_info, status):
        """sounddevice calls this from ITS OWN thread every BLOCK_MS. Keep it
        short: copy the block, measure it, and get out. Anything slow here
        causes input overflow and clipped words."""
        block = indata[:, 0].copy()
        self.level = rms(block)
        with self.lock:
            if self.recording:
                self.frames.append(block)
            else:
                self.preroll.append(block)

    def start_recording(self, why=""):
        if LAZY_MIC and self.stream is not None and not self.stream.active:
            try:
                self.stream.start()
                time.sleep(LAZY_WARMUP)          # let the headset link settle
            except Exception as e:
                print(f"[voice] could not start the microphone: {e}")
                return
        with self.lock:
            if self.recording:
                return
            # seed with the pre-roll so the first syllable is not lost
            self.frames = list(self.preroll)
            self.recording = True
            self.record_start = time.monotonic()
        self.set_state("recording")
        print(f"[voice] ● recording{' (' + why + ')' if why else ''} ...")

    def stop_recording(self):
        with self.lock:
            if not self.recording:
                return None
            self.recording = False
            frames = self.frames
            self.frames = []
        if LAZY_MIC and self.stream is not None and self.stream.active:
            try:
                self.stream.stop()               # frees the radio for the robot
            except Exception:
                pass
        dur = len(frames) * (BLOCK_MS / 1000.0)
        self.set_state("idle")
        if dur < MIN_UTTERANCE:
            print(f"[voice] (too short: {dur:.2f}s — ignored)")
            return None
        audio = np.concatenate(frames).astype(np.float32)
        # convert whatever the microphone gave us into the 16 kHz Whisper wants
        if self.capture_rate != SAMPLE_RATE:
            audio = resample_to_16k(audio, self.capture_rate)
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        print(f"[voice] ■ {dur:.1f}s captured (peak {peak:.3f}) — transcribing ...")
        if peak < 0.005:
            print("[voice]   ⚠ that was almost silent — is this the right mic, "
                  "and is it unmuted?")
        self.work.put(audio)
        return audio

    # ── ASR + dispatch ──────────────────────────────────────────────
    def asr_worker(self):
        while not self.shutdown:
            try:
                audio = self.work.get(timeout=0.3)
            except queue.Empty:
                continue
            self.set_state("transcribing")
            t0 = time.time()
            try:
                raw, logprob = self.asr.transcribe(audio)
            except Exception as e:
                print(f"[voice] transcription failed: {e}")
                self.set_state("error")
                continue
            dt = time.time() - t0
            text = clean(raw)
            conf = "" if logprob is None else f"  (avg_logprob {logprob:+.2f})"
            print(f"[voice] heard: {text!r}  [{dt:.2f}s]{conf}")
            self.set_state("listening" if self.args.mode == "auto" else "idle")
            self.dispatch(text, logprob)

    def dispatch(self, text, logprob):
        """Decide whether a transcript becomes a command.

        Order matters. PRIORITY words are checked FIRST — before the wake
        word, before the confidence floor, before the junk list — because a
        stop command that is filtered out is not a stop command. Everything
        else has to earn its way through."""
        if not text:
            return

        first = text.split()[0] if text.split() else ""
        if first in PRIORITY or any(text.startswith(p) for p in PRIORITY):
            self.publish_command(text)
            self.publish_transcript(text, True)
            return

        if any(re.search(rf"\b{re.escape(b)}\b", text) for b in BLOCKED):
            print("[voice] (refusing to forward a shutdown word spoken aloud — "
                  "quit the agent from its own terminal)")
            self.publish_transcript(text, False)
            return

        if text in JUNK or len(text) < MIN_CHARS:
            self.publish_transcript(text, False)
            return

        if logprob is not None and logprob < self.args.min_logprob:
            print(f"[voice] (low confidence {logprob:+.2f} < "
                  f"{self.args.min_logprob:+.2f} — not sent. Say it again?)")
            self.publish_transcript(text, False)
            return

        # hands-free needs the wake word; push-to-talk does not (the press IS
        # the wake signal)
        if self.args.mode == "auto":
            wake = self.args.wake.lower()
            if not text.startswith(wake):
                print(f"[voice] (no wake word '{wake}' — ignored)")
                self.publish_transcript(text, False)
                return
            text = text[len(wake):].strip(" ,")
            if len(text) < MIN_CHARS:
                print("[voice] (wake word with nothing after it)")
                self.publish_transcript(text, False)
                return

        self.publish_command(text)
        self.publish_transcript(text, True)

    # ── the two interactive modes ───────────────────────────────────
    def run_ptt(self, stream):
        # Same trap as the agent's input loop: when the GUI starts this node
        # there is no terminal, so input() would raise EOFError immediately
        # and the node would exit before it ever heard anything. With no
        # keyboard we idle and serve /vla/voice/trigger only — which is the
        # GUI's hold-to-talk button, so nothing is lost.
        if not sys.stdin or not sys.stdin.isatty():
            print("[voice] No terminal attached — running HEADLESS. "
                  f"Waiting for hold-to-talk on {TRIGGER_TOPIC}.")
            self.set_state("idle")
            try:
                while not self.shutdown:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                pass
            return
        print("\n" + "=" * 66)
        print(" PUSH TO TALK — press ENTER to start, ENTER again to stop.")
        print(" Type 'q' + ENTER to exit. The GUI mic button also works.")
        print("=" * 66 + "\n")
        while not self.shutdown:
            try:
                s = input("[ENTER]=talk > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if s in ("q", "quit", "exit"):
                break
            if self.recording:
                self.stop_recording()
                continue
            self.start_recording("push-to-talk")
            try:
                input("            ...speaking, ENTER to finish > ")
            except (EOFError, KeyboardInterrupt):
                self.stop_recording()
                break
            self.stop_recording()

    def calibrate(self, stream):
        """Measure the room. A fixed threshold that works in a quiet lab fires
        continuously next to a robot with running fans, and a threshold that
        survives the fans is deaf in a quiet room. So we measure, every run."""
        print(f"[voice] calibrating background noise for {CALIBRATE_S:.1f}s — "
              f"please stay quiet ...")
        levels = []
        t_end = time.monotonic() + CALIBRATE_S
        while time.monotonic() < t_end:
            levels.append(self.level)
            time.sleep(BLOCK_MS / 1000.0)
        levels = [l for l in levels if l > 0]
        self.noise_floor = (sorted(levels)[len(levels) // 2] if levels else 0.0)
        thr = max(SPEECH_FLOOR, self.noise_floor * SPEECH_MULT)
        print(f"[voice] noise floor {self.noise_floor:.4f} -> speech threshold "
              f"{thr:.4f}")
        return thr

    def run_auto(self, stream):
        thr = self.calibrate(stream)
        print("\n" + "=" * 66)
        print(f" HANDS-FREE — say '{self.args.wake} <command>'")
        print(f" e.g. '{self.args.wake}, go to the nearest chair'")
        print(" Ctrl-C to exit.")
        print("=" * 66 + "\n")
        self.set_state("listening")
        quiet_since = None
        while not self.shutdown:
            time.sleep(BLOCK_MS / 1000.0)
            loud = self.level > thr
            if not self.recording:
                if loud:
                    self.start_recording("voice detected")
                    quiet_since = None
            else:
                if loud:
                    quiet_since = None
                elif quiet_since is None:
                    quiet_since = time.monotonic()
                elif time.monotonic() - quiet_since > SILENCE_END:
                    self.stop_recording()
                    self.set_state("listening")
                    quiet_since = None
                if time.monotonic() - self.record_start > MAX_UTTERANCE:
                    print("[voice] (max utterance length reached)")
                    self.stop_recording()
                    self.set_state("listening")
                    quiet_since = None


def list_devices():
    """Show INPUT devices only, with the rate each one actually supports.

    The raw sounddevice dump is mostly HDMI outputs, which cannot record and
    are the reason the useful entry is hard to spot. Bluetooth headsets appear
    here only via the 'pulse' / 'default' entries, because PortAudio talks to
    ALSA and Bluetooth audio lives in PulseAudio/PipeWire — if you do not see
    'pulse' below, install libasound2-plugins."""
    import sounddevice as sd
    devs = sd.query_devices()
    print("\nINPUT DEVICES (these can record):\n")
    print(f"  {'idx':>4}  {'in':>3}  {'native Hz':>10}  name")
    print("  " + "-" * 68)
    found_pulse = False
    for i, d in enumerate(devs):
        if d.get("max_input_channels", 0) <= 0:
            continue
        name = d.get("name", "?")
        if "pulse" in name.lower() or "default" in name.lower():
            found_pulse = True
        rate = int(round(float(d.get("default_samplerate") or 0)))
        ok = []
        for r in (16000, rate, 48000, 44100):
            if not r:
                continue
            try:
                sd.check_input_settings(device=i, channels=1,
                                        dtype="float32", samplerate=r)
                ok.append(r)
            except Exception:
                pass
        usable = "opens OK" if ok else "WILL NOT OPEN"
        print(f"  {i:>4}  {d['max_input_channels']:>3}  {rate:>10}  {name}   [{usable}]")
    print()
    if found_pulse:
        print("  For a BLUETOOTH headset use the 'pulse' entry:")
        print("      python3 voice_command.py --mic pulse")
        print("  (make sure the earbuds are the system input in your sound")
        print("   settings first, and that they are in 'Headset' mode, not")
        print("   'High Fidelity' — the microphone only exists in Headset mode.)")
    else:
        print("  No 'pulse' device found, so BLUETOOTH microphones are not")
        print("  reachable yet. Install the ALSA->PulseAudio bridge:")
        print("      sudo apt install -y libasound2-plugins")
        print("  then run --list-devices again and look for 'pulse'.")
    print()


def main():
    ap = argparse.ArgumentParser(description="Speech input for the VLA agent")
    ap.add_argument("--mode", choices=("ptt", "auto"), default="ptt",
                    help="ptt = push to talk (default), auto = wake word")
    ap.add_argument("--wake", default=WAKE_DEFAULT,
                    help=f"wake word for --mode auto (default '{WAKE_DEFAULT}')")
    ap.add_argument("--model", default="small.en",
                    help="whisper model: tiny.en / base.en / small.en / medium.en")
    ap.add_argument("--device", default="auto", help="cuda | cpu | auto")
    ap.add_argument("--compute", default="auto", help="float16 | int8 | auto")
    ap.add_argument("--mic", default=None,
                    help="input device: index (4) or name fragment (pulse, usb, headset)")
    ap.add_argument("--rate", type=int, default=None,
                    help="force a capture sample rate (normally auto-detected)")
    ap.add_argument("--min-logprob", type=float, default=-1.0,
                    help="reject transcripts below this average log-probability")
    ap.add_argument("--speak", action="store_true",
                    help="read /vla/reply aloud with espeak-ng")
    ap.add_argument("--list-devices", action="store_true")
    args = ap.parse_args()

    if args.list_devices:
        list_devices(); return

    try:
        import sounddevice as sd
    except ImportError:
        print("sounddevice is not installed:\n"
              "  sudo apt install -y portaudio19-dev\n"
              "  pip install sounddevice")
        sys.exit(2)

    rclpy.init()
    node = VoiceCommandNode(args)
    print(f"[{VOICE_VERSION}] publishing to {CMD_TOPIC}")
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        dev = resolve_device(sd, args.mic)
    except SystemExit:
        raise
    rate = args.rate or pick_input_rate(sd, dev)
    if rate is None:
        print("[voice] this device would not open at ANY sample rate.\n"
              "        run  python3 voice_command.py --list-devices")
        sys.exit(2)
    node.capture_rate = rate
    try:
        dname = sd.query_devices(dev if dev is not None else None, "input")["name"]
    except Exception:
        dname = str(dev)
    conv = "" if rate == SAMPLE_RATE else f" -> resampled to {SAMPLE_RATE} Hz"
    print(f"[voice] microphone: [{dev}] {dname}  @ {rate} Hz{conv}")

    block = int(rate * BLOCK_MS / 1000)
    try:
        with sd.InputStream(samplerate=rate, channels=1, dtype="float32",
                            blocksize=block, device=dev,
                            callback=node.audio_cb) as stream:
            node.stream = stream
            if LAZY_MIC and args.mode == "ptt":
                stream.stop()                    # idle with the mic closed
                print("[voice] LAZY MIC: the microphone is closed until you "
                      "hold the talk button (keeps the robot's Wi-Fi clear).")
            elif LAZY_MIC:
                print("[voice] LAZY MIC ignored in --mode auto (it must listen "
                      "continuously).")
            if args.mode == "ptt":
                node.run_ptt(stream)
            else:
                node.run_auto(stream)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[voice] audio device error: {e}\n"
              f"        run   python3 voice_command.py --list-devices\n"
              f"        then  python3 voice_command.py --mic <index or name>\n"
              f"        for Bluetooth earbuds try  --mic pulse")
    finally:
        node.shutdown = True
        node.set_state("idle")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[voice] stopped.")


if __name__ == "__main__":
    main()