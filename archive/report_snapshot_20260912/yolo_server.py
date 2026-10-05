#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  yolo_server.py  —  YOLO-World open-vocabulary detection server
#
#  WHAT THIS IS:
#    A tiny Flask web server. The robot sends a camera frame; this runs
#    YOLO-World and returns the objects it found (name + box + confidence).
#    The agent (vla_agent.py) talks to it over HTTP, so detection runs in
#    its own process / GPU environment.
#
#  WHAT CHANGED IN THIS VERSION (v2):
#    #20 ATOMIC VOCABULARY SWAP. The old /set_classes assigned the global
#        CLASSES list BEFORE calling model.set_classes(). When the model
#        call raised, the global was already updated — so the model kept
#        detecting with the OLD embeddings while /detect looked names up
#        by index in the NEW list. Every label was shifted by however far
#        the two lists differed. Observed on hardware: a real "box" was
#        reported as "cardboard box", a real "cardboard box" as "computer
#        monitor" at 55%, and a real "shelf" as "computer tower". Nothing
#        was misdetected — everything was misnamed. The model is now
#        updated FIRST and the global is committed ONLY on success.
#    #21 SET_CLASSES FAILS SAFELY. On this machine model.set_classes()
#        raises at runtime with "Expected all tensors to be on the same
#        device": the CLIP text tokens are built on the CPU while the text
#        encoder has already been moved to cuda:0 by the first inference.
#        This is an Ultralytics-internal bug, not ours. It does NOT affect
#        the startup call (nothing has touched the GPU yet, so both sides
#        are on the CPU and they match). We therefore do not try to patch
#        Ultralytics — we roll back cleanly, keep serving with the working
#        vocabulary, and return HTTP 503 telling the caller to set
#        YOLO_CLASSES and restart instead.
#    #22 NO CLASS INHERITS THE 5% GLOBAL FLOOR. GLOBAL_CONF = 0.05 was
#        chosen for Gazebo, where flat-shaded primitives score low. On real
#        hardware it is the single source of the junk detections: "person"
#        scored 0.10 on a printed figure on a wall poster and passed,
#        because the old PER_CLASS_CONF deliberately left "person", "box"
#        and "pillar" on the global floor. Every class now has an explicit
#        floor. A real person at working range scores 0.6-0.9, so 0.35 sits
#        in empty space between the poster and the real thing.
#    #23 VOCABULARY SETTABLE FROM THE ENVIRONMENT. YOLO_CLASSES accepts a
#        comma-separated list, applied at startup where set_classes works.
#        This is how to A/B two vocabularies now that the live swap is
#        known-broken: restart with a different env var, no file edit.
#    #24 /detect's one-shot "classes" override is guarded the same way as
#        #20. If applying a request vocabulary fails, the request is
#        rejected rather than served with mismatched labels, and the
#        standing vocabulary is restored.
#
#  STILL HERE FROM BEFORE:
#    #15 WARM-UP at startup: one dummy inference is run right after
#        loading. The very first real request used to take several
#        seconds (CUDA kernels compile lazily on first use) — that lag
#        now happens at boot, not in front of your examiner.
#    #16 SAFE DECODING: a corrupt/truncated base64 image now returns a
#        clean HTTP 400 with a message instead of a 500 stack trace
#        that can wedge the request thread.
#    #17 Per-request "iou" override (alongside the existing "conf",
#        "min_area", "classes"), and an optional "per_class_conf" dict
#        so a single call can tighten one class without a restart.
#    #18 Inference time (ms) printed with every request — free numbers
#        for the thesis "system performance" table.
#    #19 DOCK VOCABULARY: "docking station" / "charging dock" added to
#        CLASSES. YOLO-World is open-vocabulary — it forces everything it
#        sees into the closest label ON THE LIST. The dock had no label,
#        so the model kept answering "chair" (the least-wrong option on a
#        multiple-choice exam that's missing the right answer). Giving it
#        the correct label makes the dock detect AS the dock. The agent
#        additionally geo-fences the dock's position (vla_agent #22), so
#        even a residual mislabel can't poison navigation or memory.
#
#    1. PER-CLASS confidence floors.
#         >>> TUNE THESE TO YOUR SCENE. Watch the printed confidences,
#             set each floor just BELOW your real objects and ABOVE the
#             false ones.
#    2. agnostic_nms = True — merges the two boxes YOLO-World stacks on
#       one primitive blob, so counts stop doubling.
#    3. Minimum box-area filter — tiny slivers are almost always noise.
#    4. Detections SORTED by confidence (best first).
#    5. Per-request overrides without restarting.
#    6. /set_classes to change the vocabulary live + a thread lock so
#       concurrent requests can't corrupt the single shared GPU model.
#
#  RESPONSE SHAPE (unchanged — safe for old callers):
#    {"detections": [ {"name", "confidence", "box":[x1,y1,x2,y2], "area"} ]}
# ─────────────────────────────────────────────────────────────────
import base64                                # decode the JPEG the agent sends
import io                                    # wrap raw bytes so PIL can open them
import os                                    # read environment-variable overrides
import threading                             # the GPU model is one shared object
import time                                  # measure inference time (#18)

from PIL import Image                        # image decoding
from flask import Flask, request, jsonify    # the HTTP server itself
from ultralytics import YOLO                 # YOLO-World implementation

# ── What the robot can recognise (open-vocabulary — edit freely) ──
# #19: "docking station"/"charging dock" are IN the list so the dock stops
# being forced into "chair". Keep them even if you never ask about the dock —
# their job is to absorb the dock's detections away from the other classes.
DEFAULT_CLASSES = ["person", "box", "cardboard box", "shelf", "door", "chair",
                   "pillar", "docking station", "charging dock"]

# #23: YOLO_CLASSES lets you swap the whole vocabulary at startup without
# editing this file, e.g.
#   YOLO_CLASSES="person,box,chair,computer monitor" python ~/yolo_server.py
# Startup is the ONLY place a vocabulary change reliably works — see #21.
_env_classes = os.environ.get("YOLO_CLASSES", "").strip()
CLASSES = ([c.strip() for c in _env_classes.split(",") if c.strip()]
           if _env_classes else list(DEFAULT_CLASSES))

PORT    = int(os.environ.get("YOLO_PORT", 5001))   # different from OpenVLA's 5000

# ── Detection tuning (all overridable per-request via the POST body) ──
# Global floor: the coarse floor handed to model.predict(). Kept LOW so the
# per-class floors below are what actually decides. Nothing is reported on
# this floor alone any more — see #22.
GLOBAL_CONF = float(os.environ.get("YOLO_CONF", 0.05))

# Per-class floor: the real filter. A class NOT listed here falls back to
# GLOBAL_CONF, which is why #22 lists every class explicitly.
#   >>> These are STARTING POINTS — tune to your own world using the
#       confidences this server prints. If a REAL chair stops being seen,
#       lower its number; if a FALSE chair appears, raise it.
PER_CLASS_CONF = {
    "person":          0.35,   # #22 was on the 0.05 floor. A printed figure on
                               # a wall poster scored 0.10 and was reported as a
                               # real person. A real person scores 0.6-0.9.
    "box":             0.35,   # #22 was on the 0.05 floor. "box" is the catch-all
                               # every rectangular thing falls into, so it needs
                               # the strictest floor of the object classes.
    "pillar":          0.30,   # #22 was on the 0.05 floor.
    "chair":           0.30,   # the main false-positive offender you hit
    "shelf":           0.25,   # counters and partitions land here at ~0.10
    "computer monitor": 0.30,  # #22 floors for the optional extended vocabulary,
    "computer tower":   0.30,  # so that if YOLO_CLASSES adds them they are not
    "poster":           0.25,  # silently sitting on the 0.05 floor.
    "door":            0.20,
    "cardboard box":   0.15,
    "docking station": 0.15,   # #19: keep the dock EASY to detect — the agent
    "charging dock":   0.15,   # geo-fences it, so a false dock is harmless.
}

IOU          = float(os.environ.get("YOLO_IOU", 0.50))      # NMS overlap
MAX_DET      = int(os.environ.get("YOLO_MAX_DET", 50))
IMG_SIZE     = int(os.environ.get("YOLO_IMGSZ", 640))
MIN_BOX_AREA = float(os.environ.get("YOLO_MIN_AREA", 100.0))  # px^2; drop tiny boxes

# ── Load YOLO-World ONCE ──
print("Loading YOLO-World model...")
model = YOLO("yolov8s-world.pt")             # weights load onto the CPU here
model.set_classes(CLASSES)                   # works: nothing is on the GPU yet (#21)
_model_lock = threading.Lock()               # the model is ONE shared GPU object

# #15: warm-up — pay the first-inference CUDA lag NOW, not during the demo.
# NOTE (#21): this is also the call that moves the CLIP text encoder to the GPU,
# which is exactly what makes every LATER set_classes() raise.
print("Warming up (first inference is always the slowest)...")
_t0 = time.perf_counter()
model.predict(Image.new("RGB", (IMG_SIZE, IMG_SIZE)), imgsz=IMG_SIZE, verbose=False)
print(f"Warm-up done in {(time.perf_counter() - _t0) * 1000:.0f} ms. "
      f"Classes = {CLASSES}. Starting server on port {PORT}.")

# #22: print the floor each class will actually be judged against, so a
# detection that "should have been filtered" can be checked against a number
# on screen instead of guessed at.
print("Confidence floors: "
      + ", ".join(f"{c}={PER_CLASS_CONF.get(c, GLOBAL_CONF):.2f}" for c in CLASSES))

app = Flask(__name__)


def _conf_floor_for(name, per_class):
    """The minimum confidence we accept for this class."""
    return per_class.get(name, GLOBAL_CONF)   # unlisted -> GLOBAL_CONF (#22 lists all)


def _try_set_classes(new_classes):
    """#20/#21: push a vocabulary into the MODEL and report honestly.

    Returns (ok, error_string). Never leaves the model and the caller's idea
    of the vocabulary out of step: on failure the model is untouched, because
    the failure happens inside Ultralytics before it commits anything, and we
    have not yet written our own global.
    """
    try:
        model.set_classes(new_classes)        # embeddings recomputed HERE
        return True, None                     # only now is the change real
    except RuntimeError as e:                 # the cuda/cpu mismatch lands here
        return False, str(e)                  # caller keeps the old vocabulary
    except Exception as e:                    # anything else: same policy
        return False, str(e)


@app.route("/", methods=["GET"])
def health():
    return jsonify({"status": "ok", "classes": CLASSES})


@app.route("/set_classes", methods=["POST"])
def set_classes():
    """Swap the detection vocabulary at runtime: {"classes": ["person", ...]}.

    #20: the model is updated FIRST. The global CLASSES is committed only if
    that succeeded, so the names /detect reports can never index a vocabulary
    the model is not actually using.
    """
    global CLASSES
    data = request.get_json(force=True, silent=True) or {}
    new = data.get("classes")
    if not new or not isinstance(new, list):
        return jsonify({"error": "send {'classes': [...]}"}), 400

    candidate = [str(c) for c in new]          # build it, do NOT commit it yet
    with _model_lock:
        ok, err = _try_set_classes(candidate)  # #20: model first
        if ok:
            CLASSES = candidate                # #20: commit only on success
        else:
            _try_set_classes(CLASSES)          # #20: force the old one back

    if not ok:
        print(f"Vocabulary change REJECTED, still using {CLASSES}\n  reason: {err}")
        return jsonify({
            "error": "live vocabulary change is not supported on this build",
            "detail": err,
            "workaround": ('restart the server with the vocabulary you want: '
                           'YOLO_CLASSES="person,box,chair" python ~/yolo_server.py'),
            "classes": CLASSES,                # what is STILL in force
        }), 503                                # 503, not 500: server is healthy

    print(f"Vocabulary updated -> {CLASSES}")
    return jsonify({"status": "ok", "classes": CLASSES})


@app.route("/detect", methods=["POST"])
def detect():
    data = request.get_json(force=True, silent=True) or {}
    if "image" not in data:
        return jsonify({"error": "send {'image': <base64 jpeg>}"}), 400

    # #16: decode defensively — a truncated frame must not 500 the server
    try:
        image_bytes = base64.b64decode(data["image"])
        image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    except Exception as e:
        return jsonify({"error": f"could not decode image: {e}"}), 400

    # Optional per-request overrides (fall back to the globals above)
    req_conf     = float(data.get("conf", GLOBAL_CONF))
    req_iou      = float(data.get("iou", IOU))                       # #17
    req_min_area = float(data.get("min_area", MIN_BOX_AREA))
    req_classes  = data.get("classes")   # optional one-shot vocabulary
    # #17: optional one-shot per-class floors, merged over the defaults
    per_class = dict(PER_CLASS_CONF)
    if isinstance(data.get("per_class_conf"), dict):
        for k, v in data["per_class_conf"].items():
            try:
                per_class[str(k)] = float(v)
            except (TypeError, ValueError):
                pass

    t0 = time.perf_counter()                                          # #18
    with _model_lock:
        # #24: a one-shot vocabulary must be PROVEN to have reached the model
        # before we name anything by index against it.
        if req_classes:
            wanted = [str(c) for c in req_classes]
            ok, err = _try_set_classes(wanted)
            if not ok:
                _try_set_classes(CLASSES)      # #24: restore before returning
                return jsonify({
                    "error": "per-request 'classes' override is not supported "
                             "on this build; omit it or restart with YOLO_CLASSES",
                    "detail": err,
                    "classes": CLASSES,
                }), 503
            active_classes = wanted            # safe: the model really has these
        else:
            active_classes = CLASSES           # the standing, proven vocabulary

        results = model.predict(
            image,
            conf=req_conf,          # coarse floor; per-class filter applied below
            iou=req_iou,
            max_det=MAX_DET,
            imgsz=IMG_SIZE,
            agnostic_nms=True,      # merge boxes stacked on the same blob
            verbose=False,
        )
        if req_classes:             # restore the standing vocabulary
            _try_set_classes(CLASSES)
    infer_ms = (time.perf_counter() - t0) * 1000                      # #18
    r = results[0]

    detections = []
    rejected = []                              # #22: what the floors threw away
    for box in r.boxes:
        idx = int(box.cls[0])
        if idx < 0 or idx >= len(active_classes):
            continue
        name = active_classes[idx]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)

        # apply the STRICTER of (per-request conf, per-class floor)
        floor = max(req_conf, _conf_floor_for(name, per_class))
        if conf < floor:
            rejected.append((name, round(conf, 2), floor))   # #22: log, don't hide
            continue
        if area < req_min_area:
            continue

        detections.append({
            "name": name,
            "confidence": conf,
            "box": [x1, y1, x2, y2],
            "area": area,
        })

    # best detections first — callers that pick "the" object get the strongest
    detections.sort(key=lambda d: d["confidence"], reverse=True)

    # printing confidences makes tuning the per-class floors easy;
    # printing the time gives you free numbers for the thesis (#18)
    print(f"[{infer_ms:6.1f} ms] Detected: "
          f"{[(d['name'], round(d['confidence'], 2)) for d in detections]}")
    # #22: seeing WHAT was filtered and by how much is how you tune the floors.
    # If a real object appears here, its floor is too high — lower it.
    if rejected:
        print(f"          filtered: "
              f"{[(n, c, f'floor {f:.2f}') for n, c, f in rejected]}")
    return jsonify({"detections": detections})


if __name__ == "__main__":
    # threaded=True is safe: every model call is guarded by _model_lock.
    app.run(host="0.0.0.0", port=PORT, threaded=True)