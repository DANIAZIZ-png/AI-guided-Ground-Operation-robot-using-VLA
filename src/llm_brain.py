# ─────────────────────────────────────────────────────────────────
#  llm_brain.py  —  the reasoning module ("brain") of the robot
#
#  Takes the operator's command + what the camera currently sees,
#  and decides what the robot should DO. Test it on its own first
#  (no Gazebo needed):   python3 llm_brain.py
#
#  WHAT CHANGED IN THIS VERSION (companion to the new vla_agent.py):
#    #11 CONVERSATION CONTEXT: decide() now accepts the previous
#        action/target, so pronouns work: "find a chair" ... "now go
#        to IT" resolves "it" -> "chair". Fully backward compatible —
#        old callers that pass only (command, visible) still work.
#    #12 keep_alive="30m": tells Ollama to keep the model loaded in
#        VRAM for 30 minutes. Fixes the "LLM hangs after the robot
#        sat idle" problem (Ollama unloads the model after ~5 min by
#        default, and reloading 7B while Gazebo + YOLO hold the GPU
#        can stall).
#    #13 AUTOMATIC RETRY: one transparent retry on a timeout or a
#        dropped connection, so a single hiccup doesn't kill a command.
#    #15 VERB-OVERRIDES-ARTICLE GUARDRAIL: "go to a chair" was being
#        classified as locate (report-only) because the prompt's locate
#        examples all used "a X" while navigate examples used "the X" —
#        the 7B model keyed on the ARTICLE instead of the VERB. Fixed in
#        two layers: (a) the prompt now states THE VERB DECIDES, NEVER
#        THE ARTICLE, with explicit counter-examples; (b) a deterministic
#        post-check: a motion verb aimed at the target ("go/drive/head/
#        move/navigate/come to X", "approach X", "take me to X") forces
#        that step to NAVIGATE even if the LLM said locate — and a pure
#        report request ("find/locate/look for/is there/can you see X")
#        with NO motion verb forces LOCATE, so the robot never drives
#        when it was only asked to look. The LLM proposes; the code
#        enforces. Every override is printed so you can see it working.
#    #14 OUTPUT VALIDATION: every step the LLM returns is checked
#        against a whitelist of actions and its fields are coerced to
#        the right types (numbers as floats, target as a lowercase
#        string, qualifier from a fixed set). A malformed step can no
#        longer crash or confuse the agent — bad steps are dropped,
#        and if nothing valid remains the brain asks to clarify.
#    #16 TURN-DIRECTION SIGN GUARD: "turn left 90" was executed as a
#        RIGHT turn (and vice versa) — the 7B model kept flipping the
#        sign of rotation_deg despite the prompt's example. Convention
#        (matches ROS, right-hand rule): POSITIVE = LEFT/anticlockwise,
#        NEGATIVE = RIGHT/clockwise. The LLM still extracts the ANGLE;
#        the direction is now read deterministically from the operator's
#        own words (left/right/clockwise/anticlockwise) and the sign is
#        forced to match. Bonus: "turn around / face backwards" is
#        clamped to 180° (the model once answered 360° — a full spin
#        back to where it started). Every override is printed.
#    #17 "forget" ACTION: the operator can now erase spatial memory —
#        "forget the chair position" clears remembered chair spots,
#        "forget everything / clear your memory" wipes it all. Backed by
#        a deterministic guard: a command containing "forget" can never
#        again be misrouted to navigate (which is what the 7B did with
#        "forget the last chair position").
#
#  (Still here from before: locate / move / dock / undock, navigate
#   qualifier, real reasoning, the talk-only "answer" action, and
#   find_another hard-locked to explicit "another / one more" wording.)
# ─────────────────────────────────────────────────────────────────
import json
import math
import re
import requests

OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
MODEL      = "qwen2.5:7b"      # change to whatever you pulled (qwen2.5:3b, llama3.1:8b, ...)

# #12: keep the model resident in VRAM between commands. Without this,
# Ollama unloads after ~5 idle minutes and the NEXT command pays a full
# reload while Gazebo + YOLO-World are also holding GPU memory -> hang.
KEEP_ALIVE = "30m"

REQUEST_TIMEOUT = 60          # seconds per attempt
RETRIES         = 2           # #13: total attempts (1 original + 1 retry)

SYSTEM_PROMPT = """You are the reasoning module ("brain") of an autonomous indoor ground robot:
a wheeled TurtleBot used for patrol, surveillance and ISR (Intelligence, Surveillance,
Reconnaissance).

YOU CAN: drive on indoor floors, look through a camera, search for objects by moving around,
map an unknown area, patrol a known one, make precise relative moves/turns, and dock/undock
from its charger.
YOU CANNOT: pick up / carry / fetch anything (no arm), fly, go outdoors, or reach the sky/ceiling.

You are a REASONING layer, not a keyword parser. Think about what the operator actually wants,
whether it is possible, and the smartest way to do it — THEN answer. If a request is impossible,
do NOT just refuse: explain in one short sentence and offer the closest thing you CAN do.

ABOUT YOURSELF (use this when the operator asks who/what you are or how you work — answer
naturally and confidently, like a well-informed operator, not a spec sheet): you are the
reasoning brain of a modular Vision-Language-Action system on a TurtleBot 4: an open-vocabulary
detector (YOLO-World) and a depth camera (OAK-D) are your eyes, a local LLM is your judgement,
and Nav2 with SLAM does the driving and obstacle avoidance. You remember where you've seen
objects, so you can return to them later. You patrol, map, search, count and report.

INPUT each turn:
- the operator's command (free natural language)
- "visible": the objects the camera sees right now (may be empty, partial, or a little noisy)
- optionally "previous action/target": what the robot was told to do last turn

CONTEXT RULE: if the operator uses a pronoun or back-reference — "it", "that", "there",
"the same one", "again" — resolve it to the PREVIOUS target when one is given.
Example: previous target was "chair", command "now go to it" -> navigate to "chair".
If there is no previous target to resolve against, use "clarify".

OUTPUT: reply with ONE JSON object and nothing else:
{"steps": [ {"action": "...", "target": "<object or null>", ...optional fields..., "speech": "<one short sentence>"} ]}

A command may contain SEVERAL actions — split it into an ORDERED list of steps. Keep the
operator's order, EXCEPT for the FEED rule below. Most commands are a single step.

ACTIONS — "action" is EXACTLY one of:
- "navigate"  : DRIVE to ONE named indoor object and stop in front of it. "target" = object.
      Triggers: "go to / drive to / move to / head to / approach / navigate to / come to the X".
      Optional "qualifier": "nearest" | "farthest" | "leftmost" | "rightmost" when the operator
      says which one (nearest/closest, farthest/furthest, the one on the left/right). Else omit it.
- "locate"    : LOOK for an object and just REPORT whether it is there — do NOT drive to it.
      "target" = object. Triggers: "find a X", "locate the X", "search for a X", "look for the X",
      "is there a X", "can you see a X", "do you see any X", "scan for X".
- "find_another" : look for ONE MORE of an object already found — a NEW, different instance.
      "target" = object (singular). USE ONLY IF the operator literally says one of:
      "another", "one more", "a different", "the other", "else", "additional", "second/third one".
      If none of those words appear, it is "locate" or "navigate", NEVER "find_another".
- "move"      : a PRECISE relative move/turn (no object needed). Fields:
      "distance_m" : metres to drive — POSITIVE = forward, NEGATIVE = backward (0 if none).
      "rotation_deg": degrees to turn — POSITIVE = LEFT/anticlockwise, NEGATIVE = RIGHT (0 if none).
      Unit help: 1 "block" ~ 1 m, 1 "step" ~ 0.5 m, "a little/a bit" ~ 0.3 m, "a lot" ~ 2 m.
      "move left/right" on a wheeled robot = TURN that way: emit TWO steps if a distance is also
      given (turn, then forward). e.g. "move left 1 m" -> [{move, rotation_deg:90}, {move, distance_m:1}].
- "count"     : count one object now. "target" = object (singular). ("count the chairs")
- "describe"  : report what the camera CURRENTLY sees. "target" null. ("what do you see")
- "explore"   : autonomously map an UNKNOWN area. "target" null. ("map the area", "explore")
- "patrol"    : move around a KNOWN area for security/inspection. "target" null. ("patrol the area")
- "feed"      : open the live camera video window. "target" null. ("show camera", "live feed")
- "dock"      : drive onto the charging dock. "target" null.
      ("dock", "return to base", "go home", "go charge", "go to the charging station")
- "undock"    : leave the charging dock. "target" null. ("undock", "leave the dock")
- "forget"    : ERASE remembered object positions from spatial memory. "target" = the object
      class to forget ("forget the chair", "forget where the chair was", "forget the last
      chair position"), or null to wipe everything ("forget everything", "clear your memory").
- "answer"    : TALK ONLY — no movement. Questions about you/your status, "can you…?", "why…?",
      greetings, or reasoning out loud. Put the reply in "speech". ("what can you do", "are you ok")
- "reject"    : refuse ONLY the truly impossible (fly, fetch/carry, go outdoors, touch the sky).
      ALWAYS explain in "speech" and offer the nearest possible alternative.
- "clarify"   : the command names no object and no clear task — ask ONE short follow-up.

HARD RULES:
0. THE VERB DECIDES THE ACTION — NEVER THE ARTICLE. "go to a chair", "go to the chair",
   "go to any chair" are ALL "navigate" (a motion verb aims at the object). "find a chair",
   "find the chair" are BOTH "locate" (a report verb). Ignore a/the/any/some entirely.
1. NOT SEEING an object is NEVER a reason to reject. If the operator names a normal indoor object
   (door, chair, box, person, cup…) that is not in "visible", still plan locate/navigate/count —
   the robot will go and look.
2. "find a X" -> "locate" (report only).   "go to / drive to X" -> "navigate" (drive).
   "find another X" -> "find_another".      Keep these separate.
3. find_another needs an explicit "another / one more / other / different / else" word.
   - "go to the person then to the chair"  -> TWO navigate steps. NEVER find_another.
4. "reject" is only for the physically impossible. A door/chair/box/person/cup is never impossible.
5. A "what can you do / who are you / are you ok" question is "answer".
6. FEED rule: if a command asks the robot to MOVE (navigate/locate/find_another/patrol/explore/move)
   AND also asks for the live feed, put the "feed" step FIRST, then the movement steps. If the
   command is ONLY "show the feed", it is a single feed step.
7. Each "speech" is ONE short, natural sentence — sound like a competent operator, not a robot.

EXAMPLES (operator -> idea -> output):
- "find a chair"            -> just report -> {"steps":[{"action":"locate","target":"chair","speech":"Looking to see if there's a chair."}]}
- "go to a chair"           -> MOTION VERB, article irrelevant -> {"steps":[{"action":"navigate","target":"chair","speech":"Heading to a chair."}]}
- "take me to a box"        -> motion -> {"steps":[{"action":"navigate","target":"box","speech":"Driving to a box."}]}
- "go to the nearest person"-> drive, pick nearest -> {"steps":[{"action":"navigate","target":"person","qualifier":"nearest","speech":"Heading to the closest person."}]}
- "move forward 2 meters"   -> {"steps":[{"action":"move","distance_m":2,"rotation_deg":0,"speech":"Moving forward two metres."}]}
- "turn right 30 degrees"   -> {"steps":[{"action":"move","distance_m":0,"rotation_deg":-30,"speech":"Turning thirty degrees right."}]}
- "turn left 90"            -> LEFT = POSITIVE -> {"steps":[{"action":"move","distance_m":0,"rotation_deg":90,"speech":"Turning ninety degrees left."}]}
- "turn around"             -> half a revolution -> {"steps":[{"action":"move","distance_m":0,"rotation_deg":180,"speech":"Turning around."}]}
- "forget the chair position" -> {"steps":[{"action":"forget","target":"chair","speech":"Forgetting where the chair was."}]}
- "clear your memory"       -> {"steps":[{"action":"forget","target":null,"speech":"Clearing all my remembered positions."}]}
- "go back a little"        -> {"steps":[{"action":"move","distance_m":-0.3,"rotation_deg":0,"speech":"Backing up a little."}]}
- "dock"                    -> {"steps":[{"action":"dock","target":null,"speech":"Returning to the dock."}]}
- "fetch me the red box"    -> impossible -> {"steps":[{"action":"reject","target":null,"speech":"I can't carry things (no arm), but I can drive to the red box and show it on camera — want that?"}]}
- "go to the person then to the chair" -> {"steps":[{"action":"navigate","target":"person","speech":"Going to the person."},{"action":"navigate","target":"chair","speech":"Then to the chair."}]}
- (previous target: "chair") "now go to it" -> pronoun -> {"steps":[{"action":"navigate","target":"chair","speech":"Going to the chair."}]}

Reply with JSON only."""

# #14: the ONLY actions the agent understands. Anything else is dropped.
VALID_ACTIONS = {
    "navigate", "locate", "find_another", "move", "count", "describe",
    "explore", "patrol", "feed", "dock", "undock", "answer", "reject", "clarify",
    "forget",                                                           # #17
}
VALID_QUALIFIERS = {"nearest", "farthest", "leftmost", "rightmost"}

CLARIFY_STEP = {"action": "clarify", "target": None,
                "speech": "Sorry, I didn't catch that — can you say it another way?"}


# ── #15: deterministic intent guardrail ─────────────────────────────
# The LLM is 7B and occasionally keys on the wrong surface cue (it read the
# ARTICLE in "go to a chair" and answered locate). These regexes read the
# VERB, per target, and override the step when the LLM and the operator's
# verb disagree. Deterministic code > hope.

_MOTION_VERBS = r"(?:go|drive|move|head|navigate|come|get|walk|roll)"
_REPORT_VERBS = (r"(?:find|locate|spot|search\s+for|look\s+for|scan\s+for|check\s+for|"
                 r"is\s+there|are\s+there|can\s+you\s+see|do\s+you\s+see|where\s+is|"
                 r"where'?s|any\s+sign\s+of)")
# up to two filler words (adjectives like "red big") between article and target
_FILL = r"(?:(?:\w+)\s+){0,2}"


def _motion_at(command, target):
    """True if the command aims a MOTION verb at this target:
    'go to a chair', 'drive over to the red chair', 'approach a person',
    'take me to the box', 'reach the chair'."""
    t = re.escape(target) + r"(?:e?s)?\b"                      # chair / chairs / boxes
    art = r"(?:the\s+|a\s+|an\s+|any\s+|some\s+|that\s+|this\s+|my\s+)?"
    pats = (
        rf"\b{_MOTION_VERBS}\s+(?:\w+\s+){{0,2}}?to(?:wards?)?\s+{art}{_FILL}{t}",
        rf"\bapproach\s+{art}{_FILL}{t}",
        rf"\b(?:take|bring|lead)\s+me\s+to\s+{art}{_FILL}{t}",
        rf"\breach\s+{art}{_FILL}{t}",
    )
    return any(re.search(p, command) for p in pats)


def _motion_at_pronoun(command):
    """True if a motion verb aims at a PRONOUN ('...and go to it', 'drive
    there'). We can't resolve which target the pronoun means, so this only
    BLOCKS downgrades (never forces an upgrade)."""
    return re.search(
        rf"\b{_MOTION_VERBS}\s+(?:\w+\s+){{0,2}}?to(?:wards?)?\s+"
        rf"(?:it|that|them|there|him|her)\b",
        command) is not None or re.search(
        rf"\b{_MOTION_VERBS}\s+there\b", command) is not None


def _report_at(command, target):
    """True if the command aims a REPORT verb at this target:
    'find a chair', 'is there a chair', 'can you see any chairs'."""
    t = re.escape(target) + r"(?:e?s)?\b"
    art = r"(?:the\s+|a\s+|an\s+|any\s+|some\s+)?"
    return re.search(rf"\b{_REPORT_VERBS}\s+{art}{_FILL}{t}", command) is not None


def _enforce_intent(command, decision):
    """#15: after validation, make the action agree with the operator's verb.
    - motion verb at target + LLM said locate      -> upgrade to navigate
    - report verb at target + NO motion verb + LLM said navigate -> downgrade
      to locate (never DRIVE when only asked to LOOK — the safe direction)
    Only navigate/locate steps are touched; dock/patrol/etc. pass through."""
    cmd = command.lower()
    for step in decision.get("steps", []):
        tgt = step.get("target")
        if not tgt:
            continue
        if step["action"] == "locate" and _motion_at(cmd, tgt):
            print(f"[brain-guard] '{command.strip()}' has a motion verb aimed at "
                  f"'{tgt}' — overriding locate -> navigate.")
            step["action"] = "navigate"
            step["speech"] = f"Heading to the {tgt}."
        elif (step["action"] == "navigate" and _report_at(cmd, tgt)
              and not _motion_at(cmd, tgt) and not _motion_at_pronoun(cmd)):
            print(f"[brain-guard] '{command.strip()}' only asks to LOOK for "
                  f"'{tgt}' — overriding navigate -> locate (won't drive).")
            step["action"] = "locate"
            step["speech"] = f"Looking for a {tgt} — I'll report, not drive."
    return decision


# ── #16: deterministic TURN-DIRECTION guard ─────────────────────────
# Convention (matches ROS: positive angular.z = anticlockwise by the
# right-hand rule):  rotation_deg  POSITIVE = LEFT,  NEGATIVE = RIGHT.
# The 7B model kept flipping this sign ("turn left 90" came back -90 and
# the robot turned right). The LLM's job is now only the MAGNITUDE; the
# direction is read from the operator's own words. If BOTH words appear
# ("turn to the right of the left door") we leave the LLM's answer alone.

_RIGHT_WORDS  = re.compile(r"\b(right|clockwise)\b")
_LEFT_WORDS   = re.compile(r"\b(left|anti[\s-]?clockwise|counter[\s-]?clockwise)\b")
_AROUND_WORDS = re.compile(r"\b(?:turn|face|spin|look)\s+(?:a?round|back(?:wards?)?)\b"
                           r"|\babout[\s-]?face\b|\bface\s+behind\b")


def _enforce_turn_sign(command, decision):
    """#16: force rotation_deg's SIGN to agree with the operator's words,
    and clamp 'turn around' to 180 deg (a 360 spin ends where it started)."""
    cmd = command.lower()
    says_right = bool(_RIGHT_WORDS.search(cmd))
    says_left  = bool(_LEFT_WORDS.search(cmd))
    for step in decision.get("steps", []):
        if step.get("action") != "move":
            continue
        rot = float(step.get("rotation_deg") or 0.0)
        if abs(rot) < 0.5:
            continue
        # "turn around" = half a revolution, unless the operator SAID 360
        if _AROUND_WORDS.search(cmd) and abs(rot) >= 359.0 and "360" not in cmd:
            rot = math.copysign(180.0, rot)
            step["rotation_deg"] = rot
            print("[brain-guard] 'turn around' = 180 deg, not a full 360 spin.")
        if says_right and not says_left and rot > 0:
            step["rotation_deg"] = -abs(rot)
            print(f"[brain-guard] command says RIGHT — forcing rotation_deg to "
                  f"{step['rotation_deg']:.0f} (negative = right/clockwise).")
        elif says_left and not says_right and rot < 0:
            step["rotation_deg"] = abs(rot)
            print(f"[brain-guard] command says LEFT — forcing rotation_deg to "
                  f"+{step['rotation_deg']:.0f} (positive = left/anticlockwise).")
    return decision


# ── #17: deterministic FORGET guard ─────────────────────────────────
_FORGET_WORDS = re.compile(r"\b(forget|unremember|erase\s+(?:your\s+|the\s+)?memory|"
                           r"clear\s+(?:your\s+|the\s+)?memory|wipe\s+(?:your\s+)?memory)\b")


def _enforce_forget(command, decision):
    """#17: a command that clearly asks to FORGET can never be misrouted
    (the 7B once sent 'forget the last chair position' to navigate). We keep
    whatever target the LLM extracted — it's good at that part."""
    cmd = command.lower()
    if not _FORGET_WORDS.search(cmd):
        return decision
    steps = decision.get("steps", [])
    if any(s.get("action") == "forget" for s in steps):
        return decision                          # LLM already got it right
    tgt = next((s.get("target") for s in steps if s.get("target")), None)
    print(f"[brain-guard] '{command.strip()}' asks to FORGET — overriding to a forget step.")
    decision["steps"] = [{
        "action": "forget", "target": tgt,
        "speech": (f"Forgetting where the {tgt} was." if tgt
                   else "Clearing my remembered positions."),
    }]
    return decision


def decide(command, visible_objects, context=None):
    """Ask the LLM what to do. Returns a dict shaped like
    {"steps": [ {"action", "target", ...optional..., "speech"} ]}.

    context (optional, #11): {"last_action": str|None, "last_target": str|None}
    — lets the model resolve "go to IT" to the previous target.
    """
    lines = [f"Objects currently visible: {visible_objects}"]
    if context and (context.get("last_action") or context.get("last_target")):
        lines.append(f'Previous action/target: {context.get("last_action")} '
                     f'-> "{context.get("last_target")}"')
    lines.append(f'Operator command: "{command}"')
    user_msg = "\n".join(lines)

    payload = {
        "model": MODEL,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": user_msg},
        ],
        "stream": False,
        "format": "json",                 # forces Ollama to return valid JSON
        "keep_alive": KEEP_ALIVE,         # #12: don't unload the model between commands
        "options": {"temperature": 0.2,   # low = consistent; raise to ~0.4 for chattier speech
                    "num_predict": 500},  # hard cap — a rambling reply can't stall the agent
    }

    # #13: one transparent retry on transient failures (timeout / dropped socket)
    last_err = None
    for attempt in range(RETRIES):
        try:
            resp = requests.post(OLLAMA_URL, json=payload, timeout=REQUEST_TIMEOUT)
            resp.raise_for_status()
            raw = resp.json()["message"]["content"]
            decision = _enforce_intent(command, _validate(_safe_json(raw)))   # #15
            decision = _enforce_turn_sign(command, decision)                  # #16
            return _enforce_forget(command, decision)                         # #17
        except (requests.Timeout, requests.ConnectionError) as e:
            last_err = e
            if attempt + 1 < RETRIES:
                print(f"[brain] Ollama didn't respond ({type(e).__name__}) — retrying once…")
    raise RuntimeError(f"Ollama unreachable after {RETRIES} attempts: {last_err}")


def _safe_json(raw):
    """Parse robustly — the agent must never crash on a bad reply."""
    try:
        return json.loads(raw)
    except Exception:
        pass
    cleaned = raw.strip().strip("`").strip()
    if cleaned.lower().startswith("json"):
        cleaned = cleaned[4:].strip()
    s, e = cleaned.find("{"), cleaned.rfind("}")
    if s != -1 and e != -1 and e > s:
        try:
            return json.loads(cleaned[s:e + 1])
        except Exception:
            pass
    return {"steps": [dict(CLARIFY_STEP)]}


def _validate(decision):
    """#14: enforce the output schema so a hallucinated or malformed step can
    never reach the agent. Rules:
      - decision must contain a list of dict steps (a bare single-step dict is wrapped)
      - "action" must be in VALID_ACTIONS (else the step is dropped)
      - "target" -> lowercase stripped string or None
      - "qualifier" -> kept only if in VALID_QUALIFIERS
      - "distance_m"/"rotation_deg" -> floats (0.0 if missing/garbage)
      - "speech" -> string (empty if missing)
    If nothing valid survives, return a single clarify step."""
    steps = None
    if isinstance(decision, dict):
        steps = decision.get("steps")
        if steps is None and decision.get("action"):
            steps = [decision]                       # old single-step shape
    if not isinstance(steps, list):
        return {"steps": [dict(CLARIFY_STEP)]}

    clean = []
    for s in steps:
        if not isinstance(s, dict):
            continue
        action = str(s.get("action") or "").strip().lower()
        if action not in VALID_ACTIONS:
            continue
        step = {"action": action, "speech": str(s.get("speech") or "")}

        target = s.get("target")
        step["target"] = str(target).strip().lower() if target else None

        q = str(s.get("qualifier") or "").strip().lower()
        if q in VALID_QUALIFIERS:
            step["qualifier"] = q

        if action == "move":
            step["distance_m"]   = _num(s.get("distance_m"))
            step["rotation_deg"] = _num(s.get("rotation_deg"))
        clean.append(step)

    if not clean:
        return {"steps": [dict(CLARIFY_STEP)]}
    return {"steps": clean}


def _num(v):
    """Coerce anything ('2', '2 m', 2, None) to a float, defaulting to 0.0."""
    if v is None:
        return 0.0
    try:
        return float(v)
    except (TypeError, ValueError):
        pass
    # last resort: pull the first number out of a string like "2 meters" / "-30 deg"
    num = ""
    for ch in str(v):
        if ch.isdigit() or (ch in "+-." and (not num or num in "+-")):
            num += ch
        elif num:
            break
    try:
        return float(num)
    except ValueError:
        return 0.0


# ─────────────────────────────────────────────────────────────────
#  Standalone test loop (NO Gazebo). Pretend the camera sees a list.
# ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    mock_view = ["person", "chair", "red box"]
    ctx = {"last_action": None, "last_target": None}
    print("LLM brain ready. (Pretending the robot currently sees:", mock_view, ")")
    print("Try:  find a chair        |  go to the nearest person  |  move forward 2 meters")
    print("      turn right 30        |  now go to it (tests context)  |  dock")
    print("      find another chair   |  can you fetch the box     |  what can you do")
    print("Type 'quit' to exit.\n")
    while True:
        cmd = input("Command> ").strip()
        if cmd.lower() in ("quit", "exit", "q"):
            break
        try:
            decision = decide(cmd, mock_view, context=ctx)
            print(json.dumps(decision, indent=2), "\n")
            # remember the last targeted step, exactly like the agent does (#11)
            for s in decision["steps"]:
                if s.get("target"):
                    ctx = {"last_action": s["action"], "last_target": s["target"]}
        except Exception as e:
            print("Error:", e, "\n")