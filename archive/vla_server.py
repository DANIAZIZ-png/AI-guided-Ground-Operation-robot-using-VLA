import base64
import io

import torch
from PIL import Image
from flask import Flask, request, jsonify
from transformers import AutoModelForVision2Seq, AutoProcessor

# The translator from Step 3 — this file MUST be in the same folder
from openvla_action_translator import vla_action_to_cmd_vel

# ─────────────────────────────────────────────────────────────────
#  Config — matches your confirmed working OpenVLA setup
# ─────────────────────────────────────────────────────────────────
MODEL_PATH = "openvla/openvla-7b"
UNNORM_KEY = "bridge_orig"
DEVICE     = "cuda:0"
DTYPE      = torch.bfloat16
PORT       = 5000

# ─────────────────────────────────────────────────────────────────
#  Load OpenVLA ONCE at startup (the ~20 second wait, just once)
# ─────────────────────────────────────────────────────────────────
print()
print("Loading OpenVLA (one-time, ~20 seconds)...")

processor = AutoProcessor.from_pretrained(MODEL_PATH, trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    MODEL_PATH,
    torch_dtype=DTYPE,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to(DEVICE)

print("Model ready.")

# ─────────────────────────────────────────────────────────────────
#  Warm up the model with ONE dummy prediction.
#  OpenVLA's first prediction is always slow (GPU warmup). Doing it
#  here at startup means the robot's first real request is fast.
# ─────────────────────────────────────────────────────────────────
print("Warming up the model (one dummy prediction, may take ~30s)...")
_dummy = Image.new("RGB", (224, 224), color=(128, 128, 128))
_warm_prompt = "In: What action should the robot take to move forward?\nOut:"
_warm_inputs = processor(_warm_prompt, _dummy).to(DEVICE, dtype=DTYPE)
with torch.no_grad():
    _ = vla.predict_action(**_warm_inputs, unnorm_key=UNNORM_KEY, do_sample=False)
print("Warmup done. Model is hot and ready for the robot.")

app = Flask(__name__)


@app.route("/", methods=["GET"])
def health():
    # Quick check: open http://127.0.0.1:5000 in a browser to see this text
    return "OpenVLA server is running."


@app.route("/predict", methods=["POST"])
def predict():
    """Receive an image + instruction, return a velocity command."""
    data = request.get_json()

    # Instruction defaults to a simple patrol command if none is sent
    instruction = data.get("instruction", "move forward")

    # Decode the incoming image: base64 text -> JPEG bytes -> PIL image
    image_bytes = base64.b64decode(data["image"])
    image = Image.open(io.BytesIO(image_bytes)).convert("RGB")

    # Build the prompt in your confirmed working format
    prompt = f"In: What action should the robot take to {instruction}?\nOut:"

    # Run OpenVLA
    inputs = processor(prompt, image).to(DEVICE, dtype=DTYPE)
    with torch.no_grad():
        action = vla.predict_action(
            **inputs,
            unnorm_key=UNNORM_KEY,
            do_sample=False,
        )

    # Translate the 7-value arm action into wheeled velocities (Step 3)
    cmd = vla_action_to_cmd_vel(action)

    # DIAGNOSTIC: print OpenVLA's raw 7 values so we can see what it output.
    # Watch move_x (index 0) and yaw (index 5) — those drive the robot.
    raw = [round(float(v), 4) for v in action.flatten()]
    print(f"  raw OpenVLA action: {raw}")

    # Print what the brain decided, so you can watch it live
    print(f"[{instruction}]  ->  "
          f"linear={cmd['linear_x']:+.3f} m/s   "
          f"angular={cmd['angular_z']:+.3f} rad/s")

    return jsonify(cmd)


if __name__ == "__main__":
    print(f"Starting server on http://0.0.0.0:{PORT}")
    print("Leave this running. Press Ctrl+C to stop.")
    print()
    # threaded=False keeps GPU requests handled one at a time (safe)
    app.run(host="0.0.0.0", port=PORT, threaded=False)