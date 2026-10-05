from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch

# ============================================================
# ✏️  CHANGE THESE TWO LINES TO EXPERIMENT
IMAGE_PATH  = "/home/danyalaziz/Pictures/VLA /table.jpeg"  # path to your image
INSTRUCTION = "move away from the object"                    # your command in plain English
# ============================================================

print("Loading model... (takes ~20 seconds, normal)")
processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to("cuda:0")
print("✅ Model ready!\n")

# Load your image
image = Image.open(IMAGE_PATH).convert("RGB")

# Build the prompt — OpenVLA always needs this exact format
prompt = f"In: What action should the robot take to {INSTRUCTION}?\nOut:"

# Run the model
inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

# ---- Results ------------------------------------------------
print(f"📸 Image     : {IMAGE_PATH}")
print(f"💬 Instruction: '{INSTRUCTION}'")
print()
print("🤖 Predicted Robot Action:")
print(f"   Move left / right     (X) : {action[0]:+.5f}")
print(f"   Move forward / back   (Y) : {action[1]:+.5f}")
print(f"   Move up / down        (Z) : {action[2]:+.5f}")
print(f"   Tilt roll             (Rx): {action[3]:+.5f}")
print(f"   Tilt pitch            (Ry): {action[4]:+.5f}")
print(f"   Tilt yaw              (Rz): {action[5]:+.5f}")
print(f"   Gripper open / close      : {action[6]:+.5f}")
print()
print(f"Full array: {action}")