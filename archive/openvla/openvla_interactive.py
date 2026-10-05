import torch
from PIL import Image
from transformers import AutoModelForVision2Seq, AutoProcessor

# ─────────────────────────────────────────────────────────────────
#  Config — these match your confirmed working setup, do not change
# ─────────────────────────────────────────────────────────────────
MODEL_PATH    = "openvla/openvla-7b"
UNNORM_KEY    = "bridge_orig"
DEVICE        = "cuda:0"
DTYPE         = torch.bfloat16

ACTION_LABELS = [
    "move X ",
    "move Y ",
    "move Z ",
    "roll   ",
    "pitch  ",
    "yaw    ",
    "gripper",
]

# ─────────────────────────────────────────────────────────────────
#  SECTION 1: Load model ONCE at startup
#  This is your ~20 second wait — it only happens here, one time.
# ─────────────────────────────────────────────────────────────────
print()
print("=" * 54)
print("  OpenVLA Interactive Loop")
print("=" * 54)
print()
print("Loading model... (one-time wait, ~20 seconds)")
print()

processor = AutoProcessor.from_pretrained(
    MODEL_PATH,
    trust_remote_code=True,
)

vla = AutoModelForVision2Seq.from_pretrained(
    MODEL_PATH,
    torch_dtype=DTYPE,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to(DEVICE)

print()
print("Model loaded and ready on GPU!")
print("Type 'quit' at any prompt to exit.")
print()

# ─────────────────────────────────────────────────────────────────
#  SECTION 2: The interactive loop
#  Model stays loaded — each query runs in seconds, not 20 seconds.
# ─────────────────────────────────────────────────────────────────
last_image_path = None   # remembers the last image so you can reuse it

while True:
    print("-" * 54)

    # Step A: Get instruction from user
    try:
        instruction = input("Instruction: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\nExiting.")
        break

    # Allow clean exit
    if instruction.lower() in ("quit", "exit", "q"):
        print("Goodbye!")
        break

    # Skip empty input
    if not instruction:
        print("[!] Please type an instruction.")
        continue

    # Step B: Get image path
    # If we have a previous image, pressing Enter reuses it
    if last_image_path:
        img_prompt = f"Image path [press Enter to reuse: {last_image_path}]: "
    else:
        img_prompt = "Image path: "

    try:
        image_path = input(img_prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print("\nExiting.")
        break

    # Use last image if user pressed Enter
    if not image_path:
        if last_image_path:
            image_path = last_image_path
        else:
            print("[!] Please provide an image path.")
            continue

    # Step C: Load the image from disk
    try:
        image = Image.open(image_path).convert("RGB")
        last_image_path = image_path      # save for next round
    except FileNotFoundError:
        print(f"[!] File not found: {image_path}")
        continue
    except Exception as e:
        print(f"[!] Could not open image: {e}")
        continue

    # Step D: Build the prompt in your confirmed working format
    prompt = f"In: What action should the robot take to {instruction}?\nOut:"

    # Step E: Run inference
    #   - torch.no_grad() prevents PyTorch from storing gradients,
    #     saving GPU memory during inference (we are not training here)
    #   - **inputs unpacks the dict — this is your confirmed working fix
    print()
    print("  Running inference...")
    inputs = processor(prompt, image).to(DEVICE, dtype=DTYPE)

    with torch.no_grad():
        action = vla.predict_action(
            **inputs,
            unnorm_key=UNNORM_KEY,
            do_sample=False,
        )

    # Step F: Print the 7-value action array with clear labels
    action_flat = action.flatten()
    print()
    print("  Action output:")
    print("  " + "─" * 40)
    for label, val in zip(ACTION_LABELS, action_flat):
        val_f = float(val)
        sign  = "+" if val_f >= 0 else ""
        # Small ASCII bar to give a quick visual feel of magnitude
        bar_len = min(int(abs(val_f) * 15), 20)
        bar = "█" * bar_len
        print(f"  {label}  {sign}{val_f:+.4f}  {bar}")
    print("  " + "─" * 40)
    print()
