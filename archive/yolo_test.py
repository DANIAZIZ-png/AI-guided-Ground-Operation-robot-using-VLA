import os

from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────────
#  What to look for (open-vocabulary — edit this list freely).
#  You can put ANY words here, not just a fixed set of classes.
# ─────────────────────────────────────────────────────────────────
CLASSES = ["person", "box", "cardboard box", "shelf", "door", "chair", "cup", "bottle"]

# ─────────────────────────────────────────────────────────────────
#  Load YOLO-World ONCE (so you can test many images without reloading)
# ─────────────────────────────────────────────────────────────────
print("Loading YOLO-World model (first run downloads it)...")
model = YOLO("yolov8s-world.pt")
model.set_classes(CLASSES)
print("Model ready.\n")
print("Tip: you can DRAG an image file into this terminal to paste its exact path.\n")

# ─────────────────────────────────────────────────────────────────
#  Interactive loop: keep asking for image paths
# ─────────────────────────────────────────────────────────────────
while True:
    image_path = input("Enter image path (or 'quit' to exit): ").strip()

    # Allow quitting
    if image_path.lower() in ("quit", "exit", "q"):
        print("Goodbye!")
        break
    if not image_path:
        continue

    # Remove surrounding quotes (in case you dragged a path with spaces)
    image_path = image_path.strip("'\"")

    # Expand ~ into your home folder
    image_path = os.path.expanduser(image_path)

    # Check the file exists FIRST, with a clear message (no crash)
    is_url = image_path.startswith("http")
    if not is_url and not os.path.exists(image_path):
        print(f"  [!] File not found: {image_path}")
        print("      Double-check the path, or drag the image into the terminal.\n")
        continue

    # Run detection
    print("  Detecting...")
    results = model.predict(image_path, conf=0.1)
    r = results[0]

    # Print what it found
    print(f"\n  Found {len(r.boxes)} object(s):")
    for box in r.boxes:
        name = CLASSES[int(box.cls[0])]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [round(v) for v in box.xyxy[0].tolist()]
        print(f"    {name:14s} confidence={conf:.2f}  box=(x1={x1}, y1={y1}, x2={x2}, y2={y2})")

    # Save an annotated image so you can SEE the boxes
    r.save(filename="detection_result.jpg")
    print("  Saved annotated image to: detection_result.jpg\n")