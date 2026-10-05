from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch

print("Loading model (normal precision)... this needs ~14GB GPU memory")
processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to("cuda:0")

image = Image.new("RGB", (224, 224), color="gray")
prompt = "In: What action should the robot take to pick up the cup?\nOut:"

inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
print("Predicted action:", action)
