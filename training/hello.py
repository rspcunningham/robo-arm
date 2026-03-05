from huggingface_hub import snapshot_download
from lerobot.policies.pi05 import PI05Policy

base_dir = "../checkpoints/pi05_base"

snapshot_download("lerobot/pi05_base", local_dir=base_dir)
model = PI05Policy.from_pretrained(base_dir)

print(model.parameters())
