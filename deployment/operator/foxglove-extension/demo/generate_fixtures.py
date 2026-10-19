"""Create synthetic UI assets. No live endpoint, recording or robot is read."""
import json
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
for name, color in (("live-rgb.jpg", (35, 65, 90)), ("live-goal.jpg", (65, 90, 35))):
    image = Image.new("RGB", (640, 480), color)
    draw = ImageDraw.Draw(image)
    draw.rectangle((160, 100, 480, 380), outline="white", width=8)
    draw.text((180, 220), "SYNTHETIC TEST IMAGE", fill="white")
    image.save(ROOT / name, quality=90)

fixture = {
    "schema_version": "imagegoal.runtime.v1", "session_id": "synthetic-read-only-preview",
    "timestamp_ns": 1, "epoch": 1, "state": "IDLE", "task": None,
    "profile": "simulated-replay", "synthetic_feedback": True, "sensors": {},
    "execution_capability": {}, "metrics": {}, "gates": ["synthetic_fixture_not_for_motion"],
}
(ROOT / "fixture-status.json").write_text(json.dumps(fixture, indent=2) + "\n")
