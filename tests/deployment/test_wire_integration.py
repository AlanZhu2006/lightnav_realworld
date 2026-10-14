"""Real socket and robot client; deterministic engine substitutes only model inference."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from websockets.asyncio.server import serve

from deployment.smoke_protocol import infer_image
from lightnav.serving.ws_server import make_handler


class Session:
    def __init__(self, client_id):
        self.client_id = client_id
        self.resets = 0
        self._buffer_len = 0
        self.images = []

    def reset(self, instruction):
        self.resets += 1
        self.instruction = instruction
        self._buffer_len = 0

    def observe(self, image):
        self.images.append(image.copy())
        self._buffer_len += 1


class DeterministicService:
    def __init__(self):
        self.sessions = []
        self.instructions = []

    def make_session(self, client_id=None):
        session = Session(client_id)
        self.sessions.append(session)
        return session

    async def predict(self, session):
        self.instructions.append(session.instruction)
        return SimpleNamespace(
            waypoints=np.tile([0.4, -0.1, 0.2], (10, 1)),
            stop=False,
            visible=True,
            raw_text="<apos><pos_412><pos_650><opos><pos_205><pos_88>",
            timings_ms={"test_engine": 0.1},
        )


async def test_robot_client_matches_server_protocol_and_resets_sessions(tmp_path):
    frame = tmp_path / "frame.jpg"
    Image.new("RGB", (848, 480), (200, 50, 30)).save(frame)
    service = DeterministicService()
    async with serve(make_handler(service), "127.0.0.1", 0) as server:
        url = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
        first = await infer_image(url, frame, "approach the plant", timeout=10)
        second = await infer_image(url, frame, "turn left", timeout=10)
    assert first["sequence"] == second["sequence"] == 1
    assert len(service.sessions) == 2
    assert service.instructions == ["approach the plant", "turn left"]
    assert all(s.instruction == "" for s in service.sessions)  # Disconnect clears the session.
    assert all(s.resets >= 2 and len(s.images) == 1 for s in service.sessions)
    assert all(s.images[0].shape == (270, 480, 3) for s in service.sessions)
    assert first["trace_id"] != second["trace_id"]
    assert first["waypoints"][0] == pytest.approx([0.4, -0.1, 0.2])
    assert json.dumps(first)  # The smoke artifact is serializable.
