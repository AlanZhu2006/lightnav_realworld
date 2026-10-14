from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from vln_web.render_recording import frame_bgr


def test_raw_recording_decodes_padded_rgb_rows():
    rows = np.array([[255, 0, 0, 0, 255, 0, 77, 88]], dtype=np.uint8)
    message = SimpleNamespace(data=rows.tobytes(), width=2, height=1, step=8, encoding="rgb8")
    assert frame_bgr(message, "raw").tolist() == [[[0, 0, 255], [0, 255, 0]]]


def test_compressed_recording_decodes_without_ros():
    original = np.full((4, 6, 3), [30, 80, 150], dtype=np.uint8)
    ok, encoded = cv2.imencode(".png", original)
    assert ok
    message = SimpleNamespace(data=encoded.tobytes())
    assert np.array_equal(frame_bgr(message, "compressed"), original)


def test_corrupt_compressed_recording_reports_failure():
    with pytest.raises(ValueError, match="decode"):
        frame_bgr(SimpleNamespace(data=b"not an image"), "compressed")
