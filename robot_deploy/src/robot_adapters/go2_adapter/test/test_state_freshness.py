import queue
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from go2_adapter.adapter_node import Go2AdapterNode


def test_latest_sdk_state_bypasses_busy_ros_queue():
    node = SimpleNamespace(
        _lock=threading.Lock(), _incoming=queue.Queue(maxsize=1), _state=None, _pending_state=None
    )
    node._incoming.put(("unrelated", 0))
    old = SimpleNamespace(received_s=1.0)
    new = SimpleNamespace(received_s=2.0)
    Go2AdapterNode._put_incoming(node, "state", old)
    Go2AdapterNode._put_incoming(node, "state", new)
    assert node._state is new and node._pending_state is new
    assert node._state.received_s == 2.0
    assert node._incoming.get_nowait() == ("unrelated", 0)
    published = []
    node._publish_odom = published.append
    Go2AdapterNode._process_incoming(node)
    assert published == [new]
    Go2AdapterNode._process_incoming(node)
    assert published == [new]


def test_real_state_timeout_still_disconnects():
    node = SimpleNamespace(_state=SimpleNamespace(received_s=1.0), state_timeout_s=0.5)
    assert Go2AdapterNode._connected_and_mode(node, 1.501) == (False, "UNKNOWN")
