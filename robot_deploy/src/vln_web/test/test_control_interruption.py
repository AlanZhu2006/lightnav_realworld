"""ROS runtime regression tests; no robot connections or motion commands."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from vln_web.web_node import VlnWebNode


def node_stub(previous, desired):
    events = []
    node = SimpleNamespace(
        _observed_control_source=previous,
        recorder=SimpleNamespace(stop=lambda reason: events.append(("record_stop", reason))),
        _mpc_desired_enabled=desired,
        web_server=SimpleNamespace(
            update_control_source=lambda s: events.append(("source", s)),
            vln_mode=lambda: "objnav",
            clear_auto_controller=lambda s: events.append(("clear", s)),
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda s: events.append(("log", s))),
        _request_mpc=lambda enabled, cid: events.append(("mpc", enabled)),
        _request_vln=lambda enabled, *args: events.append(("vln", enabled)),
    )
    return node, events


def test_unexpected_control_loss_stops_both_navigation_and_mpc():
    node, events = node_stub("auto", True)
    VlnWebNode._on_control_source(node, SimpleNamespace(data="disabled"))
    assert ("mpc", False) in events and ("vln", False) in events
    assert node._observed_control_source == "disabled"
    assert any(e[0] == "clear" for e in events)


@pytest.mark.parametrize(
    "previous,source,desired",
    [
        ("disabled", "disabled", True),
        ("disabled", "auto", True),
        ("auto", "auto", True),
        ("auto", "disabled", False),
    ],
)
def test_initial_latched_state_and_expected_stop_do_not_cancel_start(previous, source, desired):
    node, events = node_stub(previous, desired)
    VlnWebNode._on_control_source(node, SimpleNamespace(data=source))
    assert not any(e[0] in ("mpc", "vln") for e in events)


def test_recording_start_rejection_rolls_back_control():
    events = []

    def reject(*args):
        raise RuntimeError("saving video")

    node = SimpleNamespace(
        recorder=SimpleNamespace(start=reject),
        _request_mpc=lambda enabled, cid: events.append(("mpc", enabled)),
        web_server=SimpleNamespace(clear_auto_controller=lambda msg: events.append(("clear", msg))),
        _service_error=lambda *args: events.append(("error", args)),
    )
    assert VlnWebNode._request_vln(node, True, "test", "objnav", "client") is False
    assert events[0] == ("mpc", False)
    assert [e[0] for e in events] == ["mpc", "clear", "error"]


def test_atomic_start_does_not_enable_mpc_after_rejection():
    import queue

    commands = queue.Queue()
    commands.put(("start_navigation", ("test", "objnav"), "client"))
    events = []
    node = SimpleNamespace(
        _commands=commands,
        _request_vln=lambda *args: False,
        _request_mpc=lambda *args: events.append(args),
    )
    VlnWebNode._flush_commands(node)
    assert events == []


@pytest.mark.parametrize("error,deadline", [("disk failed", 0.0), ("", 1.0)])
def test_recording_failure_and_start_timeout_roll_back(error, deadline):
    node, events = node_stub("auto", True)
    node.recorder.error = error
    node.recorder.active = not bool(error)
    node._navigation_deadline = deadline
    node.web_server._snapshot = lambda: {"vln": {"waypoint_count": 0}}
    node.get_logger = lambda: SimpleNamespace(error=lambda s: events.append(("log", s)))
    VlnWebNode._check_navigation_start(node)
    assert ("mpc", False) in events and ("vln", False) in events
