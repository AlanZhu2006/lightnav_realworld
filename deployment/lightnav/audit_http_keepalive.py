"""CPU-only local TCP audit of LightNav HTTP persistence and body framing.

The model policy and executor worker are fakes. JPEG parsing, HTTP handlers,
request recording and socket shutdown use the actual server implementation.
"""
from __future__ import annotations

import argparse
import base64
from concurrent.futures import Future
from contextlib import closing
import copy
import http.client
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from unittest.mock import patch
from urllib.request import Request, urlopen

from PIL import Image

from .server import Handler, Server


class FakeWorker:
    def __init__(self):
        self.calls = []
        self.closed = False

    def submit(self, function, *args):
        assert not self.closed
        self.calls.append(function.__name__)
        result = Future()
        try:
            result.set_result(function(*args))
        except Exception as exc:
            result.set_exception(exc)
        return result

    def shutdown(self, *, wait, cancel_futures):
        assert wait is True and cancel_futures is True
        self.closed = True


class FakePolicy:
    contract = {"offline_fake": True, "model_loaded": False}

    def __init__(self):
        self.calls = []
        self.resets = []
        self.reject_next = False

    def predict(self, frames, goal, identity, instruction, mode, task_mode):
        if self.reject_next:
            self.reject_next = False
            raise ValueError("expected fake inference failure")
        assert goal.mode == "RGB" and instruction == "Navigate to the reference goal image."
        assert mode == "experimental_reference_image"
        assert task_mode == "imagegoal"
        self.calls.append(copy.deepcopy(identity))
        lineage = [{key: frame[key] for key in ("frame_id", "capture_timestamp_ns")}
                   for frame in frames]
        return {"schema_version": "lightnav.imagegoal.inference.v1", "identity": identity,
                "observation_frames": lineage, "source_frame": lineage[-1],
                "model_stop": False, "arrival_confirmed": False,
                "trajectory_body_m": [[0., 0.], [.1, 0.]],
                "conditioning_mode": mode}

    def reset(self, identity):
        self.resets.append(copy.deepcopy(identity))
        return {"ok": True, "identity": identity}


def _payload(index):
    encoded = io.BytesIO()
    Image.new("RGB", (24, 16), (32, 64, 96)).save(encoded, format="JPEG")
    jpeg = base64.b64encode(encoded.getvalue()).decode("ascii")
    identity = {"session_id": "offline-session", "task_id": "offline-task",
                "goal_id": "offline-goal", "goal_version": 1, "frame_id": f"rgb-{index}"}
    return {"identity": identity, "observations": [{"frame_id": identity["frame_id"],
            "capture_timestamp_ns": 1_800_000_000_000_000_000+index*250_000_000,
            "rgb_jpeg_base64": jpeg}], "goal": {"rgb_jpeg_base64": jpeg},
            "conditioning_mode": "experimental_reference_image", "task_mode": "imagegoal",
            "instruction": "Navigate to the reference goal image."}


def _exchange(connection, method, path, payload=None, headers=None):
    body = None if payload is None else json.dumps(payload).encode()
    request_headers = {} if body is None else {"Content-Type": "application/json"}
    request_headers.update(headers or {})
    connection.request(method, path, body=body, headers=request_headers)
    response = connection.getresponse()
    raw = response.read()
    return response, json.loads(raw)


def audit():
    checks, results = {}, {}
    before_modules = {name for name in sys.modules
                      if name.startswith(("torch", "transformers", "unitree_sdk2py"))}
    worker, policy = FakeWorker(), FakePolicy()
    handler_threads = set()
    original_setup, original_handle = Handler.setup, Handler.handle

    def short_setup(handler):
        original_setup(handler)
        # Exercise an incomplete body timeout without a 30-second test wait.
        handler.connection.settimeout(.2)

    def tracked_handle(handler):
        handler_threads.add(threading.current_thread())
        return original_handle(handler)

    with tempfile.TemporaryDirectory(prefix="lightnav-http-cpu-") as temporary, \
         patch.object(Handler, "setup", short_setup), \
         patch.object(Handler, "handle", tracked_handle), \
         patch.object(Handler, "log_message"), \
         patch("deployment.lightnav.server.logging.exception"), \
         patch("deployment.lightnav.server.logging.warning"):
        server = Server(("127.0.0.1", 0), policy, Path(temporary))
        server.worker.shutdown(wait=True, cancel_futures=True)
        server.worker = worker
        address = server.server_address
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01},
                                  name="lightnav-http-cpu-audit")
        thread.start()
        connection = http.client.HTTPConnection(*address, timeout=2.)
        try:
            response, health = _exchange(connection, "GET", "/healthz")
            reused_socket = connection.sock
            assert response.version == 11 and response.status == 200 and health["ok"]
            assert not response.will_close and reused_socket is not None
            response, _ = _exchange(connection, "GET", "/healthz")
            assert response.status == 200 and connection.sock is reused_socket
            checks["two_health_requests_share_one_http11_socket"] = True

            payloads = [_payload(1), _payload(2)]
            for payload in payloads:
                response, inferred = _exchange(connection, "POST", "/infer", payload)
                assert response.status == 200 and not response.will_close
                assert connection.sock is reused_socket
                assert inferred["identity"] == payload["identity"]
                assert inferred["source_frame"]["frame_id"] == payload["identity"]["frame_id"]
            assert policy.calls == [payload["identity"] for payload in payloads]
            checks["two_inferences_reuse_health_socket_and_preserve_session_task_frame_identity"] = True

            identity = {"session_id": "offline-session", "task_id": "offline-task"}
            response, reset = _exchange(connection, "POST", "/reset", {"identity": identity})
            assert response.status == 200 and reset["identity"] == identity
            assert policy.resets == [identity] and connection.sock is reused_socket
            checks["session_reset_preserves_identity_and_connection"] = True

            records = list(Path(temporary).iterdir())
            assert len(records) == 2
            recorded_identities = []
            for directory in records:
                request = json.loads((directory/"request.json").read_text())
                reply = json.loads((directory/"response.json").read_text())
                assert request["identity"] == reply["identity"]
                recorded_identities.append(request["identity"])
            assert sorted(item["frame_id"] for item in recorded_identities) == ["rgb-1", "rgb-2"]
            checks["request_and_response_record_contract_unchanged"] = True

            def rejects_and_closes(name, raw, expected, *, half_close=False):
                with socket.create_connection(address, timeout=2.) as sock:
                    sock.sendall(raw)
                    if half_close:
                        sock.shutdown(socket.SHUT_WR)
                    response = http.client.HTTPResponse(sock)
                    response.begin()
                    body = json.loads(response.read())
                    assert response.status == expected, (name, response.status, body)
                    assert response.getheader("Connection") == "close" and response.will_close
                    try:
                        remainder = sock.recv(1)
                    except ConnectionResetError:
                        remainder = b""
                    assert remainder == b"", (name, remainder)
                    results[name] = {"status": response.status, "closed": True}

            pipeline = b"GET /healthz HTTP/1.1\r\nHost: localhost\r\n\r\n"
            rejects_and_closes("unknown_path_unread_body", b"POST /unknown HTTP/1.1\r\nHost: localhost\r\nContent-Length: 4\r\n\r\njunk"+pipeline, 404)
            rejects_and_closes("unsupported_content_type_unread_body", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: text/plain\r\nContent-Length: 4\r\n\r\njunk"+pipeline, 415)
            rejects_and_closes("chunked_body_rejected", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nTransfer-Encoding: chunked\r\n\r\n4\r\njunk\r\n0\r\n\r\n"+pipeline, 415)
            rejects_and_closes("oversized_declared_body", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 67108865\r\n\r\n"+pipeline, 400)
            rejects_and_closes("duplicate_lengths", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 2\r\nContent-Length: 2\r\n\r\n{}"+pipeline, 400)
            rejects_and_closes("invalid_json_body", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 4\r\n\r\njunk"+pipeline, 400)
            rejects_and_closes("get_body_rejected", b"GET /healthz HTTP/1.1\r\nHost: localhost\r\nContent-Length: 4\r\n\r\njunk"+pipeline, 400)
            rejects_and_closes("body_half_packet_eof", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 9\r\n\r\n{}", 400, half_close=True)
            rejects_and_closes("body_read_timeout", b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 9\r\n\r\n{}", 408)
            assert len(policy.calls) == 2
            checks["early_rejections_close_and_never_parse_unread_or_pipelined_bytes"] = True
            checks["body_timeout_and_partial_eof_close_connection"] = True

            with closing(http.client.HTTPConnection(*address, timeout=2.)) as busy:
                server.slot.acquire()
                try:
                    response, _ = _exchange(busy, "POST", "/infer", _payload(3))
                    assert response.status == 429 and response.will_close
                finally:
                    server.slot.release()
            with closing(http.client.HTTPConnection(*address, timeout=2.)) as failed:
                policy.reject_next = True
                response, error = _exchange(failed, "POST", "/infer", _payload(3))
                assert response.status == 500 and response.will_close
                assert error["identity"] == _payload(3)["identity"]
            checks["busy_and_model_error_close_connections_without_changing_slot_contract"] = True

            with closing(http.client.HTTPConnection(*address, timeout=2.)) as recovered:
                response, recovered_result = _exchange(recovered, "POST", "/infer", _payload(4))
                assert response.status == 200 and recovered_result["identity"] == _payload(4)["identity"]
            checks["new_connection_recovers_after_bad_requests_and_model_failure"] = True

            rejects_and_closes("client_connection_close", b"GET /healthz HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n", 200)
            rejects_and_closes("http10_default_close", b"GET /healthz HTTP/1.0\r\nHost: localhost\r\n\r\n", 200)
            with urlopen(Request(f"http://127.0.0.1:{address[1]}/healthz"), timeout=2.) as response:
                assert response.status == 200 and response.headers["Connection"] == "close"
                assert json.loads(response.read())["ok"]
            checks["http10_explicit_close_and_urllib_clients_remain_compatible"] = True
            connection.close()
            connection = http.client.HTTPConnection(*address, timeout=2.)
            response, _ = _exchange(connection, "GET", "/healthz")
            assert response.status == 200 and connection.sock is not None
        finally:
            # Leave the original successful keepalive socket idle and verify
            # server_close unblocks its handler rather than waiting 30 seconds.
            server.shutdown()
            thread.join(1.)
            server.server_close()
            deadline = time.monotonic()+1.
            while time.monotonic() < deadline and any(item.is_alive() for item in handler_threads):
                time.sleep(.01)
            connection.close()
            assert not thread.is_alive() and worker.closed
            assert not any(item.is_alive() for item in handler_threads)
            with server._connections_lock:
                assert not server._connections
            checks["server_close_unblocks_idle_keepalive_handlers_and_closes_worker"] = True
    assert {name for name in sys.modules
            if name.startswith(("torch", "transformers", "unitree_sdk2py"))} == before_modules
    checks["fake_policy_and_worker_no_gpu_model_or_robot_sdk"] = True
    return {"schema_version": "lightnav.http_keepalive_audit.v1", "ok": all(checks.values()),
            "checks": checks, "socket_cases": results, "local_tcp_only": True,
            "fake_policy": True, "fake_worker": True, "sdk_initialized": False,
            "gpu_inference_executed": False, "hardware_motion_sent": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
