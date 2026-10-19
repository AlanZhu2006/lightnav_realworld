"""CPU-only HTTP/task lifecycle tests; no hardware, weights or GPU inference."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import http.client
import json
import socket
import threading
import time
import unittest
from unittest.mock import patch

from deployment.lightnav import policy as policy_module
from deployment.lightnav.policy import HistoryConflictError
from deployment.lightnav.server import Handler, Server, TRANSPORT_PROTOCOL, parse_request
from deployment.lightnav.tests.test_tracking import policy, request


@contextmanager
def running_server(instance):
    instance.contract = {"max_sessions": instance.max_sessions}
    server = Server(("127.0.0.1", 0), instance, None)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
    with patch.object(Handler, "log_message"):
        thread.start()
        try:
            yield server
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


def exchange(server, path, payload, headers=None):
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    try:
        connection.request("POST", path, json.dumps(payload),
                           {"Content-Type": "application/json", **(headers or {})})
        response = connection.getresponse()
        return response.status, json.loads(response.read())
    finally:
        connection.close()


IDENTITY = {"session_id": "session", "task_id": "task"}


class LifecyclePolicyTests(unittest.TestCase):
    def test_capacity_and_history_errors_are_distinct(self):
        instance = policy()
        instance.max_sessions = 1
        instance.predict(*parse_request(request()))
        newer_task = request()
        newer_task["identity"]["task_id"] = "next"
        with self.assertRaises(HistoryConflictError) as error:
            instance.predict(*parse_request(newer_task))
        self.assertEqual(error.exception.error_code, "session_capacity")
        self.assertFalse(error.exception.retryable)
        stale = request()
        stale["observations"][0]["capture_timestamp_ns"] += 1
        with self.assertRaises(HistoryConflictError) as error:
            instance.predict(*parse_request(stale))
        self.assertEqual(error.exception.error_code, "history_conflict")

    def test_retirement_during_observe_cannot_commit_or_generate(self):
        from lightnav.inference.policies import NavigationPolicy
        instance = policy()
        original = NavigationPolicy.observe
        def retire_after_observe(observed, frame):
            original(observed, frame)
            instance.retire(IDENTITY)
        with patch.object(NavigationPolicy, "observe", retire_after_observe):
            with self.assertRaises(HistoryConflictError) as error:
                instance.predict(*parse_request(request()))
        self.assertEqual(error.exception.error_code, "task_retired")
        snapshot = instance.session_status(IDENTITY)
        self.assertTrue(snapshot["retired"])
        self.assertIsNone(snapshot["committed"])
        self.assertEqual(len(instance.tracking_engine.samples), 0)
        instance.cleanup_retired()
        self.assertFalse(instance.sessions)

    def test_commit_survives_generation_failure_and_does_not_duplicate(self):
        instance = policy()
        with patch.object(instance.tracking_engine, "generate", side_effect=RuntimeError("model failed")):
            with self.assertRaisesRegex(RuntimeError, "model failed"):
                instance.predict(*parse_request(request()))
        committed = instance.session_status(IDENTITY)["committed"]
        self.assertEqual(committed, {"frame_id": "frame-1", "capture_timestamp_ns": 1000000000,
                                     "history_frames": 1})
        result = instance.predict(*parse_request(request()))
        self.assertEqual(result["policy_history_frames_appended"], 0)
        self.assertEqual(instance.session_status(IDENTITY)["committed"], committed)

    def test_deferred_cleanup_is_deduplicated_and_does_not_reset_other_engine(self):
        instance = policy()
        instance.predict(*parse_request(request()))
        other = request("native_language")
        other["identity"]["session_id"] = "other"
        instance.predict(*parse_request(other))
        reset_counts = (instance.engine.resets, instance.tracking_engine.resets)
        for _ in range(100):
            response = instance.retire(IDENTITY)
            self.assertEqual(response["sessions_cleared"], 0)
        self.assertEqual(instance._cleanup_pending, {("session", "task")})
        self.assertEqual(instance.cleanup_retired(), 1)
        self.assertEqual(list(instance.sessions), [("other", "task")])
        self.assertEqual((instance.engine.resets, instance.tracking_engine.resets), reset_counts)

    def test_idle_ttl_retires_only_non_inflight_and_status_does_not_renew(self):
        instance = policy()
        instance.predict(*parse_request(request()))
        state = instance.sessions[("session", "task")]
        state["last_seen"] = 0.
        instance.begin_predict(IDENTITY)
        with patch.object(policy_module.time, "monotonic", return_value=121.):
            self.assertEqual(instance.expire_idle(), 0)
            instance.session_status(IDENTITY)
            instance.session_diagnostics()
            self.assertEqual(state["last_seen"], 0.)
            instance.end_predict(IDENTITY)
            self.assertEqual(state["last_seen"], 121.)
        with patch.object(policy_module.time, "monotonic", return_value=242.):
            self.assertEqual(instance.expire_idle(), 1)
        self.assertTrue(instance.session_status(IDENTITY)["retired"])
        instance.cleanup_retired()
        with self.assertRaises(HistoryConflictError) as error:
            instance.predict(*parse_request(request()))
        self.assertEqual(error.exception.error_code, "task_retired")


class LifecycleHTTPTests(unittest.TestCase):
    def test_retire_and_status_are_immediate_while_generation_is_running(self):
        instance = policy()
        started, release = threading.Event(), threading.Event()
        original = instance.tracking_engine.generate
        def blocked_generate(*args, **kwargs):
            started.set()
            if not release.wait(2):
                raise AssertionError("test did not release fake generation")
            return original(*args, **kwargs)
        with running_server(instance) as server, ThreadPoolExecutor(max_workers=1) as caller:
            with patch.object(instance.tracking_engine, "generate", side_effect=blocked_generate):
                inference = caller.submit(exchange, server, "/infer", request())
                try:
                    self.assertTrue(started.wait(1))
                    start = time.monotonic()
                    status, retired = exchange(server, "/reset", {"identity": IDENTITY, "retire": True})
                    self.assertLess(time.monotonic() - start, .5)
                    self.assertEqual(status, 200)
                    self.assertTrue(retired["retired"])
                    self.assertTrue(retired["cleanup_pending"])
                    self.assertEqual(retired["sessions_cleared"], 0)
                    _, snapshot = exchange(server, "/session/status", {"identity": IDENTITY})
                    self.assertTrue(snapshot["inference_busy"])
                    self.assertTrue(snapshot["retired"])
                    self.assertEqual(snapshot["committed"]["frame_id"], "frame-1")
                    self.assertEqual(snapshot["transport_protocol"], TRANSPORT_PROTOCOL)
                    self.assertEqual(snapshot["server_boot_id"], retired["server_boot_id"])
                finally:
                    release.set()
                code, error = inference.result(timeout=2)
                self.assertEqual(code, 409)
                self.assertEqual(error["error_code"], "task_retired")
                deadline = time.monotonic() + 1
                while instance.cleanup_pending() and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertFalse(instance.sessions)

    def test_status_validation_is_small_strict_and_never_queues_on_model_slot(self):
        instance = policy()
        with running_server(instance) as server:
            server.slot.acquire()
            try:
                status, snapshot = exchange(server, "/session/status", {"identity": IDENTITY})
                self.assertEqual(status, 200)
                self.assertTrue(snapshot["inference_busy"])
                self.assertFalse(snapshot["exists"])
                for invalid in ({"session_id": "session"}, {**IDENTITY, "goal_id": "x"},
                                {**IDENTITY, "task_id": None}, {**IDENTITY, "session_id": "x" * 257}):
                    with self.subTest(invalid=invalid):
                        status, error = exchange(server, "/session/status", {"identity": invalid})
                        self.assertEqual(status, 400)
                        self.assertEqual(error["error_code"], "invalid_request")
                status, _ = exchange(server, "/session/status", {"identity": IDENTITY, "extra": True})
                self.assertEqual(status, 400)
            finally:
                server.slot.release()

    def test_retired_admission_headers_reject_before_any_image_body_is_read(self):
        instance = policy()
        instance.retire(IDENTITY)
        with running_server(instance) as server:
            with socket.create_connection(server.server_address, timeout=1) as connection:
                connection.sendall(b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\n"
                                   b"Content-Length: 100000\r\nX-LightNav-Session-Id: session\r\n"
                                   b"X-LightNav-Task-Id: task\r\n\r\n")
                response = http.client.HTTPResponse(connection)
                response.begin()
                self.assertEqual(response.status, 409)
                self.assertEqual(json.loads(response.read())["error_code"], "task_retired")

    def test_changed_server_boot_rejects_before_image_body_and_cannot_recreate_history(self):
        instance = policy()
        with running_server(instance) as server:
            with socket.create_connection(server.server_address, timeout=1) as connection:
                connection.sendall(b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\n"
                                   b"Content-Length: 100000\r\nX-LightNav-Server-Boot-Id: old-server-boot\r\n\r\n")
                response = http.client.HTTPResponse(connection)
                response.begin()
                self.assertEqual(response.status, 409)
                error = json.loads(response.read())
                self.assertEqual(error["error_code"], "server_restarted")
                self.assertFalse(error["retryable"])
                self.assertEqual(error["server_boot_id"], server.server_boot_id)
                self.assertFalse(instance.sessions)
            value = request()
            value["server_boot_id"] = "not-the-header"
            code, error = exchange(server, "/infer", value,
                                   {"X-LightNav-Server-Boot-Id": server.server_boot_id})
            self.assertEqual(code, 400)
            self.assertEqual(error["error_code"], "invalid_request")
            self.assertFalse(instance.sessions)

    def test_header_body_identity_mismatch_is_rejected(self):
        instance = policy()
        with running_server(instance) as server:
            code, error = exchange(server, "/infer", request(),
                                   {"X-LightNav-Session-Id": "other", "X-LightNav-Task-Id": "task"})
            self.assertEqual(code, 400)
            self.assertEqual(error["error_code"], "invalid_request")
            self.assertFalse(instance.sessions)

    def test_body_total_deadline_bounds_a_trickle_upload(self):
        with running_server(policy()) as server:
            with socket.create_connection(server.server_address, timeout=1) as connection:
                start = time.monotonic()
                connection.sendall(b"POST /infer HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\n"
                                   b"Content-Length: 1000\r\nX-LightNav-Request-Timeout-Ms: 200\r\n\r\n{")
                time.sleep(.07)
                connection.sendall(b'"')
                time.sleep(.07)
                connection.sendall(b'a')
                response = http.client.HTTPResponse(connection)
                response.begin()
                self.assertLess(time.monotonic() - start, .6)
                self.assertEqual(response.status, 408)
                self.assertEqual(json.loads(response.read())["error_code"], "request_read_timeout")

    def test_busy_response_is_structured_and_retryable(self):
        with running_server(policy()) as server:
            server.slot.acquire()
            try:
                code, error = exchange(server, "/infer", request())
            finally:
                server.slot.release()
            self.assertEqual(code, 429)
            self.assertEqual(error["error_code"], "inference_busy")
            self.assertTrue(error["retryable"])
            self.assertEqual(error["transport_protocol"], TRANSPORT_PROTOCOL)


if __name__ == "__main__":
    unittest.main()
