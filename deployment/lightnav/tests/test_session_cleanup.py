"""Own-session cleanup after failed diagnostics; no GPU/model/robot use."""
from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request

from PIL import Image

from deployment.lightnav import cleanup, client, commission
from deployment.lightnav.server import Server, parse_request
from deployment.lightnav.tests.test_tracking import policy, request
from deployment.lightnav.policy import MODEL_REVISION, MODEL_SHA256, OFFICIAL_RELEASE_COMMIT, SERVER_IMPLEMENTATION, OFFICIAL_CORE_DIGEST


class FakeClock:
    now = 0.
    def time(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


def cleanup_without_wait(base, session, timeout, post):
    clock = FakeClock()
    return cleanup.reset_owned_session(base, session, timeout, post,
                                       monotonic=clock.time, sleep=clock.sleep)


def source_manifest(root):
    frames = []
    for index in range(8):
        path = root / f"frame-{index}.jpg"
        Image.new("RGB", (32, 24), (100, 80, 60)).save(path, "JPEG")
        frames.append({"path": path.name, "frame_id": f"frame-{index}",
                       "capture_timestamp_ns": 1_000_000_000 + index * 250_000_000})
    manifest = root / "manifest.json"
    manifest.write_text(json.dumps({"timestamp_source": "sensor_recording", "frames": frames}))
    return manifest


def other_session(instance):
    value = request("native_language")
    value["identity"]["session_id"] = "another-operator-session"
    instance.predict(*parse_request(value))


class CleanupRetryTests(unittest.TestCase):
    def test_busy_reset_retries_only_the_owned_identity(self):
        requests = []
        def post(url, payload, timeout):
            requests.append(copy.deepcopy(payload))
            if len(requests) == 1:
                raise urllib.error.HTTPError(url, 429, "Inference busy", {}, None)
            return {"ok": True, "request_identity": payload["identity"], "sessions_cleared": 1, "retired": True}
        result = cleanup_without_wait("http://localhost:18884", "owned", 8, post)
        self.assertTrue(result["ok"])
        self.assertEqual(result["attempts"], 2)
        self.assertEqual(requests, [{"identity": {"session_id": "owned"}, "retire": True}] * 2)

    def test_transport_timeout_then_idle_reset_succeeds(self):
        count = 0
        def post(url, payload, timeout):
            nonlocal count
            count += 1
            if count == 1:
                raise TimeoutError("GPU work still running")
            return {"ok": True, "request_identity": payload["identity"], "sessions_cleared": 1, "retired": True}
        self.assertTrue(cleanup_without_wait("http://localhost", "owned", 8, post)["ok"])

    def test_cleanup_failure_has_a_bounded_explicit_receipt(self):
        clock = FakeClock()
        def post(url, payload, timeout):
            raise urllib.error.HTTPError(url, 429, "busy", {}, None)
        result = cleanup.reset_owned_session("http://localhost", "owned", 8, post,
                                              monotonic=clock.time, sleep=clock.sleep)
        self.assertFalse(result["ok"])
        self.assertEqual(clock.now, 120.)
        self.assertEqual(result["identity"], {"session_id": "owned"})


class ToolFinallyTests(unittest.TestCase):
    def test_client_failure_releases_only_its_own_session_after_busy_retry(self):
        for failure in ("timeout", "invalid_response"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                instance = policy()
                other_session(instance)
                output = root / "replay"
                argv = ["client", "--manifest", str(source_manifest(root)), "--output", str(output),
                        "--instruction", "Go to the doorway.", "--max-requests", "1"]
                reset_requests = []
                def post(url, payload, timeout):
                    if url.endswith("/reset"):
                        reset_requests.append(copy.deepcopy(payload))
                        if len(reset_requests) == 1:
                            raise urllib.error.HTTPError(url, 429, "busy", {}, None)
                        return instance.reset(payload["identity"], retire=payload.get("retire", False))
                    result = instance.predict(*parse_request(payload))
                    if failure == "timeout":
                        raise TimeoutError("inference timed out after accepting observations")
                    result["implementation"] = "wrong"
                    return result
                expected = TimeoutError if failure == "timeout" else ValueError
                with patch("sys.argv", argv), patch.object(client, "post", side_effect=post), \
                     patch.object(client, "reset_owned_session", side_effect=cleanup_without_wait), \
                     redirect_stdout(io.StringIO()), self.assertRaises(expected):
                    client.main()
                self.assertEqual(list(instance.sessions), [("another-operator-session", "task")])
                receipt = json.loads((output / "final_cleanup.json").read_text())
                self.assertTrue(receipt["ok"])
                self.assertEqual(receipt["attempts"], 2)
                self.assertNotEqual(receipt["identity"]["session_id"], "another-operator-session")

    def test_commission_assertion_failure_also_retries_its_own_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            instance = policy()
            other_session(instance)
            output = root / "commission"
            argv = ["commission", "--manifest", str(source_manifest(root)), "--output", str(output),
                    "--instruction", "Go to the coat rack."]
            resets = []
            def request_json(url, payload=None, timeout=120):
                if url.endswith("/healthz"):
                    return 200, {"ok": True, "model_loaded": True, "contract": {
                        "model_revision": MODEL_REVISION, "model_sha256": MODEL_SHA256,
                        "backend": "hf", "native_imagegoal_support": False,
                        "implementation": SERVER_IMPLEMENTATION, "official_release_commit": OFFICIAL_RELEASE_COMMIT,
                        "official_core_sha256": OFFICIAL_CORE_DIGEST, "imagegoal_adaptation_support": True,
                        "supported_conditioning_modes": ["native_language", "native_tracking", "experimental_reference_image"]}}
                if url.endswith("/reset"):
                    resets.append(copy.deepcopy(payload))
                    if len(resets) == 1:
                        return 429, {"error": "busy"}
                    return 200, instance.reset(payload["identity"], retire=payload.get("retire", False))
                try:
                    parsed = parse_request(payload)
                except ValueError:
                    return 400, {"error": "invalid request"}
                result = instance.predict(*parsed)
                result["implementation"] = "wrong"
                return 200, result
            with patch("sys.argv", argv), patch.object(commission, "request_json", side_effect=request_json), \
                 patch.object(commission, "reset_owned_session", side_effect=cleanup_without_wait), \
                 redirect_stdout(io.StringIO()), self.assertRaises(AssertionError):
                commission.main()
            self.assertEqual(list(instance.sessions), [("another-operator-session", "task")])
            receipt = json.loads((output / "final_cleanup.json").read_text())
            self.assertTrue(receipt["ok"])
            self.assertEqual(receipt["attempts"], 2)
            self.assertTrue(receipt["identity"]["session_id"].startswith("lightnav_commission_"))


class RetirementTests(unittest.TestCase):
    def test_retired_task_cannot_be_recreated_by_delayed_inference(self):
        from deployment.lightnav.policy import HistoryConflictError
        instance = policy()
        payload = request("native_language")
        instance.predict(*parse_request(payload))
        identity = {"session_id": "session", "task_id": "task"}
        response = instance.reset(identity, retire=True)
        self.assertTrue(response["retired"])
        self.assertEqual(response["sessions_cleared"], 1)
        with self.assertRaisesRegex(HistoryConflictError, "retired"):
            instance.predict(*parse_request(payload))
        # Ordinary resets never clear terminal retirement barriers.
        instance.reset(identity)
        with self.assertRaises(HistoryConflictError):
            instance.predict(*parse_request(payload))
        self.assertFalse(instance.sessions)

    def test_retired_session_blocks_all_late_tasks_but_preserves_other_operators(self):
        from deployment.lightnav.policy import HistoryConflictError
        instance = policy()
        other_session(instance)
        payload = request("native_language")
        instance.predict(*parse_request(payload))
        instance.reset({"session_id": "session"}, retire=True)
        payload["identity"]["task_id"] = "another-late-task"
        with self.assertRaises(HistoryConflictError):
            instance.predict(*parse_request(payload))
        self.assertEqual(list(instance.sessions), [("another-operator-session", "task")])
        self.assertEqual(instance.session_diagnostics()["retired_session_count"], 1)

    def test_task_retirement_does_not_block_other_tasks_in_same_session(self):
        instance = policy()
        value = request("native_language")
        instance.predict(*parse_request(value))
        instance.reset({"session_id": "session", "task_id": "task"}, retire=True)
        value["identity"]["task_id"] = "fresh-task"
        instance.predict(*parse_request(value))
        self.assertEqual(list(instance.sessions), [("session", "fresh-task")])

    def test_default_reset_keeps_explicit_history_reconstruction_supported(self):
        instance = policy()
        payload = request("native_language")
        instance.predict(*parse_request(payload))
        response = instance.reset({"session_id": "session", "task_id": "task"})
        self.assertFalse(response["retired"])
        restarted = instance.predict(*parse_request(payload))
        self.assertEqual(restarted["policy_history_count"], 1)
        self.assertEqual(restarted["policy_history_reset_reason"], "new_task_session")

    def test_retire_parameter_is_strictly_boolean(self):
        instance = policy()
        for invalid in (None, 1, "true"):
            with self.subTest(retire=invalid), self.assertRaises(ValueError):
                instance.reset({"session_id": "session"}, retire=invalid)

    def test_old_server_ignoring_retirement_is_not_reported_as_cleaned(self):
        def post(url, payload, timeout):
            return {"ok": True, "request_identity": payload["identity"], "sessions_cleared": 1}
        receipt = cleanup_without_wait("http://localhost", "owned", 8, post)
        self.assertFalse(receipt["ok"])
        self.assertIn("retirement", receipt["error"])


class HealthSessionTests(unittest.TestCase):
    def test_http_retirement_acknowledges_then_rejects_late_infer(self):
        instance = policy()
        value = request("native_language")
        instance.predict(*parse_request(value))
        instance.contract = {"max_sessions": 2}
        server = Server(("127.0.0.1", 0), instance, None)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            retired = client.post(base + "/reset", {"identity": {"session_id": "session", "task_id": "task"}, "retire": True}, 2)
            self.assertTrue(retired["retired"])
            # The control-plane acknowledgement is logical retirement, not a
            # claim that the serial GPU worker has already released tensors.
            self.assertEqual(retired["sessions_cleared"], 0)
            self.assertTrue(retired["cleanup_pending"])
            with self.assertRaises(urllib.error.HTTPError) as error:
                client.post(base + "/infer", value, 2)
            self.assertEqual(error.exception.code, 409)
            self.assertFalse(instance.sessions)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_health_identifies_occupied_sessions_even_when_inference_busy(self):
        instance = policy()
        other_session(instance)
        instance.contract = {"max_sessions": 2}
        server = Server(("127.0.0.1", 0), instance, None)
        server.slot.acquire()
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
        thread.start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/healthz", timeout=2) as response:
                health = json.load(response)
            self.assertTrue(health["inference_busy"])
            self.assertEqual(health["active_session_count"], 1)
            self.assertEqual(health["active_sessions"][0]["session_id"], "another-operator-session")
            self.assertEqual(health["active_sessions"][0]["history_frames"], 1)
            self.assertFalse(health["active_sessions_truncated"])
            self.assertEqual(len(instance.sessions), 1)
            url = f"http://127.0.0.1:{server.server_port}/reset"
            for invalid in (None, 1, "true"):
                request_body = json.dumps({"identity": {"session_id": "another-operator-session"}, "retire": invalid}).encode()
                http_request = urllib.request.Request(url, data=request_body, headers={"Content-Type": "application/json"})
                with self.subTest(invalid=invalid), self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(http_request, timeout=2)
                self.assertEqual(error.exception.code, 400)
            self.assertEqual(len(instance.sessions), 1)
            self.assertTrue(health["retirement_protocol"]["supported"])
        finally:
            server.slot.release()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
