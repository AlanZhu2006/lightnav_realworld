from copy import deepcopy
import json
import hashlib
import threading
import unittest

from deployment.imagegoal.foxglove_control import OperatorControl, local_preview_poses


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.state = {"session_id": "session-1", "epoch": 0, "state": "IDLE"}
        self.goal = b"\xff\xd8raw\xff\xd9"
        self.read_selections = []
        self.results, self.calls = [], []
        self.condition = threading.Condition()
        self.delayed = False
        self.start_received, self.release_start, self.stop_received = (threading.Event() for _ in range(3))
        self.adapter = OperatorControl("http://runtime", self.publish, request_json=self.request,
                                       read_goal=self.read_goal, queue_size=2)

    def tearDown(self):
        self.release_start.set()
        self.adapter.close()

    def publish(self, result):
        with self.condition:
            self.results.append(result)
            self.condition.notify_all()

    def read_goal(self, selection_id):
        self.read_selections.append(selection_id)
        return self.goal

    def result(self, request_id, count=1):
        with self.condition:
            self.condition.wait_for(lambda: sum(item["request_id"] == request_id for item in self.results) >= count, timeout=3)
            matches = [item for item in self.results if item["request_id"] == request_id]
        self.assertGreaterEqual(len(matches), count, f"missing result: {request_id}")
        return matches[count - 1]

    def send(self, request_id="request-1", action="start", payload=None, session="session-1"):
        return self.adapter.submit(json.dumps({"request_id": request_id, "session_id": session,
                                              "action": action, "payload": {"expected_epoch": 0} if payload is None else payload}))

    def request(self, url, *, timeout_s, payload=None):
        path = url.removeprefix("http://runtime")
        with self.condition:
            self.calls.append((path, deepcopy(payload)))
            if path == "/status":
                return deepcopy(self.state), 1
        if path == "/tasks" and self.delayed:
            self.start_received.set()
            self.release_start.wait(3)
        with self.condition:
            if path in {"/tasks", "/task/goal", "/goal/capture", "/goal/import", "/goal/clear"}:
                if payload["expected_epoch"] != self.state["epoch"]:
                    raise ValueError("runtime expected_epoch mismatch")
                self.state["epoch"] += 1
                if path in {"/tasks", "/task/goal"}:
                    self.state["state"] = "WAITING_INPUT"
                elif path == "/goal/clear":
                    self.state["goal_selection"] = {"mode":"live", "ready":False, "selection_id":None}
                else:
                    self.state["goal_selection"] = {
                        "session_id":"session-1", "mode":"frozen", "ready":True,
                        "selection_id":f"selection-{self.state['epoch']}",
                        "sha256":hashlib.sha256(self.goal).hexdigest(),
                        "source":"camera" if path=="/goal/capture" else "import"}
            elif path == "/task/cancel":
                self.state["epoch"] += 1
                self.state["state"] = "CANCELLED"
                self.stop_received.set()
            return deepcopy(self.state), 1

    def test_session_mismatch_does_not_forward(self):
        self.send(session="old-session")
        self.assertIn("session mismatch", self.result("request-1")["error"])
        self.assertEqual(self.calls, [("/status", None)])

    def test_epoch_mismatch_does_not_get_refreshed(self):
        self.state["epoch"] = 4
        self.send()
        self.assertIn("expected_epoch mismatch", self.result("request-1")["error"])
        self.assertEqual(self.calls, [("/status", None)])

    def test_disallowed_action_never_calls_http(self):
        self.assertFalse(self.send(action="inputs/odom"))
        self.assertFalse(self.result("request-1")["ok"])
        self.assertEqual(self.calls, [])

    def test_mutation_passes_explicit_epoch_and_atomic_session(self):
        self.send(payload={"expected_epoch": 0, "execution_mode": "shadow"})
        self.assertTrue(self.result("request-1")["ok"])
        self.assertEqual(self.calls[-1], ("/tasks", {"expected_epoch": 0, "execution_mode": "shadow", "expected_session_id": "session-1"}))

    def test_cancel_uses_fixed_reason_and_session_even_with_stale_epoch(self):
        self.send(action="cancel", payload={"reason": "untrusted", "expected_epoch": 999})
        self.assertTrue(self.result("request-1")["ok"])
        self.assertEqual(self.calls[-1], ("/task/cancel", {"reason": "operator_stop", "expected_session_id": "session-1"}))

    def test_stop_overtakes_delayed_start_and_discards_queued_goal(self):
        self.delayed = True
        self.send(request_id="delayed")
        self.assertTrue(self.start_received.wait(1))
        self.send(request_id="queued", action="replace_goal")
        self.send(request_id="stop", action="cancel", payload={})
        self.assertTrue(self.stop_received.wait(1), "Stop was blocked by a normal HTTP request")
        self.assertTrue(self.result("stop")["ok"])
        self.assertIn("superseded", self.result("queued")["error"])
        self.release_start.set()
        self.assertFalse(self.result("delayed")["ok"])
        self.assertEqual(self.state["state"], "CANCELLED")
        self.assertFalse(any(path == "/task/goal" for path, _ in self.calls))

    def test_bounded_queue_does_not_block_stop(self):
        self.delayed = True
        self.send(request_id="delayed")
        self.assertTrue(self.start_received.wait(1))
        self.assertTrue(self.send(request_id="queued-1"))
        self.assertTrue(self.send(request_id="queued-2"))
        self.assertFalse(self.send(request_id="overflow"))
        self.assertIn("queue is full", self.result("overflow")["error"])
        self.send(request_id="stop", action="cancel", payload={})
        self.assertTrue(self.result("stop")["ok"])

    def test_duplicate_id_does_not_execute_again(self):
        self.send()
        self.assertTrue(self.result("request-1")["ok"])
        self.assertFalse(self.send())
        self.assertTrue(self.result("request-1", 2)["ok"])
        self.assertEqual(sum(path == "/tasks" for path, _ in self.calls), 1)

    def test_capture_uses_raw_frame_and_never_creates_task(self):
        self.send(action="capture_goal")
        result = self.result("request-1")
        self.assertTrue(result["ok"])
        self.assertEqual(result["result"]["goal"], {"rgb_jpeg_base64": "/9hyYXf/2Q=="})
        self.assertEqual(result["result"]["status"]["state"], "IDLE")
        self.assertEqual(result["result"]["selection"]["sha256"], hashlib.sha256(self.goal).hexdigest())
        self.assertEqual(self.read_selections, ["selection-1"])
        self.assertEqual([path for path,_ in self.calls], ["/status", "/goal/capture"])

    def test_capture_rejects_non_jpeg_and_oversize(self):
        for index, body in enumerate((b"not-jpeg", b"\xff\xd8" + b"a" * (4 * 1024 * 1024))):
            self.adapter.read_goal = lambda _selection_id,body=body: body
            self.send(request_id=f"bad-{index}", action="capture_goal", payload={"expected_epoch":self.state["epoch"]})
            self.assertIn("frozen goal must be a JPEG",self.result(f"bad-{index}")["error"])

    def test_frozen_bytes_must_match_confirmed_selection_hash(self):
        self.adapter.read_goal = lambda _selection_id: b"\xff\xd8other\xff\xd9"
        self.send(action="capture_goal")
        self.assertIn("hash differs",self.result("request-1")["error"])

    def test_import_and_clear_use_bounded_selection_endpoints(self):
        self.send(action="import_goal",payload={"expected_epoch":0,"goal":{"rgb_jpeg_base64":"/9hyYXf/2Q=="}})
        result=self.result("request-1")
        self.assertTrue(result["ok"])
        self.assertEqual(result["result"]["selection"]["source"],"import")
        self.send(request_id="clear",action="clear_goal",payload={"expected_epoch":1})
        self.assertTrue(self.result("clear")["ok"])
        self.assertEqual(self.state["goal_selection"]["mode"],"live")
        self.assertFalse(any(path=="/tasks" for path,_ in self.calls))

    def test_duplicate_capture_does_not_mutate_or_retain_binary_result(self):
        self.send(action="capture_goal")
        self.assertTrue(self.result("request-1")["ok"])
        self.send(action="capture_goal")
        duplicate=self.result("request-1",2)
        self.assertFalse(duplicate["ok"])
        self.assertNotIn("result",duplicate)
        self.assertEqual(sum(path=="/goal/capture" for path,_ in self.calls),1)


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.status = {"session_id": "session-1", "epoch": 3, "state": "WAITING_INPUT",
                       "task": {"task_id": "task-1", "goal_id": "goal-1", "goal_version": 2},
                       "preview": {"epoch": 3, "goal_version": 2,
                                   "identity": {"session_id": "session-1", "task_id": "task-1", "goal_id": "goal-1", "goal_version": 2},
                                   "trajectory_body_m": [[0, 0], [1, 0]]}}

    def test_unanchored_body_path_does_not_require_odom(self):
        self.assertEqual(local_preview_poses(self.status), [(0., 0., 0.), (1., 0., 0.)])

    def test_stop_epoch_and_goal_mismatch_clear_path(self):
        for key, value in (("epoch", 4), ("state", "CANCELLED"), ("session_id", "new-session")):
            stale = deepcopy(self.status)
            stale[key] = value
            self.assertEqual(local_preview_poses(stale), [])
        self.status["task"]["goal_id"] = "new-goal"
        self.assertEqual(local_preview_poses(self.status), [])

    def test_malformed_and_nonfinite_paths_clear(self):
        for points in ([[0, float("nan")]], [[1, 2, 3]], [["1", 0]], [[True, 0]]):
            self.status["preview"]["trajectory_body_m"] = points
            self.assertEqual(local_preview_poses(self.status), [])


if __name__ == "__main__":
    unittest.main()
