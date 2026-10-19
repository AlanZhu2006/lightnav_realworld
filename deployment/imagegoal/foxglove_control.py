"""Bounded Foxglove command adapter to the existing ImageGoal HTTP API.

This module has no ROS, TF, velocity, SDK, or actuator interface. The runtime
continues to enforce its configured execution authority and all motion gates.
HTTP work never runs in a ROS subscription callback. A separate Stop worker
can invalidate an in-flight large goal POST through the runtime epoch barrier.
"""
from __future__ import annotations

import base64
import hashlib
from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import math
import re
import threading
from typing import Any, Callable
from urllib.request import urlopen
from urllib.parse import urlencode


MAX_COMMAND_BYTES = 8 * 1024 * 1024
MAX_GOAL_BYTES = 4 * 1024 * 1024
ENDPOINTS = {
    "start": "/tasks", "replace_goal": "/task/goal",
    "pause": "/task/pause", "resume": "/task/resume",
    "cancel": "/task/cancel", "complete": "/task/complete",
    "capture_goal": "/goal/capture", "import_goal": "/goal/import", "clear_goal": "/goal/clear",
    "native_start": "/native/start", "native_stop": "/native/stop",
}
ACTIONS = frozenset(ENDPOINTS)
STOP_ACTIONS = frozenset({"cancel","native_stop"})
SELECTION_ACTIONS = frozenset({"capture_goal", "import_goal", "clear_goal"})
IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9._:-]{0,127}\Z")


@dataclass(frozen=True)
class Command:
    request_id: str
    session_id: str
    action: str
    payload: dict[str, Any]


def parse_command(raw: str) -> Command:
    if not isinstance(raw, str) or len(raw) > MAX_COMMAND_BYTES or len(raw.encode("utf-8")) > MAX_COMMAND_BYTES:
        raise ValueError("operator command exceeds 8 MiB or is not text")
    def reject_constant(value):
        raise ValueError("nonfinite JSON is not allowed")
    value = json.loads(raw, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError("operator command must be an object")
    for key in ("request_id", "session_id"):
        if not isinstance(value.get(key), str) or not IDENTIFIER.fullmatch(value[key]):
            raise ValueError(f"{key} must be 1-128 identifier characters")
    action = value.get("action")
    if not isinstance(action, str) or action not in ACTIONS:
        raise ValueError("action is not allowed")
    payload = value.get("payload")
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")
    if action in {"start", "native_start", "replace_goal"} | SELECTION_ACTIONS or "expected_epoch" in payload:
        epoch = payload.get("expected_epoch")
        if type(epoch) is not int or epoch < 0:
            raise ValueError("payload.expected_epoch must be an explicit nonnegative integer")
    # Also rejects overflowing JSON exponents such as 1e999.
    json.dumps(payload, allow_nan=False)
    return Command(value["request_id"], value["session_id"], action, payload)


def local_preview_poses(status: dict[str, Any]) -> list[tuple[float, float, float]]:
    """Return only the current task's unanchored model preview, never odometry."""
    preview, task = status.get("preview"), status.get("task")
    if not isinstance(preview, dict) or not isinstance(task, dict):
        return []
    if status.get("state") in {"IDLE", "CANCELLED", "SUCCEEDED", "TIMED_OUT", "FAILED"}:
        return []
    identity = preview.get("identity")
    if not isinstance(identity, dict) or preview.get("epoch") != status.get("epoch"):
        return []
    if identity.get("session_id") != status.get("session_id"):
        return []
    if any(identity.get(key) != task.get(key) for key in ("task_id", "goal_id", "goal_version")):
        return []
    if preview.get("goal_version") != task.get("goal_version"):
        return []
    points = preview.get("trajectory_body_m")
    if not isinstance(points, list) or len(points) > 10000:
        return []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return []
        if any(type(number) not in (int, float) or not math.isfinite(number) for number in point):
            return []
    poses = []
    for index, (x, y) in enumerate(points):
        other = points[min(index + 1, len(points) - 1)]
        previous = points[max(index - 1, 0)]
        dx, dy = (other[0] - x, other[1] - y) if index + 1 < len(points) else (x - previous[0], y - previous[1])
        poses.append((float(x), float(y), math.atan2(dy, dx)))
    return poses


class OperatorControl:
    """Two bounded command lanes; Stop never waits for normal HTTP requests.

    request_json and read_goal are injectable so concurrency and barriers can be
    tested without ROS. A successful Stop discards queued normal requests and
    supersedes their generation. Start/goal always carry the client's epoch all
    the way into the runtime; no stale request is rearmed using a newer epoch.
    """
    def __init__(self, runtime_url: str, publish: Callable[[dict[str, Any]], None], *,
                 request_json: Callable, read_goal: Callable[[str], bytes] | None = None,
                 queue_size: int = 4, result_cache_size: int = 64):
        if queue_size < 1 or result_cache_size < 2 * queue_size + 2:
            raise ValueError("invalid queue/cache bounds")
        self.runtime_url = runtime_url.rstrip("/")
        self.publish = publish
        self.request_json = request_json
        self.read_goal = read_goal or self._read_goal
        self.queue_size, self.result_cache_size = queue_size, result_cache_size
        self.condition = threading.Condition()
        self.normal, self.stops = deque(), deque()
        self.generation = 0
        self.closed = False
        self.seen = OrderedDict()
        self.workers = [threading.Thread(target=self._loop, args=(stop,), name="imagegoal-foxglove-stop" if stop else "imagegoal-foxglove-command", daemon=True)
                        for stop in (False, True)]
        for worker in self.workers:
            worker.start()

    def _read_goal(self, selection_id: str) -> bytes:
        query = urlencode({"selection_id": selection_id})
        with urlopen(self.runtime_url + "/goal.jpg?" + query, timeout=1.0) as response:
            if response.headers.get("X-ImageGoal-Selection-Id") != selection_id:
                raise ValueError("frozen goal response selection identity mismatch")
            body = response.read(MAX_GOAL_BYTES + 1)
        return body

    def _emit(self, result):
        try:
            self.publish(result)
        except Exception:
            # A disconnected ROS/Foxglove subscriber must not kill Stop worker.
            pass

    def _finish(self, command, *, result=None, error=None):
        response = {"request_id": command.request_id, "action": command.action, "ok": error is None}
        response["result" if error is None else "error"] = result if error is None else str(error)[:1000]
        with self.condition:
            # Capture responses contain up to 4 MiB of JPEG. Keep the ID for
            # replay protection, but do not retain that binary in the cache.
            cached = response if command.action not in {"capture_goal", "import_goal"} or error is not None else {
                "request_id": command.request_id, "action": command.action, "ok": False,
                "error": "goal selection request already completed; recover the frozen goal from current runtime status"}
            self.seen[command.request_id] = cached
            self.seen.move_to_end(command.request_id)
            self._trim_seen()
        self._emit(response)

    def _trim_seen(self):
        while len(self.seen) > self.result_cache_size:
            complete = next((key for key, value in self.seen.items() if value is not None), None)
            if complete is None:
                break
            del self.seen[complete]

    def submit(self, raw: str) -> bool:
        """Validate and enqueue only; never perform HTTP in the caller thread."""
        try:
            command = parse_command(raw)
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            # Recover only short IDs for an actionable error; never echo payload.
            request_id, action = "", ""
            if isinstance(raw, str) and len(raw) <= MAX_COMMAND_BYTES:
                try:
                    value = json.loads(raw)
                    if isinstance(value, dict):
                        request_id = str(value.get("request_id", ""))[:128]
                        action = str(value.get("action", ""))[:64]
                except (ValueError, TypeError, RecursionError):
                    pass
            self._emit({"request_id": request_id, "action": action, "ok": False, "error": str(exc)[:1000]})
            return False
        discarded = []
        duplicate = None
        error = None
        with self.condition:
            if command.request_id in self.seen:
                duplicate = self.seen[command.request_id]
                if duplicate is None:
                    return False  # Original request will publish its result.
            elif self.closed:
                error = "operator adapter is shutting down"
            else:
                queue = self.stops if command.action in STOP_ACTIONS else self.normal
                if len(queue) >= self.queue_size:
                    error = "operator command queue is full"
                else:
                    self.seen[command.request_id] = None
                    if command.action in STOP_ACTIONS:
                        self.generation += 1
                        discarded = [entry[0] for entry in self.normal]
                        self.normal.clear()
                    queue.append((command, self.generation))
                    self.condition.notify_all()
                    self._trim_seen()
        if duplicate is not None:
            self._emit(duplicate)
            return False
        if error is not None:
            self._finish(command, error=error)
            return False
        for dropped in discarded:
            self._finish(dropped, error="superseded by operator Stop")
        return True

    def _check_generation(self, generation):
        with self.condition:
            if self.closed or generation != self.generation:
                raise ValueError("superseded by operator Stop or shutdown")

    def _execute(self, command, generation):
        stop = command.action in STOP_ACTIONS
        if command.action == "native_stop":
            # The runtime checks the session and invalidates the epoch. Send
            # Stop directly, without waiting for a preliminary status GET.
            result,_ = self.request_json(self.runtime_url+ENDPOINTS[command.action],timeout_s=.5,
                                        payload={"expected_session_id":command.session_id,"reason":"operator_native_stop"})
            return result
        if not stop:
            self._check_generation(generation)
        status, _ = self.request_json(self.runtime_url + "/status", timeout_s=1.0)
        if not isinstance(status, dict) or status.get("session_id") != command.session_id:
            raise ValueError("runtime session mismatch; reconnect and use current status")
        if not stop:
            self._check_generation(generation)
            expected = command.payload.get("expected_epoch")
            if expected is not None and expected != status.get("epoch"):
                raise ValueError("expected_epoch mismatch; command was superseded or stopped")
        payload = {"reason": "operator_stop"} if stop else dict(command.payload)
        payload["expected_session_id"] = command.session_id
        # The fixed map is the complete POST surface. In particular no inputs,
        # simulation, capture-task creation, parameters, or services are exposed.
        result, _ = self.request_json(self.runtime_url + ENDPOINTS[command.action], timeout_s=1.0 if stop else 5.0, payload=payload)
        if not stop:
            self._check_generation(generation)
        if command.action in {"capture_goal", "import_goal"}:
            selection = result.get("goal_selection") if isinstance(result,dict) else None
            if (not isinstance(selection,dict) or selection.get("ready") is not True
                    or selection.get("mode") != "frozen" or selection.get("session_id") != command.session_id
                    or not isinstance(selection.get("selection_id"),str)):
                raise ValueError("runtime did not confirm a frozen goal selection")
            body = self.read_goal(selection["selection_id"])
            self._check_generation(generation)
            if not isinstance(body,bytes) or not 0 < len(body) <= MAX_GOAL_BYTES or not body.startswith(b"\xff\xd8"):
                raise ValueError("frozen goal must be a JPEG of at most 4 MiB")
            if hashlib.sha256(body).hexdigest() != selection.get("sha256"):
                raise ValueError("frozen goal JPEG hash differs from the confirmed selection")
            return {"status":result,"selection":selection,
                    "goal":{"rgb_jpeg_base64":base64.b64encode(body).decode("ascii")}}
        return result

    def _loop(self, stop):
        queue = self.stops if stop else self.normal
        while True:
            with self.condition:
                while not queue and not self.closed:
                    self.condition.wait()
                if self.closed:
                    return
                command, generation = queue.popleft()
            try:
                result = self._execute(command, generation)
            except Exception as exc:
                self._finish(command, error=exc)
            else:
                self._finish(command, result=result)

    def close(self):
        with self.condition:
            self.closed = True
            discarded = list(self.normal) + list(self.stops)
            self.normal.clear()
            self.stops.clear()
            self.condition.notify_all()
        for command, _ in discarded:
            self._finish(command, error="operator adapter is shutting down")
        for worker in self.workers:
            worker.join(timeout=2.0)
