"""Bounded cleanup of exactly one caller-owned LightNav session."""
from __future__ import annotations

import time
import urllib.error


def reset_owned_session(server_url, session_id, timeout_s, post_json, *,
                        monotonic=time.monotonic, sleep=time.sleep):
    """Retry busy/transport failures after an inference timeout.

    Client timeouts do not cancel GPU work. Keep trying the same private session
    for at least two minutes; never clear another session or raise capacity.
    Return an explicit receipt so cleanup failure cannot mask the original error.
    """
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("Cleanup requires the caller's explicit private session ID")
    identity = {"session_id": session_id}
    deadline = monotonic() + max(120., float(timeout_s) + 5.)
    attempts, error = 0, None
    while True:
        attempts += 1
        try:
            response = post_json(server_url.rstrip("/") + "/reset", {"identity": identity, "retire": True},
                                 min(5., max(.1, deadline - monotonic())))
            if not isinstance(response, dict) or response.get("ok") is not True:
                raise ValueError("Session reset did not acknowledge success")
            if response.get("retired") is not True:
                raise ValueError("Server did not acknowledge durable session retirement")
            if response.get("request_identity") != identity:
                raise ValueError("Session reset acknowledged a different identity")
            return {"ok": True, "attempts": attempts, "identity": identity, "response": response}
        except urllib.error.HTTPError as exc:
            error = f"HTTP {exc.code}: {exc.reason}"
            retry = exc.code in (408, 429) or 500 <= exc.code < 600
        except (OSError, TimeoutError) as exc:
            error = f"{type(exc).__name__}: {exc}"
            retry = True
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            retry = False
        remaining = deadline - monotonic()
        if not retry or remaining <= 0:
            return {"ok": False, "attempts": attempts, "identity": identity, "error": error}
        sleep(min(.25, remaining))
