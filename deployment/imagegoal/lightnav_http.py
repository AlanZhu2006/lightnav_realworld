"""One owned keep-alive HTTP connection for LightNav, with no automatic replay.

Requests are serialized. Any HTTP/JSON/transport failure discards the connection
and propagates to the runtime; only a later explicit request may connect again.
close() atomically closes the client and cancels its owned in-flight request
without waiting for the network lock. No robot or model code.
"""
from __future__ import annotations

import http.client
import json
import math
import socket
import threading
import time
from email.utils import parsedate_to_datetime
from urllib.parse import urlsplit

from deployment.nomad.shadow_client import _json_loads


MAX_RESPONSE_BYTES = 32 << 20
MAX_ERROR_BYTES = 4096


class LightNavHTTPError(RuntimeError):
    """A bounded server refusal, kept separate from an uncertain transport loss."""

    def __init__(self, status: int, content: bytes, *, retry_after=None):
        self.status = self.status_code = int(status)
        self.body_truncated = len(content) > MAX_ERROR_BYTES
        self.body = content[:MAX_ERROR_BYTES].decode("utf-8", errors="replace")
        self.body = self.body.encode("utf-8")[:MAX_ERROR_BYTES].decode("utf-8", errors="ignore")
        try:
            parsed = _json_loads(content[:MAX_ERROR_BYTES])
        except (ValueError, UnicodeError):
            parsed = None
        self.payload = parsed if isinstance(parsed, dict) else None
        data = self.payload or {}
        value = data.get("error_code", data.get("code"))
        self.code = value if isinstance(value, str) else None
        self.error_code = self.code
        self.retryable = data.get("retryable") if type(data.get("retryable")) is bool else None
        value = data.get("server_boot_id")
        self.server_boot_id = value if isinstance(value, str) else None
        self.retry_after_s = self._retry_delay(retry_after)
        if self.retry_after_s is None:
            self.retry_after_s = self._retry_delay(data.get("retry_after_s"))
        self.retry_after = self.retry_after_s
        super().__init__(f"HTTP {self.status}: {self.body}")

    @staticmethod
    def _retry_delay(value):
        if value is None or isinstance(value, bool):
            return None
        try:
            delay = float(value)
        except (TypeError, ValueError):
            try:
                delay = parsedate_to_datetime(str(value)).timestamp() - time.time()
            except (TypeError, ValueError, OverflowError):
                return None
        return max(0., delay) if math.isfinite(delay) and delay >= 0 else None

    def as_dict(self):
        return {"type": type(self).__name__, "status": self.status,
                "code": self.code, "error_code": self.code,
                "retryable": self.retryable, "retry_after_s": self.retry_after_s,
                "server_boot_id": self.server_boot_id, "body": self.body,
                "body_truncated": self.body_truncated}


class LightNavRequestCancelled(RuntimeError):
    """The owner cancelled this request; a later explicit request is allowed."""


class LightNavHTTPClient:
    def __init__(self):
        self._request_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._connection = None
        self._endpoint = None
        self._inflight = False
        self._closed = False
        self._active_cancelled = None
        self._last_request_stats = None

    @property
    def last_request_stats(self):
        with self._state_lock:
            return None if self._last_request_stats is None else dict(self._last_request_stats)

    @staticmethod
    def _discard_connection(connection, *, shutdown=False):
        # close() alone need not interrupt another thread blocked in read/send.
        # This socket belongs exclusively to this client's serialized request.
        owned_socket = connection.sock
        if shutdown and owned_socket is not None:
            try:
                owned_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        connection.close()

    def cancel_inflight(self) -> bool:
        """Abort the owned live socket without closing the client or replaying.

        Returns promptly. A connection still resolving/connecting has no socket
        to shut down yet; its request deadline remains the bound, and the token
        prevents sending its body once connection setup finishes.
        """
        with self._state_lock:
            if not self._inflight or self._active_cancelled is None:
                return False
            self._active_cancelled.set()
            connection, self._connection = self._connection, None
            self._endpoint = None
        if connection is not None:
            self._discard_connection(connection, shutdown=True)
        return True

    def close(self) -> None:
        """Atomically reject future requests and abort the owned request."""
        with self._state_lock:
            self._closed = True
            if self._active_cancelled is not None:
                self._active_cancelled.set()
            connection, self._connection = self._connection, None
            self._endpoint = None
        if connection is not None:
            self._discard_connection(connection, shutdown=True)

    def request_json(self, url: str, *, timeout_s: float, payload=None, stats=None):
        """Return ``(JSON, elapsed_ms)`` once; never replay an uploaded body.

        Optional ``stats`` is updated on completion, including failures.
        ``serialized_bytes`` counts the JSON body, not TCP delivery or a server
        ACK. ``send_completed`` only means the local HTTP send returned.
        """
        if (isinstance(timeout_s,bool) or not isinstance(timeout_s,(int,float)) or
                not math.isfinite(timeout_s) or timeout_s<=0):
            raise ValueError("timeout_s must be positive and finite")
        parsed = urlsplit(url)
        if (parsed.scheme not in {"http","https"} or not parsed.hostname or
                parsed.username is not None or parsed.password is not None or parsed.fragment):
            raise ValueError("Expected an absolute HTTP(S) URL without credentials or fragment")
        port = parsed.port or (443 if parsed.scheme=="https" else 80)
        target = parsed.path or "/"
        if parsed.query:
            target += "?"+parsed.query
        endpoint = (parsed.scheme,parsed.hostname,port,target)
        body = None if payload is None else json.dumps(payload,allow_nan=False,separators=(",",":")).encode()
        headers = {"Content-Type": "application/json", "Connection": "keep-alive"}
        if parsed.path == "/infer":
            identity = payload.get("identity") if isinstance(payload, dict) else None
            for field, header in (("session_id", "X-LightNav-Session-Id"),
                                  ("task_id", "X-LightNav-Task-Id")):
                value = identity.get(field) if isinstance(identity, dict) else None
                if not isinstance(value, str) or not value or not value.isascii() or any(ord(c)<33 or ord(c)>126 for c in value):
                    raise ValueError(f"LightNav inference identity.{field} must be nonempty printable ASCII")
                headers[header] = value
            if isinstance(payload, dict) and "server_boot_id" in payload:
                value = payload["server_boot_id"]
                if not isinstance(value, str) or not value or not value.isascii() or any(ord(c)<33 or ord(c)>126 for c in value):
                    raise ValueError("LightNav inference server_boot_id must be nonempty printable ASCII")
                headers["X-LightNav-Server-Boot-Id"] = value
            headers["X-LightNav-Request-Timeout-Ms"] = str(min(8000, max(1, math.ceil(timeout_s*1000))))
        started = time.perf_counter()
        deadline = started+timeout_s
        transport_stats = {"serialized_bytes": 0 if body is None else len(body),
                           "attempted_body_bytes": 0, "send_completed": False,
                           "response_status": None, "response_bytes": 0,
                           "transport_failed": True, "cancelled": False}
        def remaining():
            result = deadline-time.perf_counter()
            if result<=0:
                raise TimeoutError("LightNav HTTP request deadline exceeded")
            return result
        with self._state_lock:
            if self._closed:
                raise RuntimeError("LightNav HTTP client is closed")
        if not self._request_lock.acquire(timeout=remaining()):
            raise TimeoutError("LightNav HTTP request serialization deadline exceeded")
        connection, discard, response, watchdog = None, True, None, None
        cancelled = threading.Event()
        deadline_expired = threading.Event()

        def check_cancelled():
            if cancelled.is_set():
                raise LightNavRequestCancelled("LightNav HTTP request cancelled by its owner")

        def expire_request():
            # A socket timeout is an idle timeout, so headers trickled by a
            # peer could otherwise exceed the total request deadline. Fence
            # this exact request while holding the short state lock; a later
            # request may reuse the socket only after this owner releases it.
            with self._state_lock:
                if self._active_cancelled is not cancelled or not self._inflight:
                    return
                deadline_expired.set()
                owned_socket = None if connection is None else connection.sock
                if owned_socket is not None:
                    try:
                        owned_socket.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
        try:
            old = None
            with self._state_lock:
                if self._closed:
                    raise RuntimeError("LightNav HTTP client is closed")
                if self._endpoint!=endpoint:
                    old, self._connection = self._connection, None
                    self._endpoint = endpoint
                if self._connection is None:
                    constructor = http.client.HTTPSConnection if parsed.scheme=="https" else http.client.HTTPConnection
                    self._connection = constructor(parsed.hostname,port,timeout=remaining())
                    # We connect explicitly below. If cancellation closes the
                    # socket just before request(), never let http.client open
                    # another socket and upload the cancelled body implicitly.
                    self._connection.auto_open = 0
                connection = self._connection
                self._inflight = True
                self._active_cancelled = cancelled
            watchdog = threading.Timer(remaining(), expire_request)
            watchdog.daemon = True
            watchdog.start()
            if old is not None:
                old.close()
            connection.timeout = remaining()
            check_cancelled()
            # Explicit connect lets cancellation fence the body even when it
            # happened while DNS/connect was still bounded by the deadline.
            if connection.sock is None:
                connection.connect()
            check_cancelled()
            if connection.sock is not None:
                connection.sock.settimeout(remaining())
            transport_stats["attempted_body_bytes"] = transport_stats["serialized_bytes"]
            connection.request("GET" if body is None else "POST",target,body=body,
                               headers=headers)
            transport_stats["send_completed"] = True
            check_cancelled()
            if connection.sock is not None:
                connection.sock.settimeout(remaining())
            response = connection.getresponse()
            check_cancelled()
            transport_stats["response_status"] = response.status
            success = 200<=response.status<300
            expected = response.getheader("Content-Length")
            if expected is not None:
                if not expected.isdecimal():
                    raise ValueError("Invalid response Content-Length")
                expected = int(expected)
                if success and expected>MAX_RESPONSE_BYTES:
                    raise ValueError("server response exceeds 32 MiB")
            limit = MAX_RESPONSE_BYTES if success else MAX_ERROR_BYTES
            chunks, size = [], 0
            while True:
                check_cancelled()
                if connection.sock is not None:
                    connection.sock.settimeout(remaining())
                else:
                    remaining()
                chunk = response.read1(min(65536,limit+1-size))
                if not chunk:
                    break
                chunks.append(chunk)
                size += len(chunk)
                transport_stats["response_bytes"] = size
                if size>limit:
                    if success:
                        raise ValueError("server response exceeds 32 MiB")
                    break
            content = b"".join(chunks)
            check_cancelled()
            if not success:
                transport_stats["transport_failed"] = False
                raise LightNavHTTPError(response.status, content,
                                        retry_after=response.getheader("Retry-After"))
            if expected is not None and size!=expected:
                raise ValueError("Incomplete HTTP response body")
            result = _json_loads(content)
            remaining()
            check_cancelled()
            discard = response.will_close
            transport_stats["transport_failed"] = False
            # read1() with an exactly consumed Content-Length can leave its
            # response wrapper open. Close that wrapper explicitly; this keeps
            # the HTTP/1.1 socket and allows getresponse() on the next request.
            response.close()
            return result,(time.perf_counter()-started)*1000.
        except Exception as exc:
            if deadline_expired.is_set():
                transport_stats["transport_failed"] = True
                transport_stats["error_type"] = "TimeoutError"
                raise TimeoutError("LightNav HTTP request deadline exceeded") from exc
            if cancelled.is_set() and not isinstance(exc, LightNavRequestCancelled):
                transport_stats["transport_failed"] = True
                transport_stats["error_type"] = "LightNavRequestCancelled"
                raise LightNavRequestCancelled("LightNav HTTP request cancelled by its owner") from exc
            transport_stats["error_type"] = type(exc).__name__
            raise
        finally:
            if watchdog is not None:
                watchdog.cancel()
            transport_stats["cancelled"] = cancelled.is_set()
            transport_stats["elapsed_ms"] = (time.perf_counter()-started)*1000.
            with self._state_lock:
                if discard or self._closed or cancelled.is_set():
                    if self._connection is connection:
                        self._connection = None
                        self._endpoint = None
                    discard = True
                self._inflight = False
                self._active_cancelled = None
                self._last_request_stats = dict(transport_stats)
            if stats is not None:
                stats.update(transport_stats)
            if discard and connection is not None:
                self._discard_connection(connection, shutdown=True)
            if response is not None:
                response.close()
            self._request_lock.release()
