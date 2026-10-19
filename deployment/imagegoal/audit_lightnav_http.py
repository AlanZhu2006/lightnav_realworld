"""CPU-only keep-alive and failure checks against a local fake HTTP/1.1 server.

No model, GPU, robot SDK, runtime service, or robot task is used.
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import unittest
from unittest.mock import patch

from .lightnav_http import LightNavHTTPClient, LightNavHTTPError, LightNavRequestCancelled, MAX_ERROR_BYTES, MAX_RESPONSE_BYTES


class FakeServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    def __init__(self):
        self.lock = threading.Lock()
        self.connection_count = 0
        self.requests = []
        self.entered = threading.Event()
        self.release = threading.Event()
        super().__init__(("127.0.0.1",0),FakeHandler)


class FakeHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def setup(self):
        super().setup()
        with self.server.lock:
            self.server.connection_count += 1
            self.connection_id = self.server.connection_count
    def log_message(self,*args):
        pass
    def do_GET(self):
        self.do_POST()
    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length","0")))
        with self.server.lock:
            self.server.requests.append({"path":self.path,"connection_id":self.connection_id,"body":body,
                                         "headers":dict(self.headers)})
        if self.path=="/drop":
            self.close_connection = True
            return
        if self.path=="/timeout":
            time.sleep(.2)
        if self.path=="/wait":
            self.server.entered.set()
            self.server.release.wait(2)
        if self.path=="/drip_headers":
            try:
                for byte in b'HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\n{}':
                    self.wfile.write(bytes([byte]))
                    self.wfile.flush()
                    time.sleep(.005)
            except (BrokenPipeError,ConnectionResetError):
                self.close_connection = True
            return
        status, content, length = 200,json.dumps({"connection_id":self.connection_id,
            "payload":None if not body else json.loads(body),"method":self.command}).encode(),None
        if self.path=="/error":
            status,content = 409,b'{"error":"fixture conflict"}'
        elif self.path in ("/capacity", "/busy"):
            busy = self.path == "/busy"
            status = 429 if busy else 409
            content = json.dumps({"error":"fixture refusal", "error_code":"inference_busy" if busy else "session_capacity",
                                  "retryable":busy, "server_boot_id":"offline-boot"}).encode()
        elif self.path=="/long_error":
            status,content = 409,b'{"error":"'+b'x'*(MAX_ERROR_BYTES*2)+b'"}'
        elif self.path=="/invalid":
            content = b"{"
        elif self.path=="/nan":
            content = b'{"value":NaN}'
        elif self.path=="/infinity":
            content = b'{"value":Infinity}'
        elif self.path=="/float_overflow":
            content = b'{"value":1e999}'
        elif self.path=="/oversize":
            content,length = b"",MAX_RESPONSE_BYTES+1
        elif self.path=="/partial":
            content,length = b'{"ok":true}',100
            self.close_connection = True
        if self.path=="/idle_close":
            self.close_connection = True
        try:
            self.send_response(status)
            self.send_header("Content-Type","application/json")
            self.send_header("Content-Length",str(len(content) if length is None else length))
            if self.path == "/busy":
                self.send_header("Retry-After", "1.25")
            if self.close_connection:
                self.send_header("Connection","close")
            self.end_headers()
            self.wfile.write(content)
            self.wfile.flush()
        except (BrokenPipeError,ConnectionResetError):
            self.close_connection = True


class HTTPClientTests(unittest.TestCase):
    def setUp(self):
        self.server = FakeServer()
        self.thread = threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.01),daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.client = LightNavHTTPClient()

    def tearDown(self):
        self.client.close()
        self.server.release.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)

    def request(self,path="/ok",*,timeout_s=1.,payload=None):
        return self.client.request_json(self.url+path,timeout_s=timeout_s,payload=payload)

    def test_reuses_one_connection_and_preserves_json_request(self):
        first, rtt = self.request(payload={"identity":{"task_id":"fixture"},"observations":[1,2]})
        second, _ = self.request(payload={"identity":{"task_id":"next"}})
        self.assertEqual(first["connection_id"],second["connection_id"])
        self.assertEqual(self.server.connection_count,1)
        self.assertEqual(first["payload"]["identity"]["task_id"],"fixture")
        self.assertGreater(rtt,0)

    def test_get_and_explicit_endpoint_change_close_old_connection(self):
        first,_ = self.request()
        self.assertEqual(first["method"],"GET")
        second,_ = self.request("/other")
        self.assertNotEqual(first["connection_id"],second["connection_id"])
        self.assertEqual(self.server.connection_count,2)

    def test_http_error_discards_connection_without_replaying(self):
        with self.assertRaisesRegex(RuntimeError,"HTTP 409"):
            self.request("/error",payload={"infer":"once"})
        self.assertIsNone(self.client._connection)
        self.assertEqual(len(self.server.requests),1)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],2)

    def test_capacity_error_preserves_status_code_retryability_and_server_boot(self):
        stats = {}
        with self.assertRaises(LightNavHTTPError) as raised:
            self.client.request_json(self.url+"/capacity", timeout_s=1, payload={"once":True},stats=stats)
        error = raised.exception
        self.assertEqual(error.status,409)
        self.assertEqual(error.code,"session_capacity")
        self.assertIs(error.retryable,False)
        self.assertEqual(error.server_boot_id,"offline-boot")
        self.assertIsNone(error.retry_after_s)
        self.assertEqual(json.loads(json.dumps(error.as_dict()))["error_code"],"session_capacity")
        self.assertIs(stats["transport_failed"],False)
        self.assertEqual(stats["response_status"],409)
        self.assertEqual(len(self.server.requests),1)
        self.assertIsNone(self.client._connection)

    def test_busy_error_preserves_retry_after_without_retrying(self):
        with self.assertRaises(LightNavHTTPError) as raised:
            self.request("/busy", payload={"once":True})
        error = raised.exception
        self.assertEqual((error.status,error.code,error.retry_after_s),(429,"inference_busy",1.25))
        self.assertIs(error.retryable,True)
        self.assertEqual(len(self.server.requests),1)

    def test_error_body_is_bounded_and_unparseable_errors_stay_typed(self):
        with self.assertRaises(LightNavHTTPError) as raised:
            self.request("/long_error")
        error = raised.exception
        self.assertLessEqual(len(error.body.encode()),MAX_ERROR_BYTES)
        self.assertTrue(error.body_truncated)
        self.assertIsNone(error.payload)
        self.assertEqual(error.status,409)
        self.assertEqual(len(self.server.requests),1)

    def test_stats_count_serialized_bytes_without_claiming_delivery_ack(self):
        stats = {}
        payload = {"identity":{"task_id":"fixture"},"observations":[1,2]}
        self.client.request_json(self.url+"/ok",timeout_s=1,payload=payload,stats=stats)
        self.assertEqual(stats["serialized_bytes"],len(self.server.requests[0]["body"]))
        self.assertEqual(stats["attempted_body_bytes"],stats["serialized_bytes"])
        self.assertIs(stats["send_completed"],True)
        self.assertIs(stats["transport_failed"],False)
        self.assertGreater(stats["elapsed_ms"],0)
        self.assertEqual(stats,self.client.last_request_stats)
        stats["elapsed_ms"] = -1
        self.assertGreater(self.client.last_request_stats["elapsed_ms"],0)

    def test_infer_identity_headers_match_body_and_deadline_is_bounded(self):
        for timeout, expected in ((.1251,"126"),(20,"8000")):
            payload = {"identity":{"session_id":"session-fixture","task_id":"task-fixture"}}
            self.request("/infer",timeout_s=timeout,payload=payload)
            row = self.server.requests[-1]
            self.assertEqual(row["headers"]["X-LightNav-Session-Id"],payload["identity"]["session_id"])
            self.assertEqual(row["headers"]["X-LightNav-Task-Id"],payload["identity"]["task_id"])
            self.assertEqual(row["headers"]["X-LightNav-Request-Timeout-Ms"],expected)
        for identity in (None,{}, {"session_id":"safe", "task_id":"unsafe\r\n"},
                         {"session_id":"safe", "task_id":"非ASCII"}):
            with self.assertRaises(ValueError):
                self.request("/infer",payload={"identity":identity})
        self.assertEqual(len(self.server.requests),2)

    def test_infer_server_boot_header_matches_payload_and_invalid_values_never_upload(self):
        identity = {"session_id":"session-fixture","task_id":"task-fixture"}
        payload = {"identity":identity,"server_boot_id":"offline-boot-123"}
        self.request("/infer",payload=payload)
        row = self.server.requests[-1]
        self.assertEqual(row["headers"]["X-LightNav-Server-Boot-Id"],payload["server_boot_id"])
        self.assertEqual(json.loads(row["body"])["server_boot_id"],payload["server_boot_id"])
        for value in (None,True,123,"","unsafe\r\n","非ASCII","has space"):
            with self.subTest(value=value),self.assertRaises(ValueError):
                self.request("/infer",payload={"identity":identity,"server_boot_id":value})
        self.assertEqual(len(self.server.requests),1)

    def test_invalid_and_nonfinite_json_are_rejected_and_connection_discarded(self):
        for path in ("/invalid","/nan","/infinity","/float_overflow"):
            with self.subTest(path=path):
                before = len(self.server.requests)
                with self.assertRaises(ValueError):
                    self.request(path)
                self.assertIsNone(self.client._connection)
                self.assertEqual(len(self.server.requests),before+1)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],5)

    def test_partial_body_and_oversize_response_are_rejected(self):
        for path in ("/partial","/oversize"):
            with self.subTest(path=path),self.assertRaises(ValueError):
                self.request(path)
            self.assertIsNone(self.client._connection)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],3)

    def test_dropped_connection_is_not_replayed_and_next_call_can_connect(self):
        with self.assertRaises(Exception):
            self.request("/drop",payload={"infer":"exactly once"})
        self.assertEqual(len(self.server.requests),1)
        self.assertIsNone(self.client._connection)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],2)

    def test_timeout_discards_connection_and_next_request_can_connect(self):
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.request("/timeout",timeout_s=.025)
        self.assertLess(time.monotonic()-started,.2)
        self.assertIsNone(self.client._connection)
        self.assertEqual(len(self.server.requests),1)
        self.assertIs(self.client.last_request_stats["transport_failed"],True)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],2)

    def test_total_deadline_interrupts_a_peer_trickling_response_headers(self):
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.request("/drip_headers",timeout_s=.04)
        self.assertLess(time.monotonic()-started,.15)
        self.assertEqual(len(self.server.requests),1)
        self.assertIsNone(self.client._connection)
        self.assertIs(self.client.last_request_stats["transport_failed"],True)

    def test_cancel_inflight_unblocks_pending_io_and_allows_new_explicit_request(self):
        self.assertIs(self.client.cancel_inflight(),False)
        results,errors,stats = [],[],{}
        def pending():
            try:
                results.append(self.client.request_json(self.url+"/wait",timeout_s=1,payload={"once":True},stats=stats))
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=pending)
        worker.start()
        self.assertTrue(self.server.entered.wait(1))
        started = time.monotonic()
        self.assertIs(self.client.cancel_inflight(),True)
        self.assertLess(time.monotonic()-started,.1)
        worker.join(timeout=.4)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors),1)
        self.assertIsInstance(errors[0],LightNavRequestCancelled)
        self.assertEqual(results,[])
        self.assertIs(stats["cancelled"],True)
        self.assertIs(stats["transport_failed"],True)
        self.assertIsNone(self.client._connection)
        self.assertEqual(len(self.server.requests),1)
        result,_ = self.request()
        self.assertEqual(result["connection_id"],2)
        self.assertIs(self.client.cancel_inflight(),False)

    def test_cancel_during_connection_setup_prevents_body_send(self):
        from http.client import HTTPConnection
        entered,release = threading.Event(),threading.Event()
        original_connect = HTTPConnection.connect
        errors = []
        def slow_connect(connection):
            entered.set()
            release.wait(1)
            original_connect(connection)
        def pending():
            try:
                self.request(payload={"must_not_upload":True})
            except Exception as exc:
                errors.append(exc)
        with patch.object(HTTPConnection,"connect",slow_connect):
            worker = threading.Thread(target=pending)
            worker.start()
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.client.cancel_inflight())
            release.set()
            worker.join(timeout=.4)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors),1)
        self.assertIsInstance(errors[0],LightNavRequestCancelled)
        self.assertEqual(self.server.requests,[])
        self.assertIsNone(self.client._connection)
        result,_ = self.request()
        self.assertEqual(result["method"],"GET")

    def test_cancel_just_before_send_cannot_implicitly_reconnect_and_upload(self):
        from http.client import HTTPConnection
        original_request = HTTPConnection.request
        def cancel_then_send(connection,*args,**kwargs):
            self.assertTrue(self.client.cancel_inflight())
            return original_request(connection,*args,**kwargs)
        with patch.object(HTTPConnection,"request",cancel_then_send):
            with self.assertRaises(LightNavRequestCancelled):
                self.request(payload={"must_not_upload":True})
        self.assertEqual(self.server.requests,[])
        self.assertEqual(self.client.last_request_stats["send_completed"],False)
        result,_ = self.request()
        self.assertEqual(result["method"],"GET")

    def test_server_closed_idle_connection_gets_new_connection_next_request(self):
        first,_ = self.request("/idle_close")
        self.assertIsNone(self.client._connection)
        second,_ = self.request("/idle_close")
        self.assertNotEqual(first["connection_id"],second["connection_id"])

    def test_close_during_inflight_is_prompt_and_prevents_all_later_requests(self):
        result = []
        errors = []
        def pending():
            try:
                result.append(self.request("/wait"))
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=pending)
        worker.start()
        self.assertTrue(self.server.entered.wait(1))
        started = time.monotonic()
        self.client.close()
        self.assertLess(time.monotonic()-started,.1)
        with self.assertRaisesRegex(RuntimeError,"closed"):
            self.request()
        self.server.release.set()
        worker.join(timeout=1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors),1)
        self.assertIsInstance(errors[0],LightNavRequestCancelled)
        self.assertEqual(result,[])
        self.assertIsNone(self.client._connection)
        self.assertEqual(len(self.server.requests),1)
        self.client.close()

    def test_close_atomically_cancels_connect_started_after_an_idle_cancel(self):
        from http.client import HTTPConnection
        self.assertFalse(self.client.cancel_inflight())
        entered,release = threading.Event(),threading.Event()
        original_connect = HTTPConnection.connect
        errors = []
        def slow_connect(connection):
            entered.set()
            release.wait(1)
            original_connect(connection)
        def pending():
            try:
                self.request(payload={"must_not_upload_after_close":True})
            except Exception as exc:
                errors.append(exc)
        with patch.object(HTTPConnection,"connect",slow_connect):
            worker = threading.Thread(target=pending)
            worker.start()
            self.assertTrue(entered.wait(1))
            started = time.monotonic()
            self.client.close()
            self.assertLess(time.monotonic()-started,.1)
            release.set()
            worker.join(timeout=.4)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors),1)
        self.assertIsInstance(errors[0],LightNavRequestCancelled)
        self.assertEqual(self.server.requests,[])
        with self.assertRaisesRegex(RuntimeError,"closed"):
            self.request(payload={"still_forbidden":True})
        self.assertIsNone(self.client._connection)

    def test_close_idle_and_close_before_use_do_not_allow_requests(self):
        self.request()
        self.client.close()
        self.assertIsNone(self.client._connection)
        with self.assertRaisesRegex(RuntimeError,"closed"):
            self.request()
        unused = LightNavHTTPClient()
        unused.close()
        with self.assertRaisesRegex(RuntimeError,"closed"):
            unused.request_json(self.url+"/ok",timeout_s=1,payload={})

    def test_serializes_simultaneous_requests_on_one_connection(self):
        results,errors = [],[]
        def submit(index):
            try:
                results.append(self.request(payload={"index":index})[0])
            except Exception as exc:
                errors.append(exc)
        workers = [threading.Thread(target=submit,args=(index,)) for index in range(4)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(timeout=2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(errors,[])
        self.assertEqual(len(results),4)
        self.assertEqual({row["connection_id"] for row in results},{1})
        self.assertEqual({row["payload"]["index"] for row in results},{0,1,2,3})

    def test_invalid_url_timeout_and_request_nan_do_not_send(self):
        for url in ("ftp://127.0.0.1/a","http://name:secret@127.0.0.1/a","http:///a",self.url+"/ok#fragment"):
            with self.assertRaises(ValueError):
                self.client.request_json(url,timeout_s=1,payload={})
        for timeout in (0,-1,float("inf"),float("nan"),True):
            with self.assertRaises(ValueError):
                self.request(timeout_s=timeout)
        with self.assertRaises(ValueError):
            self.request(payload={"value":float("nan")})
        self.assertEqual(self.server.requests,[])


if __name__=="__main__":
    unittest.main()
