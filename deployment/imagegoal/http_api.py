"""Loopback operator API, including explicit native executor Start/Stop."""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import shutil
import tempfile
from urllib.parse import parse_qs, unquote, urlsplit
import zipfile

from .runtime import ImageGoalRuntime, json_bytes


class RuntimeHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, runtime: ImageGoalRuntime, static_dir: Path | None = None):
        self.runtime = runtime
        self.static_dir = static_dir.resolve(strict=True) if static_dir else None
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server: RuntimeHTTPServer

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def _bytes(self, status: int, body: bytes, mime: str, headers: dict[str,str] | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for name,value in (headers or {}).items():
            self.send_header(name,value)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value) -> None:
        self._bytes(status,json_bytes(value),"application/json; charset=utf-8")

    def _route(self) -> str:
        path = urlsplit(self.path).path
        return path[4:] if path.startswith("/api/") else path

    def do_GET(self):
        runtime = self.server.runtime
        path = self._route()
        try:
            if path in {"/status","/healthz"}:
                self._json(200,runtime.status())
            elif path == "/native/status":
                status = runtime.status()
                self._json(200,{"native_executor":status["native_executor"],"native_execution":status["native_execution"]})
            elif path == "/events":
                after = int(parse_qs(urlsplit(self.path).query).get("after",["0"])[0])
                with runtime.lock:
                    self._json(200,{"events":[event for event in runtime.events if event["sequence"]>after],
                                    "latest_event_sequence":runtime.event_sequence})
            elif path in {"/frame/latest.jpg","/goal.jpg"}:
                selection_id = None
                with runtime.lock:
                    if path=="/goal.jpg":
                        selection_id = runtime.goal_selection["selection_id"]
                        requested = parse_qs(urlsplit(self.path).query,keep_blank_values=True).get("selection_id")
                        if requested is not None and (len(requested)!=1 or requested[0]!=selection_id):
                            raise ValueError("selection_id mismatch: refresh current goal selection")
                        jpeg = runtime.goal_jpeg if runtime.goal_selection["ready"] else None
                    else:
                        jpeg = runtime.frames[-1].jpeg if runtime.frames else None
                if jpeg is None:
                    self._json(404,{"error":"no frozen goal available" if path=="/goal.jpg" else "no RGB available"})
                else:
                    headers = {"X-ImageGoal-Selection-Id":selection_id} if selection_id else None
                    self._bytes(200,jpeg,"image/jpeg",headers)
            elif path == "/run.zip":
                self._download()
            else:
                self._static(path)
        except (ValueError,TypeError) as exc:
            self._json(400,{"error":str(exc)})
        except (BrokenPipeError,ConnectionResetError):
            pass

    def _download(self):
        runtime = self.server.runtime
        with tempfile.TemporaryFile() as buffer:
            with zipfile.ZipFile(buffer,"w",zipfile.ZIP_DEFLATED) as archive:
                for path in sorted(runtime.output.rglob("*")):
                    if path.is_file() and not path.is_symlink() and not path.name.endswith(".tmp"):
                        archive.write(path,path.relative_to(runtime.output))
            length = buffer.tell()
            buffer.seek(0)
            self.send_response(200)
            self.send_header("Content-Type","application/zip")
            self.send_header("Content-Length",str(length))
            self.send_header("Content-Disposition",f'attachment; filename="{runtime.session_id}.zip"')
            self.send_header("Cache-Control","no-store")
            self.end_headers()
            shutil.copyfileobj(buffer,self.wfile,1024*1024)

    def _static(self, path: str) -> None:
        root = self.server.static_dir
        if root is None:
            self._json(404,{"error":"unknown endpoint; static UI directory not configured"})
            return
        relative = unquote(path).lstrip("/") or "index.html"
        target = (root/relative).resolve()
        if not target.is_relative_to(root) or not target.is_file():
            self._json(404,{"error":"unknown static resource"})
            return
        self._bytes(200,target.read_bytes(),mimetypes.guess_type(target.name)[0] or "application/octet-stream")

    def do_POST(self):
        runtime = self.server.runtime
        path = self._route()
        try:
            if self.headers.get_content_type() != "application/json":
                self._json(415,{"error":"Content-Type must be application/json"})
                return
            if self.headers.get("Transfer-Encoding"):
                raise ValueError("Transfer-Encoding is not supported")
            length = int(self.headers.get("Content-Length","0"))
            if not 0 < length <= 24*1024*1024:
                self._json(413,{"error":"body must be between 1 and 24 MiB"})
                return
            body = self.rfile.read(length)
            if len(body)!=length:
                raise ValueError("incomplete body")
            value = json.loads(body,parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON")))
            if not isinstance(value,dict):
                raise ValueError("request must be a JSON object")
            if path == "/native/start":
                result = runtime.start_native(value,runtime_url=f"http://127.0.0.1:{self.server.server_address[1]}")
            elif path == "/native/stop":
                result = runtime.stop_native(value)
            elif path == "/tasks":
                result = runtime.create_task(value)
            elif path == "/goal/capture":
                result = runtime.select_goal(value,source="camera")
            elif path == "/goal/import":
                result = runtime.select_goal(value,source="import")
            elif path == "/goal/clear":
                result = runtime.clear_goal(value)
            elif path == "/task/goal":
                result = runtime.replace_goal(value)
            elif path.startswith("/task/"):
                result = runtime.task_action(path.rsplit("/",1)[1],value)
            elif path == "/inputs/rgb":
                result = runtime.ingest_rgb(value)
            elif path == "/inputs/odom":
                result = runtime.ingest_odom(value)
            elif path == "/inputs/depth":
                result = runtime.ingest_depth(value)
            elif path == "/simulation/fault":
                result = runtime.set_fault(value.get("name"),value.get("value"))
            else:
                self._json(404,{"error":"unknown endpoint"})
                return
            self._json(200,result)
        except (ValueError,TypeError,KeyError) as exc:
            self._json(400,{"error":str(exc)})
        except TimeoutError:
            self._json(408,{"error":"body read timed out"})
        except (BrokenPipeError,ConnectionResetError):
            pass
        except Exception as exc:
            with runtime.lock:
                runtime._event("api_error",path=path,detail=f"{type(exc).__name__}: {exc}")
            self._json(500,{"error":"runtime API error; inspect events"})

    def log_message(self, format, *args):
        # High-frequency state polling must not flood an operator terminal.
        return
