"""Temporarily relay one integrity-checked public model file, with HTTP ranges.

This serves exactly one explicit file, never a directory or robot endpoints.
Use only for public upstream model assets, and keep the finite duration bound.
"""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading

from .download_model import digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18904)
    parser.add_argument("--duration-s", type=float, required=True)
    args = parser.parse_args()
    if not 1 <= args.duration_s <= 14400:
        parser.error("duration must be bounded to 1..14400 seconds")
    path = args.file.resolve(strict=True)
    if not path.is_file() or digest(path) != args.expected_sha256:
        raise ValueError("Asset fails the explicit SHA256 check")
    size = path.stat().st_size

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(60)

        def do_GET(self):
            if self.path == "/healthz":
                data = json.dumps({"file": path.name, "size": size, "sha256": args.expected_sha256}).encode()
                self.send_response(200); self.send_header("Content-Length", str(len(data)))
                self.end_headers(); self.wfile.write(data); return
            if self.path != "/" + path.name:
                self.send_error(404); return
            start, end = 0, size - 1
            header = self.headers.get("Range")
            if header:
                match = re.fullmatch(r"bytes=(\d+)-(\d*)", header)
                if not match:
                    self.send_error(416); return
                start = int(match[1]); end = int(match[2]) if match[2] else end
                if not 0 <= start <= end < size:
                    self.send_error(416); return
            self.send_response(206 if header else 200)
            self.send_header("Content-Length", str(end - start + 1))
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("ETag", '"' + args.expected_sha256 + '"')
            if header:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            try:
                with path.open("rb") as stream:
                    stream.seek(start); left = end - start + 1
                    while left:
                        data = stream.read(min(1 << 20, left))
                        if not data:
                            raise OSError("Asset changed or truncated during transfer")
                        self.wfile.write(data); left -= len(data)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    timer = threading.Timer(args.duration_s, server.shutdown)
    timer.daemon = True
    timer.start()
    print(json.dumps({"host": args.host, "port": args.port, "file": path.name,
                      "sha256": args.expected_sha256, "duration_s": args.duration_s}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel(); server.server_close()


if __name__ == "__main__":
    main()
