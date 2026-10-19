"""Download one pinned, unmodified official checkpoint with integrity checks.

Uses HTTP byte ranges for the large shard, writes directly to its final-volume
partial file, and never creates a second Hugging Face weight cache.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import urllib.parse
import urllib.request
import urllib.error

MODEL = "LightOriginsHQ/LightNav-0"
REVISION = "826dc5fbfa37afa8293d2e336d329b6ffc0bfb64"


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for data in iter(lambda: stream.read(8 << 20), b""):
            h.update(data)
    return h.hexdigest()


def fetch_file(item, root, workers, shard_url=None, existing_prefix_bytes=0):
    name, size = item["rfilename"], item["size"]
    target = root / name
    expected = (item.get("lfs") or {}).get("sha256")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if target.stat().st_size == size and (not expected or digest(target) == expected):
            return {"path": name, "size": size, "sha256": digest(target)}
        raise ValueError(f"Existing file fails integrity check: {target}")
    partial = target.with_name(target.name + ".partial")
    url = item.get("download_url") or f"https://huggingface.co/{MODEL}/resolve/{REVISION}/{urllib.parse.quote(name)}"
    if shard_url and name.endswith(".safetensors"):
        url = shard_url
    if size < 64 << 20:
        with urllib.request.urlopen(url, timeout=120) as response, partial.open("wb") as out:
            while data := response.read(1 << 20):
                out.write(data)
    else:
        parts = max(1, workers)
        # Resolve a signed CDN URL once. Re-requesting the Hub redirect for
        # every short range needlessly consumes Hub rate limits and adds RTTs.
        transport_lock = threading.Lock()

        def resolve_transport():
            with urllib.request.urlopen(urllib.request.Request(url, headers={"Range": "bytes=0-0"}), timeout=30) as response:
                if response.status != 206 or response.headers.get("Content-Range") != f"bytes 0-0/{size}":
                    raise RuntimeError("Transport does not support exact byte ranges")
                response.read(1)
                return response.geturl()

        transport = [resolve_transport()]
        state_path = partial.with_name(partial.name + ".progress.json")
        starts = [size * i // parts for i in range(parts)]
        ends = [size * (i + 1) // parts - 1 for i in range(parts)]
        positions = [max(start, min(existing_prefix_bytes, end + 1)) for start, end in zip(starts, ends)]
        if state_path.exists() and partial.exists():
            state = json.loads(state_path.read_text())
            if (state.get("revision") == REVISION and state.get("size") == size
                    and state.get("parts") == parts):
                candidate = state.get("positions")
                if (isinstance(candidate, list) and len(candidate) == parts and
                        all(type(p) is int and start <= p <= end + 1
                            for p, start, end in zip(candidate, starts, ends))):
                    positions = candidate
        fd = os.open(partial, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.ftruncate(fd, size)
            state_lock = threading.Lock()

            def save_position(i, offset):
                with state_lock:
                    positions[i] = offset
                    state = {"revision": REVISION, "size": size, "parts": parts,
                             "positions": positions.copy()}
                    temporary = state_path.with_name(state_path.name + ".tmp")
                    temporary.write_text(json.dumps(state) + "\n")
                    temporary.replace(state_path)

            save_position(0, positions[0])
            resumed = sum(pos - start for pos, start in zip(positions, starts))
            print(f"{name}: resume {resumed}/{size} bytes across {parts} ranges", flush=True)

            def chunk(i):
                end = ends[i]
                offset = positions[i]
                if offset > end:
                    return
                saved = offset
                while offset <= end:
                    # Long WAN streams can stall after hundreds of MiB. Keep
                    # each request short, bounded, and independently retryable.
                    request_end = min(offset + (8 << 20) - 1, end)
                    for attempt in range(5):
                        try:
                            attempt_start = offset
                            started = time.monotonic()
                            active_url = transport[0]
                            request = urllib.request.Request(active_url, headers={"Range": f"bytes={offset}-{request_end}"})
                            with urllib.request.urlopen(request, timeout=10) as response:
                                expected_range = f"bytes {offset}-{request_end}/{size}"
                                if response.status != 206 or response.headers.get("Content-Range") != expected_range:
                                    raise RuntimeError("Server did not honor the exact requested byte range")
                                while offset <= request_end:
                                    # read1 returns currently available TLS data;
                                    # read(1MiB) can wait minutes on a trickling
                                    # connection and defeat a whole-job deadline.
                                    data = response.read1(min(64 << 10, request_end - offset + 1))
                                    if not data:
                                        raise OSError("Truncated HTTP range")
                                    written = os.pwrite(fd, data, offset)
                                    if written != len(data):
                                        raise OSError("Short write")
                                    offset += written
                                    if offset - saved >= 8 << 20 or offset > request_end:
                                        save_position(i, offset); saved = offset
                                    if time.monotonic() - started > 20 and offset <= request_end:
                                        raise TimeoutError("HTTP range exceeded its wall-time bound")
                            break
                        except Exception as exc:
                            save_position(i, offset)
                            if isinstance(exc, TimeoutError) and offset > attempt_start:
                                # A productive, budget-limited request is a
                                # completed scheduling slice, not a retry
                                # failure. Continue from its exact byte offset.
                                break
                            if isinstance(exc, urllib.error.HTTPError) and exc.code in (401, 403):
                                with transport_lock:
                                    if transport[0] == active_url:
                                        transport[0] = resolve_transport()
                            if attempt == 4:
                                raise
                            time.sleep(min(2 ** attempt, 10))
                    else:
                        raise AssertionError("unreachable")
                print(f"{name}: range {i + 1}/{parts} complete", flush=True)

            with concurrent.futures.ThreadPoolExecutor(max_workers=parts) as pool:
                list(pool.map(chunk, range(parts)))
        finally:
            os.close(fd)
    actual = digest(partial)
    if partial.stat().st_size != size or (expected and expected != actual):
        raise ValueError(f"Downloaded file fails integrity check: {name}")
    partial.replace(target)
    progress = partial.with_name(partial.name + ".progress.json")
    if progress.exists():
        progress.unlink()
    print(f"Verified {name} ({size} bytes)", flush=True)
    return {"path": name, "size": size, "sha256": actual}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--shard-url", help="Optional transport relay for the exact official model bytes; official metadata/SHA remain mandatory")
    args = parser.parse_args()
    if not 1 <= args.workers <= 32:
        parser.error("workers must be 1..32")
    args.output.mkdir(parents=True, exist_ok=True)
    api = f"https://huggingface.co/api/models/{MODEL}/revision/{REVISION}?blobs=true"
    with urllib.request.urlopen(api, timeout=60) as response:
        info = json.load(response)
    if info["sha"] != REVISION:
        raise ValueError("Model revision changed")
    files = [item for item in info["siblings"] if item["rfilename"] not in ("wechat_group.png", ".gitattributes")]
    rows = [fetch_file(item, args.output, args.workers, args.shard_url) for item in files]
    manifest = {"repository": MODEL, "revision": REVISION, "files": rows}
    (args.output / "SOURCE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"model": str(args.output), "revision": REVISION, "verified_files": len(rows)}))


if __name__ == "__main__":
    main()
