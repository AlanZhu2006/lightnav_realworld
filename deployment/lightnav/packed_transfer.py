"""Lossless, integrity-checked block transport for the one official model shard.

Packing is a transfer optimization only. The destination is reconstructed as
the original safetensors file and must match the pinned official SHA256 before
it can be used. No quantization, checkpoint edit, or second full compressed
copy on the receiving workstation is involved.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import ctypes
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import urllib.request

from .download_model import digest
from .policy import MODEL_SHA256

MODEL_NAME = "model-00001-of-00001.safetensors"
MODEL_SIZE = 9695744034


class Zstd:
    def __init__(self):
        library = ctypes.util.find_library("zstd")
        if not library:
            raise RuntimeError("System libzstd is required; no package is installed by this tool")
        self.lib = ctypes.CDLL(library)
        self.lib.ZSTD_compressBound.argtypes = [ctypes.c_size_t]
        self.lib.ZSTD_compressBound.restype = ctypes.c_size_t
        for name in ("ZSTD_compress", "ZSTD_decompress"):
            method = getattr(self.lib, name)
            method.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t] + ([ctypes.c_int] if name == "ZSTD_compress" else [])
            method.restype = ctypes.c_size_t
        self.lib.ZSTD_isError.argtypes = [ctypes.c_size_t]
        self.lib.ZSTD_isError.restype = ctypes.c_uint
        self.lib.ZSTD_getErrorName.argtypes = [ctypes.c_size_t]
        self.lib.ZSTD_getErrorName.restype = ctypes.c_char_p

    def checked(self, size):
        if self.lib.ZSTD_isError(size):
            raise ValueError(self.lib.ZSTD_getErrorName(size).decode())
        return size

    def encode(self, original):
        shuffled = original[::2] + original[1::2]
        output = ctypes.create_string_buffer(self.lib.ZSTD_compressBound(len(shuffled)))
        size = self.checked(self.lib.ZSTD_compress(output, len(output), shuffled, len(shuffled), 1))
        return output.raw[:size]

    def decode(self, packed, size):
        output = ctypes.create_string_buffer(size)
        actual = self.checked(self.lib.ZSTD_decompress(output, size, packed, len(packed)))
        if actual != size:
            raise ValueError("Wrong decompressed block size")
        shuffled = output.raw
        split = (size + 1) // 2
        original = bytearray(size)
        original[::2], original[1::2] = shuffled[:split], shuffled[split:]
        return bytes(original)


def atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, separators=(",", ":")) + "\n")
    temporary.replace(path)


def prepare(source, output):
    if source.name != MODEL_NAME or source.stat().st_size != MODEL_SIZE:
        raise ValueError("Only the pinned official public LightNav model shard is accepted")
    output.mkdir(parents=True, exist_ok=False)
    codec = Zstd()
    original_hash, packed_hash = hashlib.sha256(), hashlib.sha256()
    records, original_offset, packed_offset = [], 0, 0
    partial = output / "model.pack.partial"
    started = time.monotonic()
    with source.open("rb") as stream, partial.open("xb") as destination:
        while data := stream.read(2 << 20):
            packed = codec.encode(data)
            original_hash.update(data); packed_hash.update(packed)
            destination.write(packed)
            records.append({"original_offset": original_offset, "original_size": len(data),
                            "original_sha256": hashlib.sha256(data).hexdigest(),
                            "packed_offset": packed_offset, "packed_size": len(packed),
                            "packed_sha256": hashlib.sha256(packed).hexdigest()})
            original_offset += len(data); packed_offset += len(packed)
            if len(records) % 256 == 0:
                print(json.dumps({"packed_original_bytes": original_offset, "packed_bytes": packed_offset}), flush=True)
    if original_hash.hexdigest() != MODEL_SHA256 or original_offset != MODEL_SIZE:
        raise ValueError("Source file does not match the official model SHA256")
    partial.rename(output / "model.pack")
    index = {"schema_version": "lightnav.public_model_transport.v1", "codec": "shuffle_bytes_2_then_zstd_level_1",
             "original_filename": MODEL_NAME, "original_size": MODEL_SIZE, "original_sha256": MODEL_SHA256,
             "packed_size": packed_offset, "packed_sha256": packed_hash.hexdigest(), "blocks": records,
             "prepare_elapsed_s": time.monotonic() - started}
    atomic_json(output / "index.json", index)
    print(json.dumps({k: v for k, v in index.items() if k != "blocks"}), flush=True)


def read_range(url, start, size, total_size):
    body, failures = bytearray(), 0
    while len(body) < size:
        before, started = len(body), time.monotonic()
        offset, end = start + before, start + size - 1
        try:
            request = urllib.request.Request(url, headers={"Range": f"bytes={offset}-{end}"})
            with urllib.request.urlopen(request, timeout=10) as response:
                if response.status != 206 or response.headers.get("Content-Range") != f"bytes {offset}-{end}/{total_size}":
                    raise ValueError("Relay returned a wrong HTTP byte range")
                while len(body) < size:
                    data = response.read1(min(64 << 10, size - len(body)))
                    if not data:
                        raise OSError("Truncated packed block")
                    body.extend(data)
                    if time.monotonic() - started > 20:
                        break
        except Exception:
            if len(body) == before:
                failures += 1
                if failures >= 5:
                    raise
                time.sleep(min(2 ** failures, 10))
                continue
        failures = 0
    return bytes(body)


def download(index_path, url, output, workers):
    raw_index = index_path.read_bytes()
    index = json.loads(raw_index)
    if (index.get("schema_version") != "lightnav.public_model_transport.v1" or
            index.get("codec") != "shuffle_bytes_2_then_zstd_level_1" or
            index.get("original_filename") != MODEL_NAME or index.get("original_size") != MODEL_SIZE or
            index.get("original_sha256") != MODEL_SHA256):
        raise ValueError("Index is not the pinned official model transport")
    if output.name != MODEL_NAME:
        raise ValueError("Destination filename must match the pinned official shard")
    if output.exists():
        if digest(output) != MODEL_SHA256:
            raise ValueError("Existing final file has the wrong SHA256")
        print("Official model already verified", flush=True); return
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.name + ".partial")
    progress = output.with_name(output.name + ".packed-progress.json")
    source_progress = partial.with_name(partial.name + ".progress.json")
    intervals = []
    if source_progress.exists() and partial.exists():
        previous = json.loads(source_progress.read_text())
        if previous.get("size") == MODEL_SIZE:
            intervals = [(MODEL_SIZE * i // previous["parts"], position)
                         for i, position in enumerate(previous["positions"])]
    index_hash = hashlib.sha256(raw_index).hexdigest()
    candidates = set()
    if progress.exists() and partial.exists():
        previous = json.loads(progress.read_text())
        if previous.get("index_sha256") == index_hash:
            candidates = set(previous.get("completed_blocks", []))
    records = index["blocks"]
    # Validate every block descriptor before allocating or serving arbitrary ranges.
    raw_end, packed_end = 0, 0
    for row in records:
        if (row["original_offset"] != raw_end or row["packed_offset"] != packed_end or
                not 0 < row["original_size"] <= 2 << 20 or not 0 < row["packed_size"] <= 3 << 20):
            raise ValueError("Noncontiguous or oversized packed block index")
        raw_end += row["original_size"]; packed_end += row["packed_size"]
    if raw_end != MODEL_SIZE or packed_end != index["packed_size"]:
        raise ValueError("Packed index does not cover the complete official model")
    descriptor = os.open(partial, os.O_CREAT | os.O_RDWR, 0o600)
    codec, lock = Zstd(), threading.Lock()
    completed, wire_bytes = set(), 0
    started = time.monotonic()
    try:
        os.ftruncate(descriptor, MODEL_SIZE)
        for i, row in enumerate(records):
            start, end = row["original_offset"], row["original_offset"] + row["original_size"]
            covered = i in candidates or any(lo <= start and end <= hi for lo, hi in intervals)
            if covered:
                existing = os.pread(descriptor, row["original_size"], start)
                if hashlib.sha256(existing).hexdigest() == row["original_sha256"]:
                    completed.add(i)
        skipped = len(completed)
        print(json.dumps({"verified_existing_blocks": skipped, "total_blocks": len(records)}), flush=True)

        def transfer(i):
            nonlocal wire_bytes
            row = records[i]
            packed = read_range(url, row["packed_offset"], row["packed_size"], index["packed_size"])
            if hashlib.sha256(packed).hexdigest() != row["packed_sha256"]:
                raise ValueError("Compressed block SHA256 mismatch")
            data = codec.decode(packed, row["original_size"])
            if hashlib.sha256(data).hexdigest() != row["original_sha256"]:
                raise ValueError("Restored original block SHA256 mismatch")
            if os.pwrite(descriptor, data, row["original_offset"]) != len(data):
                raise OSError("Short model-block write")
            with lock:
                completed.add(i); wire_bytes += len(packed)
                atomic_json(progress, {"index_sha256": index_hash, "original_sha256": MODEL_SHA256,
                                       "completed_blocks": sorted(completed)})
                if len(completed) % 64 == 0:
                    print(json.dumps({"completed_blocks": len(completed), "total_blocks": len(records),
                                      "wire_bytes_this_run": wire_bytes}), flush=True)

        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
            list(pool.map(transfer, [i for i in range(len(records)) if i not in completed]))
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if digest(partial) != MODEL_SHA256:
        raise ValueError("Final restored safetensors SHA256 does not match the official model")
    partial.rename(output)
    receipt = {"official_model_sha256": MODEL_SHA256, "official_model_bytes": MODEL_SIZE,
               "verified_original_blocks": len(completed), "reused_existing_blocks": skipped,
               "wire_bytes_this_run": wire_bytes, "elapsed_s": time.monotonic() - started,
               "lossless_only": True, "model_changed": False, "second_compressed_copy_on_receiver": False}
    atomic_json(output.with_name(output.name + ".transport-receipt.json"), receipt)
    print(json.dumps(receipt), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    pack = commands.add_parser("prepare")
    pack.add_argument("--source", type=Path, required=True)
    pack.add_argument("--output", type=Path, required=True)
    fetch = commands.add_parser("download")
    fetch.add_argument("--index", type=Path, required=True)
    fetch.add_argument("--url", required=True)
    fetch.add_argument("--output", type=Path, required=True)
    fetch.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.source, args.output)
    else:
        if not 1 <= args.workers <= 32:
            parser.error("workers must be 1..32")
        download(args.index, args.url, args.output, args.workers)


if __name__ == "__main__":
    main()
