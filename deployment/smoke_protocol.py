"""Exercise the actual robot WebSocket client without ROS or a robot connection."""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "robot_deploy/src/vln_client"))

from vln_client.vln_client import VlnClient  # noqa: E402


async def infer_image(server_url: str, image: Path, instruction: str, timeout: float = 30) -> dict:
    import cv2

    bgr = cv2.imread(str(image))
    if bgr is None:
        raise ValueError(f"Cannot decode image: {image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    stamp = time.time_ns()
    client = VlnClient(logger=logging.getLogger("protocol-smoke"), server_url=server_url)
    try:
        episode = client.start(instruction)
        deadline = time.monotonic() + timeout
        offered = False
        while time.monotonic() < deadline:
            state = client.snapshot()
            if state.error:
                raise RuntimeError(state.error)
            if not offered:
                offered = client.offer_frame(
                    stamp_sec=stamp // 10**9,
                    stamp_nanosec=stamp % 10**9,
                    width=rgb.shape[1],
                    height=rgb.shape[0],
                    step=rgb.shape[1] * 3,
                    rgb=rgb.tobytes(),
                )
            results = client.take_results()
            if results:
                result = results[0]
                if result.episode != episode or result.sequence != 1:
                    raise RuntimeError("Client episode or sequence mismatch")
                if len(result.waypoints) != 10 or not all(
                    len(point) == 3 and all(math.isfinite(v) for v in point)
                    for point in result.waypoints
                ):
                    raise RuntimeError("Expected ten finite [forward, left, yaw] waypoints")
                for point in (result.apos_px, result.opos_px):
                    if point is not None and not (0 <= point[0] < 480 and 0 <= point[1] < 270):
                        raise RuntimeError("Pointing pixels are outside the transmitted image")
                if result.frame.stamp_sec * 10**9 + result.frame.stamp_nanosec != stamp:
                    raise RuntimeError("Image capture stamp was not preserved")
                return dict(
                    server_url=server_url,
                    image=str(image),
                    image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                    instruction=instruction,
                    episode=episode,
                    sequence=result.sequence,
                    capture_stamp_ns=stamp,
                    trace_id=client.snapshot().trace_id,
                    waypoints=result.waypoints,
                    stop=result.stop,
                    visible=result.visible,
                    apos_state=result.apos_state,
                    opos_state=result.opos_state,
                    apos_px=result.apos_px,
                    opos_px=result.opos_px,
                    latency_ms=result.latency_ms,
                    timings=result.timings,
                    scope="Image -> robot VlnClient -> model server -> parsed waypoints; no ROS or robot commands",
                )
            await asyncio.sleep(0.01)
        raise TimeoutError(f"No inference response after {timeout}s")
    finally:
        await asyncio.to_thread(client.close)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", required=True)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = asyncio.run(infer_image(args.server, args.image, args.instruction, args.timeout))
        value = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(value)
        print(value, end="")
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print(f"Protocol smoke failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
