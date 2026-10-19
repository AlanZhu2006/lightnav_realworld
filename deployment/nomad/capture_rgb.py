"""Capture a bounded, stationary RGB diagnostic without ROS or robot APIs.

V4L2/OpenCV provides a host dequeue timestamp, not a verified sensor exposure
timestamp. These records are suitable for model IO/shadow validation only.
They must not arm the motion stack or be presented as capture-aligned odom.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import struct
import time


def inspect_video_devices() -> list[dict]:
    """Query format metadata only; no stream is started."""
    devices = []
    for node in sorted(Path('/dev').glob('video*')):
        entry = {'device': str(node), 'formats': []}
        descriptor = None
        try:
            descriptor = os.open(node, os.O_RDONLY | os.O_NONBLOCK)
            for index in range(64):
                data = bytearray(64)
                struct.pack_into('II', data, 0, index, 1)  # VIDEO_CAPTURE
                try:
                    fcntl.ioctl(descriptor, 0xC0405602, data)  # VIDIOC_ENUM_FMT
                except OSError as exc:
                    if exc.errno == errno.EINVAL:
                        break
                    raise
                entry['formats'].append({
                    'fourcc': bytes(data[44:48]).decode('ascii', errors='replace'),
                    'description': bytes(data[12:44]).split(b'\0', 1)[0].decode(errors='replace'),
                })
        except OSError as exc:
            entry['error'] = str(exc)
        finally:
            if descriptor is not None:
                os.close(descriptor)
        devices.append(entry)
    return devices


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list-devices', action='store_true')
    parser.add_argument('--device')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--frames', type=int, default=12)
    parser.add_argument('--sample-hz', type=float, default=4.0)
    parser.add_argument('--camera-fps', type=float, default=15.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--warmup-frames', type=int, default=20)
    args = parser.parse_args(argv)
    if args.list_devices:
        print(json.dumps(inspect_video_devices(), indent=2))
        return
    if not args.device or args.output is None:
        parser.error('--device and --output are required for capture')
    if (args.frames < 4 or args.width <= 0 or args.height <= 0 or args.warmup_frames < 0
            or not math.isfinite(args.sample_hz) or args.sample_hz <= 0
            or not math.isfinite(args.camera_fps) or args.camera_fps < args.sample_hz):
        parser.error('invalid capture size, frame count or sampling rate')
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error('output must be a new run directory')
    import cv2

    output.mkdir(parents=True)
    (output / 'rgb').mkdir()
    manifest = {
        'schema_version': 'nomad.rgb_capture.v1',
        'purpose': 'stationary_rgb_io_diagnostic',
        'not_for_motion': True,
        'timestamp_source': 'host_wall_clock_after_v4l2_dequeue_not_sensor_exposure',
        'device': args.device,
        'sample_hz_requested': args.sample_hz,
        'camera_fps_requested': args.camera_fps,
        'frames': [],
    }
    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError(f'cannot open {args.device}')
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        camera.set(cv2.CAP_PROP_FPS, args.camera_fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        manifest['camera_properties'] = {
            'width': camera.get(cv2.CAP_PROP_FRAME_WIDTH),
            'height': camera.get(cv2.CAP_PROP_FRAME_HEIGHT),
            'fps_reported': camera.get(cv2.CAP_PROP_FPS),
        }
        for _ in range(args.warmup_frames):
            ok, _ = camera.read()
            if not ok:
                raise RuntimeError('camera failed during warmup')
        next_sample = time.monotonic()
        deadline = next_sample + max(15.0, args.frames / args.sample_hz * 4)
        while len(manifest['frames']) < args.frames:
            ok, image_bgr = camera.read()
            now_mono = time.monotonic()
            capture_ns = time.time_ns()
            if not ok:
                raise RuntimeError('camera frame read failed')
            if now_mono > deadline:
                raise TimeoutError('capture exceeded its bounded duration')
            if now_mono < next_sample:
                continue
            index = len(manifest['frames']) + 1
            relative = f'rgb/{index:06d}.png'
            if not cv2.imwrite(str(output / relative), image_bgr):
                raise RuntimeError('failed to save camera frame')
            manifest['frames'].append({
                'frame_id': f'rgb-{index:06d}',
                'capture_timestamp_ns': capture_ns,
                'host_monotonic_ns': int(now_mono * 1e9),
                'path': relative,
                'shape_hwc': list(image_bgr.shape),
                'stored_encoding': 'PNG_RGB_decoded_by_PIL',
            })
            next_sample += 1.0 / args.sample_hz
            if next_sample < now_mono:
                next_sample = now_mono + 1.0 / args.sample_hz
        manifest['status'] = 'complete'
        # The same-view goal is only an IO diagnostic, never a navigation trial.
        manifest['diagnostic_goal_path'] = manifest['frames'][-1]['path']
    except Exception as exc:
        manifest['status'] = 'failed'
        manifest['error'] = str(exc)
        raise
    finally:
        camera.release()
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'manifest': str(output / 'manifest.json'),
                      'frames': len(manifest['frames']), 'not_for_motion': True}))


if __name__ == '__main__':
    main()
