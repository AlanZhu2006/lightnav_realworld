"""CPU-only tests; no model, service, robot SDK, or robot task is used."""
from __future__ import annotations

import base64
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from deployment.imagegoal.lightnav_transport import (
    LightNavObservationTransport, OFFICIAL_COMMIT, OFFICIAL_SOURCE_SHA256,
    OFFICIAL_VENDOR, load_official_image_encoder,
)


def image_bytes(size=(960, 540), seed=1):
    rgb = np.random.default_rng(seed).integers(0, 256, (size[1], size[0], 3), dtype=np.uint8)
    output = io.BytesIO()
    Image.fromarray(rgb).save(output, "JPEG", quality=90)
    return output.getvalue()


def frame(jpeg=None, fid="a", stamp=1000):
    return SimpleNamespace(frame_id=fid, jpeg=jpeg or image_bytes(), stamp=stamp,
                           metadata={"capture_timestamp_ns": stamp, "timestamp_source": "source_clock"},
                           provenance={"source_frame_id": "camera_optical", "nested": {"rgb": "original"}})


class TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Independent reference: execute the complete unmodified upstream
        # module and call its actual static method. The unused networking
        # dependency is a stub; no VlnClient instance is constructed.
        name = "_lightnav_official_client_transport_reference"
        spec = importlib.util.spec_from_file_location(name, OFFICIAL_VENDOR / "vln_client.py")
        cls.official = importlib.util.module_from_spec(spec)
        previous = sys.modules.get(name)
        sys.modules[name] = cls.official
        def restore_module():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
        cls.addClassCleanup(restore_module)
        previous_aiohttp = sys.modules.get("aiohttp")
        sys.modules["aiohttp"] = ModuleType("aiohttp")
        try:
            spec.loader.exec_module(cls.official)
        finally:
            # Preserve normal imports performed by upstream (notably cv2's
            # native-module bootstrap); restore only the network stub itself.
            if previous_aiohttp is None:
                sys.modules.pop("aiohttp", None)
            else:
                sys.modules["aiohttp"] = previous_aiohttp

    def wait_until(self, predicate, timeout_s=3.):
        deadline = time.monotonic()+timeout_s
        while time.monotonic()<deadline:
            if predicate():
                return
            time.sleep(.005)
        self.fail("bounded prefetch condition did not become true")

    def test_fixed_size_and_explicit_source_lineage(self):
        source = frame()
        wire, audit = LightNavObservationTransport().observations([source])
        encoded = base64.b64decode(wire[0]["rgb_jpeg_base64"], validate=True)
        with Image.open(io.BytesIO(encoded)) as image:
            self.assertEqual(image.size, (480, 270))
            self.assertEqual(image.format, "JPEG")
        self.assertEqual(wire[0]["frame_id"], source.frame_id)
        self.assertEqual(wire[0]["capture_timestamp_ns"], source.stamp)
        self.assertTrue(wire[0]["rgb_transport"]["lossy"])
        self.assertEqual(audit["frames"][0]["source_jpeg_sha256"], hashlib.sha256(source.jpeg).hexdigest())
        self.assertEqual(audit["frames"][0]["source_metadata"], source.metadata)
        self.assertEqual(audit["frames"][0]["source_provenance"], source.provenance)
        official = wire[0]["rgb_transport"]["official_source"]
        self.assertEqual(official["commit"], OFFICIAL_COMMIT)
        self.assertEqual(official["source_sha256"], OFFICIAL_SOURCE_SHA256)
        self.assertEqual(wire[0]["rgb_transport"]["jpeg"]["quality"], 80)

    def test_existing_server_parser_accepts_official_native_language_transport(self):
        from deployment.lightnav.server import parse_request
        source = frame()
        observations, _ = LightNavObservationTransport().observations([source])
        identity = {"session_id": "offline", "task_id": "transport-audit", "goal_id": "goal",
                    "goal_version": 1, "frame_id": source.frame_id}
        parsed, parsed_goal, echoed, instruction, mode, task_mode = parse_request({
            "identity": identity, "observations": observations,
            "conditioning_mode": "native_language", "task_mode": "objnav",
            "instruction": "Go to the chair and stop.",
        })
        self.assertEqual(echoed, identity)
        self.assertEqual(parsed[0]["image"].size, (480, 270))
        self.assertEqual(parsed[0]["capture_timestamp_ns"], source.stamp)
        self.assertIsNone(parsed_goal)
        self.assertEqual((instruction, mode, task_mode),
                         ("Go to the chair and stop.", "native_language", "objnav"))

    def test_original_bytes_metadata_and_goal_never_change(self):
        source, goal = frame(), image_bytes(seed=2)
        before = copy.deepcopy(source.__dict__)
        # Caller owns the complete goal; helper only accepts observations.
        payload = {"goal": {"rgb_jpeg_base64": base64.b64encode(goal).decode()}}
        goal_before = copy.deepcopy(payload["goal"])
        payload["observations"], audit = LightNavObservationTransport().observations([source])
        self.assertEqual(source.__dict__, before)
        self.assertEqual(payload["goal"], goal_before)
        audit["frames"][0]["source_provenance"]["nested"]["rgb"] = "mutated audit"
        self.assertEqual(source.provenance["nested"]["rgb"], "original")

    def test_overlap_is_identical_after_cache_hit_or_eviction(self):
        first, other = frame(), frame(image_bytes(seed=2), "b", 2000)
        transport = LightNavObservationTransport(cache_frames=1)
        initial, _ = transport.observations([first])
        repeated, hit = transport.observations([first])
        self.assertEqual(initial, repeated)
        self.assertEqual(hit["cache_hits"], 1)
        transport.observations([other])
        recomputed, miss = transport.observations([first])
        self.assertEqual(initial, recomputed)
        self.assertEqual(miss["cache_hits"], 0)

    def test_cache_does_not_hide_reused_id_changed_source(self):
        transport = LightNavObservationTransport()
        first, _ = transport.observations([frame()])
        changed, _ = transport.observations([frame(image_bytes(seed=2))])
        self.assertNotEqual(first[0]["rgb_transport"]["source_jpeg_sha256"],
                            changed[0]["rgb_transport"]["source_jpeg_sha256"])
        self.assertNotEqual(first[0]["rgb_jpeg_base64"], changed[0]["rgb_jpeg_base64"])

    def test_caller_cannot_mutate_cache_metadata(self):
        source = frame()
        transport = LightNavObservationTransport()
        first, _ = transport.observations([source])
        first[0]["rgb_transport"]["jpeg"]["quality"] = 1
        again, _ = transport.observations([source])
        self.assertEqual(again[0]["rgb_transport"]["jpeg"]["quality"], 80)

    def test_bad_jpeg_truncated_png_and_oversized_rejected(self):
        png = io.BytesIO()
        Image.new("RGB", (10, 10)).save(png, "PNG")
        jpeg = image_bytes()
        for bad in (b"not jpeg", jpeg[:len(jpeg)//2], png.getvalue(), b"x" * ((4 << 20) + 1)):
            with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                LightNavObservationTransport().observations([frame(bad)])

    def test_bounds(self):
        for count in (0, -1, 257, True, 1.5):
            with self.assertRaises(ValueError):
                LightNavObservationTransport(count)
        for count in (0, -1, 65, True, 1.5):
            with self.assertRaises(ValueError):
                LightNavObservationTransport(prefetch_queue_frames=count)
        for values in ([], [frame()] * 65):
            with self.assertRaises(ValueError):
                LightNavObservationTransport().observations(values)

    def test_prefetch_warms_frames_before_foreground_observations(self):
        transport = LightNavObservationTransport()
        self.addCleanup(transport.close)
        self.assertFalse(transport.prefetch_status()["worker_alive"])
        sources = [frame(image_bytes(size=(320,240),seed=index),str(index),index+1) for index in range(4)]
        for source in sources:
            self.assertTrue(transport.prefetch(source))
        self.wait_until(lambda: transport.prefetch_status()["completed"]==4)
        _, audit = transport.observations(sources)
        self.assertEqual(audit["cache_hits"],4)
        self.assertEqual(audit["prefetch"]["failed"],0)
        self.assertEqual(audit["prefetch"]["pending"],0)
        self.assertFalse(transport.prefetch(sources[0]))

    def test_busy_encoder_does_not_block_producer_and_queue_is_bounded(self):
        transport = LightNavObservationTransport(prefetch_queue_frames=2)
        self.addCleanup(transport.close)
        entered, release, producer_done = threading.Event(), threading.Event(), threading.Event()
        original = transport._encode_uncached
        def blocked(*args):
            entered.set()
            if not release.wait(3):
                raise AssertionError("test encoder release timed out")
            return original(*args)
        sources = [frame(image_bytes(size=(32,24),seed=index),str(index),index+1) for index in range(5)]
        accepted = []
        with patch.object(transport,"_encode_uncached",side_effect=blocked):
            try:
                self.assertTrue(transport.prefetch(sources[0]))
                self.assertTrue(entered.wait(1))
                def produce():
                    accepted.extend(transport.prefetch(source) for source in sources[1:])
                    producer_done.set()
                producer = threading.Thread(target=produce)
                producer.start()
                self.assertTrue(producer_done.wait(.5),"producer waited for the busy encoder")
                producer.join(timeout=.5)
                status = transport.prefetch_status()
                self.assertEqual(status["queued"],2)
                self.assertLessEqual(status["pending"],3)  # queue plus one active encoder
                self.assertEqual(sum(accepted),2)
                self.assertEqual(status["queue_full"],2)
            finally:
                release.set()
                transport.close()
        self.assertFalse(transport.prefetch_status()["worker_alive"])

    def test_bad_prefetch_is_audited_and_worker_processes_next_good_frame(self):
        transport = LightNavObservationTransport()
        self.addCleanup(transport.close)
        self.assertTrue(transport.prefetch(frame(b"bad jpeg","bad")))
        self.wait_until(lambda: transport.prefetch_status()["failed"]==1)
        good = frame(image_bytes(size=(32,24)),"good",2000)
        self.assertTrue(transport.prefetch(good))
        self.wait_until(lambda: transport.prefetch_status()["completed"]==1)
        _, audit = transport.observations([good])
        self.assertEqual(audit["cache_hits"],1)
        self.assertEqual(audit["prefetch"]["last_error"]["frame_id"],"bad")
        self.assertIn("ValueError",audit["prefetch"]["last_error"]["error"])

    def test_foreground_and_background_miss_encode_same_source_once(self):
        transport = LightNavObservationTransport()
        self.addCleanup(transport.close)
        entered, release = threading.Event(), threading.Event()
        original = transport._encode_uncached
        result = []
        def blocked(*args):
            entered.set()
            if not release.wait(3):
                raise AssertionError("test encoder release timed out")
            return original(*args)
        source = frame(image_bytes(size=(32,24)))
        with patch.object(transport,"_encode_uncached",side_effect=blocked) as encode:
            try:
                self.assertTrue(transport.prefetch(source))
                self.assertTrue(entered.wait(1))
                foreground = threading.Thread(target=lambda: result.append(transport.observations([source])))
                foreground.start()
                release.set()
                foreground.join(timeout=2)
                self.assertFalse(foreground.is_alive())
                self.assertEqual(encode.call_count,1)
                self.assertEqual(result[0][1]["cache_hits"],1)
            finally:
                release.set()
                transport.close()

    def test_close_rejects_enqueue_clears_queue_and_joins_owned_worker(self):
        transport = LightNavObservationTransport()
        self.assertTrue(transport.prefetch(frame(image_bytes(size=(32,24)))))
        self.wait_until(lambda: transport.prefetch_status()["completed"]==1)
        worker = transport._prefetch_worker
        transport.close()
        self.assertFalse(worker.is_alive())
        self.assertTrue(transport.prefetch_status()["closed"])
        self.assertEqual(transport.prefetch_status()["queued"],0)
        self.assertEqual(transport.prefetch_status()["pending"],0)
        self.assertFalse(transport.prefetch(frame(image_bytes(size=(32,24),seed=2))))
        transport.close()  # idempotent, does not create a replacement worker
        self.assertIs(transport._prefetch_worker,worker)

    def test_close_wait_is_bounded_when_encoder_is_already_running(self):
        transport = LightNavObservationTransport()
        self.addCleanup(transport.close)
        entered, release = threading.Event(), threading.Event()
        original = transport._encode_uncached
        def blocked(*args):
            entered.set()
            if not release.wait(3):
                raise AssertionError("test encoder release timed out")
            return original(*args)
        with patch.object(transport,"_encode_uncached",side_effect=blocked):
            try:
                self.assertTrue(transport.prefetch(frame(image_bytes(size=(32,24)))))
                self.assertTrue(entered.wait(1))
                start = time.monotonic()
                transport.close(timeout_s=.02)
                self.assertLess(time.monotonic()-start,.3)
                self.assertTrue(transport.prefetch_status()["closed"])
                self.assertFalse(transport.prefetch(frame(image_bytes(size=(32,24),seed=2))))
            finally:
                release.set()
                transport.close()
        self.assertFalse(transport.prefetch_status()["worker_alive"])

    def test_compressed_wire_bytes_match_actual_official_method(self):
        transport = LightNavObservationTransport()
        for size in ((1920, 1080), (480, 270), (17, 31), (960, 540)):
            with self.subTest(size=size):
                jpeg = image_bytes(size=size)
                original = self.official.CameraFrame(0, 0, size[0], size[1], 0, b"", jpeg)
                expected = self.official.VlnClient._encode_image(original)
                actual, _ = transport.observations([frame(jpeg)])
                self.assertEqual(actual[0]["rgb_jpeg_base64"], expected)

    def test_actual_recorded_camera_jpeg_matches_official_method_bytes(self):
        root = Path(__file__).resolve().parents[2]
        path = root / "data/realworld/imagegoal_goal_selection_20261017/browser_live_001/captured.jpg"
        if not path.exists():
            self.skipTest("Optional recorded camera fixture is not included on this host")
        jpeg = path.read_bytes()
        expected = self.official.VlnClient._encode_image(
            self.official.CameraFrame(0, 0, 0, 0, 0, b"", jpeg))
        wire, audit = LightNavObservationTransport().observations([frame(jpeg)])
        self.assertEqual(wire[0]["rgb_jpeg_base64"], expected)
        self.assertEqual(audit["frames"][0]["source_jpeg_sha256"], hashlib.sha256(jpeg).hexdigest())

    def test_ast_reuse_preserves_official_raw_rgb_and_padded_rows_branch(self):
        helper = load_official_image_encoder()
        width, height, padding = 29, 17, 7
        rgb = np.random.default_rng(3).integers(0, 256, (height, width*3+padding), dtype=np.uint8)
        arguments = dict(stamp_sec=0, stamp_nanosec=0, width=width, height=height,
                         step=width*3+padding, rgb=rgb.tobytes())
        expected = self.official.VlnClient._encode_image(self.official.CameraFrame(**arguments))
        self.assertEqual(helper.encode(helper.frame_type(**arguments)), expected)

    def test_nomad_frame_wire_stays_original(self):
        from deployment.imagegoal.runtime import Frame
        original = image_bytes()
        source = Frame("original-frame", original, {"capture_timestamp_ns": 12345}, 0,
                       {"timestamp_source": "original_sensor_clock"})
        before = source.wire()
        LightNavObservationTransport().observations([source])
        self.assertEqual(source.wire(), before)
        self.assertEqual(base64.b64decode(source.wire()["rgb_jpeg_base64"]), original)

    def test_source_and_manifest_are_hash_locked_before_ast_execution(self):
        original = (OFFICIAL_VENDOR / "vln_client.py").read_bytes()
        manifest = json.loads((OFFICIAL_VENDOR / "SOURCE.json").read_text())
        self.assertEqual(hashlib.sha256(original).hexdigest(), OFFICIAL_SOURCE_SHA256)
        with tempfile.TemporaryDirectory(prefix="lightnav-client-source-audit-") as temporary:
            source_path, manifest_path = Path(temporary)/"vln_client.py", Path(temporary)/"SOURCE.json"
            source_path.write_bytes(original+b"\n# modified source\n")
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "integrity mismatch"):
                load_official_image_encoder(source_path, manifest_path)
            source_path.write_bytes(original)
            manifest["constants"]["JPEG_QUALITY"] = 90
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "constants differ"):
                load_official_image_encoder(source_path, manifest_path)

    def test_helper_has_no_torch_or_robot_import(self):
        helper = Path(__file__).with_name("lightnav_transport.py").read_text()
        self.assertNotIn("import torch", helper)
        self.assertNotIn("from torch", helper)
        self.assertNotIn("unitree_sdk", helper)


if __name__ == "__main__":
    unittest.main()
