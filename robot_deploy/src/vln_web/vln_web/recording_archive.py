"""Lossless ROS archive and verified per-episode inventory."""

import hashlib
import json
import multiprocessing
import queue
from pathlib import Path

TOPICS = [
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    "/camera/camera/color/metadata",
    "/camera/camera/depth/metadata",
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/aligned_depth_to_color/camera_info",
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/depth/camera_info",
    "/camera/camera/extrinsics/depth_to_color",
    "/odom",
    "/cmd_vel",
    "/mpc/cmd_vel",
    "/web/cmd_vel",
    "/mpc/status",
    "/vln/response",
    "/vln/trace",
    "/vln/status",
    "/vln/instruction",
    "/vln/task_mode",
    "/vln/path_body",
    "/vln/path_odom",
    "/vln/mode",
    "/vln/server_url_status",
    "/mpc/reference",
    "/mpc/prediction",
    "/mpc/enable",
    "/parameter_events",
    "/control/source",
    "/robot/events",
    "/diagnostics",
    "/tf",
    "/tf_static",
    "/navdp/go2/telemetry/low_state",
    "/navdp/go2/telemetry/sport_state",
    "/rosout",
]


def atomic_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    tmp.replace(path)


def manifest(directory):
    directory = Path(directory)
    entries = []
    for path in sorted(directory.rglob("*")):
        if not path.is_file() or path.name in {"manifest.json", "manifest.json.tmp"}:
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        entries.append(
            dict(
                path=path.relative_to(directory).as_posix(),
                size=path.stat().st_size,
                sha256=digest.hexdigest(),
            )
        )
    atomic_json(directory / "manifest.json", dict(version=1, files=entries))


def _archive_worker(directory, jobs, results):
    import rosbag2_py

    directory = Path(directory)
    writer = None
    counts = {}
    first_ns = last_ns = None
    error = None
    try:
        config = directory / "mcap_storage.yaml"
        config.write_text('compression: "None"\nchunkSize: 4194304\n')
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(directory / "raw_ros"),
                storage_id="mcap",
                storage_config_uri=str(config),
                max_bagfile_size=268435456,
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        while True:
            item = jobs.get()
            if item is None:
                break
            topic, type_name, payload, received_ns = item
            if topic not in counts:
                writer.create_topic(
                    rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format="cdr")
                )
                counts[topic] = 0
            writer.write(topic, payload, received_ns)
            counts[topic] += 1
            if first_ns is None:
                first_ns = received_ns
            last_ns = received_ns
    except Exception as exc:
        error = str(exc)
    finally:
        if writer is not None:
            if hasattr(writer, "close"):
                writer.close()
            del writer
        results.put(
            dict(
                ok=error is None and (directory / "raw_ros" / "metadata.yaml").exists(),
                error=error,
                topics_requested=TOPICS,
                message_counts=counts,
                first_receive_ns=first_ns,
                last_receive_ns=last_ns,
                note="Original CDR payloads and header timestamps; bag timestamps are local receive time. Unpublished topics cannot be recorded.",
            )
        )


class RosArchive:
    """Separate process prevents native bag writes from holding the ROS Python GIL."""

    def __init__(self, directory):
        ctx = multiprocessing.get_context("spawn")
        self.jobs = ctx.Queue(maxsize=512)
        self.results = ctx.Queue(maxsize=1)
        self.dropped = 0
        self.accepted_by_topic = {}
        self.dropped_by_topic = {}
        self.drop_events = []
        self.worker = ctx.Process(
            target=_archive_worker, args=(str(directory), self.jobs, self.results), daemon=True
        )
        self.worker.start()

    def write(self, topic, type_name, payload, received_ns):
        if not self.worker.is_alive():
            self._drop(topic, received_ns, "writer_not_alive")
            return
        try:
            self.jobs.put_nowait((topic, type_name, payload, received_ns))
            self.accepted_by_topic[topic] = self.accepted_by_topic.get(topic, 0) + 1
        except queue.Full:
            self._drop(topic, received_ns, "archive_queue_full")

    def _drop(self, topic, received_ns, reason):
        self.dropped += 1
        self.dropped_by_topic[topic] = self.dropped_by_topic.get(topic, 0) + 1
        if len(self.drop_events) < 256:
            self.drop_events.append(dict(topic=topic, receive_ns=received_ns, reason=reason))

    def stop(self):
        try:
            self.jobs.put(None, timeout=10)
            result = self.results.get(timeout=30)
        except queue.Empty:
            result = dict(ok=False, error="Raw archive writer timed out")
        except queue.Full:
            result = dict(ok=False, error="Raw archive queue did not drain")
        self.worker.join(timeout=3)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=3)
        result["dropped_messages"] = self.dropped
        result["accepted_by_topic"] = self.accepted_by_topic
        result["dropped_by_topic"] = self.dropped_by_topic
        result["drop_events_first_256"] = self.drop_events
        result["loss_scope"] = (
            "Application queue losses only; DDS reports separately, sensor-side losses may be unobservable."
        )
        self.jobs.cancel_join_thread()
        self.jobs.close()
        self.results.close()
        return result
