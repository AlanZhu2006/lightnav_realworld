"""Portable deployment settings; no ROS, model imports, or device side effects."""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[1]


def _path(env: Mapping[str, str], key: str, default: Path, root: Path) -> Path:
    value = Path(env.get(key) or default).expanduser()
    return (root / value).absolute() if not value.is_absolute() else value


def _integer(env: Mapping[str, str], key: str, default: int, low: int, high: int) -> int:
    value = int(env.get(key, str(default)))
    if not low <= value <= high:
        raise ValueError(f"{key} must be between {low} and {high}")
    return value


def _number(env: Mapping[str, str], key: str, default: float) -> float:
    value = float(env.get(key, str(default)))
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{key} must be finite and positive")
    return value


def _choice(env: Mapping[str, str], key: str, default: str, choices: set[str]) -> str:
    value = env.get(key, default)
    if value not in choices:
        raise ValueError(f"{key} must be one of {sorted(choices)}")
    return value


def websocket_url(value: str) -> str:
    value = value.strip()
    parsed = urlparse(value)
    if (
        not value
        or any(c.isspace() for c in value)
        or parsed.scheme not in {"ws", "wss"}
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.fragment
        or (parsed.port is not None and not 1 <= parsed.port <= 65535)
    ):
        raise ValueError("LIGHTNAV_SERVER_URL must be ws://HOST:PORT or wss://HOST:PORT")
    return value


@dataclass(frozen=True)
class ServerConfig:
    root: Path
    python: Path
    model: Path
    backend: str
    task: str
    host: str
    port: int
    gpu: str
    memory_fraction: float
    batch_size: int
    cache_entries: int
    recordings: Path
    telemetry: bool

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None, root: Path = REPO_ROOT):
        env = os.environ if env is None else env
        memory = _number(env, "LIGHTNAV_GPU_MEMORY_UTILIZATION", 0.55)
        if memory > 1:
            raise ValueError("LIGHTNAV_GPU_MEMORY_UTILIZATION must be at most 1")
        host = env.get("LIGHTNAV_HOST", "0.0.0.0").strip()
        gpu = env.get("LIGHTNAV_GPU", "0").strip()
        if not host or not gpu:
            raise ValueError("LIGHTNAV_HOST and LIGHTNAV_GPU cannot be empty")
        return cls(
            root=root,
            python=_path(env, "LIGHTNAV_SERVER_PYTHON", root / ".venv-server/bin/python", root),
            model=_path(env, "LIGHTNAV_MODEL_PATH", root / "checkpoints/LightNav-0", root),
            backend=_choice(env, "LIGHTNAV_BACKEND", "vllm_local", {"vllm_local", "hf"}),
            task=_choice(env, "LIGHTNAV_TASK", "vln", {"vln", "tracking"}),
            host=host,
            port=_integer(env, "LIGHTNAV_PORT", 8050, 1, 65535),
            gpu=gpu,
            memory_fraction=memory,
            batch_size=_integer(env, "LIGHTNAV_MAX_BATCH_SIZE", 1, 1, 1024),
            cache_entries=_integer(env, "LIGHTNAV_VIT_CACHE_ENTRIES", 256, 0, 100000),
            recordings=_path(
                env, "LIGHTNAV_SERVER_RECORDINGS", root / "recordings-inference", root
            ),
            telemetry=_choice(env, "LIGHTNAV_HOST_TELEMETRY", "1", {"0", "1"}) == "1",
        )

    def command(self, ready_file: Path) -> list[str]:
        return [
            str(self.python),
            "-m",
            "lightnav.serving.ws_server",
            "--model_path",
            str(self.model),
            "--backend",
            self.backend,
            "--task",
            self.task,
            "--host",
            self.host,
            "--port",
            str(self.port),
            "--gpu_memory_utilization",
            str(self.memory_fraction),
            "--max_batch_size",
            str(self.batch_size),
            "--vit_cache_entries",
            str(self.cache_entries),
            "--ready_file",
            str(ready_file),
            "--record_dir",
            str(self.recordings / "episodes"),
        ]

    def environment(self) -> dict[str, str]:
        env = dict(os.environ)
        env.update(
            CUDA_VISIBLE_DEVICES=self.gpu,
            PYTHONUNBUFFERED="1",
            TOKENIZERS_PARALLELISM="false",
            LIGHTNAV_REPO_ROOT=str(self.root),
        )
        env["PYTHONPATH"] = str(self.root / "src") + os.pathsep + env.get("PYTHONPATH", "")
        return env


@dataclass(frozen=True)
class RobotConfig:
    root: Path
    server_url: str
    network_interface: str
    image_topic: str
    image_transport: str
    linear_limit: float
    angular_limit: float
    web_host: str
    web_port: int
    recordings: Path

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None, root: Path = REPO_ROOT):
        env = os.environ if env is None else env
        interface = env.get("LIGHTNAV_NETWORK_INTERFACE", "enP8p1s0").strip()
        topic = env.get("LIGHTNAV_IMAGE_TOPIC", "/camera/camera/color/image_raw").strip()
        if not interface or "/" in interface or not topic.startswith("/"):
            raise ValueError("Set a network interface and an absolute ROS image topic")
        _integer(env, "ROS_DOMAIN_ID", 0, 0, 232)
        return cls(
            root=root,
            server_url=websocket_url(env.get("LIGHTNAV_SERVER_URL", "")),
            network_interface=interface,
            image_topic=topic,
            image_transport=_choice(env, "LIGHTNAV_IMAGE_TRANSPORT", "raw", {"raw", "compressed"}),
            linear_limit=_number(env, "LIGHTNAV_LINEAR_LIMIT", 0.3),
            angular_limit=_number(env, "LIGHTNAV_ANGULAR_LIMIT", 0.5),
            web_host=env.get("LIGHTNAV_WEB_HOST", "0.0.0.0"),
            web_port=_integer(env, "LIGHTNAV_WEB_PORT", 8088, 1, 65535),
            recordings=_path(env, "LIGHTNAV_RECORDINGS_DIR", root / "recordings", root),
        )

    def command(self) -> list[str]:
        values = dict(
            server_url=self.server_url,
            network_interface=self.network_interface,
            image_topic=self.image_topic,
            image_transport=self.image_transport,
            linear_limit=self.linear_limit,
            angular_limit=self.angular_limit,
            web_host=self.web_host,
            web_port=self.web_port,
            recording_directory=self.recordings,
        )
        return [
            sys.executable,
            "-c",
            "from ros2cli.cli import main; raise SystemExit(main())",
            "launch",
            "vln_bringup",
            "go2_realsense.launch.py",
            *[f"{key}:={value}" for key, value in values.items()],
        ]
