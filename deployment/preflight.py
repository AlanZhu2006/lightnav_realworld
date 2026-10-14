"""Read-only dependency checks. Never creates a ROS node or Unitree client."""

from __future__ import annotations

import importlib
import json
import socket
import sys
from importlib.metadata import version
from pathlib import Path

from .configuration import RobotConfig, ServerConfig


def checkpoint_files(model: Path) -> None:
    for name in ("config.json", "eval_config.json", "processor_config.json", "tokenizer.json"):
        if not (model / name).is_file():
            raise ValueError(f"Checkpoint is missing {model / name}")
    index = model / "model.safetensors.index.json"
    if index.exists():
        weights = set(json.loads(index.read_text())["weight_map"].values())
    else:
        weights = {"model.safetensors"}
    if not weights or any(not (model / name).is_file() for name in weights):
        raise ValueError("Checkpoint weight shards are incomplete")
    bundle = model / "action_tokenizer"
    manifest = json.loads((bundle / "manifest.json").read_text())
    if manifest.get("horizon") != 10:
        raise ValueError("This robot deployment requires a 10-waypoint action tokenizer")
    for name in [
        *manifest["codebook_files"],
        manifest["jacobian_weights_file"],
        manifest["alpha_file"],
    ]:
        if not (bundle / name).is_file():
            raise ValueError(f"Action tokenizer is missing {name}")


def free_port(host: str, port: int) -> None:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    with socket.socket(family, socket.SOCK_STREAM) as sock:
        sock.bind((host, port))


def check_server(config: ServerConfig) -> dict:
    if sys.version_info[:2] != (3, 11):
        raise ValueError("The model server requires Python 3.11")
    checkpoint_files(config.model)
    if version("transformers") != "5.8.0":
        raise ValueError("transformers must be exactly 5.8.0")
    if config.backend == "vllm_local":
        for package, expected in (("vllm", "0.19.1"), ("nvidia-cutlass-dsl", "4.5.2")):
            if version(package) != expected:
                raise ValueError(f"{package} must be exactly {expected}")
    import torch

    if not torch.cuda.is_available():
        raise ValueError("CUDA is unavailable; check the selected GPU and Torch installation")
    free_port(config.host, config.port)
    return dict(
        role="server",
        python=sys.version.split()[0],
        model=str(config.model),
        backend=config.backend,
        gpu=torch.cuda.get_device_name(0),
        port=config.port,
        note="Dependencies and files checked; startup warm-up validates inference.",
    )


def check_robot(config: RobotConfig) -> dict:
    if sys.version_info[:2] != (3, 10):
        raise ValueError("ROS 2 Humble on Jetson requires Python 3.10")
    for name in (
        "rclpy",
        "ros2cli",
        "rosbag2_py",
        "rosidl_runtime_py",
        "cv2",
        "casadi",
        "aiohttp",
        "unitree_sdk2py.core.channel",
        "unitree_sdk2py.go2.sport.sport_client",
        "unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client",
    ):
        importlib.import_module(name)
    from ament_index_python.packages import get_package_share_directory

    for name in (
        "vln_bringup",
        "vln_client",
        "vln_mpc",
        "vln_web",
        "go2_adapter",
        "rosbag2_storage_mcap",
    ):
        get_package_share_directory(name)
    if not (Path("/sys/class/net") / config.network_interface).exists():
        raise ValueError(f"Network interface does not exist: {config.network_interface}")
    free_port(config.web_host, config.web_port)
    return dict(
        role="robot",
        python=sys.version.split()[0],
        server_url=config.server_url,
        interface=config.network_interface,
        image_topic=config.image_topic,
        note="No camera stream, network connection, ROS node, or robot action was started.",
    )
