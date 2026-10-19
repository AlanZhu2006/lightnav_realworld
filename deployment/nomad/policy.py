"""ROS-free, image-goal NoMaD inference with an explicit observation history.

No robot APIs are imported here. The architecture and numerical helpers come
from the supplied official checkout. A request contains four observations;
neither the model nor its HTTP wrapper owns a camera FIFO or route cursor.
"""
from __future__ import annotations

import ast
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Tuple


@dataclass(frozen=True)
class InferenceIdentity:
    session_id: str
    task_id: str
    goal_id: str
    goal_version: int
    frame_id: str

    def validate(self) -> None:
        for name in ("session_id", "task_id", "goal_id", "frame_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip() or len(value) > 256:
                raise ValueError(f"{name} must be a nonempty string of at most 256 characters")
        if type(self.goal_version) is not int or self.goal_version < 0:
            raise ValueError("goal_version must be a nonnegative integer")


@dataclass(frozen=True)
class RGBFrame:
    """A PIL RGB image and its original camera-host capture metadata.

    Timestamps must use one clock for all four frames. They are echoed rather
    than compared with the inference host's clock; the consumer owns freshness.
    """

    image: Any
    frame_id: str
    capture_timestamp_ns: int


def validate_observations(observations: List[RGBFrame], identity: InferenceIdentity) -> None:
    """Validate ordering without importing torch or contacting any hardware."""
    identity.validate()
    if len(observations) != 4:
        raise ValueError("exactly four RGB observations are required, oldest first")
    previous_ns = -1
    seen_ids = set()
    for frame in observations:
        if not isinstance(frame, RGBFrame):
            raise ValueError("observations must contain RGBFrame objects")
        if (not isinstance(frame.frame_id, str) or not frame.frame_id.strip()
                or len(frame.frame_id) > 256 or frame.frame_id in seen_ids):
            raise ValueError("observation frame_id values must be nonempty and unique")
        if type(frame.capture_timestamp_ns) is not int or frame.capture_timestamp_ns <= previous_ns:
            raise ValueError("capture_timestamp_ns must be nonnegative and strictly increasing")
        seen_ids.add(frame.frame_id)
        previous_ns = frame.capture_timestamp_ns
    if observations[-1].frame_id != identity.frame_id:
        raise ValueError("identity.frame_id must identify the newest observation")


def _fingerprint(path: Path) -> Dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def _official_functions(path: Path, names: List[str], namespace: Dict[str, Any]) -> None:
    """Compile selected official function bodies without their ROS imports."""
    tree = ast.parse(path.read_text(), filename=str(path))
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    if {node.name for node in selected} != set(names):
        raise ValueError(f"required official functions missing from {path}: {names}")
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)


class NoMaDPolicy:
    """Fixed official NoMaD image-goal policy; output is a path, never a command.

    ``model_max_v / model_frame_rate`` is the original action calibration. It
    must not be changed to impose an operational speed limit on the robot.
    """

    def __init__(
        self,
        *,
        upstream: str | Path,
        checkpoint: str | Path,
        diffusion_policy: str | Path,
        device: str = "cpu",
        seed: int = 260806,
        model_max_v: float = 0.2,
        model_frame_rate: float = 4.0,
        expected_checkpoint_sha256: str | None = None,
    ) -> None:
        upstream = Path(upstream).expanduser().resolve()
        checkpoint = Path(checkpoint).expanduser().resolve()
        diffusion_policy = Path(diffusion_policy).expanduser().resolve()
        sources = [
            "train/config/nomad.yaml",
            "train/vint_train/data/data_config.yaml",
            "train/vint_train/models/nomad/nomad.py",
            "train/vint_train/models/nomad/nomad_vint.py",
            "train/vint_train/training/train_utils.py",
            "train/vint_train/visualizing/visualize_utils.py",
            "deployment/src/utils.py",
            "deployment/src/pd_controller.py",
        ]
        for path in [checkpoint, *(upstream / name for name in sources),
                     diffusion_policy / "diffusion_policy/model/diffusion/conditional_unet1d.py"]:
            if not path.is_file():
                raise FileNotFoundError(path)
        for name, value in (("model_max_v", model_max_v), ("model_frame_rate", model_frame_rate)):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        checkpoint_pin = _fingerprint(checkpoint)
        if expected_checkpoint_sha256 and checkpoint_pin["sha256"] != expected_checkpoint_sha256.lower():
            raise ValueError("checkpoint SHA256 does not match the configured pin")

        # Imports remain local so --help and transport validation work without a
        # GPU stack. The provided checkout/vendor directories are trusted code.
        import numpy as np
        import torch
        import yaml
        from PIL import Image as PILImage
        from torchvision import transforms
        from torchvision.transforms import functional as TF

        sys.path.insert(0, str(upstream / "train"))
        sys.path.insert(0, str(diffusion_policy))
        from vint_train.models.nomad.nomad import NoMaD, DenseNetwork
        from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn
        from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
        from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

        for module_name, expected_root in ((NoMaD.__module__, upstream / "train"),
                                           (NoMaD_ViNT.__module__, upstream / "train"),
                                           (ConditionalUnet1D.__module__, diffusion_policy)):
            loaded_path = Path(sys.modules[module_name].__file__).resolve()
            if expected_root != loaded_path and expected_root not in loaded_path.parents:
                raise RuntimeError(f"{module_name} was already loaded from another checkout: {loaded_path}")
        self.config = yaml.safe_load((upstream / sources[0]).read_text())
        expected = {"model_type": "nomad", "vision_encoder": "nomad_vint", "context_size": 3,
                    "image_size": [96, 96], "num_diffusion_iters": 10,
                    "len_traj_pred": 8, "normalize": True, "learn_angle": False}
        for name, value in expected.items():
            if self.config.get(name) != value:
                raise ValueError(f"unsupported official configuration {name}: expected {value!r}")
        self.torch, self.np, self.PILImage = torch, np, PILImage
        self.device = torch.device(device)
        if self.device.type not in ("cpu", "cuda"):
            raise ValueError("device must select CPU or CUDA")
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")
        torch.set_num_threads(1)
        torch.manual_seed(seed)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        config = self.config
        encoder = NoMaD_ViNT(
            obs_encoding_size=config["encoding_size"], context_size=config["context_size"],
            mha_num_attention_heads=config["mha_num_attention_heads"],
            mha_num_attention_layers=config["mha_num_attention_layers"],
            mha_ff_dim_factor=config["mha_ff_dim_factor"],
        )
        model = NoMaD(
            vision_encoder=replace_bn_with_gn(encoder),
            noise_pred_net=ConditionalUnet1D(
                input_dim=2, global_cond_dim=config["encoding_size"],
                down_dims=config["down_dims"], cond_predict_scale=config["cond_predict_scale"]),
            dist_pred_net=DenseNetwork(embedding_dim=config["encoding_size"]),
        )
        state = torch.load(str(checkpoint), map_location="cpu", weights_only=True)
        model.load_state_dict(state, strict=True)
        del state
        self.model = model.to(self.device).eval().requires_grad_(False)
        stats = yaml.safe_load((upstream / sources[1]).read_text())
        namespace = {
            "torch": torch, "np": np, "PILImage": PILImage, "transforms": transforms,
            "TF": TF, "List": List, "Dict": Dict, "Tuple": Tuple,
            "IMAGE_ASPECT_RATIO": 4 / 3,
            "ACTION_STATS": {key: np.array(value) for key, value in stats["action_stats"].items()},
            "MAX_V": model_max_v, "MAX_W": 0.4, "DT": 1 / model_frame_rate, "EPS": 1e-8,
        }
        _official_functions(upstream / "train/vint_train/visualizing/visualize_utils.py",
                            ["to_numpy", "from_numpy"], namespace)
        _official_functions(upstream / "train/vint_train/training/train_utils.py",
                            ["unnormalize_data", "get_action"], namespace)
        _official_functions(upstream / "deployment/src/utils.py", ["transform_images"], namespace)
        _official_functions(upstream / "deployment/src/pd_controller.py",
                            ["clip_angle", "pd_controller"], namespace)
        self._transform = namespace["transform_images"]
        self._get_action = namespace["get_action"]
        self._pd = namespace["pd_controller"]
        self._scheduler = DDPMScheduler(
            num_train_timesteps=10, beta_schedule="squaredcos_cap_v2", clip_sample=True,
            prediction_type="epsilon")
        self._lock = threading.Lock()
        self.scale_m = model_max_v / model_frame_rate
        self.contract = {
            "schema_version": "nomad.inference.v1", "checkpoint": checkpoint_pin,
            "sources": [_fingerprint(upstream / name) for name in sources],
            "diffusion_source": _fingerprint(Path(sys.modules[ConditionalUnet1D.__module__].__file__)),
            "strict_checkpoint_load": True, "device": str(self.device),
            "torch": torch.__version__, "seed": seed, "history_frames": 4,
            "preprocessing": "official PIL resize 96x96; no crop; RGB ImageNet normalization",
            "diffusion_steps": 10, "num_samples": 8, "selected_sample": 0,
            "model_max_v": model_max_v, "model_frame_rate": model_frame_rate,
            "action_scale_m": self.scale_m,
            "scale_m_per_model_unit": self.scale_m,
            "trajectory_frame": "body_at_newest_capture; x_forward_y_left",
            "arrival_detection": "none; distance_steps is a learned temporal distance",
        }

    def predict(self, observations: List[RGBFrame], goal_rgb: Any,
                identity: InferenceIdentity) -> Dict[str, Any]:
        validate_observations(observations, identity)
        for image in [*(frame.image for frame in observations), goal_rgb]:
            if not isinstance(image, self.PILImage.Image) or image.mode != "RGB":
                raise ValueError("all observations and the goal must be PIL RGB images")
        started = time.perf_counter()
        # Both the scheduler and random sampling are serialized per instance.
        with self._lock, self.torch.inference_mode():
            obs = self._transform([frame.image for frame in observations], [96, 96],
                                  center_crop=False).to(self.device)
            goal = self._transform(goal_rgb, [96, 96], center_crop=False).to(self.device)
            mask = self.torch.zeros(1, dtype=self.torch.long, device=self.device)
            conditioning = self.model("vision_encoder", obs_img=obs, goal_img=goal,
                                      input_goal_mask=mask)
            distance = self.model("dist_pred_net", obsgoal_cond=conditioning)
            conditioning = conditioning.repeat(8, 1) if conditioning.ndim == 2 else conditioning.repeat(8, 1, 1)
            action = self.torch.randn((8, 8, 2), device=self.device)
            self._scheduler.set_timesteps(10)
            for timestep in self._scheduler.timesteps:
                noise = self.model("noise_pred_net", sample=action, timestep=timestep,
                                   global_cond=conditioning)
                action = self._scheduler.step(model_output=noise, timestep=timestep,
                                              sample=action).prev_sample
            # get_action unnormalizes deltas then cumulatively sums. Copy before
            # metric scaling: the original chosen_waypoint is a view into sample
            # zero, so an in-place scale would mix units inside the full path.
            samples = self._get_action(action).detach().cpu().numpy()
            first_sample = samples[0].copy()
            trajectory_m = first_sample * self.scale_m
            distance_steps = float(distance.flatten()[0].item())
        if not self.np.isfinite(trajectory_m).all() or not math.isfinite(distance_steps):
            raise RuntimeError("NoMaD returned nonfinite output")
        diagnostic_waypoint = trajectory_m[2].copy()
        diagnostic_v, diagnostic_w = self._pd(diagnostic_waypoint)
        return {
            "schema_version": "nomad.inference.v1", "identity": asdict(identity),
            "source_frame": {"frame_id": observations[-1].frame_id,
                             "capture_timestamp_ns": observations[-1].capture_timestamp_ns},
            "observation_frames": [{"frame_id": frame.frame_id,
                                    "capture_timestamp_ns": frame.capture_timestamp_ns}
                                   for frame in observations],
            "trajectory_body_m": trajectory_m.tolist(),
            "first_sample_normalized": first_sample.tolist(),
            "action_scale_m": self.scale_m,
            "scale_m_per_model_unit": self.scale_m,
            "distance_steps": distance_steps,
            "diagnostic_pd": {"waypoint_index": 2, "waypoint_body_m": diagnostic_waypoint.tolist(),
                              "v_m_s": float(diagnostic_v), "w_rad_s": float(diagnostic_w),
                              "diagnostic_only": True},
            "inference_ms": (time.perf_counter() - started) * 1000,
        }
