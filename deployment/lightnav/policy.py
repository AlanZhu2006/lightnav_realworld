"""Official LightNav inference plus an explicit Go2 reference-image adapter.

Native language/tracking call the released generation path unchanged. The
separately selected ImageGoal profile adds a static reference segment and prompt;
it is a deployment extension, not an official released ImageGoal interface.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import threading
import time

VENDOR = Path(__file__).parent / "vendor/upstream"
MODEL_SHA256 = "ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18"
MODEL_REVISION = "826dc5fbfa37afa8293d2e336d329b6ffc0bfb64"
CONDITIONING_MODES = ("native_language", "native_tracking", "experimental_reference_image")
TASK_MODES = ("objnav", "vln", "tracking", "imagegoal")
SERVER_IMPLEMENTATION = "official_core_with_deployment_adapters"
IMAGEGOAL_IMPLEMENTATION = "go2_imagegoal_adaptation"
RETIREMENT_PROTOCOL = {"schema_version": "lightnav.owned_retirement.v1", "supported": True,
                       "lifetime": "server_process", "automatic_expiration": False}
DEFAULT_INSTRUCTION = (
    "Navigate to the place shown in the separate reference goal view and stop "
    "when the current view reaches that place. The reference is the destination, "
    "not an observation from the robot's past."
)
OFFICIAL_RELEASE_COMMIT = "3015508b70fb6e30bbd66701e94c9b916dcb8df8"
# Official-release hashes, independent of the existing robot-repository mirror.
OFFICIAL_CORE_SHA256 = {
    'src/lightnav/__init__.py': '79b2b0b1e64902dd441bc76dd6160e95e7710d0269bf5aa48b5de609f36fa236',
    'src/lightnav/cli/__init__.py': 'a059a7ecea60a694a609144db2e6be7b2f5129bf25a8d1eba170f46a772d0128',
    'src/lightnav/cli/eval_habitat.py': 'dce3727aed9946b6d816af8807f4609cf2a80a57336dedc637bfa1e1a0d4cf09',
    'src/lightnav/cli/eval_merge.py': '229999f6313fd8e3a59c2a43e1fe118459897037a61855dca8fdbc0679802167',
    'src/lightnav/cli/predict.py': '824103b62da7f1a16d248a37f019be16fe224b8e9e14b51636b679c63ca02c94',
    'src/lightnav/cli/render.py': '576ad54cd21ce1047b2eeb6e6fc294ee0b8c61da2a4802627f9c02772b32f412',
    'src/lightnav/cli/ws_client.py': '878c05ba2278807420d58b8a0f424cd00d39a245116995c6c56d587815466a60',
    'src/lightnav/data_processor.py': '3feb92fa0b0e02d1de0ec12446cf769efd3000cfece83a410250e403fa1f551a',
    'src/lightnav/eval_config.py': '5cd23cd9f8040c90e7e572c706cb0b5778750f1b68d1a6e03ed65a2f58a98318',
    'src/lightnav/habitat/__init__.py': 'cf56d36f6014741d9e1a2e636bdb4e1061741e40b74e71a3a47991c393aa3cf9',
    'src/lightnav/habitat/merge.py': 'e492b7ef6f6cded65b366444f463145c550a25dd7ec1aae1aad1ba55d17acbfb',
    'src/lightnav/habitat/policy.py': '1e7231de07a61f11719116d12c715df9f55e83ffcece88498f087b68f8edd3db',
    'src/lightnav/habitat/remote_env.py': 'a89d79d4d0c8ad89d58b8ab2ea079bdc04ba2238e4cd2fb8320ef8b3ae9bd1fe',
    'src/lightnav/habitat/results.py': 'cf7ba49a16c29d1c11977f49f59e58d480bd705d0c71fc06deb8e62ef58b6c47',
    'src/lightnav/habitat/runner.py': '6c9a7816af291128495cf3f7fa75d27162a8cf8dde8306bdd6816a8c13e08957',
    'src/lightnav/inference/__init__.py': '2b040be9a48bdcb0398c3343b2246224e4f6040e8b9e974c815dc0e79c31d2fd',
    'src/lightnav/inference/config.py': '3683ccd42037ded602764c2b1962a9b3e80f1f3841497a9c22dac1f540df70f1',
    'src/lightnav/inference/engine.py': '0c955a7bee0df7363165c64f5e4b20bd065c1f132c36c50967fe9b2530c67ec0',
    'src/lightnav/inference/frame_preprocessing.py': 'e339b9b9038dd17af27e1addb357ea12f509da0cb34f742f05ba935455a4ebfa',
    'src/lightnav/inference/model.py': 'e745d3f7755eba62834168ec5e1676b611d5cf47e866a256d44f13accdbe44c1',
    'src/lightnav/inference/policies.py': '05c6a37b11911498d25812a2320084994a0fefd965ca883c7f55c8dbf8b57c03',
    'src/lightnav/inference/samples.py': '6e9c24fc70526f7dd910d4a2601d14c856e698850eba5c50c71f2bea7d0ced69',
    'src/lightnav/inference/vit_cache.py': '267fd3c1df605a869a0f8eaddd72b3d9645a7f8a9db78630122f2792c37bd797',
    'src/lightnav/inference/vllm_utils.py': '869d7ec0eff457239efede1c6e164f2766f50a553ef5113906e72d0fe442506c',
    'src/lightnav/processing.py': '470c1bc84bce40648105492e5b1b657bfe5cc49fec1c192bed27649add18684c',
    'src/lightnav/prompts.py': '9968adadffeed91327edbe2c04bd355171c32d51b2abad63755e24009aa0fd41',
    'src/lightnav/serving/__init__.py': 'f83c9be3b60e5c6798fee78ac2ed6d60151caa010261c69f4d518f93ca1f64ae',
    'src/lightnav/serving/batcher.py': 'b8505fc35ad5e3959dba22f3e0fc6a2e303f422492d454ccff29d5db106d308a',
    'src/lightnav/serving/protocol.py': '75b42419cfbe506a5fc78aee33025fb2a2e51051db10e1e59bbf53cd409bff7a',
    'src/lightnav/serving/token_budget.py': 'd8ceb6c3b0a43c68a0f7fc1cb944e4a4f72e5de3282f37b77c7f3cc4ad8685f0',
    'src/lightnav/serving/tracking_service.py': '88a1912ce2b3ad7dede12c28f08475bbb9115756c3e5cd9b8e6018768fe17add',
    'src/lightnav/slowfast.py': 'ac7deb3535341154f06b5419e6fe6b740ca169c136b3d03ad37bfed64de162cc',
    'src/lightnav/tracking.py': '59de24911956821e00f80ca82d4cfad4b15ab0de4d85e85244436694ea805b69',
    'src/lightnav/traj_vocab.py': '84c83780ba893d468e7529f5de44223f5f2aa44c099eb46fac3edb1786c14f0d',
    'src/lightnav/velocity.py': 'aa155b4a674298c1bf234679be4295ff6d387a87818dc66d407dfa74d2ca076f',
    'src/lightnav/viz/__init__.py': 'b0ddf34f974c7c1562ba603a7cb6e06aca7e77b4f49a881c5ed924d409d7fbec',
    'src/lightnav/viz/projection.py': 'a49840f72972c0d109a4f976ae314270bcdf98abb8dbb9bf441b9f165ceef345',
    'src/lightnav/viz/recorder.py': 'a2f979127935a37ea1e1cd4568d4da5d05b03ffb333204c3a71e94dc1181bace',
    'src/lightnav/viz/render.py': 'fb70c9bfcb8b5906c4f29d5f81a7e7781a6f3e5091d8babd09b58f26d274ef1b',
    'src/lightnav/viz/render_episode.py': '8d2bf01de6beb00d38e763a3e5e546718334a7e62cefa137958c51d5b3a3822a',
    'src/lightnav/viz/video.py': '67318a2d0d5d6b21b6c40313ce7c6e30eb7373c09befe2d11b9d08b529e23a54',
    'src/lightnav/vln_utils.py': '3d2e35fbe793497f6043a45d53a9d0a7b9498e07b10d4458df4832a04b5b8bec',
}


# Stable digest of the complete verified module-hash map, independent of paths
# outside the vendored release or JSON insertion order.
OFFICIAL_CORE_DIGEST = hashlib.sha256(json.dumps(
    OFFICIAL_CORE_SHA256, sort_keys=True, separators=(",", ":"),
).encode()).hexdigest()

class HistoryConflictError(ValueError):
    http_status = 409

    def __init__(self, message, error_code="history_conflict"):
        super().__init__(message)
        self.error_code = error_code
        self.retryable = False


def validate_session_identity(identity, *, task_required=False):
    allowed = {"session_id", "task_id"}
    if not isinstance(identity, dict) or set(identity) - allowed:
        raise ValueError("identity accepts only session_id and task_id")
    required = allowed if task_required else {"session_id"}
    if not required <= set(identity):
        raise ValueError("identity requires " + ", ".join(sorted(required)))
    for value in identity.values():
        if not isinstance(value, str) or not value or len(value) > 256:
            raise ValueError("session/task identity must contain bounded nonempty strings")
    return dict(identity)


def verify_sources():
    manifest = json.loads((VENDOR / "SOURCE_MANIFEST.json").read_text())
    for item in manifest["files"]:
        if hashlib.sha256((VENDOR / item["path"]).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"Vendored LightNav source changed: {item['path']}")
    for relative, expected in OFFICIAL_CORE_SHA256.items():
        if hashlib.sha256((VENDOR / relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f"LightNav inference differs from official {OFFICIAL_RELEASE_COMMIT}: {relative}")
    return manifest


def verify_loaded_upstream_sources(modules=None):
    """Reject another ``lightnav`` package already cached in this process.

    The original ROS/WebSocket stack and this pinned HTTP adapter intentionally
    coexist in the repository. Prepending a directory to ``sys.path`` cannot
    replace modules already present in ``sys.modules``. Verify their actual
    source locations rather than claiming the verified vendor files are used
    merely because their bytes exist on disk. No module is unloaded or replaced.
    The optional mapping makes this provenance check independently CPU-testable.
    """
    loaded = sys.modules if modules is None else modules
    vendor = VENDOR.resolve()
    package_root = vendor / "src/lightnav"
    checked = 0
    for name, module in tuple(loaded.items()):
        if name != "lightnav" and not name.startswith("lightnav."):
            continue
        origin = getattr(module, "__file__", None)
        try:
            relative = Path(origin).resolve().relative_to(vendor).as_posix()
        except (TypeError, ValueError, OSError):
            relative = None
        paths = getattr(module, "__path__", ())
        try:
            package_paths_valid = all(
                Path(path).resolve().is_relative_to(package_root) for path in paths
            )
        except (TypeError, ValueError, OSError):
            package_paths_valid = False
        if relative not in OFFICIAL_CORE_SHA256 or not package_paths_valid:
            raise RuntimeError(
                f"LightNav source isolation failed for {name}: {origin!r}. "
                "Start this HTTP adapter in a separate Python process; do not "
                "preload the repository's original lightnav package."
            )
        checked += 1
    return checked


def activate_verified_upstream():
    """Select the pinned core after checking disk bytes and loaded modules."""
    manifest = verify_sources()
    verify_loaded_upstream_sources()
    source = str(VENDOR / "src")
    # Preserve all unrelated search paths and modules. A process with another
    # LightNav package already loaded fails above instead of silently mixing it.
    if not sys.path or sys.path[0] != source:
        sys.path.insert(0, source)
    return manifest


def image_fingerprint(image):
    # Identical byte sequences can describe different image geometries. Shape
    # is part of the model input identity because resizing uses width/height.
    return (image.mode, image.width, image.height, hashlib.sha256(image.tobytes()).hexdigest())


def build_tracking_engine(navigation_engine, model_path, max_new_tokens):
    """Resolve the official tracking task without loading another model copy.

    Engines/data processors hold task-specific history, pooling and prompt
    settings. The HF model and tokenizer are shared; the server serializes all
    generation. A missing tracking task is an error, never a VLN fallback.
    """
    from lightnav.eval_config import get_task_params, load_eval_config, resolve_asset_path
    from lightnav.inference.engine import VLNInferenceEngine
    from lightnav.inference.model import _build_data_processor, _make_bundle, _resolve_processing_params

    config = load_eval_config(str(model_path))
    if not config or not isinstance(config.get("tasks", {}).get("trackvla"), dict):
        raise ValueError("Native tracking requires an explicit eval_config.json tasks.trackvla entry")
    tracking = get_task_params(config, "trackvla")
    navigation = get_task_params(config, "vlnce")
    if bool(tracking.get("timestamp_relative", False)) != bool(navigation.get("timestamp_relative", False)):
        raise ValueError("Shared LightNav tasks require matching checkpoint timestamp_relative settings")
    action = tracking.get("action_tokenizer") or {}
    if (action.get("method") != "rvq" or tracking.get("predict_horizon") != 10 or
            resolve_asset_path(str(model_path), action.get("bundle_path", "")).resolve() !=
            (Path(model_path) / "action_tokenizer").resolve()):
        raise ValueError("Native tracking requires the checkpoint's ten-step RVQ action_tokenizer bundle")
    params = _resolve_processing_params(str(model_path), task_type="trackvla")
    source = navigation_engine.bundle
    processor = _build_data_processor(source.processor, params, source.position_id_func)
    bundle = _make_bundle(source.model, source.processor, processor, source.device,
                          source.position_id_func, params)
    return VLNInferenceEngine(bundle, backend="hf", max_new_tokens=max_new_tokens,
                              aspect_mode=navigation_engine.aspect_mode)


class LightNavPolicy:
    def __init__(self, model_path: Path, device="cuda", max_new_tokens=12, max_session_frames=4096,
                 max_sessions=2, session_idle_ttl_s=120.):
        import torch

        if sys.version_info[:2] != (3, 11):
            raise RuntimeError("Use the isolated Python 3.11 LightNav environment")
        if importlib.metadata.version("transformers") != "5.8.0":
            raise RuntimeError("LightNav source requires transformers==5.8.0")
        if tuple(int(x) for x in torch.__version__.split("+")[0].split(".")[:2]) < (2, 9):
            raise RuntimeError("Official LightNav requires Torch >=2.9")
        if not torch.cuda.is_available() or not str(device).startswith("cuda"):
            raise RuntimeError("This deployment requires an available CUDA device")
        source = activate_verified_upstream()
        manifest = json.loads((model_path / "SOURCE_MANIFEST.json").read_text())
        if manifest["revision"] != MODEL_REVISION:
            raise ValueError("Unexpected official model revision")
        for item in manifest["files"]:
            path = model_path / item["path"]
            h = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(8 << 20), b""):
                    h.update(chunk)
            if h.hexdigest() != item["sha256"]:
                raise ValueError(f"Model integrity check failed: {item['path']}")
        weight = next(x for x in manifest["files"] if x["path"].endswith(".safetensors"))
        if weight["sha256"] != MODEL_SHA256:
            raise ValueError("Unexpected official model shard")
        from lightnav.inference import InferenceConfig, build_engine
        from lightnav.traj_vocab import load_rvq_bundle
        from lightnav.tracking import TrackingAgent

        torch.set_grad_enabled(False)
        self.engine, self.bundle = build_engine(InferenceConfig(
            model_path=str(model_path), backend="hf", device=device,
            max_new_tokens=max_new_tokens, attn_implementation="sdpa",
        ), task_type="vlnce")
        self.tracking_engine = build_tracking_engine(self.engine, model_path, max_new_tokens)
        self.decoder = TrackingAgent(self.engine, rvq_bundle=load_rvq_bundle(
            model_path / "action_tokenizer", horizon=10,
        ))
        self.max_new_tokens = max_new_tokens
        self.sessions = {}
        self.retired_sessions = set()
        self.retired_tasks = set()
        self._lifecycle_lock = threading.RLock()
        self._cleanup_pending = set()
        self._inflight_keys = set()
        self.max_session_frames = int(max_session_frames)
        self.max_sessions = int(max_sessions)
        self.session_idle_ttl_s = float(session_idle_ttl_s)
        self.contract = {
            "model": "LightOriginsHQ/LightNav-0", "model_revision": MODEL_REVISION,
            "model_sha256": MODEL_SHA256, "source_commit": source["commit"],
            "backend": "hf", "device": str(self.bundle.device),
            "implementation": SERVER_IMPLEMENTATION, "official_release_commit": OFFICIAL_RELEASE_COMMIT,
            "official_core_sha256": OFFICIAL_CORE_DIGEST,
            "official_core_files_verified": len(OFFICIAL_CORE_SHA256),
            "source_mirror_commit": source["commit"],
            "native_imagegoal_support": False,
            "native_tracking_support": True,
            "imagegoal_adaptation_support": True,
            "supported_implementations": ["official_release", IMAGEGOAL_IMPLEMENTATION],
            "task_profiles": {
                "native_language": {"implementation": "official_release", "task_modes": ["objnav", "vln"],
                                    "instruction_required": True, "goal_image_required": False},
                "native_tracking": {"implementation": "official_release", "task_modes": ["tracking"],
                                    "instruction_required": True, "goal_image_required": False},
                "experimental_reference_image": {"implementation": IMAGEGOAL_IMPLEMENTATION,
                                    "task_modes": ["imagegoal"], "instruction_required": False,
                                    "goal_image_required": True, "official_interface": False},
            },
            "supported_conditioning_modes": list(CONDITIONING_MODES),
            "supported_task_modes": list(TASK_MODES),
            "tracking_eval_task": "trackvla",
            "tracking_goal_input": "language_instruction_only; no_goal_image",
            "conditioning_mode": "native_language", "default_conditioning_mode": "native_language",
            "instruction_required": True,
            "goal_image_input": "only experimental_reference_image; no separate official goal-image input",
            "goal_image_consumed": False, "stateful_overlapping_request_history": True,
            "max_request_frames": 64, "max_session_frames": self.max_session_frames,
            "max_sessions": self.max_sessions,
            "session_idle_ttl_s": self.session_idle_ttl_s,
            "idle_expiration": "retire_non_inflight_identity; never_silent_history_recreation",
            "history_overflow": "explicit_error_requires_new_task_or_reset; never_silent_truncation",
            "video_size": self.bundle.video_size, "video_fps": self.bundle.video_fps,
            "waypoints": {"shape": [10, 3], "units": ["m", "m", "rad"],
                          "frame": "capture_time_body", "positive_y": "left", "positive_yaw": "CCW",
                          "cumulative": True},
            "arrival_semantics": "model_stop_is_a_proposal_not_verified_arrival",
            "time_encoding": "official_frame_index_at_checkpoint_fps; caller_must_sample_history_at_4hz",
            "robot_transport": False, "retirement_protocol": dict(RETIREMENT_PROTOCOL),
        }

    def _lifecycle(self):
        # __new__-constructed CPU fixtures remain supported. Live instances
        # initialise this before being shared between HTTP/model threads.
        if not hasattr(self, "_lifecycle_lock"):
            self._lifecycle_lock = threading.RLock()
        if not hasattr(self, "_cleanup_pending"):
            self._cleanup_pending = set()
        if not hasattr(self, "retired_sessions"):
            self.retired_sessions, self.retired_tasks = set(), set()
        if not hasattr(self, "_inflight_keys"):
            self._inflight_keys = set()
        return self._lifecycle_lock

    def begin_predict(self, identity):
        with self._lifecycle():
            self.assert_active(identity, check_capacity=True)
            self._inflight_keys.add((identity["session_id"], identity["task_id"]))

    def end_predict(self, identity):
        with self._lifecycle():
            key = (identity["session_id"], identity["task_id"])
            self._inflight_keys.discard(key)
            state = self.sessions.get(key)
            if state is not None and not self._retired(identity):
                state["last_seen"] = time.monotonic()

    def expire_idle(self):
        # This lightweight control-plane pass never touches engine/cache state.
        # Status/health polling does not renew a task's lease.
        with self._lifecycle():
            now = time.monotonic()
            ttl = getattr(self, "session_idle_ttl_s", 120.)
            expired = {key for key, state in self.sessions.items()
                       if key not in self._inflight_keys and now - state["last_seen"] >= ttl}
            self.retired_tasks.update(expired)
            self._cleanup_pending.update(expired)
            return len(expired)

    def _retired(self, identity):
        return (identity["session_id"] in self.retired_sessions or
                (identity["session_id"], identity["task_id"]) in self.retired_tasks)

    def assert_active(self, identity, *, check_capacity=False):
        with self._lifecycle():
            if self._retired(identity):
                raise HistoryConflictError("Session or task was retired; use a new task identity", "task_retired")
            key = (identity["session_id"], identity["task_id"])
            if check_capacity and key not in self.sessions and len(self.sessions) >= self.max_sessions:
                raise HistoryConflictError("Active session capacity reached; explicitly reset an old session", "session_capacity")

    def session_status(self, identity):
        identity = validate_session_identity(identity, task_required=True)
        with self._lifecycle():
            key = (identity["session_id"], identity["task_id"])
            state = self.sessions.get(key)
            committed = state.get("committed") if state else None
            return {"ok": True, "identity": identity, "exists": state is not None,
                    "retired": self._retired(identity), "cleanup_pending": key in self._cleanup_pending,
                    "committed": dict(committed) if committed else None,
                    "active_session_count": len(self.sessions), "max_sessions": self.max_sessions}

    def retire(self, identity):
        """Close admission immediately; only the model worker releases tensors."""
        identity = validate_session_identity(identity)
        with self._lifecycle():
            session_id, task_id = identity["session_id"], identity.get("task_id")
            if task_id is None:
                self.retired_sessions.add(session_id)
            else:
                self.retired_tasks.add((session_id, task_id))
            selected = {key for key in self.sessions if key[0] == session_id and
                        (task_id is None or key[1] == task_id)}
            # Only resource-owning live keys enter this deduplicated queue:
            # its size cannot exceed max_sessions, regardless of retire retries.
            self._cleanup_pending.update(selected)
            pending = any(key[0] == session_id and (task_id is None or key[1] == task_id)
                          for key in self._cleanup_pending)
            return {"ok": True, "sessions_cleared": 0, "request_identity": identity,
                    "retired": True, "cleanup_pending": pending}

    def cleanup_retired(self):
        """Called exclusively on the serial model worker, never the HTTP thread."""
        with self._lifecycle():
            selected = tuple(self._cleanup_pending)
            removed = [(key, self.sessions[key]) for key in selected if key in self.sessions]
        # Do not reset either shared engine: another operator's episode may
        # still own its engine cache. Dropping only retired policies releases
        # their frame tensors and per-session caches, with no concurrent GPU use.
        for _, state in removed:
            cache = getattr(state["policy"], "_vit_cache", None)
            if cache is not None:
                cache.clear()
        with self._lifecycle():
            for key, state in removed:
                if self.sessions.get(key) is state:
                    del self.sessions[key]
            self._cleanup_pending.difference_update(selected)
        return len(removed)

    def cleanup_pending(self):
        with self._lifecycle():
            return bool(self._cleanup_pending)

    def session_diagnostics(self):
        # Snapshot references in one bounded builtin operation. Health must not
        # queue behind inference or lock/mutate the model's session state.
        with self._lifecycle():
            entries = tuple(self.sessions.items())
        limit = min(8, self.max_sessions)
        sessions = []
        for (session_id, task_id), state in entries[:limit]:
            sessions.append({"session_id": session_id, "task_id": task_id,
                             "goal_id": state["goal_key"][0], "goal_version": state["goal_key"][1],
                             "task_mode": state["task_mode"],
                             "conditioning_mode": state["conditioning_mode"],
                             "history_frames": len(state["frames"]),
                             "idle_s": max(0., time.monotonic() - state["last_seen"])})
        return {"active_session_count": len(entries), "active_sessions": sessions,
                "active_sessions_truncated": len(entries) > limit,
                "retired_session_count": len(getattr(self, "retired_sessions", ())),
                "retired_task_count": len(getattr(self, "retired_tasks", ())),
                "retirement_protocol": dict(RETIREMENT_PROTOCOL)}

    def reset(self, identity, retire=False):
        if type(retire) is not bool:
            raise ValueError("retire must be boolean")
        identity = validate_session_identity(identity)
        if retire:
            result = self.retire(identity)
            result.update(sessions_cleared=self.cleanup_retired(), cleanup_pending=False)
            return result
        with self._lifecycle():
            selected = [key for key in self.sessions if key[0] == identity["session_id"] and
                        (identity.get("task_id") is None or key[1] == identity["task_id"])]
            for key in selected:
                del self.sessions[key]
            self._cleanup_pending.difference_update(selected)
        self.engine.reset_episode_state()
        self.tracking_engine.reset_episode_state()
        return {"ok": True, "sessions_cleared": len(selected), "request_identity": identity, "retired": retire}

    def predict(self, frames, goal_image, identity, instruction=None, conditioning_mode="native_language", task_mode=None):
        import numpy as np
        import torch
        from lightnav.inference.policies import NavigationPolicy
        from lightnav.inference.samples import build_tracking_sample, build_vln_traj_sample
        from lightnav.serving.protocol import decode_prediction_signals, pointing_payload

        session_key = (identity["session_id"], identity["task_id"])
        self.assert_active(identity)
        if conditioning_mode not in CONDITIONING_MODES:
            raise ValueError("Unknown conditioning_mode")
        if instruction is not None and (not isinstance(instruction, str) or len(instruction) > 8192):
            raise ValueError("instruction must be a bounded string")
        imagegoal = conditioning_mode == "experimental_reference_image"
        tracking = conditioning_mode == "native_tracking"
        if not imagegoal and not (instruction and instruction.strip()):
            raise ValueError(f"{conditioning_mode} requires a nonempty instruction")
        if task_mode is None:
            task_mode = "imagegoal" if imagegoal else "tracking" if tracking else "objnav"
        allowed_tasks = {"imagegoal"} if imagegoal else {"tracking"} if tracking else {"objnav", "vln"}
        if not isinstance(task_mode, str) or task_mode not in allowed_tasks:
            raise ValueError("task_mode and conditioning_mode disagree")
        if imagegoal and goal_image is None:
            raise ValueError("experimental_reference_image requires a goal image")
        if not imagegoal and goal_image is not None:
            raise ValueError("Official LightNav accepts observations and an instruction, not a separate goal image")
        engine = self.tracking_engine if tracking else self.engine
        bundle = engine.bundle
        task = instruction if instruction and instruction.strip() else DEFAULT_INSTRUCTION
        goal_digest = image_fingerprint(goal_image) if imagegoal else None
        goal_key = (identity["goal_id"], identity["goal_version"])
        state = self.sessions.get(session_key)
        reset_reason = None
        if state is not None and state["goal_key"] != goal_key:
            if goal_key[1] <= state["goal_key"][1]:
                raise HistoryConflictError("Stale or conflicting goal generation")
            del self.sessions[session_key]
            state = None
            reset_reason = "new_goal_generation"
        if state is None:
            self.assert_active(identity, check_capacity=True)
            observed = NavigationPolicy(engine, num_history_frames=bundle.num_history_frames)
            observed.reset(task)
            state = {"policy": observed, "goal_key": goal_key, "goal_digest": goal_digest, "task_mode": task_mode,
                     "instruction": task, "conditioning_mode": conditioning_mode,
                     "frames": {}, "last_stamp": 0, "last_seen": time.monotonic(), "committed": None}
            with self._lifecycle():
                self.assert_active(identity, check_capacity=True)
                self.sessions[session_key] = state
            reset_reason = reset_reason or "new_task_session"
        if (state["instruction"] != task or state["conditioning_mode"] != conditioning_mode or
                state["task_mode"] != task_mode or state["goal_digest"] != goal_digest):
            raise HistoryConflictError("Goal pixels, instruction or task mode changed without a new goal generation")
        if frames[-1]["capture_timestamp_ns"] < state["last_stamp"]:
            raise HistoryConflictError("Stale newest-frame request would anchor inference to the wrong capture pose")
        observed = state["policy"]
        # Validate the entire append before mutating the episode. On a model
        # failure, appended real observations remain available for deduplicated
        # retry; a failed generation must never fabricate or drop history.
        additions, latest = [], state["last_stamp"]
        for frame in frames:
            fingerprint = (frame["capture_timestamp_ns"], image_fingerprint(frame["image"]))
            previous = state["frames"].get(frame["frame_id"])
            if previous is not None:
                if previous != fingerprint:
                    raise HistoryConflictError("Source frame ID reused with different timestamp or pixels")
                continue
            if frame["capture_timestamp_ns"] <= latest:
                raise HistoryConflictError("Cannot insert an unseen older frame into policy history")
            additions.append((frame, fingerprint))
            latest = frame["capture_timestamp_ns"]
        if len(state["frames"]) + len(additions) > self.max_session_frames:
            raise HistoryConflictError("LightNav episode history capacity reached; reset or begin a new task explicitly", "history_capacity")
        for frame, fingerprint in additions:
            self.assert_active(identity)
            observed.observe(np.asarray(frame["image"], dtype=np.uint8))
            with self._lifecycle():
                self.assert_active(identity)
                state["frames"][frame["frame_id"]] = fingerprint
                state["last_stamp"] = frame["capture_timestamp_ns"]
                state["committed"] = {"frame_id": frame["frame_id"],
                                      "capture_timestamp_ns": frame["capture_timestamp_ns"],
                                      "history_frames": len(state["frames"])}
        state["last_seen"] = time.monotonic()
        builder = build_tracking_sample if tracking else build_vln_traj_sample
        sample = builder(observed._get_video_tensor(), task,
                         observed._history_frame_ids, bundle)
        if imagegoal:
            # Qwen3-VL represents a static reference using one two-frame
            # tubelet. It never enters the real observation history/source IDs.
            target = observed._convert_frame(np.asarray(goal_image, dtype=np.uint8))
            reference_segment = {
                "video": torch.stack([target, target]), "frame_indices": [0, 0],
                "total_frames": 1, "pool_spatial": 1, "pool_mode": "avg",
            }
            sample["video_segments"] = [reference_segment] + sample["video_segments"]
            sample["conversations"][0]["value"] = (
                "Separate reference goal view (a stationary destination image, not part of the observation history): <video>. "
                "The following observations are from the navigating robot. "
                "Any pointing coordinates must refer to the most recent robot observation, not the reference goal view. "
                + sample["conversations"][0]["value"]
            )
            # Only this extension disables cache reuse: reference frame IDs must
            # never alias real observations or another goal generation.
            sample["_allow_vit_cache"] = False
        start = time.monotonic()
        self.assert_active(identity)
        with torch.inference_mode():
            if imagegoal:
                text, _ = engine.generate(sample, max_new_tokens=self.max_new_tokens)
            else:
                # Exactly the upstream BatchedTrackingService HF entry point.
                text, _ = engine.generate_from_frames(
                    observed._get_video_tensor(), task, predict_horizon=1,
                    frame_ids=list(observed._history_frame_ids),
                    max_new_tokens=self.max_new_tokens,
                    task_type="tracking" if tracking else "vlnce_traj",
                )
        self.assert_active(identity)
        waypoints, raw = self.decoder.decode_waypoints(text)
        if waypoints.shape != (10, 3) or not np.isfinite(waypoints).all():
            raise ValueError("Official RVQ decode did not produce a finite 10x3 trajectory")
        signals = decode_prediction_signals(raw, is_rvq=True, waypoints=waypoints)
        newest = frames[-1]
        history = [{"frame_id": f["frame_id"], "capture_timestamp_ns": f["capture_timestamp_ns"]} for f in frames]
        self.assert_active(identity)
        return {
            "schema_version": "lightnav.imagegoal.inference.v1", "identity": identity,
            "source_frame": history[-1], "observation_frames": history,
            "policy_history_count": len(state["frames"]), "policy_history_frames_appended": len(additions),
            "policy_history_reset_reason": reset_reason,
            "conditioning_mode": conditioning_mode, "native_imagegoal_support": False,
            "implementation": IMAGEGOAL_IMPLEMENTATION if imagegoal else "official_release",
            "official_release_commit": OFFICIAL_RELEASE_COMMIT, "official_core_sha256": OFFICIAL_CORE_DIGEST,
            "task_mode": task_mode,
            "eval_task": "trackvla" if tracking else "vlnce",
            "goal_image_consumed": imagegoal,
            "trajectory_body_m_rad": waypoints.tolist(), "trajectory_body_m": waypoints[:, :2].tolist(),
            "model_stop": signals.stop, "model_visible": signals.visible,
            "arrival_confirmed": False, "raw_text": raw,
            "pointing": pointing_payload(raw, width=newest["image"].width, height=newest["image"].height),
            "instruction": task, "inference_ms": (time.monotonic() - start) * 1000,
            "prompt": sample["conversations"][0]["value"],
            "goal_encoding": "separate_static_two_frame_video_tubelet" if imagegoal else None,
            "source_frame_intervals_s": [(b["capture_timestamp_ns"] - a["capture_timestamp_ns"]) / 1e9 for a, b in zip(frames, frames[1:])],
            "time_encoding": "official_uniform_frame_index_at_checkpoint_fps",
            "pointing_reference_frame_id": newest["frame_id"],
        }


# Import compatibility; the requested conditioning mode selects the actual path.
LightNavImageGoalPolicy = LightNavPolicy
