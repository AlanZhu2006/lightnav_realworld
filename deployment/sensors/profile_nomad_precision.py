"""Measure real NoMaD CUDA precision modes without changing the running server.

Uses four stored genuine RGB frames, eight diffusion samples and ten denoising
steps in every mode. Precision changes are research results, not automatic
deployment approval. No camera, ROS publisher, robot API, or power setting is
touched. This imports the same checked deployment policy in a separate process.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import statistics
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--goal", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    if not 2 <= args.repeat <= 20:
        parser.error("repeat must be between 2 and 20")
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import torch
    from PIL import Image
    from deployment.nomad.policy import NoMaDPolicy, RGBFrame, InferenceIdentity
    from deployment.nomad.shadow_client import load_manifest
    _, frames = load_manifest(args.manifest)
    selected = frames[-4:]
    observations = [RGBFrame(image=Image.open(row["resolved_path"]).convert("RGB"),
                             frame_id=row["frame_id"],
                             capture_timestamp_ns=row["capture_timestamp_ns"])
                    for row in selected]
    goal = Image.open(args.goal).convert("RGB")
    identity = InferenceIdentity("precision-audit", "recorded-real-rgb", "same-view-io-only", 0,
                                 selected[-1]["frame_id"])
    root = args.project_root.expanduser().resolve()
    monitor = None
    monitor_log = (output / "tegrastats.txt").open("w")
    try:
        monitor = subprocess.Popen(["/usr/bin/tegrastats", "--interval", "200"],
                                    stdout=monitor_log, stderr=subprocess.STDOUT)
    except OSError:
        pass
    measurements = {}
    report = {"schema_version": "dejaview.nomad.precision-audit.v1", "not_for_motion": True,
              "running_server_changed": False, "power_settings_changed": False,
              "torch": torch.__version__, "cuda": torch.version.cuda,
              "device": torch.cuda.get_device_name(0), "num_samples": 8, "diffusion_steps": 10,
              "manifest": str(args.manifest.resolve()), "goal": str(args.goal.resolve())}
    try:
        policy = NoMaDPolicy(upstream=root / "third_party/visualnav_transformer",
            diffusion_policy=root / "third_party/diffusion_policy",
            checkpoint=root / "assets/nomad/nomad.pth", device="cuda",
            expected_checkpoint_sha256="70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333")
        report["parameter_devices"] = sorted({str(p.device) for p in policy.model.parameters()})
        report["parameter_count"] = sum(p.numel() for p in policy.model.parameters())
        hooks, events, active = [], [], {}
        def before(name):
            def hook(module, inputs):
                start = torch.cuda.Event(enable_timing=True)
                end = torch.cuda.Event(enable_timing=True)
                start.record()
                active[name] = (start, end, time.perf_counter())
            return hook
        def after(name):
            def hook(module, inputs, result):
                start, end, cpu_start = active.pop(name)
                end.record()
                events.append((name, start, end, (time.perf_counter() - cpu_start) * 1000))
            return hook
        for name in ("vision_encoder", "noise_pred_net", "dist_pred_net"):
            module = getattr(policy.model, name)
            hooks.extend([module.register_forward_pre_hook(before(name)),
                          module.register_forward_hook(after(name))])
        baseline = None
        for name, dtype in (("fp32", None), ("autocast_fp16", torch.float16),
                            ("autocast_bfloat16", torch.bfloat16)):
            runs = []
            error = None
            try:
                for index in range(args.repeat + 2):
                    events.clear()
                    torch.manual_seed(260806)
                    torch.cuda.synchronize()
                    began = time.perf_counter()
                    context = nullcontext() if dtype is None else torch.autocast("cuda", dtype=dtype)
                    with context:
                        result = policy.predict(observations, goal, identity=identity)
                    torch.cuda.synchronize()
                    total_ms = (time.perf_counter() - began) * 1000
                    sections = {}
                    for section, start, end, cpu_ms in events:
                        entry = sections.setdefault(section, {"calls": 0, "cuda_event_ms": 0.0,
                                                               "cpu_dispatch_elapsed_ms": 0.0})
                        entry["calls"] += 1
                        entry["cuda_event_ms"] += start.elapsed_time(end)
                        entry["cpu_dispatch_elapsed_ms"] += cpu_ms
                    if index >= 2:
                        runs.append({"wall_ms": total_ms, "policy_reported_ms": result["inference_ms"],
                                     "sections": sections})
                path = np.asarray(result["trajectory_body_m"], dtype=np.float64)
                if baseline is None:
                    baseline = path.copy()
                measurements[name] = {"runs": runs, "median_wall_ms": statistics.median(
                    row["wall_ms"] for row in runs), "trajectory_body_m": path.tolist(),
                    "distance_steps": result["distance_steps"],
                    "finite": bool(np.all(np.isfinite(path))),
                    "max_absolute_path_difference_from_fp32_m": float(np.max(np.abs(path - baseline))),
                    "deployment_equivalence_assessed": False}
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                measurements[name] = {"error": error, "runs": runs}
            print(json.dumps({"precision": name, **{k:v for k,v in measurements[name].items()
                                                    if k not in ("runs", "trajectory_body_m")}}), flush=True)
        for hook in hooks:
            hook.remove()
        report["measurements"] = measurements
        from concurrent.futures import ThreadPoolExecutor
        def predict_timed():
            torch.cuda.synchronize()
            began = time.perf_counter()
            torch.manual_seed(260806)
            result = policy.predict(observations, goal, identity=identity)
            torch.cuda.synchronize()
            return {"wall_ms": (time.perf_counter() - began) * 1000,
                    "policy_reported_ms": result["inference_ms"],
                    "torch_num_threads": torch.get_num_threads(),
                    "torch_num_interop_threads": torch.get_num_interop_threads()}
        def fresh_thread():
            result, errors = [], []
            def run():
                try:
                    result.append(predict_timed())
                except Exception as exc:
                    errors.append(f"{type(exc).__name__}: {exc}")
            worker = threading.Thread(target=run)
            worker.start()
            worker.join(timeout=60)
            if worker.is_alive() or errors or not result:
                raise RuntimeError(f"fresh thread failed: {errors}")
            return result[0]
        thread_results = {}
        thread_results["fresh_thread_per_call"] = [fresh_thread() for _ in range(args.repeat)]
        with ThreadPoolExecutor(max_workers=1) as executor:
            first = executor.submit(predict_timed).result(timeout=60)
            thread_results["persistent_single_worker_first"] = first
            thread_results["persistent_single_worker_warm"] = [
                executor.submit(predict_timed).result(timeout=60) for _ in range(args.repeat)]
        thread_results["main_thread_warm"] = [predict_timed() for _ in range(args.repeat)]
        report["fp32_thread_experiment"] = thread_results
        print(json.dumps({"fp32_thread_experiment": thread_results}), flush=True)
    finally:
        if monitor is not None:
            monitor.terminate()
            try:
                monitor.wait(timeout=5)
            except subprocess.TimeoutExpired:
                monitor.kill()
                monitor.wait(timeout=5)
        monitor_log.close()
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(output), "parameter_devices": report.get("parameter_devices")}, indent=2))


if __name__ == "__main__":
    main()
