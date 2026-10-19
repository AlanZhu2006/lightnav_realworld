"""Read-only LightNav environment/source/asset preflight; never loads robot APIs."""
from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path
import shutil
import sys

from .policy import MODEL_REVISION, MODEL_SHA256, verify_sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", type=Path, default=Path(__file__).parent / ".runtime/weights")
    args = parser.parse_args()
    result = {"python": sys.version.split()[0], "source_files_verified": len(verify_sources()["files"]),
              "backend": "hf", "robot_sdk_imported": False, "packages": {}, "errors": []}
    if sys.version_info[:2] != (3, 11):
        result["errors"].append("Python 3.11 is required")
    for name in ("torch", "torchvision", "transformers", "tokenizers", "numpy", "safetensors"):
        try:
            result["packages"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result["errors"].append(f"Missing package: {name}")
    if result["packages"].get("transformers") != "5.8.0":
        result["errors"].append("Transformers must be 5.8.0")
    torch_version = result["packages"].get("torch")
    if torch_version and tuple(int(x) for x in torch_version.split("+")[0].split(".")[:2]) < (2, 9):
        result["errors"].append("Torch >=2.9 is required")
    try:
        import torch
        result["cuda_available"] = torch.cuda.is_available()
        if not result["cuda_available"]:
            result["errors"].append("CUDA device unavailable")
        else:
            result["gpu"] = torch.cuda.get_device_name()
            result["cuda_build"] = torch.version.cuda
            result["compute_capability"] = torch.cuda.get_device_capability()
            value = torch.ones(8, device="cuda")
            assert float((value @ value).cpu()) == 8.0
            result["cuda_kernel_check"] = True
    except Exception as exc:
        result["errors"].append(f"CUDA import/kernel check failed: {exc}")
    manifest = args.model_path / "SOURCE_MANIFEST.json"
    if manifest.exists():
        value = json.loads(manifest.read_text())
        model = next((x for x in value["files"] if x["path"].endswith(".safetensors")), {})
        result["model_manifest_valid"] = value.get("revision") == MODEL_REVISION and model.get("sha256") == MODEL_SHA256
        if not result["model_manifest_valid"]:
            result["errors"].append("Model manifest does not identify the pinned official checkpoint")
        for item in value["files"]:
            path = args.model_path / item["path"]
            if not path.is_file() or path.stat().st_size != item["size"]:
                result["errors"].append(f"Missing/truncated model asset: {item['path']}")
    else:
        result["errors"].append("Integrity-checked model manifest not yet present")
    result["model_note"] = "Serving rehashes every model file before loading; this preflight inspects the manifest"
    existing = next(path for path in (args.model_path, *args.model_path.parents) if path.exists())
    result["free_disk_bytes"] = shutil.disk_usage(existing).free
    result["ok"] = not result["errors"]
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["ok"] else 1)


if __name__ == "__main__":
    main()
