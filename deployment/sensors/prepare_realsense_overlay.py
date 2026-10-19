"""Extract pinned official Humble RealSense packages into a user-owned prefix.

No package database, kernel module, device firmware, service, or system file is
changed. Each download must match the configured APT metadata SHA256. The host
must already provide ROS Humble and the dependencies reported by apt-get -s.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


PACKAGES = {
    "libglfw3": "3.3.6-1",
    "libglfw3-dev": "3.3.6-1",
    "ros-humble-librealsense2": "2.58.4-1jammy.20260831.021400",
    "ros-humble-realsense2-camera-msgs": "4.58.4-1jammy.20260907.223031",
    "ros-humble-realsense2-camera": "4.58.4-1jammy.20260916.045339",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    if root.exists():
        parser.error("choose a new overlay directory; existing files are never replaced")
    if not Path("/opt/ros/humble/setup.bash").is_file():
        parser.error("host ROS Humble is required")
    root.mkdir(parents=True)
    cache, unpack = root / "debs", root / "unpacked"
    cache.mkdir()
    unpack.mkdir()
    records = []
    for name, version in PACKAGES.items():
        spec = f"{name}={version}"
        metadata_text = subprocess.check_output(["apt-cache", "show", spec], text=True)
        metadata = dict(line.split(": ", 1) for line in metadata_text.split("\n\n")[0].splitlines()
                        if ": " in line and not line.startswith(" "))
        if metadata.get("Package") != name or metadata.get("Version") != version:
            raise RuntimeError(f"APT did not return pinned package {spec}")
        subprocess.run(["apt-get", "download", spec], cwd=cache, check=True)
        path = cache / Path(metadata["Filename"]).name
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != metadata["SHA256"]:
            raise RuntimeError(f"SHA256 mismatch: {path}")
        subprocess.run(["dpkg-deb", "-x", str(path), str(unpack)], check=True)
        records.append({"package": name, "version": version, "sha256": digest,
                        "filename": path.name})
    prefix = unpack / "opt/ros/humble"
    quote = shlex.quote
    (root / "setup.bash").write_text(
        "# Official RealSense user-space overlay; no system installation.\n"
        "source /opt/ros/humble/setup.bash\n"
        f"export AMENT_PREFIX_PATH={quote(str(prefix))}:\"${{AMENT_PREFIX_PATH:-}}\"\n"
        f"export CMAKE_PREFIX_PATH={quote(str(prefix))}:\"${{CMAKE_PREFIX_PATH:-}}\"\n"
        f"export LD_LIBRARY_PATH={quote(str(prefix / 'lib'))}:{quote(str(prefix / 'lib/aarch64-linux-gnu'))}:{quote(str(unpack / 'usr/lib/aarch64-linux-gnu'))}:\"${{LD_LIBRARY_PATH:-}}\"\n"
        f"export PYTHONPATH={quote(str(prefix / 'local/lib/python3.10/dist-packages'))}:\"${{PYTHONPATH:-}}\"\n"
        f"export PATH={quote(str(prefix / 'bin'))}:\"$PATH\"\n")
    manifest = {"packages": records, "prefix": str(prefix), "setup": str(root / "setup.bash"),
                "system_install_performed": False, "services_started": False}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
