import hashlib
import json

from vln_web.observability import snapshot


def test_snapshot_captures_web_and_launch_sources_excludes_local_config(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    files = {
        "robot_deploy/src/vln_web/web/app.js": "const version = 1;",
        "robot_deploy/src/vln_web/web/index.html": "<html></html>",
        "deployment/server/start.sh": "#!/bin/bash\n",
        "config/robot.env.example": 'LIGHTNAV_SERVER_URL=""\n',
        "config/robot.env": 'PRIVATE_TEST_SETTING="local-only"\n',
    }
    for name, value in files.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    directory = tmp_path / "snapshot"
    report = snapshot(directory, repo)
    paths = {entry["path"] for entry in report["source_files"]}
    assert paths == set(files) - {"config/robot.env"}
    for entry in report["source_files"]:
        source = directory / "source" / entry["path"]
        assert hashlib.sha256(source.read_bytes()).hexdigest() == entry["sha256"]
    assert (
        json.loads((directory / "runtime.json").read_text())["source_files"]
        == report["source_files"]
    )
