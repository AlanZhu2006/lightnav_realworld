"""Read-only Go2 link/feedback checks performed before DDS or SportClient init."""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
from pathlib import Path
import subprocess
import time


def load_config(path=None):
    config = json.loads(Path(path or Path(__file__).with_name("config.json")).read_text())
    if config.get("schema_version") != "dejaview.go2.config.v1":
        raise ValueError("unexpected Go2 config schema")
    if not isinstance(config.get("net_if"), str) or not config["net_if"].strip():
        raise ValueError("net_if must be explicit")
    ipaddress.ip_address(config["robot_ip"])
    if config.get("bridge_enabled") is not False:
        raise ValueError("deployment default bridge_enabled must remain false")
    for name in ("feedback_max_age_s","max_vx_m_s","max_vy_m_s","max_wz_rad_s"):
        value = config.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
    return config


def _run(command, timeout=4):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        return {"command": command, "returncode": result.returncode,
                "stdout": result.stdout[-8000:], "stderr": result.stderr[-2000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "returncode": -1, "error": str(exc)}


def check_link(config):
    checks, details = {}, {}
    address = _run(["ip", "-j", "address", "show", "dev", config["net_if"]])
    details["address"] = address
    try:
        interface = json.loads(address["stdout"])[0]
        checks["interface_up"] = "UP" in interface["flags"] and "LOWER_UP" in interface["flags"]
        target = ipaddress.ip_address(config["robot_ip"])
        checks["same_subnet"] = any(target in ipaddress.ip_network(
            f"{row['local']}/{row['prefixlen']}", strict=False)
            for row in interface["addr_info"] if row["family"] == "inet")
    except (KeyError, ValueError, IndexError, TypeError):
        checks.update(interface_up=False, same_subnet=False)
    route = _run(["ip", "-j", "route", "get", config["robot_ip"]])
    details["route"] = route
    try:
        checks["route_uses_expected_interface"] = json.loads(route["stdout"])[0]["dev"] == config["net_if"]
    except (KeyError, ValueError, IndexError, TypeError):
        checks["route_uses_expected_interface"] = False
    ping = _run(["ping", "-n", "-c", "1", "-W", "1", "-I", config["net_if"], config["robot_ip"]])
    details["ping"] = ping
    checks["robot_ping"] = ping["returncode"] == 0
    return checks, details


def check_feedback(path, config):
    try:
        report = json.loads(Path(path).expanduser().read_text())
        now_ns = time.monotonic_ns()
        # Age derives from the recorded reception, not the heartbeat write.
        sport_age = (now_ns - report["sport_received_monotonic_ns"]) / 1e9
        low_age = (now_ns - report["lowstate_received_monotonic_ns"]) / 1e9
        checks = {
            "feedback_schema": report.get("schema_version") == "dejaview.go2.telemetry.v1",
            "feedback_same_machine": report.get("boot_id") == Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "feedback_source_real": report.get("synthetic") is False,
            "feedback_interface": report.get("net_if") == config["net_if"],
            "feedback_sport_fresh": 0 <= sport_age <= config["feedback_max_age_s"],
            "feedback_lowstate_fresh": 0 <= low_age <= config["feedback_max_age_s"],
            "feedback_progressing": report.get("source_samples", 0) >= 3,
            "feedback_no_latched_fault": not report.get("latched_fault"),
            "feedback_ready": report.get("ready") is True,
            "remote_priority_ready": report.get("remote_axes_valid") is True,
            "remote_sticks_idle": bool(report.get("lowstate",{}).get("remote_axes")) and all(
                abs(axis) <= .12 for axis in report.get("lowstate",{}).get("remote_axes",[])),
        }
        return checks, {"status_file": str(Path(path).expanduser()), "sport_age_s": sport_age,
                        "lowstate_age_s": low_age, "status": report}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {"feedback_available": False}, {"status_file": str(path), "error": str(exc)}


def preflight(config, feedback_status=None):
    checks, details = check_link(config)
    if feedback_status is not None:
        feedback_checks, feedback_details = check_feedback(feedback_status, config)
        checks.update(feedback_checks)
        details["feedback"] = feedback_details
    blocked = [name for name, ok in checks.items() if not ok]
    return {"schema_version": "dejaview.go2.preflight.v1", "timestamp_ns": time.time_ns(),
            "ok": not blocked, "blocked": blocked, "checks": checks, "details": details,
            "net_if": config["net_if"], "robot_ip": config["robot_ip"],
            "dds_initialized": False, "sport_client_initialized": False,
            "motion_sent": False, "navigation_validated": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--feedback-status", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = preflight(load_config(args.config), args.feedback_status)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["ok"] else 2)


if __name__ == "__main__":
    main()
