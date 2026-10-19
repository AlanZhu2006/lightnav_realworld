"""Hash-traceable installation and clock receipts for explicit operator runs.

Receipt verification checks recorded evidence and source contracts. It cannot
measure a camera installation or network delay by itself; absent physical
measurements are a failed preflight, never fabricated defaults.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024),b""):
            digest.update(block)
    return digest.hexdigest()


def host_boot_id() -> str:
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


@dataclass(frozen=True)
class CalibrationBundle:
    robot_id: str
    mount: dict | None
    timing: dict | None
    mount_sha256: str | None
    timing_sha256: str | None
    blocking_reasons: tuple[str,...]
    commissioning: bool

    @property
    def verified(self) -> bool:
        return not self.blocking_reasons

    def summary(self) -> dict:
        return {"schema_version":"imagegoal.calibration_bundle.v1","verified":self.verified,
                "robot_id":self.robot_id,"mount_sha256":self.mount_sha256,"timing_sha256":self.timing_sha256,
                "commissioning":self.commissioning,"blocking_reasons":list(self.blocking_reasons),
                "body_frame":self.mount.get("body_frame") if self.mount else None,
                "host_boot_id":self.timing.get("host_boot_id") if self.timing else None,
                "max_alignment_error_s":self.timing.get("max_alignment_error_s") if self.timing else None}

    def correct_metadata(self, stream: str, metadata: dict) -> dict:
        """Apply the measured fixed stamp correction once, preserving raw time."""
        result = dict(metadata)
        if self.verified:
            source = self.timing["sources"][stream]
            correction_ns = int(round(source["stamp_correction_s"]*1e9))
            result.update(raw_capture_timestamp_ns=result["capture_timestamp_ns"],stamp_correction_ns=correction_ns,
                          timing_receipt_sha256=self.timing_sha256)
            result["capture_timestamp_ns"] += correction_ns
        return result

    def source_reasons(self, sensors: dict) -> list[str]:
        if not self.verified:
            return list(self.blocking_reasons)
        reasons = []
        for name in ("rgb","depth","odom"):
            item = sensors.get(name)
            if not isinstance(item,dict):
                reasons.append(f"calibration_{name}_source_missing")
                continue
            expected = self.timing["sources"][name]
            if item.get("frame_name") != expected["frame_name"] or item.get("timestamp_source") != expected["timestamp_source"]:
                reasons.append(f"calibration_{name}_source_mismatch")
            if item.get("clock_domain")!="unix_epoch_ns":
                reasons.append(f"calibration_{name}_clock_mismatch")
            if item.get("timing_receipt_sha256")!=self.timing_sha256:
                reasons.append(f"calibration_{name}_stamp_correction_missing")
        odom = sensors.get("odom") or {}
        if odom.get("body_frame_name")!=self.mount["body_frame"]:
            reasons.append("calibration_odom_body_frame_mismatch")
        if self.timing.get("host_boot_id")!=host_boot_id():
            reasons.append("calibration_host_boot_changed")
        return reasons


def _load_receipt(path, kind: str, robot_id: str, allow_commissioning: bool, reasons: list[str]):
    if path is None:
        reasons.append(f"{kind}_receipt_missing")
        return None,None
    filename = Path(path).expanduser().resolve()
    try:
        raw = filename.read_bytes()
        value = json.loads(raw)
        digest = hashlib.sha256(raw).hexdigest()
        if not isinstance(value,dict):
            raise ValueError("receipt must be an object")
        expected = "imagegoal.mount_receipt.v1" if kind=="mount" else "imagegoal.time_alignment_receipt.v1"
        if value.get("schema_version")!=expected:
            reasons.append(f"{kind}_receipt_schema_mismatch")
        if value.get("robot_id")!=robot_id or not robot_id:
            reasons.append(f"{kind}_receipt_robot_mismatch")
        if value.get("verified") is not True or not isinstance(value.get("verified_by"),str) or not value["verified_by"].strip():
            reasons.append(f"{kind}_receipt_not_verified")
        verified_at = value.get("verified_at_timestamp_ns")
        if type(verified_at) is not int or not 0<verified_at<=time.time_ns()+50_000_000:
            reasons.append(f"{kind}_receipt_verification_time_invalid")
        if not isinstance(value.get("commissioning"),bool) or (value.get("commissioning") and not allow_commissioning):
            reasons.append(f"{kind}_receipt_commissioning_not_physical")
        evidence = value.get("evidence")
        if not isinstance(evidence,list) or not 1<=len(evidence)<=16:
            reasons.append(f"{kind}_receipt_evidence_missing")
        else:
            for index,item in enumerate(evidence):
                if not isinstance(item,dict) or not isinstance(item.get("path"),str) or not isinstance(item.get("sha256"),str):
                    reasons.append(f"{kind}_evidence_{index}_invalid")
                    continue
                evidence_file = (filename.parent/item["path"]).resolve(strict=True)
                if sha256_file(evidence_file)!=item["sha256"]:
                    reasons.append(f"{kind}_evidence_{index}_sha256_mismatch")
        return value,digest
    except Exception as exc:
        reasons.append(f"{kind}_receipt_unreadable:{type(exc).__name__}")
        return None,None


def load_calibration(mount_path, timing_path, robot_id: str, *, allow_commissioning: bool = False) -> CalibrationBundle:
    reasons = []
    mount,mount_digest = _load_receipt(mount_path,"mount",robot_id,allow_commissioning,reasons)
    timing,timing_digest = _load_receipt(timing_path,"timing",robot_id,allow_commissioning,reasons)
    if mount:
        if mount.get("camera_heading_aligned_to_body") is not True:
            reasons.append("mount_camera_heading_not_verified")
        for key in ("body_frame","rgb_frame","odom_frame"):
            if not isinstance(mount.get(key),str) or not mount[key].strip():
                reasons.append(f"mount_{key}_missing")
        try:
            transform = np.asarray(mount.get("T_body_rgb"),dtype=float)
            valid = (transform.shape==(4,4) and np.isfinite(transform).all() and
                     np.allclose(transform[3],[0,0,0,1],atol=1e-7) and
                     np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5) and
                     math.isclose(float(np.linalg.det(transform[:3,:3])),1.,abs_tol=1e-5))
            if not valid:
                reasons.append("mount_extrinsic_not_rigid_SE3")
            elif transform[0,2]<=0:
                # RGB optical +z must face into the forward body hemisphere;
                # the recorded heading check handles the measured tolerance.
                reasons.append("mount_optical_forward_not_body_forward")
        except (TypeError,ValueError):
            reasons.append("mount_extrinsic_invalid")
    if timing:
        error = timing.get("max_alignment_error_s")
        if type(error) not in (int,float) or not math.isfinite(error) or not 0<=error<=.05:
            reasons.append("timing_alignment_error_unverified_or_above_50ms")
        if timing.get("host_boot_id")!=host_boot_id() or timing.get("clock_domain")!="unix_epoch_ns":
            reasons.append("timing_host_boot_or_clock_mismatch")
        sources = timing.get("sources")
        if not isinstance(sources,dict):
            reasons.append("timing_sources_missing")
        else:
            for name in ("rgb","depth","odom"):
                item = sources.get(name)
                if not isinstance(item,dict):
                    reasons.append(f"timing_{name}_source_missing")
                    continue
                for key in ("frame_name","timestamp_source"):
                    if not isinstance(item.get(key),str) or not item[key].strip():
                        reasons.append(f"timing_{name}_{key}_missing")
                correction = item.get("stamp_correction_s")
                if type(correction) not in (int,float) or not math.isfinite(correction) or abs(correction)>.5:
                    reasons.append(f"timing_{name}_correction_invalid")
                if mount and item.get("frame_name")!=mount.get("odom_frame" if name=="odom" else "rgb_frame"):
                    reasons.append(f"timing_{name}_mount_frame_mismatch")
    commissioning = bool((mount or {}).get("commissioning") or (timing or {}).get("commissioning"))
    return CalibrationBundle(robot_id,mount,timing,mount_digest,timing_digest,tuple(dict.fromkeys(reasons)),commissioning)
