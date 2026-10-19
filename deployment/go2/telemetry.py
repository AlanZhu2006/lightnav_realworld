"""Read-only SDK2 SportModeState/LowState to ROS telemetry.

DDS initialization happens only in main after link preflight succeeds. This
module has no SportClient import, command publisher, service call, or motion
API. --offline-audit and --import-check never initialize DDS.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import struct
import threading
import time

from .preflight import load_config, preflight


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


class TelemetryBuffer:
    """Pure source-progress and clock checks, reusable without ROS or SDK."""
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.poses = deque(maxlen=6)
        self.sport = None
        self.lowstate = None
        self.source_samples = 0
        self.last_source_stamp_ns = 0
        self.last_low_tick = None
        self.low_samples = 0
        self.latched_fault = None
        self.previous_receive = None

    def on_sport(self, message, wall_ns=None, mono_ns=None):
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        mono_ns = time.monotonic_ns() if mono_ns is None else mono_ns
        stamp = int(message.stamp.sec) * 1_000_000_000 + int(message.stamp.nanosec)
        x, y, yaw = float(message.position[0]), float(message.position[1]), float(message.imu_state.rpy[2])
        with self.lock:
            if self.latched_fault or stamp <= 0 or not _finite((x, y, yaw)):
                return None
            if stamp < self.last_source_stamp_ns:
                self.latched_fault = "sport_source_clock_reset_requires_new_telemetry_and_task"
                return None
            if stamp == self.last_source_stamp_ns:
                return None
            if self.previous_receive:
                prev_wall, prev_mono = self.previous_receive
                if abs((wall_ns - prev_wall) - (mono_ns - prev_mono)) > 200_000_000:
                    self.latched_fault = "host_clock_step_exceeds_200ms"
                    return None
            self.previous_receive = (wall_ns, mono_ns)
            self.last_source_stamp_ns = stamp
            self.source_samples += 1
            self.poses.append((stamp, x, y, yaw))
            velocity = [0.0, 0.0, 0.0]
            velocity_valid = len(self.poses) >= 3
            if velocity_valid:
                first = self.poses[0]
                dt = (stamp - first[0]) / 1e9
                if not 0.001 <= dt <= 0.5:
                    velocity_valid = False
                else:
                    yaw_delta = math.atan2(math.sin(yaw-first[3]), math.cos(yaw-first[3]))
                    middle_yaw = first[3] + yaw_delta / 2
                    dx, dy = x-first[1], y-first[2]
                    velocity = [(dx*math.cos(middle_yaw)+dy*math.sin(middle_yaw))/dt,
                                (-dx*math.sin(middle_yaw)+dy*math.cos(middle_yaw))/dt,
                                yaw_delta/dt]
            self.sport = {"source_stamp_ns": stamp, "receive_timestamp_ns": wall_ns,
                          "received_monotonic_ns": mono_ns, "pose_m_rad": [x,y,yaw],
                          "velocity_body_vx_vy_wz": velocity, "velocity_valid": velocity_valid,
                          "mode": int(getattr(message, "mode", -1)),
                          "gait_type": int(getattr(message, "gait_type", -1))}
            return dict(self.sport) if velocity_valid else None

    def on_lowstate(self, message, wall_ns=None, mono_ns=None):
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        mono_ns = time.monotonic_ns() if mono_ns is None else mono_ns
        with self.lock:
            tick = getattr(message, "tick", None)
            if tick is None:
                return False
            tick = int(tick)
            if self.last_low_tick is not None:
                progression = (tick - self.last_low_tick) & 0xffffffff
                if progression == 0:
                    return False
                if progression >= 0x80000000:
                    self.latched_fault = "lowstate_source_tick_reset_requires_new_telemetry_and_task"
                    return False
            self.last_low_tick = tick
            try:
                raw = bytes(message.wireless_remote)
                axes = [struct.unpack("<f", raw[start:start+4])[0] for start in (4,8,12,20)]
                axes_valid = _finite(axes) and all(abs(axis) <= 2 for axis in axes)
            except (AttributeError, ValueError, TypeError, struct.error):
                axes, axes_valid = [], False
            self.low_samples += 1
            self.lowstate = {"source_tick": tick, "receive_timestamp_ns": wall_ns,
                             "received_monotonic_ns": mono_ns, "remote_axes": axes,
                             "remote_axes_valid": bool(axes_valid)}
            return True

    def status(self, now_mono_ns=None):
        now = time.monotonic_ns() if now_mono_ns is None else now_mono_ns
        with self.lock:
            sport, low = self.sport or {}, self.lowstate or {}
            sport_age = (now-sport.get("received_monotonic_ns",0))/1e9
            low_age = (now-low.get("received_monotonic_ns",0))/1e9
            ready = (self.source_samples >= 3 and self.low_samples >= 2 and not self.latched_fault
                     and sport.get("velocity_valid",False) and low.get("remote_axes_valid",False)
                     and 0 <= sport_age <= self.config["feedback_max_age_s"]
                     and 0 <= low_age <= self.config["feedback_max_age_s"])
            return {"schema_version":"dejaview.go2.telemetry.v1", "timestamp_ns":time.time_ns(),
                    "boot_id":Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                    "synthetic":False, "read_only":True, "motion_api_imported":False,
                    "net_if":self.config["net_if"], "ready":bool(ready),
                    "source_samples":self.source_samples, "lowstate_samples":self.low_samples,
                    "sport_received_monotonic_ns":sport.get("received_monotonic_ns",0),
                    "lowstate_received_monotonic_ns":low.get("received_monotonic_ns",0),
                    "sport_age_s":sport_age,"lowstate_age_s":low_age,
                    "remote_axes_valid":low.get("remote_axes_valid",False),
                    "latched_fault":self.latched_fault,"sport":sport,"lowstate":low,
                    "timestamp_source":"go2_dds_receive_host_system_time_not_sensor_source_time",
                    "velocity_source":"derived_body_velocity_from_sport_pose_and_source_timestamp",
                    "capture_pose_clock_alignment_verified":False}


def offline_audit():
    from types import SimpleNamespace as NS
    config = load_config()
    buffer = TelemetryBuffer(config)
    start_wall, start_mono = time.time_ns(), time.monotonic_ns()
    last = None
    for index in range(8):
        source_ns = 1_000_000_000 + index*20_000_000
        message = NS(stamp=NS(sec=source_ns//1_000_000_000,nanosec=source_ns%1_000_000_000),
                     position=[index*0.002,0,0],imu_state=NS(rpy=[0,0,0]),mode=1,gait_type=0)
        last = buffer.on_sport(message,start_wall+index*20_000_000,start_mono+index*20_000_000)
        buffer.on_lowstate(NS(tick=index,wireless_remote=bytes(40)),
                           start_wall+index*20_000_000,start_mono+index*20_000_000)
    snapshot = buffer.status(start_mono+140_000_000)
    duplicate = buffer.on_sport(message,start_wall+150_000_000,start_mono+150_000_000)
    stale = buffer.status(start_mono+1_000_000_000)
    message.stamp.sec = 0
    message.stamp.nanosec = 100
    reset = buffer.on_sport(message,start_wall+160_000_000,start_mono+160_000_000)
    checks = {"fake_transport_no_sdk":True,"pose_velocity_finite":bool(last and _finite(last["velocity_body_vx_vy_wz"])),
              "derived_forward_velocity":bool(last and abs(last["velocity_body_vx_vy_wz"][0]-.1)<1e-9),
              "progressing_sources_ready":snapshot["ready"],"duplicate_not_fresh":duplicate is None,
              "stale_not_ready":not stale["ready"],"reset_latches_fault":reset is None and bool(buffer.latched_fault)}
    print(json.dumps({"offline_diagnostic":True,"synthetic":True,"checks":checks,
                      "dds_initialized":False,"motion_sent":False},indent=2))
    return all(checks.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--status-file",type=Path,default=Path(".runtime/go2/telemetry.json"))
    parser.add_argument("--duration-s",type=float,default=30)
    parser.add_argument("--watch",action="store_true")
    parser.add_argument("--import-check",action="store_true")
    parser.add_argument("--offline-audit",action="store_true")
    args = parser.parse_args()
    if args.offline_audit:
        raise SystemExit(0 if offline_audit() else 1)
    config = load_config(args.config)
    if args.import_check:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, LowState_
        print(json.dumps({"sdk_imports":True,"dds_initialized":False,"sport_client_imported":False}))
        return
    if args.duration_s <= 0 or not math.isfinite(args.duration_s):
        parser.error("duration must be positive and finite")
    report = preflight(config)
    if not report["ok"]:
        print(json.dumps(report,indent=2))
        raise SystemExit(2)
    # Imports and participant creation are intentionally beyond all early exits.
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, LowState_
    import sys
    vendor = Path(__file__).parent / "vendor/gem"
    sys.path.insert(0,str(vendor))
    from go2_battery_monitor import sample_from_low_state, battery_message
    buffer = TelemetryBuffer(config)
    rclpy.init()
    node = Node("dejaview_go2_readonly_telemetry")
    odom_pub = node.create_publisher(Odometry,config["odom_topic"],10)
    battery_pub = node.create_publisher(BatteryState,config["battery_topic"],10)
    status_pub = node.create_publisher(String,config["status_topic"],10)
    battery = [None]
    args.status_file.parent.mkdir(parents=True,exist_ok=True)
    def on_sport(message):
        sample = buffer.on_sport(message)
        if sample is None:
            return
        odom = Odometry()
        stamp = sample["receive_timestamp_ns"]
        odom.header.stamp.sec,odom.header.stamp.nanosec = divmod(stamp,1_000_000_000)
        odom.header.frame_id = config["odom_frame"]
        odom.child_frame_id = config["body_frame"]
        x,y,yaw = sample["pose_m_rad"]
        odom.pose.pose.position.x,odom.pose.pose.position.y = x,y
        odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        vx,vy,wz = sample["velocity_body_vx_vy_wz"]
        odom.twist.twist.linear.x,odom.twist.twist.linear.y = vx,vy
        odom.twist.twist.angular.z = wz
        # Pose uncertainty has not been measured; zero does not claim certainty.
        # Retain the original GEM zero/unspecified covariance convention.
        # The telemetry status explicitly does not certify pose uncertainty.
        odom_pub.publish(odom)
    def on_low(message):
        if buffer.on_lowstate(message):
            battery[0] = sample_from_low_state(message)
    def publish_status():
        report = buffer.status()
        text = json.dumps(report,allow_nan=False)
        msg = String(); msg.data = text; status_pub.publish(msg)
        temporary = args.status_file.with_suffix(".tmp")
        temporary.write_text(text+"\n"); temporary.replace(args.status_file)
        msg = battery_message(battery[0],now_monotonic=time.monotonic(),offline_timeout_s=.5)
        msg.header.stamp = node.get_clock().now().to_msg(); battery_pub.publish(msg)
    ChannelFactoryInitialize(config["dds_domain_id"],config["net_if"])
    sport = ChannelSubscriber(config["sport_topic"],SportModeState_)
    low = ChannelSubscriber(config["lowstate_topic"],LowState_)
    sport.Init(on_sport,10); low.Init(on_low,10)
    node.create_timer(.1,publish_status)
    began = time.monotonic()
    try:
        while rclpy.ok() and (args.watch or time.monotonic()-began < args.duration_s):
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        for subscriber in (sport,low):
            subscriber.Close()
        buffer.latched_fault = "telemetry_stopped"
        publish_status()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
