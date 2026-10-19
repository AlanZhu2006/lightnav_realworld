"""Identity-bound command lease and independently guarded Sport transport."""
from __future__ import annotations

import threading
import time
import math
import json

from .execution_envelope import validate_envelope, validate_commissioning_candidate


class CommandLease:
    def __init__(self,binding,config,*,commissioning=False,wall_clock=time.time_ns,mono_clock=time.monotonic_ns):
        self.binding,self.config,self.commissioning = dict(binding),config,commissioning
        self.lock = threading.RLock()
        self.decision = None
        self.received_monotonic_ns = 0
        self.evaluated_timestamp_ns = 0
        self.accepted_sequence = -1
        self.accepted_evaluation_signature = None
        self.last_reason = "no_authorized_envelope"
        self.latched_fault = None
        self.wall_clock,self.mono_clock = wall_clock,mono_clock
        self.minimum_post_stop_source_ns = 0
        self.observed_generation = {key:binding.get(key) for key in ("goal_id","goal_version","epoch")}

    def invalidate(self,reason,*,latch=False):
        with self.lock:
            if self.decision is not None:
                self.minimum_post_stop_source_ns = max(self.minimum_post_stop_source_ns,self.wall_clock())
            self.decision = None
            self.last_reason = reason
            if latch: self.latched_fault = reason

    def mark_stop_dispatched(self):
        """Fence source time after successful zero dispatch, not standstill."""
        with self.lock:
            self.minimum_post_stop_source_ns = max(self.minimum_post_stop_source_ns,self.wall_clock())
            self.decision = None
            self.last_reason = "waiting_plan_captured_after_zero_dispatch"

    @staticmethod
    def _evaluation_signature(envelope):
        """Bind immutable controller output, excluding live sensor/status ages.

        A status poll may report newer sensor observations without a new MPC
        evaluation. Those observations are validated independently below; they
        cannot renew the accepted controller command or its lease.
        """
        fields = ("schema_version","session_id","task_id","goal_id","goal_version","epoch",
                  "source_stamp_ns","request_id","evaluated_timestamp_ns","expires_timestamp_ns","sequence",
                  "command","robot_id","calibration","transport_authority","authority_schema",
                  "execution_mode","not_for_motion","synthetic","commissioning")
        return json.dumps({key:envelope.get(key) for key in fields},sort_keys=True,allow_nan=False)

    def submit(self,envelope,*,now_ns=None,mono_ns=None,runtime_state=None):
        now = self.wall_clock() if now_ns is None else now_ns
        mono = self.mono_clock() if mono_ns is None else mono_ns
        validate = validate_commissioning_candidate if self.commissioning else validate_envelope
        with self.lock:
            if self.latched_fault:
                return False
            state = runtime_state or envelope.get("task_state")
            if state in {"CANCELLED","FAILED","TIMED_OUT","SUCCEEDED"}:
                self.invalidate(f"operator_run_terminal:{state}",latch=True)
                return False
            if any(envelope.get(key)!=self.binding.get(key) for key in ("session_id","task_id")):
                self.invalidate("operator_run_session_or_task_changed",latch=True)
                return False
            generation = {key:envelope.get(key) for key in ("goal_id","goal_version","epoch")}
            if (not isinstance(generation["goal_id"],str) or not generation["goal_id"]
                    or any(type(generation[key]) is not int or generation[key]<0 for key in ("goal_version","epoch"))):
                self.invalidate("invalid_goal_generation")
                return False
            current = self.observed_generation
            if any(generation[key]<current[key] for key in ("goal_version","epoch")):
                # Late replies cannot revive an old goal or interrupt the
                # already accepted newer generation.
                self.last_reason = "discarded_old_goal_or_epoch"
                return False
            if generation["goal_version"]==current["goal_version"] and generation["goal_id"]!=current["goal_id"]:
                self.invalidate("goal_identity_changed_without_version",latch=True)
                return False
            advanced = any(generation[key]>current[key] for key in ("goal_version","epoch"))
            if advanced:
                self.invalidate("generation_advanced_waiting_post_stop_plan")
                self.observed_generation = generation
            evaluated,sequence = envelope.get("evaluated_timestamp_ns"),envelope.get("sequence")
            if type(evaluated) is not int or type(sequence) is not int or sequence<0:
                self.last_reason = "invalid_controller_evaluation_identity"
                return False
            if evaluated<self.evaluated_timestamp_ns or sequence<self.accepted_sequence:
                self.last_reason = "discarded_old_controller_evaluation"
                return False
            try:
                signature = self._evaluation_signature(envelope)
            except (TypeError,ValueError):
                self.last_reason = "invalid_controller_evaluation_signature"
                return False
            repeated = evaluated==self.evaluated_timestamp_ns or sequence==self.accepted_sequence
            if repeated and (evaluated!=self.evaluated_timestamp_ns or sequence!=self.accepted_sequence
                             or signature!=self.accepted_evaluation_signature):
                # Reject a conflicting replay without rolling back or stopping
                # a newer already-accepted command. Its original lease remains
                # bounded and is never refreshed by this packet.
                self.last_reason = "discarded_conflicting_controller_evaluation"
                return False
            trial_binding = {**self.binding,**self.observed_generation}
            result = validate(envelope,binding=trial_binding,config=self.config,now_ns=now)
            if not result.accepted:
                hard = any(reason in {"binding_session_id_mismatch","binding_task_id_mismatch","binding_robot_id_mismatch",
                                      "calibration_receipt_binding_mismatch"}
                           for reason in result.reasons)
                self.invalidate(";".join(result.reasons),latch=hard)
                return False
            if envelope["source_stamp_ns"] <= self.minimum_post_stop_source_ns:
                self.invalidate("waiting_new_plan_captured_after_stop")
                return False
            if repeated:
                # 20 Hz polling of a 10 Hz controller normally sees each
                # evaluation twice. An identical read is harmless, but must
                # not extend either reception age or producer expiry.
                valid = self.current(now_ns=now,mono_ns=mono) is not None
                if valid: self.last_reason = "same_controller_evaluation_lease_preserved"
                return valid
            self.decision = result
            self.binding.update(self.observed_generation)
            self.evaluated_timestamp_ns = evaluated
            self.accepted_sequence = sequence
            self.accepted_evaluation_signature = signature
            self.received_monotonic_ns = mono
            self.last_reason = "authorized_commissioning_fake_only" if self.commissioning else "authorized_operator_run"
            return True

    def current(self,*,now_ns=None,mono_ns=None):
        now = self.wall_clock() if now_ns is None else now_ns
        mono = self.mono_clock() if mono_ns is None else mono_ns
        with self.lock:
            decision = self.decision
            if self.latched_fault or decision is None:
                return None
            if not 0 <= mono-self.received_monotonic_ns <= 250_000_000:
                self.invalidate("envelope_receive_watchdog_expired")
                return None
            if now >= decision.expires_timestamp_ns or now < self.evaluated_timestamp_ns-50_000_000:
                self.invalidate("execution_lease_expired_or_clock_regressed")
                return None
            return decision


class CheckedSportTransport:
    """Final SDK boundary rechecks the lease inside each serialized Move call.

    A separate watchdog can stop while the ROS executor is idle. All RPCs are
    serialized, and the real SDK timeout is configured to 0.20 s by the caller.
    A failed RPC latches the lease; a fresh model response cannot clear it.
    """
    def __init__(self,client,lease):
        if lease.commissioning and getattr(client,"is_fake_transport",False) is not True:
            raise ValueError("commissioning lease requires an explicitly fake transport")
        self.client,self.lease = client,lease
        self.lock = threading.RLock()
        self.command_active = False
        self.remote_active = lambda: False
        self.stop_event = threading.Event()
        self.watchdog = None
        self.last_error = None

    def Move(self,vx,vy,wz):
        with self.lock:
            limits = (self.lease.config["max_vx_m_s"],self.lease.config["max_vy_m_s"],self.lease.config["max_wz_rad_s"])
            if not all(math.isfinite(v) and abs(v)<=limit+1e-12 for v,limit in zip((vx,vy,wz),limits)):
                self.lease.invalidate("final_sdk_velocity_invalid",latch=True)
                self.safe_stop("final_sdk_velocity_invalid")
                return -1
            moving = any(value != 0 for value in (vx,vy,wz))
            was_active = self.command_active
            if moving and (self.lease.current() is None or self.remote_active()):
                if self.remote_active(): self.lease.invalidate("hand_controller_takeover")
                self.safe_stop("final_move_authority_or_remote_rejected")
                return -1
            if moving: self.command_active = True
            try:
                code = self.client.Move(vx,vy,wz)
            except Exception as exc:
                self.last_error = f"SportClient.Move raised {type(exc).__name__}: {exc}"
                self.lease.invalidate(self.last_error,latch=True)
                raise
            if code != 0:
                self.last_error = f"SportClient.Move returned {code}"
                self.lease.invalidate(self.last_error,latch=True)
                raise RuntimeError(self.last_error)
            self.command_active = moving
            if was_active and not moving: self.lease.mark_stop_dispatched()
            return code

    def StopMove(self):
        with self.lock:
            code = self.client.StopMove()
            if code != 0:
                self.last_error = f"SportClient.StopMove returned {code}"
                self.lease.invalidate(self.last_error,latch=True)
            return code

    def safe_stop(self,reason):
        with self.lock:
            if not self.command_active:
                return
            try:
                code = self.client.Move(0.,0.,0.)
                if code != 0: raise RuntimeError(f"zero Move returned {code}")
                self.command_active = False
                self.lease.mark_stop_dispatched()
                code = self.client.StopMove()
                if code != 0: raise RuntimeError(f"StopMove returned {code}")
            except Exception as exc:
                self.last_error = f"{reason}: {type(exc).__name__}: {exc}"
                self.lease.invalidate(self.last_error,latch=True)

    def watchdog_once(self):
        remote = self.remote_active()
        if remote: self.lease.invalidate("hand_controller_takeover")
        if self.lease.current() is None or remote:
            self.safe_stop("independent_authority_watchdog")

    def start_watchdog(self):
        def run():
            while not self.stop_event.wait(.02): self.watchdog_once()
        self.watchdog = threading.Thread(target=run,name="go2-independent-watchdog",daemon=True)
        self.watchdog.start()

    def close(self):
        self.stop_event.set()
        self.lease.invalidate("transport_closed",latch=True)
        self.safe_stop("transport_closed")
        if self.watchdog is not None: self.watchdog.join(timeout=1)


def make_validated_gem_bridge(gem_module):
    class ValidatedGEMBridge(gem_module.Go2CmdBridge):
        def __init__(self,transport,lease):
            self.lease = lease
            super().__init__(transport)
            transport.remote_active = self.remote_is_active

        def on_cmd_vel(self,_message):
            # The inherited ROS Twist subscription is deliberately inert.
            # Only the envelope path below can call the original method.
            return

        def on_operator_stop(self,message):
            self.lease.invalidate("operator_STOP",latch=True)
            super().on_operator_stop(message)

        def publish_to_go2(self):
            decision = self.lease.current()
            if decision is None:
                self.latest_cmd = None
            else:
                twist = gem_module.Twist()
                twist.linear.x,twist.linear.y,twist.angular.z = decision.command_vx_vy_wz
                gem_module.Go2CmdBridge.on_cmd_vel(self,twist)
            super().publish_to_go2()
    return ValidatedGEMBridge
