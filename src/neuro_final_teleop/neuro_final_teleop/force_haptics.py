"""Force/torque haptics and safety helpers for the NeuroFinal teleop node."""

import json
import math
import time
from typing import List, Optional, Tuple

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from neuro_final_teleop.control.ft_guard import FTDataState, StalePolicy
from neuro_final_teleop.input.gamepad import InputSnapshot
from neuro_final_teleop.control.math_utils import clamp, tool_z_axis_from_rpy_deg
from neuro_final_teleop.control.state_machine import TeleopState


class ForceHapticsMixin:
    def _ft_force_ratio(self) -> float:
        r = self.ft_guard.last_reading
        cfg = self.ft_guard.cfg
        force_norm = math.sqrt(r.fx ** 2 + r.fy ** 2 + r.fz ** 2)
        ratios = [
            abs(r.fx) / max(cfg.fx_limit_n, 1e-6),
            abs(r.fy) / max(cfg.fy_limit_n, 1e-6),
            abs(r.fz) / max(cfg.fz_limit_n, 1e-6),
            force_norm / max(cfg.force_norm_limit_n, 1e-6),
        ]
        return max(ratios)

    def _reset_v7_sensor_lpf(self) -> None:
        self._v7_sensor_lpf_fx = 0.0
        self._v7_sensor_lpf_fy = 0.0
        self._v7_sensor_lpf_fz = 0.0
        self._v7_sensor_lpf_mx = 0.0
        self._v7_sensor_lpf_my = 0.0
        self._v7_sensor_lpf_mz = 0.0
        self._v7_sensor_lpf_initialized = False
        self._v7_sensor_lpf_last_sample_stamp_s = 0.0

    def _ft_relative_values_from_reading(self) -> Tuple[List[float], List[float]]:
        r = self.ft_guard.last_reading
        z = self._v7_ft_haptic_zero
        if z is None:
            z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        f_rel = [float(r.fx) - z[0], float(r.fy) - z[1], float(r.fz) - z[2]]
        t_rel = [float(r.tx) - z[3], float(r.ty) - z[4], float(r.tz) - z[5]]
        return f_rel, t_rel

    def _update_v7_sensor_lpf(self, now_s: float) -> Optional[dict]:
        """Continuously filter FT data for measurement/debug, not haptic gating."""
        self._refresh_ft_fallback(now_s)
        data_state = self.ft_guard.data_state(now_s)
        r = self.ft_guard.last_reading
        f_rel, t_rel = self._ft_relative_values_from_reading()
        sample_stamp_s = float(r.stamp_s)
        ft_fresh = data_state == FTDataState.FRESH

        if ft_fresh and sample_stamp_s != self._v7_sensor_lpf_last_sample_stamp_s:
            alpha = self.v7_ft_sensor_lpf_alpha
            if not self._v7_sensor_lpf_initialized:
                self._v7_sensor_lpf_fx, self._v7_sensor_lpf_fy, self._v7_sensor_lpf_fz = f_rel
                self._v7_sensor_lpf_mx, self._v7_sensor_lpf_my, self._v7_sensor_lpf_mz = t_rel
                self._v7_sensor_lpf_initialized = True
            else:
                self._v7_sensor_lpf_fx = alpha * f_rel[0] + (1.0 - alpha) * self._v7_sensor_lpf_fx
                self._v7_sensor_lpf_fy = alpha * f_rel[1] + (1.0 - alpha) * self._v7_sensor_lpf_fy
                self._v7_sensor_lpf_fz = alpha * f_rel[2] + (1.0 - alpha) * self._v7_sensor_lpf_fz
                self._v7_sensor_lpf_mx = alpha * t_rel[0] + (1.0 - alpha) * self._v7_sensor_lpf_mx
                self._v7_sensor_lpf_my = alpha * t_rel[1] + (1.0 - alpha) * self._v7_sensor_lpf_my
                self._v7_sensor_lpf_mz = alpha * t_rel[2] + (1.0 - alpha) * self._v7_sensor_lpf_mz
            self._v7_sensor_lpf_last_sample_stamp_s = sample_stamp_s

        raw_force_norm = math.sqrt(f_rel[0] * f_rel[0] + f_rel[1] * f_rel[1] + f_rel[2] * f_rel[2])
        filtered_force_norm = math.sqrt(
            self._v7_sensor_lpf_fx * self._v7_sensor_lpf_fx
            + self._v7_sensor_lpf_fy * self._v7_sensor_lpf_fy
            + self._v7_sensor_lpf_fz * self._v7_sensor_lpf_fz
        )
        raw_sensor_force_norm = math.sqrt(r.fx * r.fx + r.fy * r.fy + r.fz * r.fz)
        return {
            "ft_fresh": bool(ft_fresh),
            "raw_sensor": {
                "Fx": float(r.fx), "Fy": float(r.fy), "Fz": float(r.fz),
                "Mx": float(r.tx), "My": float(r.ty), "Mz": float(r.tz),
            },
            "raw_relative_sensor": {
                "Fx": f_rel[0], "Fy": f_rel[1], "Fz": f_rel[2],
                "Mx": t_rel[0], "My": t_rel[1], "Mz": t_rel[2],
            },
            "filtered_sensor": {
                "Fx": float(self._v7_sensor_lpf_fx),
                "Fy": float(self._v7_sensor_lpf_fy),
                "Fz": float(self._v7_sensor_lpf_fz),
                "Mx": float(self._v7_sensor_lpf_mx),
                "My": float(self._v7_sensor_lpf_my),
                "Mz": float(self._v7_sensor_lpf_mz),
            },
            "raw_force_norm_n": float(raw_force_norm),
            "raw_sensor_force_norm_n": float(raw_sensor_force_norm),
            "filtered_force_norm_n": float(filtered_force_norm),
            "lpf_alpha": float(self.v7_ft_sensor_lpf_alpha),
        }

    def _selected_force_from_sensor_debug(self, sensor_debug: Optional[dict]) -> float:
        """Select contact force from continuous filtered sensor values."""
        if not sensor_debug:
            return 0.0
        f = sensor_debug.get("filtered_sensor", {})
        fx = float(f.get("Fx", 0.0))
        fy = float(f.get("Fy", 0.0))
        fz = float(f.get("Fz", 0.0))
        src = self.v7_ft_haptic_force_source
        if src == "max_axis":
            return max(abs(fx), abs(fy), abs(fz))
        if src == "insertion_axis":
            try:
                code_p, pose_sdk = self.arm.get_position(is_radian=False)
            except Exception:
                code_p, pose_sdk = -1, None
            if code_p == 0 and pose_sdk:
                tool_z = tool_z_axis_from_rpy_deg(
                    float(pose_sdk[3]), float(pose_sdk[4]), float(pose_sdk[5])
                )
                return abs((fx * tool_z[0]) + (fy * tool_z[1]) + (fz * tool_z[2]))
        return math.sqrt((fx * fx) + (fy * fy) + (fz * fz))

    def _contact_gate_thresholds(self) -> Tuple[float, float, float, float]:
        if self.v7_contact_gate_enable:
            return (
                float(self.v7_contact_on_threshold_n),
                float(self.v7_contact_off_threshold_n),
                float(self.v7_contact_on_dwell_s),
                float(self.v7_contact_off_dwell_s),
            )
        full_scale = max(float(self.v7_ft_haptic_full_scale_n), 1e-6)
        on_n = self.v7_ft_haptic_deadband_n + (
            self.v7_ft_haptic_contact_on_ratio
            * max(full_scale - self.v7_ft_haptic_deadband_n, 1e-6)
        )
        off_n = self.v7_ft_haptic_deadband_n + (
            self.v7_ft_haptic_contact_off_ratio
            * max(full_scale - self.v7_ft_haptic_deadband_n, 1e-6)
        )
        if off_n > on_n:
            off_n = on_n
        return (
            float(on_n),
            float(off_n),
            float(self.v7_ft_haptic_contact_on_dwell_s),
            float(self.v7_ft_haptic_contact_off_dwell_s),
        )

    def _update_contact_gate(self, now_s: float, selected_force_n: float) -> bool:
        # When the shift gate is enabled it is the SOLE source of truth for
        # contact detection. The legacy absolute thresholds (v7_contact_on/off)
        # are not OR'd in, because on a noisy 100/150 mN FT sensor with a tool
        # mounted the absolute baseline easily crosses 0.35 N during free-space
        # motion and would falsely latch contact. The absolute force safety
        # stop is still enforced separately by _apply_global_ft_guard.
        on_n, off_n, on_dwell_s, off_dwell_s = self._contact_gate_thresholds()
        shift_delta_n = 0.0
        shift_dwell_done = False

        if self.v7_contact_shift_gate_enable:
            if not self._v7_contact_baseline_initialized:
                self._v7_contact_baseline_n = float(selected_force_n)
                self._v7_contact_baseline_initialized = True
            shift_delta_n = max(
                0.0, float(selected_force_n) - float(self._v7_contact_baseline_n)
            )
            shift_candidate_raw = shift_delta_n >= self.v7_contact_shift_threshold_n
            if shift_candidate_raw:
                if self._v7_ft_contact_shift_candidate_since_s <= 0.0:
                    self._v7_ft_contact_shift_candidate_since_s = now_s
                shift_dwell_done = (
                    (now_s - self._v7_ft_contact_shift_candidate_since_s)
                    >= self.v7_contact_shift_dwell_s
                )
            else:
                self._v7_ft_contact_shift_candidate_since_s = 0.0
            contact_candidate = bool(shift_dwell_done)
        else:
            self._v7_ft_contact_shift_candidate_since_s = 0.0
            contact_candidate = bool(selected_force_n >= on_n)

        if not self._v7_ft_contact_active:
            self._v7_ft_contact_off_candidate_since_s = 0.0
            if contact_candidate:
                if self.v7_contact_shift_gate_enable:
                    # The shift gate has its own persistence test
                    # (v7_contact_shift_dwell_s). Stacking another on_dwell on
                    # top adds visible latency to the boundary acknowledgement
                    # the operator is supposed to feel "immediately", so we
                    # bypass it whenever the shift gate is in charge.
                    self._v7_ft_contact_active = True
                    self._v7_ft_contact_on_candidate_since_s = 0.0
                    self._v7_ft_contact_off_candidate_since_s = 0.0
                else:
                    if self._v7_ft_contact_on_candidate_since_s <= 0.0:
                        self._v7_ft_contact_on_candidate_since_s = now_s
                    if (now_s - self._v7_ft_contact_on_candidate_since_s) >= on_dwell_s:
                        self._v7_ft_contact_active = True
                        self._v7_ft_contact_on_candidate_since_s = 0.0
                        self._v7_ft_contact_off_candidate_since_s = 0.0
            else:
                self._v7_ft_contact_on_candidate_since_s = 0.0
                # Free-space training: baseline only advances when no candidate
                # is forming, so a real touch cannot be silently absorbed.
                if self.v7_contact_shift_gate_enable:
                    alpha = self.v7_contact_baseline_alpha
                    self._v7_contact_baseline_n = (
                        (alpha * float(selected_force_n))
                        + ((1.0 - alpha) * float(self._v7_contact_baseline_n))
                    )
        else:
            self._v7_ft_contact_on_candidate_since_s = 0.0
            if self.v7_contact_shift_gate_enable:
                release_candidate = (
                    shift_delta_n <= self.v7_contact_shift_off_threshold_n
                )
            else:
                release_candidate = selected_force_n <= off_n
            if release_candidate:
                if self._v7_ft_contact_off_candidate_since_s <= 0.0:
                    self._v7_ft_contact_off_candidate_since_s = now_s
                if (now_s - self._v7_ft_contact_off_candidate_since_s) >= off_dwell_s:
                    self._v7_ft_contact_active = False
                    self._v7_ft_contact_on_candidate_since_s = 0.0
                    self._v7_ft_contact_off_candidate_since_s = 0.0
                    self._v7_ft_contact_shift_candidate_since_s = 0.0
            else:
                self._v7_ft_contact_off_candidate_since_s = 0.0

        self._v7_debug_contact_candidate = bool(contact_candidate)
        self._v7_debug_contact_shift_candidate = bool(shift_dwell_done)
        self._v7_debug_contact_shift_delta_n = float(shift_delta_n)
        self._v7_debug_contact_baseline_n = float(self._v7_contact_baseline_n)
        return bool(contact_candidate)

    def _update_haptic_decision_from_sensor(self, now_s: float, sensor_debug: Optional[dict]) -> Tuple[float, float, float, float, bool]:
        if self._v7_ft_haptic_zero is None:
            # No tare yet: do not advance the contact gate so the first FT
            # reading does not snapshot a biased baseline.
            self._v7_ft_contact_active = False
            self._v7_ft_contact_on_candidate_since_s = 0.0
            self._v7_ft_contact_off_candidate_since_s = 0.0
            self._v7_ft_contact_shift_candidate_since_s = 0.0
            self._v7_contact_baseline_initialized = False
            self._v7_debug_contact_candidate = False
            self._v7_debug_contact_shift_candidate = False
            self._v7_debug_contact_shift_delta_n = 0.0
            self._v7_debug_contact_baseline_n = 0.0
            self._v7_debug_selected_force_value = 0.0
            self._v7_debug_selected_force_after_deadband = 0.0
            self._v7_debug_target_ratio_before_gate = 0.0
            self._v7_debug_target_ratio_after_gate = 0.0
            self._v7_debug_target_haptic_ratio = 0.0
            self._v7_debug_haptic_reason = "ft_zero_not_captured"
            return (0.0, 0.0, 0.0, 0.0, False)

        selected_force_n = self._selected_force_from_sensor_debug(sensor_debug)
        selected_after_deadband_n = max(
            0.0, selected_force_n - self.v7_ft_haptic_deadband_n
        )
        # Advance contact gate FIRST so the baseline / shift_delta we read
        # afterwards reflect the current sample.
        self._update_contact_gate(now_s, selected_force_n)

        if self.v7_contact_shift_gate_enable:
            # Map "shift past the off-threshold" -> [0..1] over full_scale_n.
            # This is the same signal the GUI shows in the contact-debug plot,
            # so graph and adaptive-trigger curve stay locked together.
            shift_after_band = max(
                0.0,
                float(selected_force_n)
                - float(self._v7_contact_baseline_n)
                - self.v7_contact_shift_off_threshold_n,
            )
            span = max(
                self.v7_ft_haptic_full_scale_n
                - self.v7_contact_shift_off_threshold_n,
                1e-6,
            )
            target_raw_ratio = clamp(shift_after_band / span, 0.0, 1.0)
        else:
            _, off_n, _, _ = self._contact_gate_thresholds()
            force_after_on = max(0.0, selected_force_n - off_n)
            force_span = max(self.v7_ft_haptic_full_scale_n - off_n, 1e-6)
            target_raw_ratio = clamp(force_after_on / force_span, 0.0, 1.0)

        target_shaped_ratio = target_raw_ratio ** self.v7_ft_haptic_shape_exp

        if self._v7_ft_contact_active:
            target_ratio_before_gate = clamp(
                max(
                    float(self.v7_contact_min_haptic_ratio),
                    float(target_shaped_ratio),
                ),
                0.0,
                self.v7_ft_haptic_max_trigger_resistance,
            )
            target_ratio_after_gate = target_ratio_before_gate
            self._v7_debug_haptic_reason = "contact_active_sending"
        else:
            target_ratio_before_gate = 0.0
            target_ratio_after_gate = 0.0
            self._v7_debug_haptic_reason = (
                "waiting_contact_dwell"
                if self._v7_debug_contact_candidate
                else "below_contact_threshold"
            )

        self._v7_ft_force_filtered_n = selected_force_n
        self._v7_ft_haptic_ratio_filtered = target_raw_ratio
        self._v7_debug_selected_force_value = selected_force_n
        self._v7_debug_selected_force_after_deadband = selected_after_deadband_n
        self._v7_debug_target_ratio_before_gate = target_ratio_before_gate
        self._v7_debug_target_ratio_after_gate = target_ratio_after_gate
        self._v7_debug_target_haptic_ratio = target_ratio_after_gate
        return (
            selected_force_n,
            selected_after_deadband_n,
            target_raw_ratio,
            target_ratio_after_gate,
            bool(self._v7_ft_contact_active),
        )

    def _capture_ft_haptic_zero(self, log: bool = False) -> None:
        r = self.ft_guard.last_reading
        self._v7_ft_haptic_zero = (
            float(r.fx), float(r.fy), float(r.fz),
            float(r.tx), float(r.ty), float(r.tz),
        )
        self._reset_v7_sensor_lpf()
        self._v7_ft_feedback_filtered = 0.0
        self._v7_ft_force_filtered_n = 0.0
        self._v7_ft_haptic_ratio_filtered = 0.0
        self._v7_ft_hard_block_active = False
        self._v7_ft_contact_active = False
        self._v7_ft_prev_contact_active = False
        self._v7_ft_contact_on_candidate_since_s = 0.0
        self._v7_ft_contact_off_candidate_since_s = 0.0
        self._v7_ft_contact_shift_candidate_since_s = 0.0
        self._v7_contact_baseline_initialized = False
        self._v7_contact_baseline_n = 0.0
        self._v7_debug_contact_candidate = False
        self._v7_debug_contact_shift_candidate = False
        self._v7_debug_contact_shift_delta_n = 0.0
        self._v7_debug_contact_baseline_n = 0.0
        self._v7_debug_target_ratio_before_gate = 0.0
        self._v7_debug_target_ratio_after_gate = 0.0
        self._v7_debug_selected_force_value = 0.0
        self._v7_debug_selected_force_after_deadband = 0.0
        self._v7_debug_actual_sent_ratio = 0.0
        self._v7_debug_haptics_enabled = False
        self._v7_debug_haptic_reason = "haptics_disabled"
        self._v7_filtered_fx = 0.0
        self._v7_filtered_fy = 0.0
        self._v7_filtered_fz = 0.0
        self._v7_prev_ft_feedback = 0.0
        self._set_ft_trigger_feedback(0.0, self._v7_depth_active_side)
        self._set_ft_body_rumble(0.0)
        if log:
            self.get_logger().info(
                "[V7][HAPTICS] FT haptic baseline captured "
                f"F=({r.fx:+.2f},{r.fy:+.2f},{r.fz:+.2f})N "
                f"T=({r.tx:+.3f},{r.ty:+.3f},{r.tz:+.3f})Nm"
            )

    def _ft_haptic_relative_components(self) -> Tuple[List[float], List[float]]:
        r = self.ft_guard.last_reading
        z = self._v7_ft_haptic_zero
        if z is None:
            z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        f = [float(r.fx) - z[0], float(r.fy) - z[1], float(r.fz) - z[2]]
        t = [float(r.tx) - z[3], float(r.ty) - z[4], float(r.tz) - z[5]]
        return f, t

    def _ft_haptic_force_n(self, pose_mm_deg: Optional[List[float]]) -> float:
        f, _ = self._ft_haptic_relative_components()
        force_norm = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])

        src = self.v7_ft_haptic_force_source
        if src == "norm" or pose_mm_deg is None:
            return force_norm
        if src == "max_axis":
            return max(abs(f[0]), abs(f[1]), abs(f[2]))

        # insertion_axis: project FT onto tool axis so operator feels drilling load
        # instead of unrelated lateral spikes.
        tool_z = tool_z_axis_from_rpy_deg(
            float(pose_mm_deg[3]), float(pose_mm_deg[4]), float(pose_mm_deg[5])
        )
        f_axis = abs((f[0] * tool_z[0]) + (f[1] * tool_z[1]) + (f[2] * tool_z[2]))
        return f_axis

    def _ft_haptic_force_torque_ratio(self, pose_mm_deg: Optional[List[float]]) -> float:
        cfg = self.ft_guard.cfg
        force_n = self._ft_haptic_force_n(pose_mm_deg)
        force_eff_n = max(0.0, force_n - self.v7_ft_haptic_deadband_n)
        full_scale_n = (
            self.v7_ft_haptic_full_scale_n
            if self.v7_ft_haptic_full_scale_n > 0.0
            else float(cfg.force_norm_limit_n)
        )
        # Map deadband..full_scale to 0..1 so full_scale_n is the real 100% point.
        force_span_n = max(full_scale_n - self.v7_ft_haptic_deadband_n, 1e-6)
        force_ratio = force_eff_n / force_span_n
        if not self.v7_ft_haptic_include_torque:
            return clamp(force_ratio, 0.0, 1.0)

        _, t = self._ft_haptic_relative_components()
        torque_norm = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
        torque_dead = self.v7_ft_haptic_torque_deadband_nm
        full_scale_torque = (
            self.v7_ft_haptic_full_scale_torque_nm
            if self.v7_ft_haptic_full_scale_torque_nm > 0.0
            else float(cfg.torque_norm_limit_nm)
        )
        torque_ratios = [
            max(0.0, abs(t[0]) - torque_dead) / max(float(cfg.tx_limit_nm), 1e-6),
            max(0.0, abs(t[1]) - torque_dead) / max(float(cfg.ty_limit_nm), 1e-6),
            max(0.0, abs(t[2]) - torque_dead) / max(float(cfg.tz_limit_nm), 1e-6),
            max(0.0, torque_norm - torque_dead) / max(full_scale_torque, 1e-6),
        ]
        return clamp(max(force_ratio, max(torque_ratios)), 0.0, 1.0)

    def _set_ft_trigger_feedback(self, force_ratio: float, side: str, hard_block: bool = False):
        # Per-channel disable so the operator can A/B-test vibration-only vs
        # trigger-only modes. hard_block still fires because that channel
        # represents the safety lock, not the proportional feedback.
        if not self.v7_ft_haptic_trigger_enable and not hard_block:
            force_ratio = 0.0
        gain = max(float(self.v7_haptic_feedback_gain), 0.0)
        ratio = clamp(
            force_ratio * gain, 0.0, max(self.v7_ft_haptic_max_trigger_resistance, 0.01)
        )
        if hard_block:
            ratio = max(self.v7_ft_haptic_max_trigger_resistance, 0.01)
        progressive = self.v7_ft_haptic_progressive_trigger_enable
        start_pos = int(self.get_parameter("v7_ft_haptic_trigger_start_position").value)
        max_strength = int(self.get_parameter("v7_ft_haptic_trigger_max_strength").value)
        # Route adaptive trigger resistance to the active insertion side.
        if side == "left":
            if hard_block and hasattr(self.input, "set_trigger_hard_block_left"):
                self.input.set_trigger_hard_block_left(start_position=start_pos, strength=max_strength)
            elif hard_block and hasattr(self.input, "set_trigger_profile_left"):
                self.input.set_trigger_profile_left(start_position=start_pos, strength=max_strength)
            elif progressive and hasattr(self.input, "set_drilling_feedback_progressive_left"):
                self.input.set_drilling_feedback_progressive_left(ratio)
            elif hasattr(self.input, "set_drilling_feedback_left"):
                self.input.set_drilling_feedback_left(ratio)
            elif hasattr(self.input, "set_trigger_profile_left"):
                self.input.set_trigger_profile_left(start_position=0, strength=int(round(ratio * 8.0)))
            if progressive and hasattr(self.input, "set_drilling_feedback_progressive"):
                self.input.set_drilling_feedback_progressive(0.0)
            elif hasattr(self.input, "set_drilling_feedback"):
                self.input.set_drilling_feedback(0.0)
        else:
            if hard_block and hasattr(self.input, "set_trigger_hard_block"):
                self.input.set_trigger_hard_block(start_position=start_pos, strength=max_strength)
            elif hard_block and hasattr(self.input, "set_trigger_profile"):
                self.input.set_trigger_profile(start_position=start_pos, strength=max_strength)
            elif progressive and hasattr(self.input, "set_drilling_feedback_progressive"):
                self.input.set_drilling_feedback_progressive(ratio)
            elif hasattr(self.input, "set_drilling_feedback"):
                self.input.set_drilling_feedback(ratio)
            if progressive and hasattr(self.input, "set_drilling_feedback_progressive_left"):
                self.input.set_drilling_feedback_progressive_left(0.0)
            elif hasattr(self.input, "set_drilling_feedback_left"):
                self.input.set_drilling_feedback_left(0.0)

    def _update_v7_depth_active_side(self, inp: InputSnapshot) -> None:
        rt = float(inp.rt)
        lt = float(inp.lt)
        dead = 0.05
        # R2 is inward/down in the live setup. While it is pressed, keep force
        # feedback on R2 so small L2 noise cannot steal the active haptic side.
        if rt > dead:
            self._v7_depth_active_side = "right"
        elif lt > dead:
            self._v7_depth_active_side = "left"

    def _set_ft_body_rumble(self, feedback_ratio: float) -> None:
        if time.monotonic() < getattr(self, "_v7_event_pulse_until_s", 0.0):
            return
        if self.dry_run or not hasattr(self, "input"):
            return
        onset = clamp(self.v7_ft_haptic_vibration_min_ratio, 0.0, 0.95)
        if (not self.v7_ft_haptic_rumble_enable) or feedback_ratio <= onset:
            # Avoid flooding the DualSense HID endpoint with repeated zero packets
            # while the robot is in free space.
            if self._v7_last_ft_body_rumble == 0.0:
                return
            self._v7_last_ft_body_rumble = 0.0
            try:
                if hasattr(self.input, "set_body_rumble"):
                    self.input.set_body_rumble(0.0, 0.0)
                elif hasattr(self.input, "rumble"):
                    self.input.rumble(strength=0.0, duration_ms=1)
            except Exception:
                pass
            return

        x = clamp((feedback_ratio - onset) / max(1.0 - onset, 1e-6), 0.0, 1.0)
        shaped = x ** self.v7_ft_haptic_rumble_shape_exp
        level = clamp(
            self.v7_ft_haptic_rumble_min_strength + (
                (self.v7_ft_haptic_rumble_max_strength - self.v7_ft_haptic_rumble_min_strength)
                * shaped
            ),
            0.0,
            1.0,
        )
        if self.v7_ft_haptic_rumble_continuous_enable:
            self._v7_last_ft_body_rumble = level
            try:
                if hasattr(self.input, "set_body_rumble"):
                    self.input.set_body_rumble(level, level)
            except Exception:
                pass
            return

        interval_s = self.v7_ft_haptic_rumble_max_interval_s - (
            (self.v7_ft_haptic_rumble_max_interval_s - self.v7_ft_haptic_rumble_min_interval_s) * math.sqrt(x)
        )
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._v7_last_ft_rumble_s < interval_s:
            return
        self._v7_last_ft_rumble_s = now_s
        self._v7_last_ft_body_rumble = level
        duration_ms = int(round(
            self.v7_ft_haptic_rumble_min_duration_ms
            + ((self.v7_ft_haptic_rumble_max_duration_ms - self.v7_ft_haptic_rumble_min_duration_ms) * math.sqrt(x))
        ))
        try:
            if hasattr(self.input, "rumble"):
                self.input.rumble(strength=level, duration_ms=duration_ms)
            elif hasattr(self.input, "set_body_rumble"):
                self.input.set_body_rumble(level, level)
        except Exception:
            pass

    def _should_fault_on_vc_failure(self, ret: int, cmd: List[float]) -> bool:
        if int(ret) in self.v7_vc_reject_soft_codes:
            self._warn_throttle(
                "v7_vc_soft_reject",
                f"[V7] VC rejected code={ret}; holding without latching fault",
                0.6,
            )
            try:
                self.arm.vc_set_cartesian_velocity(
                    [0.0] * 6,
                    is_radian=False,
                    is_tool_coord=False,
                    duration=self.velocity_watchdog_s,
                )
            except Exception:
                pass
            self._last_cmd_sent = [0.0] * 6
            self.rate_limiter.reset()
            return False
        return True

    def _check_robot_health(self) -> bool:
        if not self.arm.has_err_warn:
            return False
        err = int(getattr(self.arm, "error_code", 0) or 0)
        warn = int(getattr(self.arm, "warn_code", 0) or 0)
        if err == 0 and warn == 0:
            return False
        if self.v7_contact_error_auto_recover and err == self.v7_contact_error_code:
            now = time.monotonic()
            if now - self._v7_last_contact_recovery_s < self.v7_contact_recovery_cooldown_s:
                self._warn_throttle(
                    "v7_contact_recover_wait",
                    f"[V7] contact error {err}; waiting before retry",
                    0.5,
                )
                return True
            self._v7_last_contact_recovery_s = now
            self._warn_throttle(
                "v7_contact_auto_recover",
                f"[V7] contact error {err}; auto-recovering to IDLE",
                0.2,
            )
            try:
                self.arm.clean_error()
                self.arm.clean_warn()
                self.arm.motion_enable(True)
                self.arm.set_mode(0)
                self.arm.set_state(0)
                time.sleep(0.05)
                self._switch_mode(5)
                self._last_cmd_sent = [0.0] * 6
                self._idle_zero_sent = True
                self.rate_limiter.reset()
                if self.sm.state != TeleopState.IDLE:
                    self.sm.transition_to(TeleopState.IDLE, "v7_contact_auto_recovered")
                return True
            except Exception as exc:
                self._warn_throttle(
                    "v7_contact_recover_exc",
                    f"[V7] contact auto-recovery failed: {exc}",
                    0.8,
                )
        return super()._check_robot_health()

    def _refresh_ft_fallback(self, now_s: float):
        """Update FT guard directly from SDK when ROS FT topic is stale/missing."""
        if self.dry_run or not self.v7_ft_poll_fallback_enable:
            return
        if self.ft_guard.data_state(now_s) == FTDataState.FRESH:
            return
        if (now_s - self._v7_last_ft_poll_s) < self.v7_ft_poll_min_interval_s:
            return
        self._v7_last_ft_poll_s = now_s

        try:
            code, ft_data = self.arm.get_ft_sensor_data()
        except Exception as exc:
            self._warn_throttle("v7_ft_poll_exc", f"[V7] direct FT poll exception: {exc}", 2.0)
            return

        if code != 0 or not ft_data or len(ft_data) < 6:
            if self.v7_ft_poll_auto_init and not self._v7_ft_auto_init_attempted:
                self._v7_ft_auto_init_attempted = True
                try:
                    if hasattr(self.arm, "set_tgpio_modbus_baudrate"):
                        self.arm.set_tgpio_modbus_baudrate(int(self.v7_ft_modbus_baudrate))
                    if hasattr(self.arm, "ft_sensor_enable"):
                        self.arm.ft_sensor_enable(1)
                    self.get_logger().warn(
                        "[V7] direct FT poll unavailable; attempted FT sensor auto-init"
                    )
                except Exception as exc:
                    self._warn_throttle(
                        "v7_ft_poll_init_exc",
                        f"[V7] FT auto-init attempt failed: {exc}",
                        2.0,
                    )
            self._warn_throttle(
                "v7_ft_poll_bad",
                f"[V7] direct FT poll unavailable (code={code})",
                2.0,
            )
            return

        self.ft_guard.update(
            fx=float(ft_data[0]),
            fy=float(ft_data[1]),
            fz=float(ft_data[2]),
            tx=float(ft_data[3]),
            ty=float(ft_data[4]),
            tz=float(ft_data[5]),
            now_s=now_s,
        )
        if not self._v7_direct_ft_logged:
            self._v7_direct_ft_logged = True
            self.get_logger().info("[V7] FT fallback active: using direct SDK FT polling")

    def _update_motion_haptics(self, cmd: List[float], inp: InputSnapshot):
        if not self.v7_motion_haptic_enable:
            return
        if self._v7_prev_ft_feedback > 0.05:
            # Keep force-based haptics dominant during contact.
            self._v7_last_motion_feedback = 0.0
            return

        self._update_v7_depth_active_side(inp)

        lin_norm = math.sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2])
        lin_ratio = lin_norm / max(self.max_linear_mm_s, 1e-6)
        depth_ratio = abs(float(inp.lt - inp.rt))
        motion_ratio = clamp(max(lin_ratio, depth_ratio), 0.0, 1.0)

        if motion_ratio <= self.v7_motion_haptic_deadband_ratio:
            if self._v7_last_motion_feedback > 0.0:
                try:
                    if hasattr(self.input, "set_trigger_vibration_left"):
                        self.input.set_trigger_vibration_left(intensity=0.0)
                    if hasattr(self.input, "set_trigger_vibration"):
                        self.input.set_trigger_vibration(intensity=0.0)
                except Exception:
                    pass
            self._v7_last_motion_feedback = 0.0
            return

        intensity = clamp(self.v7_motion_haptic_gain * math.sqrt(motion_ratio), 0.0, 1.0)
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._v7_last_motion_rumble_s >= self.v7_motion_haptic_rumble_interval_s:
            self._v7_last_motion_rumble_s = now_s
            try:
                self.input.rumble(strength=clamp(0.20 + 0.60 * intensity, 0.0, 1.0), duration_ms=80)
            except Exception:
                pass
        try:
            if self._v7_depth_active_side == "left" and hasattr(self.input, "set_trigger_vibration_left"):
                self.input.set_trigger_vibration_left(intensity=intensity)
                if hasattr(self.input, "set_trigger_vibration"):
                    self.input.set_trigger_vibration(intensity=0.0)
            elif hasattr(self.input, "set_trigger_vibration"):
                self.input.set_trigger_vibration(intensity=intensity)
                if hasattr(self.input, "set_trigger_vibration_left"):
                    self.input.set_trigger_vibration_left(intensity=0.0)
        except Exception:
            pass
        self._v7_last_motion_feedback = intensity

    def _publish_ft_haptic_debug(
        self,
        now_s: float,
        selected_force_value: Optional[float] = None,
        selected_force_after_deadband: Optional[float] = None,
        target_haptic_ratio: Optional[float] = None,
        smoothed_haptic_ratio: Optional[float] = None,
        depth_input_active: Optional[bool] = None,
        actual_sent_ratio: Optional[float] = None,
        contact_candidate: Optional[bool] = None,
        target_ratio_before_contact_gate: Optional[float] = None,
        target_ratio_after_contact_gate: Optional[float] = None,
        haptics_enabled: Optional[bool] = None,
        deadman_active: Optional[bool] = None,
        motion_active: Optional[bool] = None,
        reason: Optional[str] = None,
        sensor_debug: Optional[dict] = None,
    ) -> None:
        """Publish the exact backend FT/haptic state for GUI analysis only."""
        pub = getattr(self, "_v7_ft_haptic_debug_pub", None)
        if pub is None:
            return
        if selected_force_value is not None:
            self._v7_debug_selected_force_value = float(selected_force_value)
        if selected_force_after_deadband is not None:
            self._v7_debug_selected_force_after_deadband = float(selected_force_after_deadband)
        if target_haptic_ratio is not None:
            self._v7_debug_target_haptic_ratio = float(target_haptic_ratio)
        if smoothed_haptic_ratio is not None:
            self._v7_debug_smoothed_haptic_ratio = float(smoothed_haptic_ratio)
        if actual_sent_ratio is not None:
            self._v7_debug_actual_sent_ratio = float(actual_sent_ratio)
        elif smoothed_haptic_ratio is not None:
            self._v7_debug_actual_sent_ratio = float(smoothed_haptic_ratio)
        if depth_input_active is not None:
            self._v7_debug_depth_input_active = bool(depth_input_active)
        if contact_candidate is not None:
            self._v7_debug_contact_candidate = bool(contact_candidate)
        if target_ratio_before_contact_gate is not None:
            self._v7_debug_target_ratio_before_gate = float(target_ratio_before_contact_gate)
        if target_ratio_after_contact_gate is not None:
            self._v7_debug_target_ratio_after_gate = float(target_ratio_after_contact_gate)
        if haptics_enabled is not None:
            self._v7_debug_haptics_enabled = bool(haptics_enabled)
        if deadman_active is not None:
            self._v7_debug_deadman_active = bool(deadman_active)
        if motion_active is not None:
            self._v7_debug_motion_active = bool(motion_active)
        if reason is not None:
            self._v7_debug_haptic_reason = str(reason)

        if sensor_debug is None:
            sensor_debug = self._update_v7_sensor_lpf(now_s)
        if sensor_debug is None:
            return

        min_dt = 1.0 / max(self.v7_ft_haptic_debug_topic_max_hz, 1.0)
        if (now_s - self._v7_last_ft_haptic_debug_pub_s) < min_dt:
            return
        self._v7_last_ft_haptic_debug_pub_s = now_s

        try:
            debug_depth_active = bool(self._v7_debug_depth_input_active)
            if self.sm.state == TeleopState.IDLE:
                debug_depth_active = False
            debug_haptics_enabled = bool(
                self._v7_debug_haptics_enabled
                and self.sm.state != TeleopState.IDLE
            )
            active_side = self._v7_depth_active_side if debug_haptics_enabled else "none"
            actual_ratio = self._v7_debug_actual_sent_ratio if debug_haptics_enabled else 0.0
            debug_contact_active = bool(self._v7_ft_contact_active)
            debug_hard_block_active = bool(self._v7_ft_hard_block_active and actual_ratio > 0.0)
            on_n, off_n, _, _ = self._contact_gate_thresholds()
            raw_sensor = sensor_debug["raw_sensor"]
            filtered_sensor = sensor_debug["filtered_sensor"]
            sensor_analysis = {
                "raw_force_norm_n": float(sensor_debug["raw_force_norm_n"]),
                "raw_sensor_force_norm_n": float(sensor_debug["raw_sensor_force_norm_n"]),
                "filtered_force_norm_n": float(sensor_debug["filtered_force_norm_n"]),
                "selected_force_before_deadband_n": float(self._v7_debug_selected_force_value),
                "selected_force_after_deadband_n": float(self._v7_debug_selected_force_after_deadband),
                # Explicit aliases requested by the operator: these are the
                # exact signals the backend uses for contact detection and
                # haptic mapping. GUI plots must read these (not derive them).
                "selected_force_n": float(self._v7_debug_selected_force_value),
                "baseline_force_n": float(self._v7_debug_contact_baseline_n),
                "shift_from_baseline_n": float(self._v7_debug_contact_shift_delta_n),
                "noise_band_n": float(self.v7_contact_shift_threshold_n),
                "noise_band_off_n": float(self.v7_contact_shift_off_threshold_n),
                "force_source": str(self.v7_ft_haptic_force_source),
                "deadband_n": float(self.v7_ft_haptic_deadband_n),
                "full_scale_n": float(self.v7_ft_haptic_full_scale_n),
                "contact_on_threshold_n": float(on_n),
                "contact_off_threshold_n": float(off_n),
                "contact_baseline_n": float(self._v7_debug_contact_baseline_n),
                "contact_shift_delta_n": float(self._v7_debug_contact_shift_delta_n),
                "contact_shift_threshold_n": float(self.v7_contact_shift_threshold_n),
                "contact_shift_off_threshold_n": float(self.v7_contact_shift_off_threshold_n),
                "shift_gate_enabled": bool(self.v7_contact_shift_gate_enable),
                "lpf_alpha": float(sensor_debug["lpf_alpha"]),
                "ft_fresh": bool(sensor_debug["ft_fresh"]),
            }
            haptic_output = {
                "contact_candidate": bool(self._v7_debug_contact_candidate),
                "contact_shift_candidate": bool(self._v7_debug_contact_shift_candidate),
                "target_ratio": float(self._v7_debug_target_haptic_ratio),
                "target_haptic_ratio": float(self._v7_debug_target_haptic_ratio),
                "target_ratio_before_contact_gate": float(self._v7_debug_target_ratio_before_gate),
                "target_ratio_after_contact_gate": float(self._v7_debug_target_ratio_after_gate),
                "smoothed_haptic_ratio": float(self._v7_debug_smoothed_haptic_ratio),
                "actual_sent_ratio": float(actual_ratio),
                "contact_active": debug_contact_active,
                "contact_gate_enabled": bool(self.v7_contact_gate_enable),
                "shift_gate_enabled": bool(self.v7_contact_shift_gate_enable),
                "trigger_enable": bool(self.v7_ft_haptic_trigger_enable),
                "rumble_enable": bool(self.v7_ft_haptic_rumble_enable),
                "hard_block_active": debug_hard_block_active,
                "deadman_active": bool(self._v7_debug_deadman_active and self.sm.state != TeleopState.IDLE),
                "depth_input_active": bool(debug_depth_active),
                "motion_active": bool(self._v7_debug_motion_active),
                "haptics_enabled": bool(debug_haptics_enabled),
                "active_trigger_side": active_side,
                # Single source of truth for "why does PS5 see what it sees".
                "haptic_block_reason": str(self._v7_debug_haptic_reason),
                "reason": str(self._v7_debug_haptic_reason),
            }
            payload = {
                "stamp": float(now_s),
                "raw_sensor": raw_sensor,
                "raw_relative_sensor": sensor_debug["raw_relative_sensor"],
                "filtered_sensor": filtered_sensor,
                "sensor_analysis": sensor_analysis,
                "haptic_output": haptic_output,
                "torque_filtered": True,
                # Backward-compatible keys used by the current GUI/debug tools.
                "raw": raw_sensor,
                "filtered": filtered_sensor,
                "force_norm_raw": float(sensor_debug["raw_force_norm_n"]),
                "force_norm_filtered": float(sensor_debug["filtered_force_norm_n"]),
                "selected_force_value": float(self._v7_debug_selected_force_value),
                "selected_force_before_deadband_n": float(self._v7_debug_selected_force_value),
                "selected_force_after_deadband_n": float(self._v7_debug_selected_force_after_deadband),
                "selected_force_n": float(self._v7_debug_selected_force_value),
                "baseline_force_n": float(self._v7_debug_contact_baseline_n),
                "shift_from_baseline_n": float(self._v7_debug_contact_shift_delta_n),
                "noise_band_n": float(self.v7_contact_shift_threshold_n),
                "force_source": str(self.v7_ft_haptic_force_source),
                "deadband_n": float(self.v7_ft_haptic_deadband_n),
                "full_scale_n": float(self.v7_ft_haptic_full_scale_n),
                "contact_on_threshold_n": float(on_n),
                "contact_off_threshold_n": float(off_n),
                "contact_baseline_n": float(self._v7_debug_contact_baseline_n),
                "contact_shift_delta_n": float(self._v7_debug_contact_shift_delta_n),
                "contact_shift_threshold_n": float(self.v7_contact_shift_threshold_n),
                "contact_shift_off_threshold_n": float(self.v7_contact_shift_off_threshold_n),
                "shift_gate_enabled": bool(self.v7_contact_shift_gate_enable),
                "target_haptic_ratio": float(self._v7_debug_target_haptic_ratio),
                "target_ratio_before_contact_gate": float(self._v7_debug_target_ratio_before_gate),
                "target_ratio_after_contact_gate": float(self._v7_debug_target_ratio_after_gate),
                "smoothed_haptic_ratio": float(self._v7_debug_smoothed_haptic_ratio),
                "contact_active": debug_contact_active,
                "contact_candidate": bool(self._v7_debug_contact_candidate),
                "contact_shift_candidate": bool(self._v7_debug_contact_shift_candidate),
                "contact_gate_enabled": bool(self.v7_contact_gate_enable),
                "trigger_enable": bool(self.v7_ft_haptic_trigger_enable),
                "rumble_enable": bool(self.v7_ft_haptic_rumble_enable),
                "hard_block_active": debug_hard_block_active,
                "active_trigger_side": active_side,
                "depth_input_active": bool(debug_depth_active),
                "deadman_active": bool(self._v7_debug_deadman_active and self.sm.state != TeleopState.IDLE),
                "motion_active": bool(self._v7_debug_motion_active),
                "haptics_enabled": bool(debug_haptics_enabled),
                "actual_sent_ratio": float(actual_ratio),
                "haptic_block_reason": str(self._v7_debug_haptic_reason),
                "reason": str(self._v7_debug_haptic_reason),
                "lpf_alpha": float(self.v7_ft_sensor_lpf_alpha),
            }
            pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        except Exception:
            pass

    def _v7_sensor_debug_timer_cb(self) -> None:
        # Continuous, deadman-independent sensor measurement + baseline train.
        # The actual PS5 output is still gated by deadman/contact in
        # _update_ft_force_haptics; here we only advance the noise model so
        # contact detection is ready the instant the operator engages.
        now_s = self.get_clock().now().nanoseconds / 1e9
        sensor_debug = self._update_v7_sensor_lpf(now_s)
        if sensor_debug is not None and sensor_debug.get("ft_fresh", False):
            self._update_haptic_decision_from_sensor(now_s, sensor_debug)
            if self.sm.state == TeleopState.IDLE:
                # Override the "below_contact_threshold" / "contact_active_..."
                # decision-side reason with the real reason output is zero.
                self._v7_debug_haptic_reason = "idle_deadman_not_active"
        self._publish_ft_haptic_debug(now_s, sensor_debug=sensor_debug)

    def _update_ft_force_haptics(self, inp: InputSnapshot):
        if not self.v7_ft_haptic_enable:
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        sensor_debug = self._update_v7_sensor_lpf(now_s)
        if self.ft_guard.data_state(now_s) != FTDataState.FRESH:
            self._warn_throttle("v7_ft_stale_haptic", "[V7] FT data stale for haptics", 2.0)
            self._set_ft_trigger_feedback(0.0, self._v7_depth_active_side)
            self._v7_ft_feedback_filtered = 0.0
            self._v7_ft_haptic_ratio_filtered = 0.0
            self._v7_ft_hard_block_active = False
            self._v7_prev_ft_feedback = 0.0
            self._v7_ft_contact_active = False
            self._v7_ft_prev_contact_active = False
            self._v7_ft_contact_on_candidate_since_s = 0.0
            self._v7_ft_contact_off_candidate_since_s = 0.0
            self._set_ft_body_rumble(0.0)
            self._publish_ft_haptic_debug(
                now_s,
                target_haptic_ratio=0.0,
                smoothed_haptic_ratio=0.0,
                actual_sent_ratio=0.0,
                depth_input_active=False,
                contact_candidate=False,
                target_ratio_before_contact_gate=0.0,
                target_ratio_after_contact_gate=0.0,
                haptics_enabled=False,
                deadman_active=False,
                motion_active=False,
                reason="ft_stale",
                sensor_debug=sensor_debug,
            )
            return

        if self.v7_ft_haptic_auto_tare_enable and self._v7_ft_haptic_zero is None:
            self._capture_ft_haptic_zero(log=True)
            return

        self._update_v7_depth_active_side(inp)
        trigger_travel = float(inp.rt) if self._v7_depth_active_side == "right" else float(inp.lt)
        depth_input_active = trigger_travel >= self.v7_ft_haptic_trigger_travel_deadband
        haptic_input_active = True
        if self.v7_ft_haptic_require_depth_input:
            haptic_input_active = (
                depth_input_active
                and max(float(inp.lt), float(inp.rt)) >= self.v7_ft_haptic_depth_input_min
            )
        deadman_active = self.sm.state != TeleopState.IDLE
        motion_active = (
            max(
                abs(float(inp.lx)), abs(float(inp.ly)),
                abs(float(inp.rx)), abs(float(inp.ry)),
                abs(float(inp.lt)), abs(float(inp.rt)),
            ) > 0.05
        )

        (
            selected_force_n,
            selected_after_deadband_n,
            raw_ratio,
            target_after_gate,
            contact_active,
        ) = self._update_haptic_decision_from_sensor(now_s, sensor_debug)

        haptics_enabled = bool(deadman_active and haptic_input_active and contact_active)

        # Decide why output is what it is. The decision-side reason
        # (set by _update_haptic_decision_from_sensor) only knows about contact
        # state; here we override it with operator-side reasons.
        haptic_reason = self._v7_debug_haptic_reason
        if not deadman_active:
            haptic_reason = "idle_deadman_not_active"
        elif self.v7_ft_haptic_require_depth_input and not haptic_input_active:
            haptic_reason = "depth_input_required_but_inactive"

        # First-contact tactile acknowledgement: one short rumble pulse on the
        # rising edge of contact_active (and only when the operator is engaged,
        # so the cue is meaningful). Subsequent ticks use the proportional loop.
        rising_edge_contact = bool(
            contact_active
            and not self._v7_ft_prev_contact_active
            and deadman_active
        )
        if rising_edge_contact and self.v7_ft_haptic_contact_pulse_enable:
            self._pulse_haptic(
                strength=self.v7_ft_haptic_contact_pulse_strength,
                duration_ms=int(self.v7_ft_haptic_contact_pulse_duration_ms),
                cooldown_s=0.0,
            )
        self._v7_ft_prev_contact_active = bool(contact_active)

        if not haptics_enabled:
            # Contact / deadman / depth-trigger gate not satisfied: PS5 silent.
            self._v7_ft_feedback_filtered = 0.0
            self._v7_prev_ft_feedback = 0.0
            self._v7_ft_hard_block_active = False
            self._set_ft_trigger_feedback(0.0, self._v7_depth_active_side)
            self._set_ft_body_rumble(0.0)
            self._publish_ft_haptic_debug(
                now_s,
                selected_force_value=selected_force_n,
                selected_force_after_deadband=selected_after_deadband_n,
                target_haptic_ratio=target_after_gate,
                smoothed_haptic_ratio=0.0,
                actual_sent_ratio=0.0,
                depth_input_active=depth_input_active,
                contact_candidate=self._v7_debug_contact_candidate,
                target_ratio_before_contact_gate=self._v7_debug_target_ratio_before_gate,
                target_ratio_after_contact_gate=target_after_gate,
                haptics_enabled=False,
                deadman_active=deadman_active,
                motion_active=motion_active,
                reason=haptic_reason,
                sensor_debug=sensor_debug,
            )
            return

        if self.v7_ft_haptic_hard_block_enable:
            if raw_ratio >= self.v7_ft_haptic_hard_block_on_ratio:
                self._v7_ft_hard_block_active = True
            elif raw_ratio <= self.v7_ft_haptic_hard_block_off_ratio:
                self._v7_ft_hard_block_active = False
        else:
            self._v7_ft_hard_block_active = False

        if self._v7_ft_hard_block_active:
            # Safety cue: max-force contact becomes a distinct mechanical block
            # instead of merely another proportional resistance value.
            feedback_ratio = self.v7_ft_haptic_max_trigger_resistance
            self._v7_ft_feedback_filtered = feedback_ratio
            self._set_ft_trigger_feedback(feedback_ratio, self._v7_depth_active_side, hard_block=True)
            self._set_ft_body_rumble(feedback_ratio)
            self._v7_prev_ft_feedback = feedback_ratio
            self._publish_ft_haptic_debug(
                now_s,
                selected_force_value=selected_force_n,
                selected_force_after_deadband=selected_after_deadband_n,
                target_haptic_ratio=feedback_ratio,
                smoothed_haptic_ratio=feedback_ratio,
                actual_sent_ratio=feedback_ratio,
                depth_input_active=depth_input_active,
                contact_candidate=self._v7_debug_contact_candidate,
                target_ratio_before_contact_gate=self._v7_debug_target_ratio_before_gate,
                target_ratio_after_contact_gate=target_after_gate,
                haptics_enabled=True,
                deadman_active=deadman_active,
                motion_active=motion_active,
                reason="hard_block_saturated",
                sensor_debug=sensor_debug,
            )
            return

        target_feedback = clamp(target_after_gate, 0.0, self.v7_ft_haptic_max_trigger_resistance)

        # Fast attack, softer release: contact should be felt immediately, but
        # tiny FT noise should not buzz the controller when force decreases.
        if target_feedback >= self._v7_ft_feedback_filtered:
            smooth_alpha = max(self.v7_ft_haptic_smooth_alpha, 0.85)
        else:
            smooth_alpha = min(self.v7_ft_haptic_smooth_alpha, 0.35)
        feedback_ratio = (
            (smooth_alpha * target_feedback)
            + ((1.0 - smooth_alpha) * self._v7_ft_feedback_filtered)
        )
        feedback_ratio = clamp(
            feedback_ratio,
            0.0,
            self.v7_ft_haptic_max_trigger_resistance,
        )
        self._v7_ft_feedback_filtered = feedback_ratio
        self._set_ft_trigger_feedback(feedback_ratio, self._v7_depth_active_side)
        self._set_ft_body_rumble(feedback_ratio)
        self._publish_ft_haptic_debug(
            now_s,
            selected_force_value=selected_force_n,
            selected_force_after_deadband=selected_after_deadband_n,
            target_haptic_ratio=target_feedback,
            smoothed_haptic_ratio=feedback_ratio,
            actual_sent_ratio=feedback_ratio,
            depth_input_active=depth_input_active,
            contact_candidate=self._v7_debug_contact_candidate,
            target_ratio_before_contact_gate=self._v7_debug_target_ratio_before_gate,
            target_ratio_after_contact_gate=target_after_gate,
            haptics_enabled=True,
            deadman_active=deadman_active,
            motion_active=motion_active,
            reason="contact_active_sending",
            sensor_debug=sensor_debug,
        )

        release = clamp(self.v7_ft_haptic_release_ratio, 0.0, 0.50)
        if self.v7_ft_haptic_debug and (now_s - self._v7_last_ft_haptic_dbg_s) >= self.v7_ft_haptic_debug_interval_s:
            self._v7_last_ft_haptic_dbg_s = now_s
            f_rel, t_rel = self._ft_haptic_relative_components()
            self.get_logger().info(
                "[V7][HAPTIC_DBG] "
                f"relF=({f_rel[0]:+.3f},{f_rel[1]:+.3f},{f_rel[2]:+.3f})N "
                f"relT=({t_rel[0]:+.3f},{t_rel[1]:+.3f},{t_rel[2]:+.3f})Nm "
                f"source={self.v7_ft_haptic_force_source} force={self._v7_ft_force_filtered_n:.3f}N "
                f"raw={raw_ratio:.3f} target={target_feedback:.3f} "
                f"contact={self._v7_ft_contact_active} "
                f"trigger={feedback_ratio:.3f} side={self._v7_depth_active_side}"
            )

        if feedback_ratio > self.v7_ft_haptic_vibration_min_ratio:
            # Trigger vibration is a separate DualSense trigger effect and can
            # overwrite continuous resistance. Keep it optional; body rumble is
            # the held-contact cue that does not destroy the resistance profile.
            if self.v7_ft_haptic_trigger_vibration_enable:
                try:
                    vib = clamp(0.08 + 0.70 * feedback_ratio, 0.0, 1.0)
                    if self._v7_depth_active_side == "left" and hasattr(self.input, "set_trigger_vibration_left"):
                        self.input.set_trigger_vibration_left(intensity=vib)
                    elif hasattr(self.input, "set_trigger_vibration"):
                        self.input.set_trigger_vibration(intensity=vib)
                except Exception:
                    pass

        if (
            self.v7_ft_haptic_release_pulse_enable
            and self._v7_prev_ft_feedback >= 0.25
            and feedback_ratio <= release
        ):
            self._pulse_haptic(0.75, 120, 0.0)
        self._v7_prev_ft_feedback = feedback_ratio

    def _retreat_cmd_from_force_limit(self, cmd: List[float], fx: float, fy: float, fz: float) -> List[float]:
        lin = [float(cmd[0]), float(cmd[1]), float(cmd[2])]
        lin_norm = math.sqrt(lin[0] * lin[0] + lin[1] * lin[1] + lin[2] * lin[2])
        f_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        if lin_norm < 1e-6 or f_norm < 1e-6:
            return [0.0] * 6
        retreat_cos = self._force_retreat_cos(cmd, fx, fy, fz)
        if retreat_cos < self.v7_ft_retreat_min_cos:
            return [0.0] * 6
        s = self.v7_ft_retreat_speed_scale * clamp(retreat_cos, 0.0, 1.0)
        return [lin[0] * s, lin[1] * s, lin[2] * s, 0.0, 0.0, 0.0]

    def _force_retreat_cos(self, cmd: List[float], fx: float, fy: float, fz: float) -> float:
        lin = [float(cmd[0]), float(cmd[1]), float(cmd[2])]
        lin_norm = math.sqrt(lin[0] * lin[0] + lin[1] * lin[1] + lin[2] * lin[2])
        f_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        if lin_norm < 1e-6 or f_norm < 1e-6:
            return 0.0
        dot = fx * lin[0] + fy * lin[1] + fz * lin[2]
        return (-dot) / max(f_norm * lin_norm, 1e-9)

    def _log_dualsense_haptics_status(self):
        dev = getattr(self.input, "bound_device_path", None)
        writable = bool(getattr(self.input, "_haptics_writable", False))
        conn_err = getattr(self.input, "_connect_error", None)
        if conn_err:
            self.get_logger().warn(f"[V7][HAPTICS] DualSense connect warning: {conn_err}")
        self.get_logger().info(
            f"[V7][HAPTICS] DualSense device={dev or 'unknown'} write_access={'yes' if writable else 'no'}"
        )
        if self.v7_haptics_boot_test and writable:
            self._pulse_haptic(1.0, 220, 0.0)
            self._set_ft_trigger_feedback(0.0, self._v7_depth_active_side)
            self.get_logger().info("[V7][HAPTICS] Boot test pulse sent")

    def _on_haptic_eval_parameter_change(self, params):
        # Refresh the cached per-channel enables when the GUI evaluation
        # panel pushes a ROS parameter update. No control or FT processing
        # logic is touched here -- this only mirrors the new value into the
        # already-existing self.* attributes that the haptic output gates
        # (trigger / body rumble / contact pulse / etc.) read every cycle.
        flags = getattr(self, "_v7_haptic_runtime_flags", ())
        for p in params:
            if p.name in flags:
                try:
                    setattr(self, p.name, bool(p.value))
                except Exception:
                    pass
        return SetParametersResult(successful=True)

    def _pulse_haptic(self, strength: float = 0.7, duration_ms: int = 100, cooldown_s: float = 0.4):
        now = time.monotonic()
        if now - self._last_haptic_s < cooldown_s:
            return
        self._last_haptic_s = now
        eff_strength = clamp(max(float(strength), 0.30) * max(self.v7_haptic_event_gain, 0.0), 0.0, 1.0)
        eff_duration = max(int(duration_ms), int(self.v7_haptic_event_min_duration_ms))
        try:
            self.input.rumble(strength=eff_strength, duration_ms=eff_duration)
            self._v7_event_pulse_until_s = time.monotonic() + (eff_duration / 1000.0)
        except Exception:
            pass

    def _apply_global_ft_guard(self, cmd: List[float]) -> List[float]:
        if not self.v7_ft_global_stop_enable:
            return cmd
        if all(abs(c) < 1e-6 for c in cmd):
            return cmd
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._refresh_ft_fallback(now_s)
        ds = self.ft_guard.data_state(now_s)
        cfg = self.ft_guard.cfg
        if ds != FTDataState.FRESH:
            if cfg.stale_policy == StalePolicy.BLOCK_INWARD:
                return [0.0] * 6
            return cmd

        r = self.ft_guard.last_reading
        force_norm = math.sqrt(r.fx * r.fx + r.fy * r.fy + r.fz * r.fz)
        ratios = [
            abs(r.fx) / max(cfg.fx_limit_n, 1e-6),
            abs(r.fy) / max(cfg.fy_limit_n, 1e-6),
            abs(r.fz) / max(cfg.fz_limit_n, 1e-6),
            force_norm / max(cfg.force_norm_limit_n, 1e-6),
        ]
        if self.v7_ft_global_include_torque:
            torque_norm = math.sqrt(r.tx * r.tx + r.ty * r.ty + r.tz * r.tz)
            ratios.extend([
                abs(r.tx) / max(cfg.tx_limit_nm, 1e-6),
                abs(r.ty) / max(cfg.ty_limit_nm, 1e-6),
                abs(r.tz) / max(cfg.tz_limit_nm, 1e-6),
                torque_norm / max(cfg.torque_norm_limit_nm, 1e-6),
            ])
        max_ratio = max(ratios)
        retreat_cos = self._force_retreat_cos(cmd, r.fx, r.fy, r.fz)
        retreat_requested = retreat_cos >= self.v7_ft_retreat_min_cos
        if max_ratio >= 1.0:
            self._warn_throttle("v7_ft_global_block", f"[V7] FT global stop ratio={max_ratio:.2f}", 0.35)
            if self.v7_ft_allow_retreat_on_limit and retreat_requested:
                retreat_cmd = self._retreat_cmd_from_force_limit(cmd, r.fx, r.fy, r.fz)
                self._warn_throttle(
                    "v7_ft_global_retreat",
                    f"[V7] FT limit active: operator retreat allowed (ratio={max_ratio:.2f})",
                    0.7,
                )
                return retreat_cmd
            # Edge-triggered safety pulse: fire once on entry to hard-stop only.
            # The FT haptic pipeline already conveys the saturated trigger to the
            # operator; re-pulsing every loop felt like rapid recoil.
            if not self._v7_ft_global_stop_latched:
                self._v7_ft_global_stop_latched = True
                self._pulse_haptic(0.95, 110, 0.25)
            return [0.0] * 6
        # Out of hard-stop band: clear latch so a future entry can pulse again.
        self._v7_ft_global_stop_latched = False
        if self.v7_ft_global_soft_slowdown and max_ratio >= cfg.warning_ratio:
            if retreat_requested:
                return cmd
            span = max(1.0 - cfg.warning_ratio, 1e-6)
            scale = clamp(1.0 - (max_ratio - cfg.warning_ratio) / span, 0.0, 1.0)
            scale = max(scale, self.v7_ft_global_soft_min_scale)
            return [c * scale for c in cmd]
        return cmd
    def _joint_ft_scale(self) -> float:
        if not self.v7_ft_global_stop_enable:
            return 1.0
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._refresh_ft_fallback(now_s)
        ds = self.ft_guard.data_state(now_s)
        cfg = self.ft_guard.cfg
        if ds != FTDataState.FRESH:
            return 0.0 if cfg.stale_policy == StalePolicy.BLOCK_INWARD else 1.0

        r = self.ft_guard.last_reading
        force_norm = math.sqrt(r.fx * r.fx + r.fy * r.fy + r.fz * r.fz)
        ratios = [
            abs(r.fx) / max(cfg.fx_limit_n, 1e-6),
            abs(r.fy) / max(cfg.fy_limit_n, 1e-6),
            abs(r.fz) / max(cfg.fz_limit_n, 1e-6),
            force_norm / max(cfg.force_norm_limit_n, 1e-6),
        ]
        if self.v7_ft_global_include_torque:
            torque_norm = math.sqrt(r.tx * r.tx + r.ty * r.ty + r.tz * r.tz)
            ratios.extend([
                abs(r.tx) / max(cfg.tx_limit_nm, 1e-6),
                abs(r.ty) / max(cfg.ty_limit_nm, 1e-6),
                abs(r.tz) / max(cfg.tz_limit_nm, 1e-6),
                torque_norm / max(cfg.torque_norm_limit_nm, 1e-6),
            ])
        max_ratio = max(ratios)
        if max_ratio >= 1.0:
            self._warn_throttle("v7_jik_ft_stop", f"[V7][JIK] FT stop ratio={max_ratio:.2f}", 0.35)
            if not self._v7_ft_jik_stop_latched:
                self._v7_ft_jik_stop_latched = True
                self._pulse_haptic(0.95, 110, 0.25)
            return 0.0
        self._v7_ft_jik_stop_latched = False
        if self.v7_ft_global_soft_slowdown and max_ratio >= cfg.warning_ratio:
            span = max(1.0 - cfg.warning_ratio, 1e-6)
            return max(
                clamp(1.0 - (max_ratio - cfg.warning_ratio) / span, 0.0, 1.0),
                self.v7_ft_global_soft_min_scale,
            )
        return 1.0
    def _tare_sensor_action(self):
        if self.v7_ft_hardware_tare_enable:
            super()._tare_sensor_action()
        else:
            self.get_logger().info(
                "[V7][HAPTICS] Local FT haptic tare applied; hardware FT zero skipped"
            )
        self._v7_ft_haptic_zero = None
        self._reset_v7_sensor_lpf()
        self._v7_filtered_fx = 0.0
        self._v7_filtered_fy = 0.0
        self._v7_filtered_fz = 0.0
        self._v7_ft_feedback_filtered = 0.0
        self._v7_ft_force_filtered_n = 0.0
        self._v7_ft_haptic_ratio_filtered = 0.0
        self._v7_ft_hard_block_active = False
        self._v7_ft_contact_active = False
        self._v7_ft_prev_contact_active = False
        self._v7_ft_contact_on_candidate_since_s = 0.0
        self._v7_ft_contact_off_candidate_since_s = 0.0
        self._v7_ft_contact_shift_candidate_since_s = 0.0
        self._v7_contact_baseline_initialized = False
        self._v7_contact_baseline_n = 0.0
        self._v7_debug_contact_candidate = False
        self._v7_debug_contact_shift_candidate = False
        self._v7_debug_contact_shift_delta_n = 0.0
        self._v7_debug_contact_baseline_n = 0.0
        self._v7_debug_target_ratio_before_gate = 0.0
        self._v7_debug_target_ratio_after_gate = 0.0
        self._v7_debug_selected_force_value = 0.0
        self._v7_debug_selected_force_after_deadband = 0.0
        self._v7_debug_actual_sent_ratio = 0.0
        self._v7_debug_haptics_enabled = False
        self._v7_debug_haptic_reason = "haptics_disabled"
        self._v7_prev_ft_feedback = 0.0
        self._set_ft_trigger_feedback(0.0, self._v7_depth_active_side)
        self._set_ft_body_rumble(0.0)
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._refresh_ft_fallback(now_s)
        if self.ft_guard.data_state(now_s) == FTDataState.FRESH:
            self._capture_ft_haptic_zero(log=True)
        else:
            self.get_logger().info("[V7][HAPTICS] FT haptic baseline will recapture on next fresh sample")
