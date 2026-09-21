"""Motion-mode helpers for the NeuroFinal teleop node."""

import math
import time
from typing import List, Optional, Tuple

from neuro_final_teleop.input.gamepad import InputSnapshot
from neuro_final_teleop.control.math_utils import (
    clamp,
    rpy_deg_to_rotmat,
    sigmoid_shape,
    tool_z_axis_from_rpy_deg,
    vec3_add,
    vec3_cross,
    vec3_norm,
    vec3_scale,
)
from neuro_final_teleop.control.state_machine import TeleopState


class MotionModesMixin:
    def _tip_lock_feedback_pose_mm_deg(self, joints_rad: Optional[List[float]] = None) -> Optional[List[float]]:
        """Pose source used by Tip-Lock capture/feedback loop."""
        if self.v7_tip_lock_pose_source == "sdk_tcp":
            try:
                code_p, pose_sdk = self.arm.get_position(is_radian=False)
            except Exception:
                code_p, pose_sdk = -1, None
            if code_p == 0 and pose_sdk:
                return [float(pose_sdk[i]) for i in range(6)]
            # Fall back to model pose if SDK read fails.
        if joints_rad is None:
            try:
                code_j, joints = self.arm.get_servo_angle(is_radian=True)
            except Exception:
                code_j, joints = -1, None
            if code_j != 0 or not joints:
                return None
            joints_rad = list(joints)
        return self._effective_tip_pose_mm_deg(joints_rad)

    def _on_state_enter(self, old: TeleopState, new: TeleopState, reason: str):
        super()._on_state_enter(old, new, reason)
        #if new == TeleopState.TIP_LOCK_ACTIVE:
        #    self._v7_constrained_settle_until_s = time.monotonic() + self.v7_mode_switch_settle_s
        #else:
        #    self._v7_constrained_settle_until_s = 0.0
        if new in (TeleopState.TIP_LOCK_ACTIVE, TeleopState.RCM_ACTIVE):
            self._v7_constrained_settle_until_s = (
                time.monotonic() + self.v7_mode_switch_settle_s
            )
        else:
            self._v7_constrained_settle_until_s = 0.0
        self._v7_idle_since_s = 0.0
        if new == TeleopState.TIP_LOCK_ACTIVE:
            # Tighten tip-lock correction for v7 and re-anchor on state entry
            # to avoid visible step/shift right after capture.
            self.tip_lock.cfg.correction_gain = float(self.v7_tip_lock_correction_gain)
            self.tip_lock.cfg.correction_deadband_mm = float(self.v7_tip_lock_correction_deadband_mm)
            if hasattr(self, "_v7_fixed_point_ik"):
                self._v7_fixed_point_ik.reset()
            self._v7_tip_lock_joint_cmd_rad_s = [0.0] * 7
            self._v7_last_joint_cmd_rad_s = [0.0] * 7
            if self.v7_tip_lock_reanchor_on_enter:
                try:
                    code_j, joints = self.arm.get_servo_angle(is_radian=True)
                except Exception:
                    code_j, joints = -1, None
                if code_j == 0 and joints:
                    pose = self._tip_lock_feedback_pose_mm_deg(list(joints))
                    if pose:
                        self.tip_lock.locked_pos_mm = [float(pose[0]), float(pose[1]), float(pose[2])]
            self.rate_limiter.reset()

    def _planar_sticks(self, inp: InputSnapshot) -> Tuple[float, float]:
        # Operator-facing convention used in v6:
        # return (forward_cmd, right_cmd)
        return float(inp.ly), float(inp.lx)

    def _planar_sticks_raw(self, inp: InputSnapshot) -> Tuple[float, float]:
        return float(inp.raw_ly), float(inp.raw_lx)

    def _tip_lock_sticks(self, inp: InputSnapshot) -> Tuple[float, float]:
        if self.v7_tip_lock_right_stick_control:
            return float(inp.ry), float(inp.rx)
        return self._planar_sticks(inp)

    def _tip_lock_sticks_raw(self, inp: InputSnapshot) -> Tuple[float, float]:
        if self.v7_tip_lock_right_stick_control:
            return float(inp.raw_ry), float(inp.raw_rx)
        return self._planar_sticks_raw(inp)

    def _tip_lock_operator_angular_rad_s(
        self,
        pose: List[float],
        fwd_cmd: float,
        right_cmd: float,
        max_w_rad_s: float,
        op_scale: float,
    ) -> List[float]:
        # Operator convention is unchanged: stick up/down and left/right map to
        # pitch/roll-like angular commands. In tool-frame mode those commands
        # are first expressed in the current end-effector frame, then rotated
        # into base frame because xArm Cartesian velocity is sent in base coords.
        local_w = [
            -float(right_cmd) * float(op_scale) * float(max_w_rad_s),
            float(fwd_cmd) * float(op_scale) * float(max_w_rad_s),
            0.0,
        ]
        if not self.v7_tip_lock_tool_frame_control:
            return local_w
        R = rpy_deg_to_rotmat(float(pose[3]), float(pose[4]), float(pose[5]))
        base_w = R @ local_w
        return [float(base_w[0]), float(base_w[1]), float(base_w[2])]

    def _depth_axis_inward(self, inp: InputSnapshot) -> float:
        """Operator depth convention: R2 is inward/insertion, L2 is outward."""
        return float(inp.rt) - float(inp.lt)

    def _insertion_sign(self) -> float:
        return -1.0 if self.insertion_along_neg_tool_z else 1.0

    def _tool_depth_velocity_mm_s(self, pose: List[float], inward_speed_mm_s: float) -> List[float]:
        tool_z = tool_z_axis_from_rpy_deg(pose[3], pose[4], pose[5])
        return vec3_scale(tool_z, self._insertion_sign() * float(inward_speed_mm_s))

    def _base_depth_velocity_mm_s(self, inward_speed_mm_s: float) -> List[float]:
        # Base-frame operator convention: R2 inward/down is base -Z.
        return [0.0, 0.0, -float(inward_speed_mm_s)]

    def _operator_is_idle(self, inp: InputSnapshot) -> bool:
        dead = 0.10
        hard_dead = dead * 1.35
        fwd_cmd, right_cmd = self._planar_sticks(inp)
        raw_fwd_cmd, raw_right_cmd = self._planar_sticks_raw(inp)
        return (
            abs(fwd_cmd) < dead
            and abs(right_cmd) < dead
            and abs(raw_fwd_cmd) < hard_dead
            and abs(raw_right_cmd) < hard_dead
            and abs(float(inp.lt)) < dead
            and abs(float(inp.rt)) < dead
            and abs(float(inp.rx)) < dead
            and abs(float(inp.ry)) < dead
        )

    def _handle_speed_scale_dpad(self, inp: InputSnapshot) -> None:
        _, dpad_y = inp.dpad
        if dpad_y == 0:
            self._v7_last_dpad_y = 0
            return
        if dpad_y == self._v7_last_dpad_y:
            return
        self._v7_last_dpad_y = dpad_y
        old = self._v7_speed_scale
        self._v7_speed_scale = clamp(
            old + (float(dpad_y) * self.v7_speed_scale_step),
            self.v7_speed_scale_min,
            self.v7_speed_scale_max,
        )
        if abs(self._v7_speed_scale - old) > 1e-6:
            self.get_logger().info(f"[V7] Speed scale {100.0 * self._v7_speed_scale:.0f}%")
            self._pulse_haptic(0.45, 60, 0.05)

    def _apply_v7_speed_scale(self, cmd: List[float]) -> List[float]:
        s = float(self._v7_speed_scale)
        return [cmd[0] * s, cmd[1] * s, cmd[2] * s, cmd[3] * s, cmd[4] * s, cmd[5] * s]

    def _clamp_linear_speed(self, v_xyz: List[float], max_mm_s: float) -> List[float]:
        v_norm = vec3_norm(v_xyz)
        lim = max(float(max_mm_s), 1e-6)
        if v_norm <= lim:
            return v_xyz
        s = lim / max(v_norm, 1e-9)
        return vec3_scale(v_xyz, s)
    def _vel_free_teleop(self, inp: InputSnapshot, scale: float, joints_rad: List[float]) -> List[float]:
        sg = self.sigmoid_gain
        fwd_cmd, right_cmd = self._planar_sticks(inp)
        depth_axis_raw = self._depth_axis_inward(inp)
        if abs(depth_axis_raw) > 0.05:
            if abs(fwd_cmd) < self.v7_depth_planar_deadband:
                fwd_cmd = 0.0
            if abs(right_cmd) < self.v7_depth_planar_deadband:
                right_cmd = 0.0
        vx = sigmoid_shape(fwd_cmd, sg) * self.max_linear_mm_s * scale
        vy = sigmoid_shape(right_cmd, sg) * self.max_linear_mm_s * scale
        # Positive depth_speed is inward/insertion: R2 positive, L2 negative.
        depth_speed = sigmoid_shape(depth_axis_raw, sg) * self.max_z_mm_s
        if abs(depth_speed) < 1e-6:
            return [vx, vy, 0.0, 0.0, 0.0, 0.0]
        if self.v7_use_tool_z_depth_in_free:
            pose = self._effective_tip_pose_mm_deg(joints_rad)
            if pose:
                v_ins = self._tool_depth_velocity_mm_s(pose, depth_speed * scale)
                return [vx + v_ins[0], vy + v_ins[1], v_ins[2], 0.0, 0.0, 0.0]
        # Base-Z mode is operator-facing vertical motion: R2 = down (-Z),
        # L2 = up (+Z), independent of tool/needle orientation.
        v_depth = self._base_depth_velocity_mm_s(depth_speed * scale)
        return [vx, vy, v_depth[2], 0.0, 0.0, 0.0]

    def _tip_lock_joint_ik_active(self) -> bool:
        return bool(
            self.v7_tip_lock_joint_ik_enable
            and self.sm.state == TeleopState.TIP_LOCK_ACTIVE
            and self.tip_lock.active
        )
    
    def _rcm_joint_ik_active(self) -> bool:
        """Return True while RCM owns joint-space motion."""
        return (
            self.v7_rcm_enable
            and self.sm.state == TeleopState.RCM_ACTIVE
            and self._v7_rcm_controller.active
        )

    def _joint_ik_active(self) -> bool:
        """True when either constrained controller uses mode 4."""
        return self._tip_lock_joint_ik_active() or self._rcm_joint_ik_active()

    def _vel_rcm_joint_ik(
        self,
        inp: InputSnapshot,
        scale: float,
        joints_rad: List[float],
    ) -> List[float]:
        """Use the right stick to rotate about the captured RCM point."""

        # Hold zero briefly after capture/mode transition.
        if time.monotonic() < self._v7_constrained_settle_until_s:
            self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
            return [0.0] * 6

        rx = 0.0 if abs(inp.rx) < 0.10 else sigmoid_shape(inp.rx, self.sigmoid_gain)
        ry = 0.0 if abs(inp.ry) < 0.10 else sigmoid_shape(inp.ry, self.sigmoid_gain)

        stick_norm = math.hypot(rx, ry)

        if stick_norm > 1.0:
            rx /= stick_norm
            ry /= stick_norm

        max_w = math.radians(self.v7_rcm_max_angular_deg_s)
        speed = max_w * self._v7_speed_scale * scale
        desired_insertion_m_s = 0.0

        if self.v7_rcm_insertion_enable:
            depth_axis = self._depth_axis_inward(inp)

            depth_cmd = (
                0.0
                if abs(depth_axis) < 0.10
                else sigmoid_shape(
                    depth_axis,
                    self.sigmoid_gain,
                )
            )

            desired_insertion_m_s = (
                depth_cmd
                * self.v7_rcm_max_insertion_mm_s
                * 0.001
                * self._v7_speed_scale
                * scale
            )

        # Right stick commands tool-frame X/Y rotation.
        local_w = [-rx * speed, ry * speed, 0.0]

        pose = self._effective_tip_pose_mm_deg(joints_rad)
        if not pose:
            self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
            return [0.0] * 6

        R = rpy_deg_to_rotmat(pose[3], pose[4], pose[5])
        base_w = R @ local_w

        tcp_m = [
            value * 0.001
            for value in self._effective_tcp_translation_mm()
        ]

        try:
            result = self._v7_rcm_controller.solve(
                q_rad=joints_rad[:7],
                desired_angular_rad_s=base_w.tolist(),
                desired_insertion_m_s=desired_insertion_m_s,
                tcp_offset_m=tcp_m,
                dt_s=self.dt,
                tcp_rotation=self._robot_tcp_rotation(),
            )
        except Exception as exc:
            self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
            self._warn_throttle(
                "rcm_solve",
                f"[V7][RCM] Solve failed: {exc}",
                0.5,
            )
            return [0.0] * 6

        self._v7_rcm_joint_cmd_rad_s = result.qdot_rad_s

        if hasattr(self, "_publish_rcm_diagnostics"):
            self._publish_rcm_diagnostics(
                result,
                base_w.tolist(),
                joints_rad[:7],
            )

        # Keep six-axis values only for existing diagnostics.
        self._last_cmd_sent = [
            *result.correction_mm_s,
            *[math.degrees(w) for w in result.angular_rad_s],
        ]

        if result.lateral_error_mm > 1.0:
            self._warn_throttle(
                "rcm_error",
                f"[V7][RCM] Entry error: {result.lateral_error_mm:.2f} mm",
                0.25,
            )

        # The real command is the stored seven-joint velocity.
        return [0.0] * 6


    def _vel_tip_lock_joint_ik(
        self,
        inp: InputSnapshot,
        scale: float,
        joints_rad: List[float],
        pose: List[float],
    ) -> List[float]:
        dead = 0.10
        fwd_cmd, right_cmd = self._tip_lock_sticks(inp)
        fwd_cmd = 0.0 if abs(fwd_cmd) < dead else float(fwd_cmd)
        right_cmd = 0.0 if abs(right_cmd) < dead else float(right_cmd)
        rt = float(inp.rt) if float(inp.rt) > dead else 0.0
        lt = float(inp.lt) if float(inp.lt) > dead else 0.0
        depth_axis = rt - lt

        op_scale = float(self._v7_speed_scale)
        max_w_rad_s = math.radians(float(self.tip_lock.cfg.max_angular_deg_s))
        desired_w = self._tip_lock_operator_angular_rad_s(
            pose, fwd_cmd, right_cmd, max_w_rad_s, op_scale
        )

        depth_input = sigmoid_shape(depth_axis, self.sigmoid_gain)
        depth_speed = depth_input * self.max_depth_mm_s * self.depth_gain * op_scale
        if abs(depth_speed) > 0.001:
            if self.v7_use_tool_z_depth_in_tip_lock:
                depth_delta = vec3_scale(
                    self._tool_depth_velocity_mm_s(pose, depth_speed),
                    self.dt,
                )
            else:
                depth_delta = vec3_scale(self._base_depth_velocity_mm_s(depth_speed), self.dt)
            self.tip_lock.locked_pos_mm = vec3_add(
                self.tip_lock.locked_pos_mm,
                depth_delta,
            )

        tcp_m = [v * 0.001 for v in self._effective_tcp_translation_mm()]
        result = self._v7_fixed_point_ik.solve(
            joints_rad,
            list(self.tip_lock.locked_pos_mm),
            [float(pose[0]), float(pose[1]), float(pose[2])],
            desired_w,
            tcp_m,
            self.dt,
        )
        self.tip_lock.last_pos_err_mm = float(result.tip_error_mm)
        self._v7_tip_lock_joint_cmd_rad_s = list(result.qdot_rad_s)
        self._last_cmd_sent = [
            result.correction_mm_s[0] * scale,
            result.correction_mm_s[1] * scale,
            result.correction_mm_s[2] * scale,
            math.degrees(result.angular_rad_s[0]) * scale,
            math.degrees(result.angular_rad_s[1]) * scale,
            math.degrees(result.angular_rad_s[2]) * scale,
        ]
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.v7_tip_lock_debug and now_s - self._v7_last_joint_ik_dbg_s >= self.v7_tip_lock_debug_interval_s:
            self._v7_last_joint_ik_dbg_s = now_s
            qmax = max(abs(v) for v in result.qdot_rad_s) if result.qdot_rad_s else 0.0
            self.get_logger().warn(
                "[V7][JIK] "
                f"tip_err={result.tip_error_mm:.2f}mm "
                f"qdot_max={qmax:.3f}rad/s "
                f"limited={result.limited} "
                f"corr=({result.correction_mm_s[0]:+.1f},"
                f"{result.correction_mm_s[1]:+.1f},{result.correction_mm_s[2]:+.1f})mm/s"
            )
        return [0.0] * 6

    def _vel_tip_lock(self, inp: InputSnapshot, scale: float, joints_rad: List[float]) -> List[float]:
        if time.monotonic() < self._v7_constrained_settle_until_s:
            return [0.0] * 6
        pose = self._tip_lock_feedback_pose_mm_deg(joints_rad)
        if not pose:
            return [0.0] * 6
        if self._tip_lock_joint_ik_active():
            return self._vel_tip_lock_joint_ik(inp, scale, joints_rad, pose)
        dead = 0.10
        hard_dead = dead * 1.25
        fwd_cmd, right_cmd = self._tip_lock_sticks(inp)
        raw_fwd_cmd, raw_right_cmd = self._tip_lock_sticks_raw(inp)
        rt = float(inp.rt) if float(inp.rt) > dead else 0.0
        lt = float(inp.lt) if float(inp.lt) > dead else 0.0
        # Positive depth axis is inward/insertion: R2 positive, L2 negative.
        depth_axis = rt - lt
        if (
            abs(fwd_cmd) < dead
            and abs(right_cmd) < dead
            and abs(raw_fwd_cmd) < hard_dead
            and abs(raw_right_cmd) < hard_dead
            and abs(depth_axis) < dead
        ):
            lin_v, _ = self.tip_lock.compute_velocity(pose, 0.0, 0.0)
            tip_err = float(getattr(self.tip_lock, "last_pos_err_mm", 0.0))
            dynamic_cap = min(
                self.max_linear_mm_s,
                max(float(self.v7_tip_lock_max_linear_mm_s), 0.5),
            )
            lin_v = self._clamp_linear_speed(lin_v, dynamic_cap)
            if vec3_norm(lin_v) > 1e-3:
                self._v7_idle_since_s = 0.0
                return [
                    lin_v[0] * scale, lin_v[1] * scale, lin_v[2] * scale,
                    0.0, 0.0, 0.0,
                ]

            now_mono = time.monotonic()
            if self._v7_idle_since_s <= 0.0:
                self._v7_idle_since_s = now_mono
            idle_for_s = now_mono - self._v7_idle_since_s
            # Re-anchor only if explicitly enabled, operator stayed idle long enough,
            # and lock error is already small. This prevents lock-point jumps.
            if (
                self.v7_tip_idle_reanchor
                and self.tip_lock.active
                and idle_for_s >= self.v7_tip_idle_reanchor_dwell_s
                and tip_err <= self.v7_tip_idle_reanchor_max_err_mm
            ):
                self.tip_lock.locked_pos_mm = [float(pose[0]), float(pose[1]), float(pose[2])]
            self.rate_limiter.reset()
            return [0.0] * 6
        self._v7_idle_since_s = 0.0

        op_scale = float(self._v7_speed_scale)
        lin_v, ang_v = self.tip_lock.compute_velocity(pose, 0.0, 0.0)
        max_w_rad_s = math.radians(float(self.tip_lock.cfg.max_angular_deg_s))
        w_rad_tool_or_base = self._tip_lock_operator_angular_rad_s(
            pose, fwd_cmd, right_cmd, max_w_rad_s, op_scale
        )
        ang_v = [math.degrees(float(w)) for w in w_rad_tool_or_base]
        tip_lin_cap = float(self.v7_tip_lock_max_linear_mm_s)
        rec_err = float(self.v7_tip_lock_recovery_err_mm)
        rec_gain = float(self.v7_tip_lock_recovery_gain_mm_s_per_mm)
        min_ang_scale = float(self.v7_tip_lock_min_angular_scale)
        tip_err = float(getattr(self.tip_lock, "last_pos_err_mm", 0.0))
        if tip_err > rec_err:
            over = tip_err - rec_err
            dynamic_cap = tip_lin_cap + rec_gain * over
            ang_scale = max(min_ang_scale, rec_err / max(tip_err, 1e-6))
            ang_v = [ang_scale * a for a in ang_v]
        else:
            dynamic_cap = tip_lin_cap

        # Additional "error brake": if tip drift grows, strongly damp angular
        # command so linear correction can recenter the lock point.
        if self.v7_tip_lock_error_brake_enable and tip_err > self.v7_tip_lock_error_brake_mm:
            ratio = self.v7_tip_lock_error_brake_mm / max(tip_err, 1e-6)
            brake_scale = clamp(ratio ** self.v7_tip_lock_error_brake_exp, min_ang_scale, 1.0)
            ang_v = [brake_scale * a for a in ang_v]
        dynamic_cap = min(self.max_linear_mm_s, max(dynamic_cap, 0.5))

        # Compensate rotation-induced tip translation caused by nonzero TCP offset.
        # When the low-level controller effectively rotates about flange, this term
        # cancels the induced tip motion: v = -omega x r_tcp.
        if self.v7_tip_lock_tcp_rotation_comp_enable:
            eff_tcp = self._effective_tcp_translation_mm()
            off_z_mm = float(eff_tcp[2]) if len(eff_tcp) >= 3 else 0.0
            if abs(off_z_mm) > 1e-6:
                tool_z = tool_z_axis_from_rpy_deg(pose[3], pose[4], pose[5])
                r_tcp = vec3_scale(tool_z, off_z_mm)
                w_rad = [
                    math.radians(float(ang_v[0])),
                    math.radians(float(ang_v[1])),
                    math.radians(float(ang_v[2])),
                ]
                v_rot_comp = vec3_scale(
                    vec3_cross(w_rad, r_tcp), -float(self.v7_tip_lock_tcp_rotation_comp_gain)
                )
                lin_v = vec3_add(lin_v, v_rot_comp)

        code_p, pose_sdk = -1, None
        if self.v7_tip_lock_pose_source != "sdk_tcp":
            try:
                code_p, pose_sdk = self.arm.get_position(is_radian=False)
            except Exception:
                code_p, pose_sdk = -1, None
        if code_p == 0 and pose_sdk:
            d = [
                float(pose[0] - pose_sdk[0]),
                float(pose[1] - pose_sdk[1]),
                float(pose[2] - pose_sdk[2]),
            ]
            d_norm = vec3_norm(d)
            w_rad = [
                math.radians(float(ang_v[0])),
                math.radians(float(ang_v[1])),
                math.radians(float(ang_v[2])),
            ]
            w_norm = vec3_norm(w_rad)
            if d_norm > 1e-3 and w_norm > 1e-6:
                w_cap = (0.85 * dynamic_cap) / d_norm
                if w_norm > w_cap and w_cap > 1e-6:
                    s = w_cap / w_norm
                    ang_v = [a * s for a in ang_v]
                    w_rad = [w * s for w in w_rad]
                v_ff = vec3_scale(vec3_cross(w_rad, d), -1.0)
                lin_v = vec3_add(lin_v, v_ff)
        # Allow insertion/extraction while fixed-tip mode is active by shifting
        # the lock point with the trigger command.
        depth_input = sigmoid_shape(depth_axis, self.sigmoid_gain)
        depth_speed = depth_input * self.max_depth_mm_s * self.depth_gain * op_scale
        if abs(depth_speed) > 0.001:
            if self.v7_use_tool_z_depth_in_tip_lock:
                v_ins = self._tool_depth_velocity_mm_s(pose, depth_speed)
            else:
                v_ins = self._base_depth_velocity_mm_s(depth_speed)
            lin_v = vec3_add(lin_v, v_ins)
            if self.tip_lock.active:
                self.tip_lock.locked_pos_mm = vec3_add(
                    self.tip_lock.locked_pos_mm,
                    vec3_scale(v_ins, self.dt),
                )
        lin_v = self._clamp_linear_speed(lin_v, dynamic_cap)
        if self.v7_tip_lock_debug:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self._v7_last_tip_dbg_s >= self.v7_tip_lock_debug_interval_s:
                self._v7_last_tip_dbg_s = now_s
                lock = list(getattr(self.tip_lock, "locked_pos_mm", [0.0, 0.0, 0.0]))
                err_v = [float(lock[i] - pose[i]) for i in range(3)]
                err_n = math.sqrt(err_v[0] * err_v[0] + err_v[1] * err_v[1] + err_v[2] * err_v[2])
                if err_n >= self.v7_tip_lock_debug_err_threshold_mm:
                    try:
                        code_sdk, pose_sdk = self.arm.get_position(is_radian=False)
                    except Exception:
                        code_sdk, pose_sdk = -1, None
                    if code_sdk == 0 and pose_sdk:
                        model_vs_sdk = [
                            float(pose[0] - pose_sdk[0]),
                            float(pose[1] - pose_sdk[1]),
                            float(pose[2] - pose_sdk[2]),
                        ]
                        model_vs_sdk_n = math.sqrt(
                            model_vs_sdk[0] * model_vs_sdk[0]
                            + model_vs_sdk[1] * model_vs_sdk[1]
                            + model_vs_sdk[2] * model_vs_sdk[2]
                        )
                    else:
                        model_vs_sdk = [0.0, 0.0, 0.0]
                        model_vs_sdk_n = -1.0
                    self.get_logger().warn(
                        "[V7][TIPDBG] "
                        f"err_mm={err_n:.2f} "
                        f"err_v=({err_v[0]:+.2f},{err_v[1]:+.2f},{err_v[2]:+.2f}) "
                        f"lock=({lock[0]:.1f},{lock[1]:.1f},{lock[2]:.1f}) "
                        f"tip=({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}) "
                        f"lin_cmd=({lin_v[0]:+.2f},{lin_v[1]:+.2f},{lin_v[2]:+.2f}) "
                        f"ang_cmd=({ang_v[0]:+.2f},{ang_v[1]:+.2f},{ang_v[2]:+.2f}) "
                        f"m_vs_sdk_mm={model_vs_sdk_n:.2f} "
                        f"m_vs_sdk_v=({model_vs_sdk[0]:+.2f},{model_vs_sdk[1]:+.2f},{model_vs_sdk[2]:+.2f}) "
                        f"depth_axis={depth_axis:+.3f}"
                    )
        return [
            lin_v[0] * scale, lin_v[1] * scale, lin_v[2] * scale,
            ang_v[0] * scale, ang_v[1] * scale, ang_v[2] * scale,
        ]

    def _desired_control_mode(self) -> int:
        return 4 if self._joint_ik_active() else 5

    def _rate_limit_is_constrained(self) -> bool:
        # Joint controllers apply their own acceleration limits.
        if self._joint_ik_active():
            return False
        return super()._rate_limit_is_constrained()

    def _ensure_mode4(self) -> bool:
        mode = getattr(self.arm, "mode", None)
        state = getattr(self.arm, "state", None)
        err = int(getattr(self.arm, "error_code", 0) or 0)
        warn = int(getattr(self.arm, "warn_code", 0) or 0)
        if err != 0 or warn != 0:
            return False
        if mode == 4 and state in (0, 1, 2):
            self._current_mode = 4
            return True
        try:
            self.arm.motion_enable(True)
            if mode != 4:
                ret_m = self.arm.set_mode(4)
                if isinstance(ret_m, int) and ret_m != 0:
                    return False
            ret_s = self.arm.set_state(0)
            if isinstance(ret_s, int) and ret_s != 0:
                return False
            time.sleep(0.01)
        except Exception as exc:
            self._warn_throttle("mode4_rec", f"Mode4 recover: {exc}", 0.5)
            return False
        mode = getattr(self.arm, "mode", None)
        state = getattr(self.arm, "state", None)
        if mode == 4 and state in (0, 1, 2):
            self._current_mode = 4
            return True
        return False
    def _send_joint_zero(self, reason: str = "") -> None:
        try:
            if not self._ensure_mode4():
                return
            zero = [0.0] * 7
            self.arm.vc_set_joint_velocity(
                zero,
                is_radian=True,
                is_sync=True,
                duration=max(self.velocity_watchdog_s, 0.02),
            )
            #self._v7_last_joint_cmd_rad_s = zero
            #self._v7_tip_lock_joint_cmd_rad_s = zero
            #self._v7_fixed_point_ik.reset()
            self._v7_last_joint_cmd_rad_s = zero
            self._v7_tip_lock_joint_cmd_rad_s = zero
            self._v7_rcm_joint_cmd_rad_s = zero
            self._v7_fixed_point_ik.reset()
        except Exception as exc:
            self._warn_throttle("v7_jik_zero", f"[V7][JIK] zero failed ({reason}): {exc}", 0.5)

    def _send_velocity(self, cmd: List[float]):
        """Send Cartesian or joint velocity for the active controller."""

        if not self._joint_ik_active():
            if self._v7_joint_velocity_active:
                self._v7_joint_velocity_active = False
                self._current_mode = None
            return super()._send_velocity(cmd)

        if not self._ensure_mode4():
            self._warn_throttle(
                "joint_mode",
                "[V7] Joint command skipped: mode 4 unavailable",
                0.4,
            )
            return

        # Select the joint command produced during this control cycle.
        if self._rcm_joint_ik_active():
            source = self._v7_rcm_joint_cmd_rad_s
        else:
            source = self._v7_tip_lock_joint_cmd_rad_s

        # Apply the existing force/torque safety scale.
        ft_scale = self._joint_ft_scale()
        qdot = [float(v) * ft_scale for v in source[:7]]

        if max(abs(v) for v in qdot) < 1e-5:
            qdot = [0.0] * 7

        ret = self.arm.vc_set_joint_velocity(
            qdot,
            is_radian=True,
            is_sync=False,
            duration=self.velocity_watchdog_s,
        )

        if isinstance(ret, int) and ret != 0:
            self._warn_throttle(
                "joint_send",
                f"[V7] Joint velocity failed: code={ret}",
                0.25,
            )
            self._send_joint_zero("send_failure")
            return

        self._v7_joint_velocity_active = True
        self._v7_last_joint_cmd_rad_s = qdot

    def _on_state_exit(self, old: TeleopState, new: TeleopState, reason: str):
        leaving_joint_mode = (
            old == TeleopState.RCM_ACTIVE
            or (
                old == TeleopState.TIP_LOCK_ACTIVE
                and self.v7_tip_lock_joint_ik_enable
            )
        )

        if leaving_joint_mode:
            # Stop mode-4 motion before clearing controller state.
            self._send_joint_zero(f"exit_{old.value}")
            self._v7_joint_velocity_active = False
            self._current_mode = None

        if (
            old == TeleopState.TIP_LOCK_ACTIVE
            and new != TeleopState.TIP_LOCK_ACTIVE
        ):
            self._v7_tip_lock_started_by_r3 = False

        if (
            old == TeleopState.RCM_ACTIVE
            and new != TeleopState.RCM_ACTIVE
        ):
            self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
            self._v7_rcm_controller.reset()

        super()._on_state_exit(
            old,
            new,
            reason,
        )

    def _compute_velocity(self, inp: InputSnapshot, scale: float, joints_rad: List[float]) -> List[float]:
        st = self.sm.state
        self._handle_speed_scale_dpad(inp)
        operator_idle = self._operator_is_idle(inp)
        if st == TeleopState.FREE_TELEOP and operator_idle:
            if self._v7_idle_since_s <= 0.0:
                self._v7_idle_since_s = time.monotonic()
            self.rate_limiter.reset()
            cmd = [0.0] * 6
            self._update_ft_force_haptics(inp)
            self._update_motion_haptics(cmd, inp)
            return self._apply_global_ft_guard(cmd)
        if not (st == TeleopState.TIP_LOCK_ACTIVE and operator_idle):
            self._v7_idle_since_s = 0.0
        #if st == TeleopState.FREE_TELEOP:
        #    cmd = self._vel_free_teleop(inp, scale, joints_rad)
        #elif st == TeleopState.TIP_LOCK_ACTIVE:
        #    cmd = self._vel_tip_lock(inp, scale, joints_rad)
        #else:
        #    cmd = [0.0] * 6
        if st == TeleopState.FREE_TELEOP:
            cmd = self._vel_free_teleop(inp, scale, joints_rad)
        elif st == TeleopState.TIP_LOCK_ACTIVE:
            cmd = self._vel_tip_lock(inp, scale, joints_rad)
        elif st == TeleopState.RCM_ACTIVE:
            cmd = self._vel_rcm_joint_ik(inp, scale, joints_rad)
        else:
            cmd = [0.0] * 6
        #if st != TeleopState.TIP_LOCK_ACTIVE:
        #    cmd = self._apply_v7_speed_scale(cmd)
        if st not in (TeleopState.TIP_LOCK_ACTIVE, TeleopState.RCM_ACTIVE):
            cmd = self._apply_v7_speed_scale(cmd)
        self._update_ft_force_haptics(inp)
        self._update_motion_haptics(cmd, inp)
        return self._apply_global_ft_guard(cmd)

    def _do_tip_lock_capture(self):
        """Capture Tip-Lock anchor using selected v7 pose source."""
        try:
            code_j, joints = self.arm.get_servo_angle(is_radian=True)
        except Exception:
            code_j, joints = -1, None
        joints_for_fallback = list(joints) if (code_j == 0 and joints) else None
        pose = self._tip_lock_feedback_pose_mm_deg(joints_for_fallback)
        if not pose:
            self._warn_throttle("tip_cap_v7", "[V7] Tip-Lock capture: pose unavailable", 0.5)
            self._v7_tip_lock_started_by_r3 = False
            self.sm.transition_to(TeleopState.FREE_TELEOP, "capture_pose_fail")
            return
        self.tip_lock.capture(pose)
        self.get_logger().info(
            f"[V7] TIP-LOCK captured ({self.v7_tip_lock_pose_source}) at "
            f"({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f})"
        )
        self._pulse_haptic(1.0, 180, 0.0)
        self.sm.transition_to(TeleopState.TIP_LOCK_ACTIVE, "tip_captured")
    
    def _do_rcm_capture(self):
        """Capture the current effective tool tip as the RCM entry point."""

        try:
            code_j, joints = self.arm.get_servo_angle(
                is_radian=True
            )
        except Exception as exc:
            self._warn_throttle(
                "rcm_capture_joint_exception",
                f"[V7][RCM] Joint read exception: {exc}",
                0.5,
            )
            self.sm.transition_to(
                TeleopState.FREE_TELEOP,
                "rcm_capture_joint_exception",
            )
            return

        if (
            code_j != 0
            or not joints
            or len(joints) < 7
        ):
            self._warn_throttle(
                "rcm_capture_joint_failure",
                "[V7][RCM] Capture failed: joints unavailable",
                0.5,
            )
            self.sm.transition_to(
                TeleopState.FREE_TELEOP,
                "rcm_capture_joint_failure",
            )
            return

        tcp_offset_m = [
            float(value) * 0.001
            for value
            in self._effective_tcp_translation_mm()
        ]

        captured = self._v7_rcm_controller.capture(
            [float(value) for value in joints[:7]],
            tcp_offset_m,
            tcp_rotation=self._robot_tcp_rotation(),
        )

        if not captured:
            self._warn_throttle(
                "rcm_capture_failure",
                "[V7][RCM] Entry-point capture failed",
                0.5,
            )
            self.sm.transition_to(
                TeleopState.FREE_TELEOP,
                "rcm_capture_failure",
            )
            return

        entry = self._v7_rcm_controller.entry_point_m
        axis = self._v7_rcm_controller.captured_axis

        if entry is None or axis is None:
            self._v7_rcm_controller.reset()
            self.sm.transition_to(
                TeleopState.FREE_TELEOP,
                "rcm_capture_invalid_result",
            )
            return

        self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
        self._v7_last_joint_cmd_rad_s = [0.0] * 7

        self.get_logger().info(
            "[V7][RCM] Entry captured at "
            f"({entry[0] * 1000.0:.1f},"
            f"{entry[1] * 1000.0:.1f},"
            f"{entry[2] * 1000.0:.1f}) mm | "
            "shaft axis="
            f"({axis[0]:+.3f},"
            f"{axis[1]:+.3f},"
            f"{axis[2]:+.3f})"
        )

        self._pulse_haptic(
            1.0,
            180,
            0.0,
        )

        self.sm.transition_to(
            TeleopState.RCM_ACTIVE,
            "rcm_entry_captured",
        )

    def _handle_button_actions(self, inp: InputSnapshot):
        st = self.sm.state

        if st == TeleopState.FREE_TELEOP:
            # Options enters RCM capture.
            if (
                self.v7_rcm_enable
                and getattr(inp, "options_edge", False)
            ):
                if not self._constrained_modes_enabled:
                    self._warn_throttle(
                        "v7_rcm_kinematics_not_validated",
                        "[V7][RCM] Capture blocked: "
                        "kinematic model not validated",
                        1.0,
                    )
                    return

                self._v7_tip_lock_started_by_r3 = False

                self.sm.transition_to(
                    TeleopState.RCM_CAPTURE,
                    "v7_options_rcm_capture",
                )
                return

            # Square performs FT tare.
            if inp.x_edge:
                self._tare_sensor_action()

            # L1 moves to the initial pose.
            elif inp.lb_edge:
                if not self.enable_initial_pose_action:
                    self._warn_throttle(
                        "v7_init_pose_disabled",
                        "[V7] Initial-pose action disabled",
                        1.0,
                    )
                    return

                self._start_initial_pose()

            # Keep the existing R3 Tip Lock behavior unchanged.
            elif (
                self.v7_tip_lock_r3_toggle_enable
                and getattr(inp, "r3_edge", False)
            ):
                if not self._constrained_modes_enabled:
                    self._warn_throttle(
                        "v7_kin_nv",
                        "[V7] R3 fixed-tip blocked: "
                        "kinematic model not validated",
                        1.0,
                    )
                    return

                self._v7_tip_lock_started_by_r3 = True

                self.sm.transition_to(
                    TeleopState.TIP_LOCK_CAPTURE,
                    "v7_r3_click_fixed_tip",
                )

            # Cross enters Tip Lock.
            elif inp.a_edge:
                if not self._constrained_modes_enabled:
                    self._warn_throttle(
                        "v7_kin_nv",
                        "[V7] Fixed-tip blocked: "
                        "kinematic model not validated",
                        1.0,
                    )
                    return

                self._v7_tip_lock_started_by_r3 = False

                self.sm.transition_to(
                    TeleopState.TIP_LOCK_CAPTURE,
                    "v7_btn_cross_fixed_tip",
                )

            # Triangle starts alignment.
            elif inp.y_edge:
                self._start_alignment()

        elif st == TeleopState.TIP_LOCK_ACTIVE:
            if inp.a_edge:
                self._v7_tip_lock_started_by_r3 = False

                self.sm.transition_to(
                    TeleopState.FREE_TELEOP,
                    "v7_fixed_tip_unlock",
                )

        elif st == TeleopState.RCM_ACTIVE:
            # Options exits RCM mode.
            if getattr(inp, "options_edge", False):
                self.sm.transition_to(
                    TeleopState.FREE_TELEOP,
                    "v7_options_rcm_exit",
                )
