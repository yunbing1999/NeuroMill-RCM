"""
TeleopV4Node -- shared base ROS 2 node for neuro_final_teleop.

Orchestrates state machine, input, safety, kinematics, and constrained
controllers in a single-threaded timer loop.  Uses xArm SDK for robot
connection and PyKDL for kinematic computations.

Every velocity command is issued with a positive duration (watchdog).
If the control loop stalls, the robot firmware stops automatically.

Dry run (no robot, no joystick):
  - ROS: ``ros2 run neuro_final_teleop neuro_final_teleop --ros-args -p dry_run:=true``
  - Or set environment variable ``NEURO_FINAL_DRY_RUN=1`` (same effect as default param + env OR).
  - For automated tests: ``-p enable_control_timer:=false`` to skip the periodic timer;
    call ``_control_loop_inner()`` manually if needed.
"""

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


def _import_xarm_api():
    """Lazy import so imports work without xarm-python-sdk when dry_run."""
    try:
        from xarm.wrapper import XArmAPI
    except ImportError as exc:
        raise ImportError(
            "xarm-python-sdk is required when dry_run is false. "
            "Install the SDK, or run with dry_run:=true or NEURO_FINAL_DRY_RUN=1."
        ) from exc
    return XArmAPI


from .ft_guard import FTGuard, FTGuardConfig, StalePolicy
from neuro_final_teleop.input.gamepad import InputConfig, InputSnapshot, XboxInput
from .kinematics import (KDLKinModel,load_joint_origins,)
from .math_utils import clamp, rpy_deg_to_rotmat, sigmoid_shape
from .tip_lock_controller import TipLockConfig, TipLockController
from .safety import (
    JointRiskConfig,
    JointRiskMonitor,
    RateLimiter,
    RateLimiterConfig,
)
from .state_machine import StateMachine, TeleopState


class TeleopV4Node(Node):

    def __init__(self, node_name: str = "base_teleop_node"):
        super().__init__(node_name)

        self._declare_params()
        self._load_params()

        # Build the kinematic model before connecting to the robot.
        if self.kinematics_yaml:
            origins = load_joint_origins(
                self.kinematics_yaml
            )
            self.kin = KDLKinModel(origins)

            self.get_logger().info(
                "Using calibrated kinematics: "
                f"{self.kinematics_yaml}"
            )
        else:
            self.kin = KDLKinModel()

            self.get_logger().warning(
                "No kinematics_yaml provided; "
                "using nominal xArm7 kinematics"
            )

        if self.dry_run:
            self.get_logger().info("DRY RUN mode -- no robot connection")
            self.arm = _FakeArm()
        else:
            self.get_logger().info(f"Connecting to xArm at {self.robot_ip}")
            #XArmAPI = _import_xarm_api()
            #self.arm = XArmAPI(self.robot_ip)
            #self.arm.motion_enable(True)
            XArmAPI = _import_xarm_api()
            self.arm = XArmAPI(self.robot_ip)

            sim_mode = bool(self.arm.is_simulation_robot)
            self.get_logger().info(f"Controller simulation mode: {sim_mode}")

            if self.require_simulation_robot and not sim_mode:
                self.arm.disconnect()
                raise RuntimeError("Startup blocked: controller is not in Sim mode")

            self.arm.motion_enable(True)
            
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.4)
            self.arm.set_tcp_jerk(3000)
            self.arm.set_tcp_maxacc(1500)
            self.arm.set_joint_jerk(800, is_radian=False)
            self.arm.set_joint_maxacc(350, is_radian=False)
        self._log_robot_config()
        self._initial_joint_target_deg = self._resolve_initial_joint_target()
        
        self._constrained_modes_enabled = False
        self._validate_kinematics()

        if self.dry_run:
            self.input = _FakeInput()
        else:
            self.input = XboxInput(self._build_input_config())
        self.ft_guard = FTGuard(self._build_ft_config())
        self.rate_limiter = RateLimiter(self._build_rate_config())
        self.joint_risk = JointRiskMonitor(
            self._build_joint_risk_config(), kin_model=self.kin,
        )
        self.tip_lock = TipLockController(self._build_tip_lock_config())

        self.sm = StateMachine(
            on_exit=self._on_state_exit,
            on_enter=self._on_state_enter,
            logger=self.get_logger(),
        )

        self.ft_sub = self.create_subscription(
            WrenchStamped, "/xarm/ft_data", self._on_ft_msg, 20,
        )

        self._current_mode: Optional[int] = None
        self._idle_zero_sent = False
        self._align_done_time = 0.0
        self._last_dpad_x = 0
        self._last_warn_times: Dict[str, float] = {}
        self._last_haptic_s = 0.0
        self._last_diag_s = 0.0
        self._last_cmd_sent = [0.0] * 6
        self._risk_scale_smooth = 1.0
        self._abort_motion_requested = False
        self._last_vctime_mono: Optional[float] = None
        self._last_deadman_zero_attempt_s = 0.0

        self._switch_mode(5)

        self.dt = 1.0 / max(self.loop_hz, 1.0)
        if self.enable_control_timer:
            self.timer = self.create_timer(self.dt, self.control_loop)
        else:
            self.timer = None
            self.get_logger().info(
                "Control timer disabled (enable_control_timer:=false)",
            )
        self.get_logger().info(
            "TeleopV4 ready | RB=deadman  A=TipLock  "
            "Y=Align  LB=GoInitialPose  BACK=recovery/quit  Dpad=J7trim"
        )

    # =====================================================================
    # Parameters
    # =====================================================================

    def _declare_params(self):
        P = self.declare_parameter
        P("robot_ip", "192.168.1.243")
        P("kinematics_yaml", "")
        P("dry_run", False)
        P("require_simulation_robot", False)
        P("enable_control_timer", True)
        P("loop_hz", 100.0)
        P("velocity_watchdog_s", 0.20)
        P("deadzone", 0.10)
        P("axis_lx_idx", 0); P("axis_ly_idx", 1)
        P("axis_rx_idx", 3); P("axis_ry_idx", 4)
        P("axis_lt_idx", 2); P("axis_rt_idx", 5)
        P("lx_sign", 1.0); P("ly_sign", 1.0)
        P("rx_sign", 1.0); P("ry_sign", 1.0)
        P("btn_deadman_idx", 5)
        P("btn_tip_lock_idx", 0)
        P("btn_tare_idx", 2)
        P("btn_align_idx", 3)
        P("btn_align_alt_idx", -1)
        P("btn_lb_idx", 4)
        P("btn_back_idx", 6)
        P("btn_r3_idx", 10)
        P("enable_back_quit", True)
        P("input_filter_alpha", 0.50)
        P("input_release_alpha", 0.92)
        P("input_zero_snap", 0.02)
        P("sigmoid_gain", 0.5)
        P("button_debounce_s", 0.35)
        P("max_linear_mm_s", 50.0)
        P("max_z_mm_s", 30.0)
        P("tip_lock_max_angular_deg_s", 10.0)
        P("tip_lock_correction_gain", 3.0)
        P("tip_lock_correction_deadband_mm", 0.3)
        P("insertion_along_neg_tool_z", False)
        P("depth_gain", 1.0)
        P("max_depth_mm_s", 18.0)
        P("ft_fx_limit_n", 25.0); P("ft_fy_limit_n", 25.0)
        P("ft_fz_limit_n", 20.0)
        P("ft_tx_limit_nm", 2.0); P("ft_ty_limit_nm", 2.0)
        P("ft_tz_limit_nm", 1.5)
        P("ft_force_norm_limit_n", 30.0)
        P("ft_torque_norm_limit_nm", 2.5)
        P("ft_hysteresis_ratio", 0.80)
        P("ft_warning_ratio", 0.70)
        P("ft_stale_policy", "allow")
        P("ft_max_age_s", 1.0)
        P("max_linear_acc_mm_s2", 600.0)
        P("max_linear_dec_mm_s2", 1400.0)
        P("max_angular_acc_deg_s2", 600.0)
        P("max_angular_dec_deg_s2", 1600.0)
        P("joint_speed_limit_rad_s", 3.0)
        P("joint_speed_warn_rad_s", 2.0)
        P("joint_margin_deg", 15.0)
        P("singularity_warn_sv", 0.05)
        P("singularity_hard_sv", 0.01)
        P("orthogonal_roll_deg", 180.0)
        P("orthogonal_pitch_deg", 0.0)
        P("orthogonal_compensate_tcp_rp", True)
        P("orthogonal_speed", 12.0)
        P("orthogonal_cooldown_s", 0.8)
        P("nullspace_j7_deg_s", 15.0)
        P("dpad_trim_pulse_s", 0.20)
        P("verbose_input_logging", True)
        P("debug_dynamics_logging", True)
        P("dynamics_log_interval_s", 0.2)
        P("diag_log_interval_s", 0.5)
        # True: stick LR -> base Y (forward/back), stick UD -> base X (left/right)
        # in FREE and the same pairing for Tip/Entry planar inputs.
        P("left_stick_swap_xy", True)
        P("alignment_timeout_s", 120.0)
        P("enable_initial_pose_action", True)
        P("initial_pose_use_startup", True)
        P("initial_pose_joint_deg_csv", "")
        P("initial_pose_speed_deg_s", 20.0)
        P("initial_pose_mvacc_deg_s2", 200.0)
        P("initial_pose_timeout_s", 20.0)
        # Software-only tip refinement (mm, flange-frame); never sent to controller.
        P("virtual_tip_x_mm", 0.0)
        P("virtual_tip_y_mm", 0.0)
        P("virtual_tip_z_mm", 0.0)
        P("validate_with_tcp", True)
        P("tcp_validation_tol_mm", 15.0)
        P("tcp_validation_grace_mm", 5.0)

    def _load_params(self):
        G = lambda name: self.get_parameter(name).value
        env_dry = os.environ.get("NEURO_FINAL_DRY_RUN", "").strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.dry_run = bool(G("dry_run")) or env_dry
        self.require_simulation_robot = bool(G("require_simulation_robot"))
        self.enable_control_timer = bool(G("enable_control_timer"))
        self.robot_ip = str(G("robot_ip"))
        self.kinematics_yaml = str(G("kinematics_yaml")).strip()
        self.loop_hz = float(G("loop_hz"))
        self.velocity_watchdog_s = float(G("velocity_watchdog_s"))
        self.sigmoid_gain = float(G("sigmoid_gain"))
        self.max_linear_mm_s = float(G("max_linear_mm_s"))
        self.max_z_mm_s = float(G("max_z_mm_s"))
        self.insertion_along_neg_tool_z = bool(G("insertion_along_neg_tool_z"))
        self.depth_gain = float(G("depth_gain"))
        self.max_depth_mm_s = float(G("max_depth_mm_s"))
        self.orthogonal_roll_deg = float(G("orthogonal_roll_deg"))
        self.orthogonal_pitch_deg = float(G("orthogonal_pitch_deg"))
        self.orthogonal_compensate_tcp_rp = bool(G("orthogonal_compensate_tcp_rp"))
        self.orthogonal_speed = float(G("orthogonal_speed"))
        self.orthogonal_cooldown_s = float(G("orthogonal_cooldown_s"))
        self.nullspace_j7_deg_s = float(G("nullspace_j7_deg_s"))
        self.dpad_trim_pulse_s = float(G("dpad_trim_pulse_s"))
        self.enable_back_quit = bool(G("enable_back_quit"))
        self.verbose = bool(G("verbose_input_logging"))
        self.debug_dyn = bool(G("debug_dynamics_logging"))
        self.diag_interval = float(G("diag_log_interval_s"))
        self.left_stick_swap_xy = bool(G("left_stick_swap_xy"))
        self.alignment_timeout_s = float(G("alignment_timeout_s"))
        self.enable_initial_pose_action = bool(G("enable_initial_pose_action"))
        self.initial_pose_use_startup = bool(G("initial_pose_use_startup"))
        self.initial_pose_joint_deg_csv = str(G("initial_pose_joint_deg_csv")).strip()
        self.initial_pose_speed_deg_s = float(G("initial_pose_speed_deg_s"))
        self.initial_pose_mvacc_deg_s2 = float(G("initial_pose_mvacc_deg_s2"))
        self.initial_pose_timeout_s = float(G("initial_pose_timeout_s"))
        self.virtual_tip_x_mm = float(G("virtual_tip_x_mm"))
        self.virtual_tip_y_mm = float(G("virtual_tip_y_mm"))
        self.virtual_tip_z_mm = float(G("virtual_tip_z_mm"))
        self.validate_with_tcp = bool(G("validate_with_tcp"))
        self.tcp_validation_tol_mm = float(G("tcp_validation_tol_mm"))
        self.tcp_validation_grace_mm = float(G("tcp_validation_grace_mm"))

    def _build_input_config(self) -> InputConfig:
        G = lambda n: self.get_parameter(n).value
        back_idx = int(G("btn_back_idx"))
        tare_idx = int(G("btn_tare_idx"))
        align_alt_idx = int(G("btn_align_alt_idx"))
        if align_alt_idx == tare_idx and align_alt_idx >= 0:
            align_alt_idx = -1
            self._warn_throttle(
                "align_tare_idx_conflict",
                "btn_align_alt_idx equals btn_tare_idx; disabling alternate align index to avoid overlap.",
                2.0,
            )
        # If BACK quit is disabled, stop polling BACK to avoid accidental
        # collisions on controller layouts where a trigger shares this index.
        if not self.enable_back_quit:
            back_idx = -1
        return InputConfig(
            axis_lx_idx=int(G("axis_lx_idx")), axis_ly_idx=int(G("axis_ly_idx")),
            axis_rx_idx=int(G("axis_rx_idx")), axis_ry_idx=int(G("axis_ry_idx")),
            axis_lt_idx=int(G("axis_lt_idx")), axis_rt_idx=int(G("axis_rt_idx")),
            lx_sign=float(G("lx_sign")), ly_sign=float(G("ly_sign")),
            rx_sign=float(G("rx_sign")), ry_sign=float(G("ry_sign")),
            btn_deadman_idx=int(G("btn_deadman_idx")),
            btn_tip_lock_idx=int(G("btn_tip_lock_idx")),
            btn_tare_idx=tare_idx,
            btn_align_idx=int(G("btn_align_idx")),
            btn_align_alt_idx=align_alt_idx,
            btn_lb_idx=int(G("btn_lb_idx")),
            btn_back_idx=back_idx,
            btn_r3_idx=int(G("btn_r3_idx")),
            deadzone=float(G("deadzone")),
            filter_alpha=float(G("input_filter_alpha")),
            release_alpha=float(G("input_release_alpha")),
            zero_snap=float(G("input_zero_snap")),
            debounce_s=float(G("button_debounce_s")),
        )

    def _build_ft_config(self) -> FTGuardConfig:
        G = lambda n: self.get_parameter(n).value
        policy_str = str(G("ft_stale_policy")).strip().lower()
        policy = StalePolicy.BLOCK_INWARD if policy_str == "block_inward" else StalePolicy.ALLOW
        return FTGuardConfig(
            fx_limit_n=float(G("ft_fx_limit_n")), fy_limit_n=float(G("ft_fy_limit_n")),
            fz_limit_n=float(G("ft_fz_limit_n")),
            tx_limit_nm=float(G("ft_tx_limit_nm")), ty_limit_nm=float(G("ft_ty_limit_nm")),
            tz_limit_nm=float(G("ft_tz_limit_nm")),
            force_norm_limit_n=float(G("ft_force_norm_limit_n")),
            torque_norm_limit_nm=float(G("ft_torque_norm_limit_nm")),
            hysteresis_ratio=float(G("ft_hysteresis_ratio")),
            warning_ratio=float(G("ft_warning_ratio")),
            stale_policy=policy, max_age_s=float(G("ft_max_age_s")),
        )

    def _build_rate_config(self) -> RateLimiterConfig:
        G = lambda n: self.get_parameter(n).value
        return RateLimiterConfig(
            max_linear_acc_mm_s2=float(G("max_linear_acc_mm_s2")),
            max_linear_dec_mm_s2=float(G("max_linear_dec_mm_s2")),
            max_angular_acc_deg_s2=float(G("max_angular_acc_deg_s2")),
            max_angular_dec_deg_s2=float(G("max_angular_dec_deg_s2")),
        )

    def _build_joint_risk_config(self) -> JointRiskConfig:
        G = lambda n: self.get_parameter(n).value
        return JointRiskConfig(
            joint_margin_deg=float(G("joint_margin_deg")),
            singularity_warn_sv=float(G("singularity_warn_sv")),
            singularity_hard_sv=float(G("singularity_hard_sv")),
            joint_speed_limit_rad_s=float(G("joint_speed_limit_rad_s")),
            joint_speed_warn_rad_s=float(G("joint_speed_warn_rad_s")),
        )

    def _build_tip_lock_config(self) -> TipLockConfig:
        G = lambda n: self.get_parameter(n).value
        return TipLockConfig(
            max_angular_deg_s=float(G("tip_lock_max_angular_deg_s")),
            correction_gain=float(G("tip_lock_correction_gain")),
            correction_deadband_mm=float(G("tip_lock_correction_deadband_mm")),
        )

    # =====================================================================
    # State-machine callbacks  (zero-on-transition)
    # =====================================================================

    def _on_state_exit(self, old: TeleopState, new: TeleopState, reason: str):
        self._send_zero(f"exit_{old.value}")

    def _on_state_enter(self, old: TeleopState, new: TeleopState, reason: str):
        self.input.reset_filters()
        self.rate_limiter.reset()
        self._idle_zero_sent = False
        self._send_zero(f"enter_{new.value}")
        if new == TeleopState.FREE_TELEOP:
            self.tip_lock.clear()

    # =====================================================================
    # Control loop
    # =====================================================================

    def control_loop(self):
        try:
            self._control_loop_inner()
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            self._warn_throttle("loop_exc", f"Control loop exception: {exc}", 0.3)
            self._send_zero(f"exception:{exc}")

    def _control_loop_inner(self):
        now = time.monotonic()

        # 1. Read joystick
        inp = self.input.read()
        if not inp.connected:
            self.sm.force_fault("joystick_disconnected")
            self._send_zero("joystick_disconnected")
            return

        # 2. BACK button: quit or recovery
        if inp.back_edge:
            if self.sm.state == TeleopState.FAULT_LATCHED:
                self._attempt_recovery()
                return
            if self.enable_back_quit:
                self.get_logger().info("[ACTION] BACK pressed: shutting down")
                raise KeyboardInterrupt
            # Quietly ignore BACK when quit is disabled.

        # 3. Robot health
        if self._check_robot_health():
            return

        # 4. Deadman (RB)
        if not inp.btn_rb:
            if self.sm.state not in (TeleopState.IDLE, TeleopState.FAULT_LATCHED):
                self.sm.transition_to(TeleopState.IDLE, "deadman_released")
            if self.velocity_watchdog_s <= 0.0:
                # Only send an explicit stop after a nonzero VC was accepted.
                # At startup the arm can still be settling into mode 5, and
                # hammering zero VC there only produces SDK code=9 noise.
                if any(abs(c) >= 0.001 for c in self._last_cmd_sent):
                    self._send_zero("deadman_hold_zero")
            return

        if self.sm.state == TeleopState.IDLE:
            self.sm.transition_to(TeleopState.FREE_TELEOP, "deadman_pressed")

        # 5. Fault -- only BACK can recover
        if self.sm.state == TeleopState.FAULT_LATCHED:
            return

        # 6. ALIGN_BUSY -- wait for cooldown
        if self.sm.state == TeleopState.ALIGN_BUSY:
            if now >= self._align_done_time:
                self.sm.transition_to(TeleopState.FREE_TELEOP, "alignment_complete")
            return

        # 7. Button actions
        self._handle_button_actions(inp)

        # 8. Transient capture states
        if self.sm.state == TeleopState.TIP_LOCK_CAPTURE:
            self._do_tip_lock_capture()
            return
        if self.sm.state == TeleopState.RCM_CAPTURE:
            self._do_rcm_capture()
            return

        # 9. D-pad J7 trim
        dpad_x, _ = inp.dpad
        if dpad_x != 0 and self.sm.is_motion_allowed():
            if abs(inp.lx) < 0.15 and abs(inp.ly) < 0.15:
                if dpad_x != self._last_dpad_x:
                    self._last_dpad_x = dpad_x
                    self._j7_trim(dpad_x)
                    return
        else:
            self._last_dpad_x = 0

        # 10. Ensure requested velocity-control mode
        desired_mode = self._desired_control_mode()
        if self._current_mode != desired_mode:
            self._switch_mode(desired_mode)

        # 11. Read joints
        code_j, joints_rad = self.arm.get_servo_angle(is_radian=True)
        if code_j != 0 or not joints_rad or len(joints_rad) < 7:
            self._warn_throttle("joint_read", "Joint read failed", 0.5)
            self._send_zero("joint_read_fail")
            return

        # 12. Joint speeds for governor
        js_deg = None
        try:
            js_raw = getattr(self.arm, "realtime_joint_speeds", None)
            if js_raw and len(js_raw) >= 7:
                js_deg = [float(v) for v in js_raw[:7]]
        except Exception:
            pass

        # 13. Risk assessment
        risk = self.joint_risk.compute_risk(joints_rad, js_deg)
        alpha_r = 0.15 if risk.scale < self._risk_scale_smooth else 0.05
        self._risk_scale_smooth += alpha_r * (risk.scale - self._risk_scale_smooth)
        combined_scale = clamp(self._risk_scale_smooth, 0.1, 1.0)

        if combined_scale < 0.55 and risk.reason:
            self._warn_throttle("risk", f"Risk: {risk.reason} s={combined_scale:.2f}", 0.4)
            self._pulse_haptic(0.55, 60, 0.3)

        # 14. Measured dt for rate limiter (teleop.py-style clamp)
        t_cmd = time.monotonic()
        if self._last_vctime_mono is None:
            dt_dyn = self.dt
        else:
            dt_dyn = clamp(t_cmd - self._last_vctime_mono, 0.001, 0.05)
        self._last_vctime_mono = t_cmd

        # 15. Compute velocity
        cmd = self._compute_velocity(inp, combined_scale, joints_rad)

        # 16. Rate limit
        cmd = self.rate_limiter.limit(cmd, dt_dyn, self._rate_limit_is_constrained())

        # 17. Send with watchdog
        self._send_velocity(cmd)

        # 18. Diagnostics
        self._log_diagnostics(risk, now)

    # =====================================================================
    # Button actions
    # =====================================================================

    def _handle_button_actions(self, inp: InputSnapshot):
        st = self.sm.state
        if st == TeleopState.FREE_TELEOP:
            if inp.x_edge:
                self._tare_sensor_action()
            elif inp.lb_edge:
                if not self.enable_initial_pose_action:
                    self._warn_throttle(
                        "init_pose_disabled",
                        "Initial-pose action disabled (enable_initial_pose_action:=false)",
                        1.0,
                    )
                    return
                self._start_initial_pose()
            elif inp.a_edge:
                if not self._constrained_modes_enabled:
                    self._warn_throttle("kin_nv", "Tip-Lock blocked: kinematic model not validated", 1.0)
                    return
                self.sm.transition_to(TeleopState.TIP_LOCK_CAPTURE, "btn_a_tip_lock")
            elif inp.y_edge:
                self._start_alignment()
        elif st == TeleopState.TIP_LOCK_ACTIVE:
            if inp.a_edge:
                self.sm.transition_to(TeleopState.FREE_TELEOP, "tip_lock_unlock")

    def _rate_limit_is_constrained(self) -> bool:
        return self.sm.is_constrained()

    def _desired_control_mode(self) -> int:
        return 5

    def _tare_sensor_action(self):
        if self.dry_run:
            self.get_logger().info("[ACTION] FT tare requested (dry-run)")
            self._pulse_haptic(0.7, 100, 0.0)
            return
        try:
            ret = self.arm.ft_sensor_set_zero()
            if isinstance(ret, int) and ret != 0:
                self._warn_throttle("ft_tare", f"FT tare failed code={ret}", 0.8)
                return
            self.get_logger().info("[ACTION] FT sensor tare applied")
            self._pulse_haptic(0.85, 120, 0.0)
        except Exception as exc:
            self._warn_throttle("ft_tare_exc", f"FT tare exception: {exc}", 0.8)

    # =====================================================================
    # Capture handlers  (transient single-frame states)
    # =====================================================================

    def _do_tip_lock_capture(self):
        code_j, joints = self.arm.get_servo_angle(is_radian=True)
        if code_j != 0 or not joints:
            self._warn_throttle("tip_cap", "Tip-Lock capture: joints unavailable", 0.5)
            self.sm.transition_to(TeleopState.FREE_TELEOP, "capture_pose_fail")
            return
        pose = self._effective_tip_pose_mm_deg(joints)
        if not pose:
            self._warn_throttle("tip_cap", "Tip-Lock capture: pose unavailable", 0.5)
            self.sm.transition_to(TeleopState.FREE_TELEOP, "capture_pose_fail")
            return
        self.tip_lock.capture(pose)
        eff = self._effective_tcp_translation_mm()
        self.get_logger().info(
            f"TIP-LOCK captured (effective tip, base mm) at ({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}) | "
            f"eff_tcp_trans_mm=({eff[0]:.2f},{eff[1]:.2f},{eff[2]:.2f})"
        )
        self._pulse_haptic(1.0, 180, 0.0)
        self.sm.transition_to(TeleopState.TIP_LOCK_ACTIVE, "tip_captured")

    # =====================================================================
    # Velocity computation per state
    # =====================================================================

    def _planar_sticks(self, inp: InputSnapshot) -> Tuple[float, float]:
        """Stick pair (sx, sy) for XY / planar-angular mapping.

        When ``left_stick_swap_xy`` is True: physical LR (lx) maps like old ``ly``
        to vy, physical UD (ly) maps like old ``lx`` to vx (see ``_vel_free_teleop``).
        """
        if self.left_stick_swap_xy:
            return float(inp.ly), float(inp.lx)
        return float(inp.lx), float(inp.ly)

    def _robot_tcp_offset_list(self) -> List[float]:
        try:
            return list(self.arm.tcp_offset) if self.arm.tcp_offset else [0.0] * 6
        except Exception:
            return [0.0] * 6

    def _virtual_tip_xyz_mm(self) -> List[float]:
        """Software tip translation in the flange frame.

        A future drill-axis extension should instead be expressed in the
        rotated tool frame. Current behavior is retained for compatibility.
        """
        return [self.virtual_tip_x_mm, self.virtual_tip_y_mm, self.virtual_tip_z_mm]

    def _robot_tcp_rotation(self) -> np.ndarray:
        """Return controller flange-to-TCP rotation from degree RPY values."""
        tcp_offset = self._robot_tcp_offset_list()
        if len(tcp_offset) < 6:
            return np.eye(3)
        try:
            return rpy_deg_to_rotmat(
                *[float(value) for value in tcp_offset[3:6]]
            )
        except (TypeError, ValueError):
            return np.eye(3)

    def _effective_tcp_translation_mm(self) -> List[float]:
        return self.kin.effective_tcp_translation_mm(
            self._robot_tcp_offset_list(), self._virtual_tip_xyz_mm(),
        )

    def _effective_tip_pose_mm_deg(self, joints_rad: List[float]) -> Optional[List[float]]:
        """Base-frame pose (mm, deg) of the **effective surgical tip**.

        Position: SDK ``get_position`` when effective TCP translation is ~0;
        else ``fk_tool`` with effective translation (mm→m) and SDK roll/pitch/yaw
        (same convention as ``tool_z_axis_from_rpy_deg``).
        """
        code_p, pose_sdk = self.arm.get_position(is_radian=False)
        if code_p != 0 or not pose_sdk:
            return None
        eff = self._effective_tcp_translation_mm()
        if abs(eff[0]) + abs(eff[1]) + abs(eff[2]) < 1e-3:
            return [float(pose_sdk[i]) for i in range(6)]
        tcp_m = [eff[i] / 1000.0 for i in range(3)]
        pos_m, _ = self.kin.fk_tool(list(joints_rad)[:7], tcp_m)
        return [
            pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0,
            float(pose_sdk[3]), float(pose_sdk[4]), float(pose_sdk[5]),
        ]

    def _compute_velocity(self, inp: InputSnapshot, scale: float, joints_rad: List[float]) -> List[float]:
        st = self.sm.state
        if st == TeleopState.FREE_TELEOP:
            return self._vel_free_teleop(inp, scale)
        elif st == TeleopState.TIP_LOCK_ACTIVE:
            return self._vel_tip_lock(inp, scale, joints_rad)
        return [0.0] * 6

    def _vel_free_teleop(self, inp: InputSnapshot, scale: float) -> List[float]:
        sg = self.sigmoid_gain
        sx, sy = self._planar_sticks(inp)
        vx = sigmoid_shape(-sx, sg) * self.max_linear_mm_s * scale
        vy = sigmoid_shape(-sy, sg) * self.max_linear_mm_s * scale
        vz = sigmoid_shape(-inp.ry, sg) * self.max_z_mm_s * scale
        return [vx, vy, vz, 0.0, 0.0, 0.0]

    def _vel_tip_lock(self, inp: InputSnapshot, scale: float, joints_rad: List[float]) -> List[float]:
        pose = self._effective_tip_pose_mm_deg(joints_rad)
        if not pose:
            return [0.0] * 6
        sx, sy = self._planar_sticks(inp)
        lin_v, ang_v = self.tip_lock.compute_velocity(pose, sx, sy)
        return [
            lin_v[0] * scale, lin_v[1] * scale, lin_v[2] * scale,
            ang_v[0] * scale, ang_v[1] * scale, ang_v[2] * scale,
        ]

    def _parse_joint_deg_csv(self, text: str) -> Optional[List[float]]:
        txt = (text or "").strip()
        if not txt:
            return None
        for sep in (";", "|", "\t"):
            txt = txt.replace(sep, ",")
        parts = [p.strip() for p in txt.split(",") if p.strip()]
        if len(parts) != 7:
            self.get_logger().warn(
                f"initial_pose_joint_deg_csv ignored: expected 7 values, got {len(parts)}"
            )
            return None
        try:
            return [float(v) for v in parts]
        except ValueError:
            self.get_logger().warn("initial_pose_joint_deg_csv ignored: values must be numeric")
            return None

    def _resolve_initial_joint_target(self) -> Optional[List[float]]:
        fallback = self._parse_joint_deg_csv(self.initial_pose_joint_deg_csv)
        if self.initial_pose_use_startup:
            try:
                code, joints_deg = self.arm.get_servo_angle(is_radian=False)
                if code == 0 and joints_deg and len(joints_deg) >= 7:
                    target = [float(joints_deg[i]) for i in range(7)]
                    self.get_logger().info(
                        "[INIT] Startup joint target captured (deg): "
                        f"[{', '.join(f'{v:.2f}' for v in target)}]"
                    )
                    return target
                self.get_logger().warn(
                    f"[INIT] Failed to capture startup joints (code={code}); "
                    "falling back to initial_pose_joint_deg_csv if provided."
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"[INIT] Startup joint capture exception: {exc}; "
                    "falling back to initial_pose_joint_deg_csv if provided."
                )
        if fallback is not None:
            self.get_logger().info(
                "[INIT] Using configured initial_pose_joint_deg_csv target (deg): "
                f"[{', '.join(f'{v:.2f}' for v in fallback)}]"
            )
            return fallback
        self.get_logger().warn(
            "[INIT] No initial-pose target available. "
            "Set initial_pose_joint_deg_csv or keep initial_pose_use_startup:=true."
        )
        return None

    def _start_initial_pose(self):
        target = self._initial_joint_target_deg
        if not target or len(target) < 7:
            self._warn_throttle(
                "init_pose_missing",
                "Initial-pose action skipped: no target captured/configured.",
                1.0,
            )
            return
        ok = self.sm.transition_to(TeleopState.ALIGN_BUSY, "go_initial_pose")
        if not ok:
            return
        move_issued = False
        move_success = False
        try:
            self._switch_mode(0)
            ret = self.arm.set_servo_angle(
                angle=[float(v) for v in target[:7]],
                is_radian=False,
                speed=float(self.initial_pose_speed_deg_s),
                mvacc=float(self.initial_pose_mvacc_deg_s2),
                wait=False,
            )
            if isinstance(ret, int) and ret != 0:
                self._warn_throttle(
                    "init_pose_cmd",
                    f"Initial-pose command rejected: code={ret}",
                    0.6,
                )
                return
            move_issued = True
            t0 = time.monotonic()
            deadline = t0 + max(self.initial_pose_timeout_s, 1.0)
            min_done_s = 0.15
            while time.monotonic() < deadline:
                if self._abort_motion_requested:
                    break
                try:
                    moving = self.arm.get_is_moving()
                except Exception:
                    moving = False
                if not moving and (time.monotonic() - t0) >= min_done_s:
                    move_success = True
                    break
                time.sleep(0.02)
            else:
                if move_issued and not self._abort_motion_requested:
                    self._warn_throttle(
                        "init_pose_tout",
                        "Initial-pose move timed out waiting for motion end",
                        2.0,
                    )
        except KeyboardInterrupt:
            self._abort_motion_requested = True
            raise
        except Exception as exc:
            self.get_logger().error(f"Initial-pose move failed: {exc}")
        finally:
            if move_issued and not move_success and not self.dry_run:
                try:
                    self.arm.emergency_stop()
                except Exception:
                    try:
                        self.arm.set_state(4)
                    except Exception:
                        pass
            try:
                self._switch_mode(5)
                self._send_zero("initial_pose_finally")
            except Exception:
                pass
            if move_success and not self._abort_motion_requested:
                self._align_done_time = time.monotonic() + self.orthogonal_cooldown_s
                self.get_logger().info(
                    "[ACTION] Initial pose reached "
                    f"(deg): [{', '.join(f'{v:.2f}' for v in target[:7])}]"
                )
                self._pulse_haptic(1.0, 180, 0.0)
            elif self.sm.state == TeleopState.ALIGN_BUSY:
                reason = (
                    "init_pose_aborted" if self._abort_motion_requested else "init_pose_stopped"
                )
                self._align_done_time = time.monotonic()
                self.sm.transition_to(TeleopState.FREE_TELEOP, reason)

    # =====================================================================
    # Alignment  (mode 0 blocking move)
    # =====================================================================

    def _start_alignment(self):
        ok = self.sm.transition_to(TeleopState.ALIGN_BUSY, "orthogonal_align")
        if not ok:
            return
        pose_fail = False
        move_issued = False
        align_success = False
        try:
            self._switch_mode(0)
            code, pose = self.arm.get_position(is_radian=False)
            if code != 0 or not pose:
                self._warn_throttle("align_pose", "Alignment: pose unavailable", 0.5)
                pose_fail = True
                return
            tcp_off = list(self.arm.tcp_offset) if self.arm.tcp_offset else [0.0] * 6
            if self.orthogonal_compensate_tcp_rp:
                roll_t = self.orthogonal_roll_deg - float(tcp_off[3])
                pitch_t = self.orthogonal_pitch_deg - float(tcp_off[4])
            else:
                roll_t = self.orthogonal_roll_deg
                pitch_t = self.orthogonal_pitch_deg
            self.arm.set_position(
                x=float(pose[0]), y=float(pose[1]), z=float(pose[2]),
                roll=roll_t, pitch=pitch_t, yaw=float(pose[5]),
                speed=self.orthogonal_speed, mvacc=120.0, wait=False,
            )
            move_issued = True
            t0 = time.monotonic()
            deadline = t0 + max(self.alignment_timeout_s, 1.0)
            min_done_s = 0.15
            while time.monotonic() < deadline:
                if self._abort_motion_requested:
                    break
                try:
                    moving = self.arm.get_is_moving()
                except Exception:
                    moving = False
                if not moving and (time.monotonic() - t0) >= min_done_s:
                    align_success = True
                    break
                time.sleep(0.02)
            else:
                if move_issued and not self._abort_motion_requested:
                    self._warn_throttle(
                        "align_tout", "Alignment: timed out waiting for motion end", 2.0,
                    )
        except KeyboardInterrupt:
            self._abort_motion_requested = True
            raise
        except Exception as exc:
            self.get_logger().error(f"Alignment failed: {exc}")
        finally:
            if move_issued and not align_success and not self.dry_run:
                try:
                    self.arm.emergency_stop()
                except Exception:
                    try:
                        self.arm.set_state(4)
                    except Exception:
                        pass
            try:
                self._switch_mode(5)
                self._send_zero("align_finally")
            except Exception:
                pass
            if pose_fail:
                self.sm.transition_to(TeleopState.FREE_TELEOP, "align_pose_fail")
            elif align_success and not self._abort_motion_requested:
                self._align_done_time = time.monotonic() + self.orthogonal_cooldown_s
                self.get_logger().info("[ACTION] Orthogonal alignment done")
                self._pulse_haptic(0.9, 140, 0.0)
            elif self.sm.state == TeleopState.ALIGN_BUSY:
                reason = (
                    "align_aborted" if self._abort_motion_requested else "align_stopped"
                )
                self._align_done_time = time.monotonic()
                self.sm.transition_to(TeleopState.FREE_TELEOP, reason)

    # =====================================================================
    # J7 null-space trim  (mode 4 pulse)
    # =====================================================================

    def _j7_trim(self, dpad_x: int):
        try:
            self._switch_mode(4)
            speeds = [0.0] * 7
            speeds[6] = float(dpad_x) * self.nullspace_j7_deg_s
            self.arm.vc_set_joint_velocity(
                speeds, is_radian=False, is_sync=True,
                duration=self.dpad_trim_pulse_s + 0.1,
            )
            time.sleep(self.dpad_trim_pulse_s)
            self.arm.vc_set_joint_velocity(
                [0.0] * 7, is_radian=False, is_sync=True, duration=0.1,
            )
        except Exception as exc:
            self._warn_throttle("j7_trim", f"J7 trim failed: {exc}", 0.8)
        finally:
            self._switch_mode(5)
            self._send_zero("j7_trim_done")

    # =====================================================================
    # Robot utilities
    # =====================================================================

    def _switch_mode(self, target: int):
        reported = getattr(self.arm, "mode", None)
        if self._current_mode == target and reported == target:
            return
        self.arm.set_mode(target)
        self.arm.set_state(0)
        time.sleep(0.04)
        self._current_mode = target

    def _ensure_mode5(self) -> bool:
        mode = getattr(self.arm, "mode", None)
        state = getattr(self.arm, "state", None)
        err = int(getattr(self.arm, "error_code", 0) or 0)
        warn = int(getattr(self.arm, "warn_code", 0) or 0)
        if err != 0 or warn != 0:
            # Avoid retry storms while controller is faulted.
            return False
        # During mode-5 VC, state 2 (moving) is normal — do not force set_state each tick.
        if mode == 5 and state in (0, 1, 2):
            self._current_mode = 5
            return True
        try:
            self.arm.motion_enable(True)
            if mode != 5:
                ret_m = self.arm.set_mode(5)
                if isinstance(ret_m, int) and ret_m != 0:
                    return False
            ret_s = self.arm.set_state(0)
            if isinstance(ret_s, int) and ret_s != 0:
                return False
            time.sleep(0.02)
        except Exception as exc:
            self._warn_throttle("mode5_rec", f"Mode5 recover: {exc}", 0.5)
        mode = getattr(self.arm, "mode", None)
        state = getattr(self.arm, "state", None)
        if mode == 5 and state in (0, 1, 2):
            self._current_mode = 5
            return True
        return False

    def _send_velocity(self, cmd: List[float]):
        is_zero = all(abs(c) < 0.001 for c in cmd)
        # With watchdog==0.0, keep streaming zero commands continuously.
        # Deduplicating zeros in this mode can leave stale nonzero VC latched.
        if is_zero and self._idle_zero_sent and self.velocity_watchdog_s > 0.0:
            return
        if not self._ensure_mode5():
            self._warn_throttle("mode_nr", f"vc skip: mode={self.arm.mode} state={self.arm.state}", 0.4)
            return
        self._last_cmd_sent = list(cmd)
        ret = self.arm.vc_set_cartesian_velocity(
            cmd, is_radian=False, is_tool_coord=False,
            duration=self.velocity_watchdog_s,
        )
        if isinstance(ret, int) and ret != 0:
            self._warn_throttle("vc_fail", f"vc_set_cartesian_velocity failed code={ret}", 0.25)
            if self._should_fault_on_vc_failure(ret, cmd):
                self.sm.force_fault(f"vc_command_fail(code={ret})")
            self._idle_zero_sent = False
            return
        self._idle_zero_sent = is_zero

    def _should_fault_on_vc_failure(self, ret: int, cmd: List[float]) -> bool:
        return True

    def _send_zero(self, reason: str = ""):
        try:
            if reason == "deadman_hold_zero":
                now = time.monotonic()
                if now - self._last_deadman_zero_attempt_s < 0.5:
                    return
                self._last_deadman_zero_attempt_s = now
            if not self._ensure_mode5():
                return
            zero = [0.0] * 6
            ret = self.arm.vc_set_cartesian_velocity(
                zero, is_radian=False, is_tool_coord=False,
                duration=self.velocity_watchdog_s,
            )
            if isinstance(ret, int) and ret != 0:
                self._warn_throttle("vc_zero_fail", f"zero vc failed code={ret} ({reason})", 0.5)
                return
            self._last_cmd_sent = zero
            self._idle_zero_sent = True
            self.rate_limiter.reset()
        except Exception:
            pass

    def _check_robot_health(self) -> bool:
        if not self.arm.has_err_warn:
            return False
        if self.arm.error_code != 0 or self.arm.warn_code != 0:
            self._warn_throttle("arm_err", f"Robot err={self.arm.error_code} warn={self.arm.warn_code}", 0.8)
            self.sm.force_fault(f"robot_error(e={self.arm.error_code},w={self.arm.warn_code})")
            return True
        return False

    def _attempt_recovery(self):
        self.get_logger().info("[RECOVERY] Attempting fault recovery...")
        try:
            self._send_zero("recovery")
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.1)
            self._switch_mode(5)
            self.sm.transition_to(TeleopState.IDLE, "recovery_complete")
            self.get_logger().info("[RECOVERY] Success -> IDLE")
        except Exception as exc:
            self.get_logger().error(f"[RECOVERY] Failed: {exc}")

    def _log_robot_config(self):
        try:
            tcp_off = list(self.arm.tcp_offset) if self.arm.tcp_offset else [0.0] * 6
        except Exception:
            tcp_off = [0.0] * 6
        try:
            tcp_ld = list(self.arm.tcp_load) if self.arm.tcp_load else [0.0] * 4
        except Exception:
            tcp_ld = [0.0] * 4
        self.get_logger().info(f"Robot config | tcp_offset={tcp_off} | tcp_load={tcp_ld}")
        tcp_rotation = self._robot_tcp_rotation()
        tcp_rpy = (tcp_off + [0.0] * 6)[3:6]
        tool_z = tcp_rotation[:, 2]
        self.get_logger().info(
            "Robot TCP rotation | "
            f"rpy_deg=({tcp_rpy[0]:.2f},{tcp_rpy[1]:.2f},{tcp_rpy[2]:.2f}) "
            f"| tool_z_flange=({tool_z[0]:.5f},{tool_z[1]:.5f},{tool_z[2]:.5f})"
        )
        self.get_logger().info(
            f"Tip model | virtual_tip_mm=({self.virtual_tip_x_mm:.2f},{self.virtual_tip_y_mm:.2f},"
            f"{self.virtual_tip_z_mm:.2f}) | validate_with_tcp={self.validate_with_tcp} "
            f"| tcp_validation_tol_mm={self.tcp_validation_tol_mm:.2f}"
        )

    # =====================================================================
    # Kinematic validation
    # =====================================================================

    def _validate_kinematics(self):
        """FK vs SDK ``get_position`` (flange if tcp trans ~0, else TCP)."""
        try:
            code_j, joints = self.arm.get_servo_angle(is_radian=True)
            code_p, pose = self.arm.get_position(is_radian=False)
            if code_j != 0 or code_p != 0 or not joints or not pose:
                self.get_logger().warn("[KIN] Validation skipped: cannot read robot state")
                return
            try:
                tcp_off = list(self.arm.tcp_offset) if self.arm.tcp_offset else [0.0] * 6
            except Exception:
                tcp_off = [0.0] * 6
            virt = self._virtual_tip_xyz_mm()
            result = self.kin.validate_against_sdk(
                pose,
                joints,
                tolerance_mm=self.tcp_validation_tol_mm,
                tcp_offset_mm_deg=tcp_off,
                virtual_tip_offset_mm=virt,
                validate_with_tcp=self.validate_with_tcp,
                tcp_rotation=self._robot_tcp_rotation(),
            )
            self.get_logger().info(f"[KIN] Kinematic validation: {result.detail}")
            if result.orientation_error_deg > 1.0:
                self.get_logger().warn(
                    "[KIN] TCP orientation mismatch: "
                    f"{result.orientation_error_deg:.3f}deg > 1.000deg "
                    "(warning only)"
                )
            eff = self.kin.effective_tcp_translation_mm(tcp_off, virt)
            tcp_mode = self.validate_with_tcp and (abs(eff[0]) + abs(eff[1]) + abs(eff[2]) > 1.0)
            if result.valid:
                self._constrained_modes_enabled = True
                self.get_logger().info(
                    "[KIN] Model VALIDATED -- Tip-Lock enabled"
                )
            elif tcp_mode and result.position_error_mm <= (self.tcp_validation_tol_mm + self.tcp_validation_grace_mm):
                self._constrained_modes_enabled = True
                self.get_logger().warn(
                    "[KIN] Validation in grace band (TCP approximation). "
                    f"err={result.position_error_mm:.2f}mm <= tol+grace "
                    f"{self.tcp_validation_tol_mm + self.tcp_validation_grace_mm:.2f}mm -- "
                    "constrained modes enabled. Verify tip behavior carefully."
                )
            else:
                self._constrained_modes_enabled = False
                self.get_logger().warn(
                    "[KIN] Model NOT validated -- constrained modes DISABLED. "
                    "Check URDF vs arm, joint units, or tcp_offset vs Studio."
                )
        except Exception as exc:
            self.get_logger().warn(f"[KIN] Validation exception: {exc}")
            self._constrained_modes_enabled = False

    # =====================================================================
    # ROS subscriber callback
    # =====================================================================

    def _on_ft_msg(self, msg: WrenchStamped):
        now_s = self.get_clock().now().nanoseconds / 1e9
        self.ft_guard.update(
            fx=float(msg.wrench.force.x), fy=float(msg.wrench.force.y),
            fz=float(msg.wrench.force.z), tx=float(msg.wrench.torque.x),
            ty=float(msg.wrench.torque.y), tz=float(msg.wrench.torque.z),
            now_s=now_s,
        )

    # =====================================================================
    # Utilities
    # =====================================================================

    def _warn_throttle(self, key: str, text: str, interval: float = 0.5):
        now = time.monotonic()
        if now - self._last_warn_times.get(key, 0.0) > interval:
            self._last_warn_times[key] = now
            self.get_logger().warn(text)

    def _pulse_haptic(self, strength: float = 0.7, duration_ms: int = 100, cooldown_s: float = 0.4):
        now = time.monotonic()
        if now - self._last_haptic_s >= cooldown_s:
            self._last_haptic_s = now
            self.input.rumble(strength=strength, duration_ms=duration_ms)

    def _log_diagnostics(self, risk, now: float):
        if not self.debug_dyn:
            return
        if now - self._last_diag_s < self.diag_interval:
            return
        self._last_diag_s = now
        st = self.sm.state.value
        sent = self._last_cmd_sent
        extra = ""
        if self.sm.state == TeleopState.TIP_LOCK_ACTIVE:
            extra = f" tip_err={self.tip_lock.last_pos_err_mm:.2f}mm"
        self.get_logger().info(
            f"[DIAG] {st} risk={risk.scale:.2f} sv={risk.min_singular_value:.4f} "
            f"jm={risk.worst_joint_margin_deg:.1f}deg "
            f"cmd=({sent[0]:+.1f},{sent[1]:+.1f},{sent[2]:+.1f}|"
            f"{sent[3]:+.1f},{sent[4]:+.1f},{sent[5]:+.1f}){extra} "
            f"mode={self.arm.mode} state={self.arm.state}"
        )

    # =====================================================================
    # Shutdown
    # =====================================================================

    def _request_motion_abort(self) -> None:
        """Stop arm motion (alignment / VC). Safe to call from signal or rclpy shutdown."""
        self._abort_motion_requested = True
        if self.dry_run:
            return
        try:
            self.arm.emergency_stop()
        except Exception:
            try:
                self.arm.set_state(4)
            except Exception:
                pass

    def shutdown_hook(self):
        self._abort_motion_requested = True
        self.get_logger().info("base teleop shutting down safely")
        if not self.dry_run:
            try:
                self.arm.emergency_stop()
            except Exception:
                try:
                    self.arm.set_state(4)
                except Exception:
                    pass
            try:
                self._send_zero("shutdown")
            except Exception:
                pass
            try:
                self.arm.disconnect()
            except Exception:
                pass


# =====================================================================
# Offline stubs for dry-run mode
# =====================================================================

class _FakeArm:
    """Stub that mimics the XArmAPI surface used by TeleopV4Node.

    Returns safe, plausible values so the control loop can run
    end-to-end without hardware.
    """

    def __init__(self):
        self.mode = 0
        self.state = 0
        self.error_code = 0
        self.warn_code = 0
        self.has_err_warn = False
        self.tcp_offset = [0.0] * 6
        self.tcp_load = [0.0] * 4
        self.realtime_joint_speeds = [0.0] * 7
        self._joints_rad = [0.5, -0.3, 0.2, 1.0, -0.5, 0.8, 0.1]
        self._pose = [469.0, 281.8, 323.0, 148.6, -24.9, 56.9]
        self._moving = False

    def motion_enable(self, *a, **kw): pass
    def clean_error(self): pass
    def clean_warn(self): pass
    def set_mode(self, m): self.mode = m
    def set_state(self, s): self.state = s
    def set_tcp_jerk(self, *a): pass
    def set_tcp_maxacc(self, *a): pass
    def set_joint_jerk(self, *a, **kw): pass
    def set_joint_maxacc(self, *a, **kw): pass
    def disconnect(self): pass

    def get_servo_angle(self, is_radian=False):
        if is_radian:
            return 0, list(self._joints_rad)
        import math
        return 0, [math.degrees(j) for j in self._joints_rad]

    def get_position(self, is_radian=False):
        return 0, list(self._pose)

    def set_position(self, **kw):
        self._moving = False

    def set_servo_angle(self, angle=None, is_radian=False, **kw):
        if angle is not None and len(angle) >= 7:
            if is_radian:
                self._joints_rad = [float(v) for v in angle[:7]]
            else:
                self._joints_rad = [math.radians(float(v)) for v in angle[:7]]
        self._moving = False
        return 0

    def get_is_moving(self):
        return self._moving

    def emergency_stop(self):
        self.state = 4
        self._moving = False

    def ft_sensor_set_zero(self):
        return 0

    def vc_set_cartesian_velocity(self, speeds, **kw): pass
    def vc_set_joint_velocity(self, speeds, **kw): pass


class _FakeInput:
    """Stub replacing XboxInput for dry-run tests."""

    def __init__(self):
        self._snapshot = InputSnapshot()
        self._call_count = 0

    def read(self):
        self._call_count += 1
        return self._snapshot

    def reset_filters(self): pass
    def rumble(self, **kw): pass

    def inject(self, **overrides):
        """For testing: set specific fields for the next read."""
        from dataclasses import replace
        self._snapshot = replace(self._snapshot, **overrides)

# =====================================================================
# Entry point
# =====================================================================

TeleopBaseNode = TeleopV4Node

def main(args=None):
    # Drop whitespace-only argv tokens (e.g. ``--ros-args \  -p``) — they cause
    # rclpy UnknownROSArgsError: [' '].
    if args is None:
        import sys
        args = [a for a in sys.argv if a.strip()]
    rclpy.init(args=args)
    node = TeleopV4Node()
    try:
        ctx = rclpy.get_default_context()
        ctx.on_shutdown(lambda: node._request_motion_abort())
    except AttributeError:
        pass
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
