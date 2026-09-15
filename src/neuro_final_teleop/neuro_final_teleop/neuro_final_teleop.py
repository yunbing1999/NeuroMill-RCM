"""
NeuroFinalTeleopNode

PS5-focused telemanipulation node for experiment:
- Free teleop + fixed-tip mode (Tip-Lock).
- Continuous FT-driven DualSense haptics (rumble + adaptive triggers).
"""

from typing import Optional, Tuple

import math
import json
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from neuro_final_teleop.control.math_utils import clamp
from neuro_final_teleop.control.base_node import TeleopV4Node
from neuro_final_teleop.force_haptics import ForceHapticsMixin
from neuro_final_teleop.motion_modes import MotionModesMixin
from neuro_final_teleop.input.dualsense import DualSenseInput, InputConfig as DSInputConfig
from neuro_final_teleop.control.fixed_point_ik import FixedPointIKConfig, FixedPointIKSolver
from neuro_final_teleop.control.rcm_controller import RCMConfig, RCMController


class NeuroFinalTeleopNode(ForceHapticsMixin, MotionModesMixin, TeleopV4Node):
    def _declare_params(self):
        super()._declare_params()
        self.declare_parameter("v7_force_watchdog_zero", True)
        self.declare_parameter("v7_use_tool_z_depth_in_free", False)
        self.declare_parameter("v7_use_tool_z_depth_in_tip_lock", False)
        self.declare_parameter("v7_depth_planar_deadband", 0.16)
        self.declare_parameter("v7_mode_switch_settle_s", 0.20)
        self.declare_parameter("v7_tip_idle_reanchor", False)
        self.declare_parameter("v7_tip_idle_reanchor_dwell_s", 0.60)
        self.declare_parameter("v7_tip_idle_reanchor_max_err_mm", 0.8)
        self.declare_parameter("v7_tip_lock_max_linear_mm_s", 12.0)
        self.declare_parameter("v7_tip_lock_recovery_err_mm", 6.0)
        self.declare_parameter("v7_tip_lock_recovery_gain_mm_s_per_mm", 0.8)
        self.declare_parameter("v7_tip_lock_min_angular_scale", 0.15)
        self.declare_parameter("v7_tip_lock_error_brake_enable", True)
        self.declare_parameter("v7_tip_lock_error_brake_mm", 2.0)
        self.declare_parameter("v7_tip_lock_error_brake_exp", 1.25)
        self.declare_parameter("v7_tip_lock_pose_source", "sdk_tcp")
        self.declare_parameter("v7_tip_lock_tcp_rotation_comp_enable", True)
        self.declare_parameter("v7_tip_lock_tcp_rotation_comp_gain", 0.8)
        self.declare_parameter("v7_tip_lock_correction_gain", 6.0)
        self.declare_parameter("v7_tip_lock_correction_deadband_mm", 0.10)
        self.declare_parameter("v7_tip_lock_reanchor_on_enter", True)
        self.declare_parameter("v7_tip_lock_r3_toggle_enable", True)
        self.declare_parameter("v7_tip_lock_right_stick_control", True)
        self.declare_parameter("v7_tip_lock_tool_frame_control", True)
        self.declare_parameter("v7_tip_lock_debug", False)
        self.declare_parameter("v7_tip_lock_debug_interval_s", 0.20)
        self.declare_parameter("v7_tip_lock_debug_err_threshold_mm", 2.0)
        self.declare_parameter("v7_tip_lock_joint_ik_enable", True)
        self.declare_parameter("v7_tip_lock_joint_ik_pos_gain_s", 14.0)
        self.declare_parameter("v7_tip_lock_joint_ik_max_correction_mm_s", 12.0)
        self.declare_parameter("v7_tip_lock_joint_ik_damping", 0.025)
        self.declare_parameter("v7_tip_lock_joint_ik_qdot_limit_rad_s", 0.55)
        self.declare_parameter("v7_tip_lock_joint_ik_qddot_limit_rad_s2", 2.0)
        self.declare_parameter("v7_tip_lock_joint_ik_nullspace_gain", 0.06)
        self.declare_parameter("v7_tip_lock_joint_ik_hold_wz", False)

        # Remote-center-of-motion controller.
        self.declare_parameter("v7_rcm_enable", True)
        self.declare_parameter("v7_rcm_correction_gain_s", 12.0)
        self.declare_parameter("v7_rcm_max_correction_mm_s", 10.0)
        self.declare_parameter("v7_rcm_damping", 0.025)
        self.declare_parameter("v7_rcm_qdot_limit_rad_s", 0.40)
        self.declare_parameter("v7_rcm_qddot_limit_rad_s2", 1.50)
        self.declare_parameter("v7_rcm_shaft_axis_sign", 1.0)
        self.declare_parameter("v7_rcm_max_angular_deg_s", 5.0)

        # Keep insertion disabled until rotation-only RCM is validated
        # on the physical robot.
        self.declare_parameter("v7_rcm_insertion_enable", False)
        self.declare_parameter("v7_rcm_max_insertion_mm_s", 5.0)
        self.declare_parameter("v7_rcm_max_insertion_depth_mm",20.0,)
        self.declare_parameter("v7_rcm_max_withdrawal_depth_mm",10.0,)
        self.declare_parameter("v7_rcm_travel_slowdown_mm", 5.0)

        self.declare_parameter("v7_haptic_feedback_gain", 1.0)
        self.declare_parameter("v7_haptics_boot_test", False)
        self.declare_parameter("v7_haptic_event_gain", 1.0)
        self.declare_parameter("v7_haptic_event_min_duration_ms", 160)
        self.declare_parameter("v7_ft_haptic_enable", True)
        self.declare_parameter("v7_ft_haptic_warn_ratio", 0.05)
        self.declare_parameter("v7_ft_haptic_release_ratio", 0.08)
        self.declare_parameter("v7_ft_haptic_force_source", "insertion_axis")
        self.declare_parameter("v7_ft_haptic_require_depth_input", False)
        self.declare_parameter("v7_ft_haptic_depth_input_min", 0.12)
        self.declare_parameter("v7_ft_haptic_full_scale_n", 20.0)
        self.declare_parameter("v7_ft_haptic_full_scale_torque_nm", 3.0)
        self.declare_parameter("v7_ft_haptic_include_torque", False)
        self.declare_parameter("v7_ft_haptic_force_alpha", 0.15)
        self.declare_parameter("v7_ft_haptic_deadband_n", 0.12)
        self.declare_parameter("v7_ft_haptic_torque_deadband_nm", 0.03)
        self.declare_parameter("v7_ft_haptic_shape_exp", 1.8)
        self.declare_parameter("v7_ft_haptic_smooth_alpha", 0.6)
        self.declare_parameter("v7_ft_haptic_contact_on_ratio", 0.020)
        self.declare_parameter("v7_ft_haptic_contact_off_ratio", 0.008)
        self.declare_parameter("v7_ft_haptic_contact_on_dwell_s", 0.12)
        self.declare_parameter("v7_ft_haptic_contact_off_dwell_s", 0.18)
        self.declare_parameter("v7_ft_haptic_min_contact_feedback", 0.02)
        self.declare_parameter("v7_ft_haptic_max_trigger_resistance", 1.0)
        self.declare_parameter("v7_ft_haptic_trigger_travel_deadband", 25.0 / 255.0)
        self.declare_parameter("v7_ft_haptic_hard_block_enable", True)
        self.declare_parameter("v7_ft_haptic_hard_block_on_ratio", 1.0)
        self.declare_parameter("v7_ft_haptic_hard_block_off_ratio", 0.94)
        self.declare_parameter("v7_ft_haptic_trigger_start_position", 0)
        self.declare_parameter("v7_ft_haptic_trigger_end_position", 9)
        self.declare_parameter("v7_ft_haptic_trigger_preload_strength", 1)
        self.declare_parameter("v7_ft_haptic_trigger_max_strength", 8)
        self.declare_parameter("v7_ft_haptic_progressive_trigger_enable", False)
        self.declare_parameter("v7_ft_haptic_auto_tare_enable", True)
        self.declare_parameter("v7_ft_hardware_tare_enable", False)
        self.declare_parameter("v7_ft_haptic_rumble_enable", True)
        self.declare_parameter("v7_ft_haptic_rumble_continuous_enable", True)
        self.declare_parameter("v7_ft_haptic_rumble_interval_s", 0.14)
        self.declare_parameter("v7_ft_haptic_vibration_min_ratio", 0.0)
        self.declare_parameter("v7_ft_haptic_rumble_min_strength", 0.04)
        self.declare_parameter("v7_ft_haptic_rumble_max_strength", 0.85)
        self.declare_parameter("v7_ft_haptic_rumble_shape_exp", 1.25)
        self.declare_parameter("v7_ft_haptic_rumble_min_interval_s", 0.08)
        self.declare_parameter("v7_ft_haptic_rumble_max_interval_s", 0.45)
        self.declare_parameter("v7_ft_haptic_rumble_min_duration_ms", 28)
        self.declare_parameter("v7_ft_haptic_rumble_max_duration_ms", 90)
        self.declare_parameter("v7_ft_haptic_release_pulse_enable", False)
        self.declare_parameter("v7_ft_haptic_trigger_vibration_enable", False)
        # Per-channel enables so trigger-only vs vibration-only studies are
        # possible without touching the contact-detection logic.
        self.declare_parameter("v7_ft_haptic_trigger_enable", True)
        # One-shot rumble pulse on the contact_active rising edge (boundary
        # acknowledgement before the proportional loop kicks in).
        self.declare_parameter("v7_ft_haptic_contact_pulse_enable", True)
        self.declare_parameter("v7_ft_haptic_contact_pulse_strength", 0.55)
        self.declare_parameter("v7_ft_haptic_contact_pulse_duration_ms", 90)
        self.declare_parameter("v7_ft_haptic_debug", False)
        self.declare_parameter("v7_ft_haptic_debug_interval_s", 0.25)
        self.declare_parameter("v7_ft_haptic_debug_topic_enable", True)
        self.declare_parameter("v7_ft_haptic_debug_topic_max_hz", 30.0)
        self.declare_parameter("v7_ft_sensor_lpf_alpha", 0.5)
        self.declare_parameter("v7_contact_gate_enable", True)
        self.declare_parameter("v7_contact_on_threshold_n", 1.5)
        self.declare_parameter("v7_contact_off_threshold_n", 0.75)
        self.declare_parameter("v7_contact_on_dwell_s", 0.15)
        self.declare_parameter("v7_contact_off_dwell_s", 0.30)
        self.declare_parameter("v7_contact_min_haptic_ratio", 0.0)
        self.declare_parameter("v7_contact_shift_gate_enable", True)
        self.declare_parameter("v7_contact_shift_threshold_n", 0.12)
        self.declare_parameter("v7_contact_shift_off_threshold_n", 0.06)
        self.declare_parameter("v7_contact_shift_dwell_s", 0.18)
        self.declare_parameter("v7_contact_baseline_alpha", 0.02)
        self.declare_parameter("v7_ft_poll_fallback_enable", True)
        self.declare_parameter("v7_ft_poll_min_interval_s", 0.04)
        self.declare_parameter("v7_ft_poll_auto_init", True)
        self.declare_parameter("v7_ft_modbus_baudrate", 2000000)
        self.declare_parameter("v7_ft_global_stop_enable", True)
        self.declare_parameter("v7_ft_global_soft_slowdown", True)
        self.declare_parameter("v7_ft_global_soft_min_scale", 0.20)
        self.declare_parameter("v7_ft_global_include_torque", False)
        self.declare_parameter("v7_ft_allow_retreat_on_limit", False)
        self.declare_parameter("v7_ft_retreat_speed_scale", 0.40)
        self.declare_parameter("v7_ft_retreat_min_cos", 0.15)
        self.declare_parameter("v7_contact_error_auto_recover", True)
        self.declare_parameter("v7_contact_error_code", 31)
        self.declare_parameter("v7_contact_recovery_cooldown_s", 0.70)
        self.declare_parameter("v7_vc_reject_soft_codes", [1])
        self.declare_parameter("v7_motion_haptic_enable", False)
        self.declare_parameter("v7_motion_haptic_gain", 0.45)
        self.declare_parameter("v7_motion_haptic_deadband_ratio", 0.03)
        self.declare_parameter("v7_motion_haptic_rumble_interval_s", 0.12)
        self.declare_parameter("dualsense_startup_grace_s", 1.0)
        self.declare_parameter("v7_deadman_button", "circle")
        self.declare_parameter("v7_speed_scale_default", 0.75)
        self.declare_parameter("v7_speed_scale_min", 0.15)
        self.declare_parameter("v7_speed_scale_max", 1.0)
        self.declare_parameter("v7_speed_scale_step", 0.05)
        self.declare_parameter("target_vendor_id", 0x054C)
        self.declare_parameter("target_product_id", 0x0CE6)

    def _load_params(self):
        super()._load_params()
        self.v7_force_watchdog_zero = bool(
            self.get_parameter("v7_force_watchdog_zero").value
        )
        self.v7_use_tool_z_depth_in_free = bool(
            self.get_parameter("v7_use_tool_z_depth_in_free").value
        )
        self.v7_use_tool_z_depth_in_tip_lock = bool(
            self.get_parameter("v7_use_tool_z_depth_in_tip_lock").value
        )
        self.v7_depth_planar_deadband = clamp(
            float(self.get_parameter("v7_depth_planar_deadband").value), 0.0, 0.6
        )
        self.v7_mode_switch_settle_s = max(
            0.0, float(self.get_parameter("v7_mode_switch_settle_s").value)
        )
        self.v7_tip_idle_reanchor = bool(
            self.get_parameter("v7_tip_idle_reanchor").value
        )
        self.v7_tip_idle_reanchor_dwell_s = max(
            0.0, float(self.get_parameter("v7_tip_idle_reanchor_dwell_s").value)
        )
        self.v7_tip_idle_reanchor_max_err_mm = max(
            0.0, float(self.get_parameter("v7_tip_idle_reanchor_max_err_mm").value)
        )
        self.v7_tip_lock_max_linear_mm_s = float(
            self.get_parameter("v7_tip_lock_max_linear_mm_s").value
        )
        self.v7_tip_lock_recovery_err_mm = max(
            0.1, float(self.get_parameter("v7_tip_lock_recovery_err_mm").value)
        )
        self.v7_tip_lock_recovery_gain_mm_s_per_mm = max(
            0.0, float(self.get_parameter("v7_tip_lock_recovery_gain_mm_s_per_mm").value)
        )
        self.v7_tip_lock_min_angular_scale = clamp(
            float(self.get_parameter("v7_tip_lock_min_angular_scale").value), 0.0, 1.0
        )
        self.v7_tip_lock_error_brake_enable = bool(
            self.get_parameter("v7_tip_lock_error_brake_enable").value
        )
        self.v7_tip_lock_error_brake_mm = max(
            0.1, float(self.get_parameter("v7_tip_lock_error_brake_mm").value)
        )
        self.v7_tip_lock_error_brake_exp = max(
            0.2, float(self.get_parameter("v7_tip_lock_error_brake_exp").value)
        )
        self.v7_tip_lock_pose_source = str(
            self.get_parameter("v7_tip_lock_pose_source").value
        ).strip().lower()
        if self.v7_tip_lock_pose_source not in ("sdk_tcp", "kdl_effective"):
            self.get_logger().warn(
                f"[V7] invalid v7_tip_lock_pose_source={self.v7_tip_lock_pose_source!r}; using sdk_tcp"
            )
            self.v7_tip_lock_pose_source = "sdk_tcp"
        self.v7_tip_lock_tcp_rotation_comp_enable = bool(
            self.get_parameter("v7_tip_lock_tcp_rotation_comp_enable").value
        )
        self.v7_tip_lock_tcp_rotation_comp_gain = clamp(
            float(self.get_parameter("v7_tip_lock_tcp_rotation_comp_gain").value), 0.0, 2.0
        )
        self.v7_tip_lock_correction_gain = max(
            0.0, float(self.get_parameter("v7_tip_lock_correction_gain").value)
        )
        self.v7_tip_lock_correction_deadband_mm = max(
            0.0, float(self.get_parameter("v7_tip_lock_correction_deadband_mm").value)
        )
        self.v7_tip_lock_reanchor_on_enter = bool(
            self.get_parameter("v7_tip_lock_reanchor_on_enter").value
        )
        self.v7_tip_lock_r3_toggle_enable = bool(
            self.get_parameter("v7_tip_lock_r3_toggle_enable").value
        )
        self.v7_tip_lock_right_stick_control = bool(
            self.get_parameter("v7_tip_lock_right_stick_control").value
        )
        self.v7_tip_lock_tool_frame_control = bool(
            self.get_parameter("v7_tip_lock_tool_frame_control").value
        )
        self.v7_tip_lock_debug = bool(
            self.get_parameter("v7_tip_lock_debug").value
        )
        self.v7_tip_lock_debug_interval_s = max(
            0.05, float(self.get_parameter("v7_tip_lock_debug_interval_s").value)
        )
        self.v7_tip_lock_debug_err_threshold_mm = max(
            0.0, float(self.get_parameter("v7_tip_lock_debug_err_threshold_mm").value)
        )
        self.v7_tip_lock_joint_ik_enable = bool(
            self.get_parameter("v7_tip_lock_joint_ik_enable").value
        )
        self.v7_tip_lock_joint_ik_pos_gain_s = max(
            0.0, float(self.get_parameter("v7_tip_lock_joint_ik_pos_gain_s").value)
        )
        self.v7_tip_lock_joint_ik_max_correction_mm_s = max(
            0.1, float(self.get_parameter("v7_tip_lock_joint_ik_max_correction_mm_s").value)
        )
        self.v7_tip_lock_joint_ik_damping = max(
            1e-5, float(self.get_parameter("v7_tip_lock_joint_ik_damping").value)
        )
        self.v7_tip_lock_joint_ik_qdot_limit_rad_s = max(
            0.01, float(self.get_parameter("v7_tip_lock_joint_ik_qdot_limit_rad_s").value)
        )
        self.v7_tip_lock_joint_ik_qddot_limit_rad_s2 = max(
            0.01, float(self.get_parameter("v7_tip_lock_joint_ik_qddot_limit_rad_s2").value)
        )
        self.v7_tip_lock_joint_ik_nullspace_gain = max(
            0.0, float(self.get_parameter("v7_tip_lock_joint_ik_nullspace_gain").value)
        )
        self.v7_tip_lock_joint_ik_hold_wz = bool(
            self.get_parameter("v7_tip_lock_joint_ik_hold_wz").value
        )

        self.v7_rcm_enable = bool(
            self.get_parameter("v7_rcm_enable").value
        )

        self.v7_rcm_correction_gain_s = max(
            0.0,
            float(
                self.get_parameter(
                    "v7_rcm_correction_gain_s"
                ).value
            ),
        )

        self.v7_rcm_max_correction_mm_s = max(
            0.1,
            float(
                self.get_parameter(
                    "v7_rcm_max_correction_mm_s"
                ).value
            ),
        )

        self.v7_rcm_damping = max(
            1e-6,
            float(
                self.get_parameter(
                    "v7_rcm_damping"
                ).value
            ),
        )

        self.v7_rcm_qdot_limit_rad_s = max(
            0.01,
            float(
                self.get_parameter(
                    "v7_rcm_qdot_limit_rad_s"
                ).value
            ),
        )

        self.v7_rcm_qddot_limit_rad_s2 = max(
            0.01,
            float(
                self.get_parameter(
                    "v7_rcm_qddot_limit_rad_s2"
                ).value
            ),
        )

        shaft_axis_sign = float(
            self.get_parameter(
                "v7_rcm_shaft_axis_sign"
            ).value
        )

        self.v7_rcm_shaft_axis_sign = (
            -1.0 if shaft_axis_sign < 0.0 else 1.0
        )

        self.v7_rcm_max_angular_deg_s = max(
            0.1,
            float(
                self.get_parameter(
                    "v7_rcm_max_angular_deg_s"
                ).value
            ),
        )

        self.v7_rcm_insertion_enable = bool(
            self.get_parameter(
                "v7_rcm_insertion_enable"
            ).value
        )

        self.v7_rcm_max_insertion_mm_s = max(
            0.1,
            float(
                self.get_parameter(
                    "v7_rcm_max_insertion_mm_s"
                ).value
            ),
        )

        self.v7_rcm_max_insertion_depth_mm = max(0.0, float(self.get_parameter("v7_rcm_max_insertion_depth_mm").value))
        self.v7_rcm_max_withdrawal_depth_mm = max(0.0, float(self.get_parameter("v7_rcm_max_withdrawal_depth_mm").value))
        self.v7_rcm_travel_slowdown_mm = max(0.0, float(self.get_parameter("v7_rcm_travel_slowdown_mm").value))



        self.v7_haptic_feedback_gain = float(
            self.get_parameter("v7_haptic_feedback_gain").value
        )
        self.v7_haptics_boot_test = bool(
            self.get_parameter("v7_haptics_boot_test").value
        )
        self.v7_haptic_event_gain = float(
            self.get_parameter("v7_haptic_event_gain").value
        )
        self.v7_haptic_event_min_duration_ms = int(
            self.get_parameter("v7_haptic_event_min_duration_ms").value
        )
        self.v7_ft_haptic_enable = bool(
            self.get_parameter("v7_ft_haptic_enable").value
        )
        self.v7_ft_haptic_warn_ratio = float(
            self.get_parameter("v7_ft_haptic_warn_ratio").value
        )
        self.v7_ft_haptic_release_ratio = float(
            self.get_parameter("v7_ft_haptic_release_ratio").value
        )
        self.v7_ft_haptic_force_source = str(
            self.get_parameter("v7_ft_haptic_force_source").value
        ).strip().lower()
        if self.v7_ft_haptic_force_source not in ("insertion_axis", "norm", "max_axis"):
            self.get_logger().warn(
                f"[V7] invalid v7_ft_haptic_force_source={self.v7_ft_haptic_force_source!r}; using insertion_axis"
            )
            self.v7_ft_haptic_force_source = "insertion_axis"
        self.v7_ft_haptic_require_depth_input = bool(
            self.get_parameter("v7_ft_haptic_require_depth_input").value
        )
        self.v7_ft_haptic_depth_input_min = clamp(
            float(self.get_parameter("v7_ft_haptic_depth_input_min").value), 0.0, 1.0
        )
        self.v7_ft_haptic_full_scale_n = max(
            0.0, float(self.get_parameter("v7_ft_haptic_full_scale_n").value)
        )
        self.v7_ft_haptic_full_scale_torque_nm = max(
            0.0, float(self.get_parameter("v7_ft_haptic_full_scale_torque_nm").value)
        )
        self.v7_ft_haptic_include_torque = bool(
            self.get_parameter("v7_ft_haptic_include_torque").value
        )
        self.v7_ft_haptic_force_alpha = clamp(
            float(self.get_parameter("v7_ft_haptic_force_alpha").value), 0.01, 1.0
        )
        self.v7_ft_haptic_deadband_n = max(
            0.0, float(self.get_parameter("v7_ft_haptic_deadband_n").value)
        )
        self.v7_ft_haptic_torque_deadband_nm = max(
            0.0, float(self.get_parameter("v7_ft_haptic_torque_deadband_nm").value)
        )
        self.v7_ft_haptic_shape_exp = max(
            0.3, float(self.get_parameter("v7_ft_haptic_shape_exp").value)
        )
        self.v7_ft_haptic_smooth_alpha = clamp(
            float(self.get_parameter("v7_ft_haptic_smooth_alpha").value), 0.01, 1.0
        )
        self.v7_ft_haptic_contact_on_ratio = clamp(
            float(self.get_parameter("v7_ft_haptic_contact_on_ratio").value), 0.0, 1.0
        )
        self.v7_ft_haptic_contact_off_ratio = clamp(
            float(self.get_parameter("v7_ft_haptic_contact_off_ratio").value), 0.0, 1.0
        )
        if self.v7_ft_haptic_contact_off_ratio > self.v7_ft_haptic_contact_on_ratio:
            self.v7_ft_haptic_contact_off_ratio = self.v7_ft_haptic_contact_on_ratio
        self.v7_ft_haptic_contact_on_dwell_s = max(
            0.0, float(self.get_parameter("v7_ft_haptic_contact_on_dwell_s").value)
        )
        self.v7_ft_haptic_contact_off_dwell_s = max(
            0.0, float(self.get_parameter("v7_ft_haptic_contact_off_dwell_s").value)
        )
        self.v7_ft_haptic_min_contact_feedback = clamp(
            float(self.get_parameter("v7_ft_haptic_min_contact_feedback").value), 0.0, 0.5
        )
        self.v7_ft_haptic_max_trigger_resistance = clamp(
            float(self.get_parameter("v7_ft_haptic_max_trigger_resistance").value), 0.05, 1.0
        )
        self.v7_ft_haptic_trigger_travel_deadband = clamp(
            float(self.get_parameter("v7_ft_haptic_trigger_travel_deadband").value), 0.0, 1.0
        )
        self.v7_ft_haptic_hard_block_enable = bool(
            self.get_parameter("v7_ft_haptic_hard_block_enable").value
        )
        self.v7_ft_haptic_hard_block_on_ratio = clamp(
            float(self.get_parameter("v7_ft_haptic_hard_block_on_ratio").value), 0.0, 1.0
        )
        self.v7_ft_haptic_hard_block_off_ratio = clamp(
            float(self.get_parameter("v7_ft_haptic_hard_block_off_ratio").value), 0.0, 1.0
        )
        if self.v7_ft_haptic_hard_block_off_ratio > self.v7_ft_haptic_hard_block_on_ratio:
            self.v7_ft_haptic_hard_block_off_ratio = self.v7_ft_haptic_hard_block_on_ratio
        self.v7_ft_haptic_progressive_trigger_enable = bool(
            self.get_parameter("v7_ft_haptic_progressive_trigger_enable").value
        )
        self.v7_ft_haptic_auto_tare_enable = bool(
            self.get_parameter("v7_ft_haptic_auto_tare_enable").value
        )
        self.v7_ft_hardware_tare_enable = bool(
            self.get_parameter("v7_ft_hardware_tare_enable").value
        )
        self.v7_ft_haptic_rumble_enable = bool(
            self.get_parameter("v7_ft_haptic_rumble_enable").value
        )
        self.v7_ft_haptic_rumble_continuous_enable = bool(
            self.get_parameter("v7_ft_haptic_rumble_continuous_enable").value
        )
        self.v7_ft_haptic_rumble_interval_s = float(
            self.get_parameter("v7_ft_haptic_rumble_interval_s").value
        )
        self.v7_ft_haptic_vibration_min_ratio = clamp(
            float(self.get_parameter("v7_ft_haptic_vibration_min_ratio").value), 0.0, 1.0
        )
        self.v7_ft_haptic_rumble_min_strength = clamp(
            float(self.get_parameter("v7_ft_haptic_rumble_min_strength").value), 0.0, 1.0
        )
        self.v7_ft_haptic_rumble_max_strength = clamp(
            float(self.get_parameter("v7_ft_haptic_rumble_max_strength").value),
            self.v7_ft_haptic_rumble_min_strength,
            1.0,
        )
        self.v7_ft_haptic_rumble_shape_exp = max(
            0.3, float(self.get_parameter("v7_ft_haptic_rumble_shape_exp").value)
        )
        self.v7_ft_haptic_rumble_min_interval_s = max(
            0.01, float(self.get_parameter("v7_ft_haptic_rumble_min_interval_s").value)
        )
        self.v7_ft_haptic_rumble_max_interval_s = max(
            self.v7_ft_haptic_rumble_min_interval_s,
            float(self.get_parameter("v7_ft_haptic_rumble_max_interval_s").value),
        )
        self.v7_ft_haptic_rumble_min_duration_ms = max(
            1, int(self.get_parameter("v7_ft_haptic_rumble_min_duration_ms").value)
        )
        self.v7_ft_haptic_rumble_max_duration_ms = max(
            self.v7_ft_haptic_rumble_min_duration_ms,
            int(self.get_parameter("v7_ft_haptic_rumble_max_duration_ms").value),
        )
        self.v7_ft_haptic_release_pulse_enable = bool(
            self.get_parameter("v7_ft_haptic_release_pulse_enable").value
        )
        self.v7_ft_haptic_trigger_vibration_enable = bool(
            self.get_parameter("v7_ft_haptic_trigger_vibration_enable").value
        )
        self.v7_ft_haptic_trigger_enable = bool(
            self.get_parameter("v7_ft_haptic_trigger_enable").value
        )
        self.v7_ft_haptic_contact_pulse_enable = bool(
            self.get_parameter("v7_ft_haptic_contact_pulse_enable").value
        )
        self.v7_ft_haptic_contact_pulse_strength = clamp(
            float(self.get_parameter("v7_ft_haptic_contact_pulse_strength").value),
            0.0,
            1.0,
        )
        self.v7_ft_haptic_contact_pulse_duration_ms = max(
            1, int(self.get_parameter("v7_ft_haptic_contact_pulse_duration_ms").value)
        )
        self.v7_ft_haptic_debug = bool(
            self.get_parameter("v7_ft_haptic_debug").value
        )
        self.v7_ft_haptic_debug_interval_s = max(
            0.05, float(self.get_parameter("v7_ft_haptic_debug_interval_s").value)
        )
        self.v7_ft_haptic_debug_topic_enable = bool(
            self.get_parameter("v7_ft_haptic_debug_topic_enable").value
        )
        self.v7_ft_haptic_debug_topic_max_hz = max(
            1.0, float(self.get_parameter("v7_ft_haptic_debug_topic_max_hz").value)
        )
        self.v7_ft_sensor_lpf_alpha = clamp(
            float(self.get_parameter("v7_ft_sensor_lpf_alpha").value), 0.001, 1.0
        )
        self.v7_contact_gate_enable = bool(
            self.get_parameter("v7_contact_gate_enable").value
        )
        self.v7_contact_on_threshold_n = max(
            0.0, float(self.get_parameter("v7_contact_on_threshold_n").value)
        )
        self.v7_contact_off_threshold_n = max(
            0.0, float(self.get_parameter("v7_contact_off_threshold_n").value)
        )
        if self.v7_contact_off_threshold_n > self.v7_contact_on_threshold_n:
            self.v7_contact_off_threshold_n = self.v7_contact_on_threshold_n
        self.v7_contact_on_dwell_s = max(
            0.0, float(self.get_parameter("v7_contact_on_dwell_s").value)
        )
        self.v7_contact_off_dwell_s = max(
            0.0, float(self.get_parameter("v7_contact_off_dwell_s").value)
        )
        self.v7_contact_min_haptic_ratio = clamp(
            float(self.get_parameter("v7_contact_min_haptic_ratio").value), 0.0, 0.5
        )
        self.v7_contact_shift_gate_enable = bool(
            self.get_parameter("v7_contact_shift_gate_enable").value
        )
        self.v7_contact_shift_threshold_n = max(
            0.0, float(self.get_parameter("v7_contact_shift_threshold_n").value)
        )
        self.v7_contact_shift_off_threshold_n = max(
            0.0, float(self.get_parameter("v7_contact_shift_off_threshold_n").value)
        )
        if self.v7_contact_shift_off_threshold_n > self.v7_contact_shift_threshold_n:
            self.v7_contact_shift_off_threshold_n = self.v7_contact_shift_threshold_n
        self.v7_contact_shift_dwell_s = max(
            0.0, float(self.get_parameter("v7_contact_shift_dwell_s").value)
        )
        self.v7_contact_baseline_alpha = clamp(
            float(self.get_parameter("v7_contact_baseline_alpha").value), 0.001, 1.0
        )
        self.v7_ft_poll_fallback_enable = bool(
            self.get_parameter("v7_ft_poll_fallback_enable").value
        )
        self.v7_ft_poll_min_interval_s = max(
            0.01, float(self.get_parameter("v7_ft_poll_min_interval_s").value)
        )
        self.v7_ft_poll_auto_init = bool(
            self.get_parameter("v7_ft_poll_auto_init").value
        )
        self.v7_ft_modbus_baudrate = int(
            self.get_parameter("v7_ft_modbus_baudrate").value
        )
        self.v7_ft_global_stop_enable = bool(
            self.get_parameter("v7_ft_global_stop_enable").value
        )
        self.v7_ft_global_soft_slowdown = bool(
            self.get_parameter("v7_ft_global_soft_slowdown").value
        )
        self.v7_ft_global_soft_min_scale = clamp(
            float(self.get_parameter("v7_ft_global_soft_min_scale").value), 0.0, 1.0
        )
        self.v7_ft_global_include_torque = bool(
            self.get_parameter("v7_ft_global_include_torque").value
        )
        self.v7_ft_allow_retreat_on_limit = bool(
            self.get_parameter("v7_ft_allow_retreat_on_limit").value
        )
        self.v7_ft_retreat_speed_scale = clamp(
            float(self.get_parameter("v7_ft_retreat_speed_scale").value), 0.05, 1.0
        )
        self.v7_ft_retreat_min_cos = clamp(
            float(self.get_parameter("v7_ft_retreat_min_cos").value), 0.0, 1.0
        )
        self.v7_contact_error_auto_recover = bool(
            self.get_parameter("v7_contact_error_auto_recover").value
        )
        self.v7_contact_error_code = int(
            self.get_parameter("v7_contact_error_code").value
        )
        self.v7_contact_recovery_cooldown_s = max(
            0.05, float(self.get_parameter("v7_contact_recovery_cooldown_s").value)
        )
        soft_codes = self.get_parameter("v7_vc_reject_soft_codes").value
        self.v7_vc_reject_soft_codes = {int(c) for c in soft_codes}
        self.v7_motion_haptic_enable = bool(
            self.get_parameter("v7_motion_haptic_enable").value
        )
        self.v7_motion_haptic_gain = max(
            0.0, float(self.get_parameter("v7_motion_haptic_gain").value)
        )
        self.v7_motion_haptic_deadband_ratio = clamp(
            float(self.get_parameter("v7_motion_haptic_deadband_ratio").value), 0.0, 0.95
        )
        self.v7_motion_haptic_rumble_interval_s = max(
            0.05, float(self.get_parameter("v7_motion_haptic_rumble_interval_s").value)
        )
        self.v7_speed_scale_min = clamp(
            float(self.get_parameter("v7_speed_scale_min").value), 0.01, 1.0
        )
        self.v7_speed_scale_max = clamp(
            float(self.get_parameter("v7_speed_scale_max").value), self.v7_speed_scale_min, 1.0
        )
        self.v7_speed_scale_default = clamp(
            float(self.get_parameter("v7_speed_scale_default").value),
            self.v7_speed_scale_min,
            self.v7_speed_scale_max,
        )
        self.v7_speed_scale_step = clamp(
            float(self.get_parameter("v7_speed_scale_step").value), 0.01, 0.50
        )
        self.v7_deadman_button = str(
            self.get_parameter("v7_deadman_button").value
        ).strip().lower()
        if self.v7_force_watchdog_zero and self.velocity_watchdog_s != 0.0:
            self.get_logger().warn(
                f"[V7] forcing velocity_watchdog_s from {self.velocity_watchdog_s:.3f} to 0.0 for smooth stream"
            )
            self.velocity_watchdog_s = 0.0

    def __init__(self):
        super().__init__(node_name="neuro_final_teleop")
        self._v7_constrained_settle_until_s = 0.0
        self._v7_idle_since_s = 0.0
        self._v7_prev_ft_feedback = 0.0
        self._v7_last_ft_rumble_s = 0.0
        self._v7_event_pulse_until_s = 0.0
        self._v7_last_ft_poll_s = 0.0
        self._v7_last_ft_haptic_debug_pub_s = 0.0
        self._v7_ft_haptic_debug_pub = None
        self._v7_ft_haptic_debug_timer = None
        if self.v7_ft_haptic_debug_topic_enable:
            self._v7_ft_haptic_debug_pub = self.create_publisher(
                String, "/neuro_final/ft_haptic_debug", 10
            )
            self._v7_ft_haptic_debug_timer = self.create_timer(
                1.0 / max(self.v7_ft_haptic_debug_topic_max_hz, 1.0),
                self._v7_sensor_debug_timer_cb,
            )
        # Keep adaptive feedback aligned with live operator mapping:
        # current robot behavior uses R2 for inward/down.
        self._v7_depth_active_side = "right"
        self._v7_last_tip_dbg_s = 0.0
        self._v7_sensor_lpf_fx = 0.0
        self._v7_sensor_lpf_fy = 0.0
        self._v7_sensor_lpf_fz = 0.0
        self._v7_sensor_lpf_mx = 0.0
        self._v7_sensor_lpf_my = 0.0
        self._v7_sensor_lpf_mz = 0.0
        self._v7_sensor_lpf_initialized = False
        self._v7_sensor_lpf_last_sample_stamp_s = 0.0
        self._v7_debug_selected_force_value = 0.0
        self._v7_debug_target_haptic_ratio = 0.0
        self._v7_debug_smoothed_haptic_ratio = 0.0
        self._v7_debug_actual_sent_ratio = 0.0
        self._v7_debug_selected_force_after_deadband = 0.0
        self._v7_debug_contact_candidate = False
        self._v7_debug_contact_shift_candidate = False
        self._v7_debug_contact_shift_delta_n = 0.0
        self._v7_debug_contact_baseline_n = 0.0
        # One-line explanation of why actual_sent_ratio is what it is. The GUI
        # surfaces this as haptic_block_reason so "graph shows shift but no
        # feedback" is never ambiguous.
        self._v7_debug_haptic_reason = "haptics_disabled"
        self._v7_debug_target_ratio_before_gate = 0.0
        self._v7_debug_target_ratio_after_gate = 0.0
        self._v7_debug_haptics_enabled = False
        self._v7_debug_deadman_active = False
        self._v7_debug_motion_active = False
        self._v7_debug_depth_input_active = False
        self._v7_ft_feedback_filtered = 0.0
        self._v7_ft_force_filtered_n = 0.0
        self._v7_ft_haptic_ratio_filtered = 0.0
        self._v7_ft_hard_block_active = False
        self._v7_ft_contact_active = False
        # Track previous contact_active so we can detect the rising edge once
        # per touch and fire the boundary acknowledgement pulse exactly once.
        self._v7_ft_prev_contact_active = False
        self._v7_ft_contact_on_candidate_since_s = 0.0
        self._v7_ft_contact_off_candidate_since_s = 0.0
        self._v7_ft_contact_shift_candidate_since_s = 0.0
        self._v7_contact_baseline_n = 0.0
        self._v7_contact_baseline_initialized = False
        # Edge-trigger state for safety hard-stop pulse. The legacy code emitted
        # an event pulse every loop iteration while above the FT safety limit,
        # which felt like rapid recoil/buzzing on the trigger. We now pulse only
        # on the rising edge (False -> True).
        self._v7_ft_global_stop_latched = False
        self._v7_ft_jik_stop_latched = False
        self._v7_ft_haptic_zero: Optional[Tuple[float, float, float, float, float, float]] = None
        self._v7_direct_ft_logged = False
        self._v7_ft_auto_init_attempted = False
        self._v7_last_motion_rumble_s = 0.0
        self._v7_last_motion_feedback = 0.0
        self._v7_last_ft_body_rumble = 0.0
        self._v7_last_ft_haptic_dbg_s = 0.0
        self._v7_last_contact_recovery_s = 0.0
        self._v7_tip_lock_started_by_r3 = False
        self._v7_speed_scale = float(self.v7_speed_scale_default)
        self._v7_last_dpad_y = 0
        self._v7_tip_lock_joint_cmd_rad_s = [0.0] * 7
        self._v7_last_joint_cmd_rad_s = [0.0] * 7
        self._v7_joint_velocity_active = False
        self._v7_last_joint_ik_dbg_s = 0.0
        self._v7_fixed_point_ik = FixedPointIKSolver(
            self.kin,
            FixedPointIKConfig(
                pos_gain_s=self.v7_tip_lock_joint_ik_pos_gain_s,
                max_correction_mm_s=self.v7_tip_lock_joint_ik_max_correction_mm_s,
                damping=self.v7_tip_lock_joint_ik_damping,
                qdot_limit_rad_s=self.v7_tip_lock_joint_ik_qdot_limit_rad_s,
                qddot_limit_rad_s2=self.v7_tip_lock_joint_ik_qddot_limit_rad_s2,
                nullspace_gain=self.v7_tip_lock_joint_ik_nullspace_gain,
                hold_wz=self.v7_tip_lock_joint_ik_hold_wz,
            ),
        )

        self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
        self._v7_last_rcm_debug_s = 0.0

        self._v7_rcm_controller = RCMController(
            self.kin,
            RCMConfig(
                correction_gain_s=(
                    self.v7_rcm_correction_gain_s
                ),
                max_correction_mm_s=(
                    self.v7_rcm_max_correction_mm_s
                ),
                damping=self.v7_rcm_damping,
                characteristic_length_m=(
                    self.v7_rcm_max_insertion_mm_s
                    * 0.001
                    / math.radians(
                        self.v7_rcm_max_angular_deg_s
                    )
                ),
                max_insertion_depth_mm=self.v7_rcm_max_insertion_depth_mm,
                travel_slowdown_mm=self.v7_rcm_travel_slowdown_mm,
                max_withdrawal_depth_mm=self.v7_rcm_max_withdrawal_depth_mm,
                qdot_limit_rad_s=(
                    self.v7_rcm_qdot_limit_rad_s
                ),
                qddot_limit_rad_s2=(
                    self.v7_rcm_qddot_limit_rad_s2
                ),
                shaft_axis_sign=(
                    self.v7_rcm_shaft_axis_sign
                ),
            ),
        )

        self._v7_rcm_diag_pub = self.create_publisher(
            String,
            "/neuro_final/rcm_diagnostics",
            10,
        )
        self._v7_last_rcm_diag_s = 0.0

        if not self.dry_run:
            cfg = DSInputConfig(
                deadzone=float(self.get_parameter("deadzone").value),
                filter_alpha=float(self.get_parameter("input_filter_alpha").value),
                release_alpha=float(self.get_parameter("input_release_alpha").value),
                zero_snap=float(self.get_parameter("input_zero_snap").value),
                debounce_s=float(self.get_parameter("button_debounce_s").value),
                startup_grace_s=float(self.get_parameter("dualsense_startup_grace_s").value),
                lx_sign=float(self.get_parameter("lx_sign").value),
                ly_sign=float(self.get_parameter("ly_sign").value),
                rx_sign=float(self.get_parameter("rx_sign").value),
                ry_sign=float(self.get_parameter("ry_sign").value),
                target_vendor_id=int(self.get_parameter("target_vendor_id").value),
                target_product_id=int(self.get_parameter("target_product_id").value),
                trigger_start_position=int(
                    self.get_parameter("v7_ft_haptic_trigger_start_position").value
                ),
                trigger_end_position=int(
                    self.get_parameter("v7_ft_haptic_trigger_end_position").value
                ),
                trigger_preload_strength=int(
                    self.get_parameter("v7_ft_haptic_trigger_preload_strength").value
                ),
                trigger_max_strength=int(
                    self.get_parameter("v7_ft_haptic_trigger_max_strength").value
                ),
                deadman_button=self.v7_deadman_button,
            )
            self.input = DualSenseInput(cfg)
            self.get_logger().info("[V7] PS5 DualSense input layer enabled")
            self._log_dualsense_haptics_status()
        #self.get_logger().info(
        #    "[Teleop ready | Circle/R1 deadman | D-pad up/down speed | "
        #    "Cross fixed-tip toggle | R3 click + right stick fixed-tip | "
        #    "Square FT tare | L2/R2 depth in free mode"
        #)
        self.get_logger().info(
            "[Teleop ready | Circle/R1 deadman | "
            "D-pad up/down speed | "
            "Cross fixed-tip | Options RCM | "
            "Right stick constrained rotation | "
            "Square FT tare | L2/R2 depth"
        )

        # Live mirror of per-channel haptic enables so the GUI evaluation
        # panel (Vibration-only / Adaptive-trigger-only / Off / Both) can
        # toggle the existing output gates at runtime without touching any
        # FT, contact, or control logic. This is the same set of flags that
        # are already consulted by the haptic setters; we just refresh the
        # cached attributes when ROS parameter updates arrive.
        self._v7_haptic_runtime_flags = (
            "v7_ft_haptic_trigger_enable",
            "v7_ft_haptic_rumble_enable",
            "v7_ft_haptic_contact_pulse_enable",
            "v7_ft_haptic_release_pulse_enable",
            "v7_ft_haptic_trigger_vibration_enable",
            "v7_motion_haptic_enable",
        )
        self.add_on_set_parameters_callback(self._on_haptic_eval_parameter_change)

    def _publish_rcm_diagnostics(self, result, desired_w, joints):
        """Publish compact RCM measurements at a maximum of 25 Hz."""

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._v7_last_rcm_diag_s < 0.04:
            return
        self._v7_last_rcm_diag_s = now

        desired_w = [float(v) for v in desired_w]
        achieved_w = [float(v) for v in result.angular_rad_s]

        data = {
            "time_s": now,
            "lateral_error_mm": result.lateral_error_mm,
            "desired_angular_speed_rad_s": math.sqrt(sum(v * v for v in desired_w)),
            "achieved_angular_speed_rad_s": math.sqrt(sum(v * v for v in achieved_w)),
            "angular_error_rad_s": math.sqrt(
                sum((d - a) ** 2 for d, a in zip(desired_w, achieved_w))
            ),
            "requested_insertion_mm_s": result.requested_insertion_mm_s,
            "target_insertion_mm_s": result.target_insertion_mm_s,
            "achieved_insertion_mm_s": result.insertion_mm_s,
            "insertion_depth_mm": result.insertion_depth_mm,
            "max_joint_speed_rad_s": max(abs(v) for v in result.qdot_rad_s),
            "travel_limited": result.travel_limited,
            "joint_velocity_limited": result.joint_velocity_limited,
            "joint_acceleration_limited": result.joint_acceleration_limited,
        }

        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self._v7_rcm_diag_pub.publish(msg)

TeleopV7Node = NeuroFinalTeleopNode


def main(args=None):
    if args is None:
        import sys

        args = [a for a in sys.argv if a.strip()]
    rclpy.init(args=args)
    node = NeuroFinalTeleopNode()
    try:
        ctx = rclpy.get_default_context()
        ctx.on_shutdown(lambda: node._request_motion_abort())
    except AttributeError:
        pass
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
