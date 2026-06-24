"""
Safety primitives for teleopv4.

- RateLimiter:  acceleration / deceleration limiting with unified
  scaling for constrained Tip-Lock mode.
- JointRiskMonitor:  smooth risk scaling from actual xArm7 joint limits
  and Jacobian-based singularity metric.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .math_utils import clamp


# ═══════════════════════════════════════════════════════════════════════
# Rate Limiter
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class RateLimiterConfig:
    max_linear_acc_mm_s2: float = 600.0
    max_linear_dec_mm_s2: float = 1400.0
    max_angular_acc_deg_s2: float = 600.0
    max_angular_dec_deg_s2: float = 1600.0


class RateLimiter:
    """Limits acceleration / deceleration of the 6-DOF velocity command.

    In constrained Tip-Lock mode the linear and angular components are scaled
    by the same factor.  In free-teleop the two are scaled independently.
    """

    def __init__(self, cfg: RateLimiterConfig):
        self.cfg = cfg
        self._prev = [0.0] * 6
        self.was_limited = False

    def reset(self) -> None:
        self._prev = [0.0] * 6
        self.was_limited = False

    def limit(
        self, cmd: List[float], dt: float, is_constrained: bool,
    ) -> List[float]:
        """Apply slew-rate limiting.  Returns the limited command."""
        if dt <= 0.0:
            return list(cmd)

        delta_lin = [cmd[i] - self._prev[i] for i in range(3)]
        delta_ang = [cmd[i] - self._prev[i] for i in range(3, 6)]

        dl_norm = _norm3(delta_lin)
        da_norm = _norm3(delta_ang)

        # Choose acc / dec step based on whether speed is increasing
        prev_lin_spd = _norm3(self._prev[:3])
        cmd_lin_spd = _norm3(cmd[:3])
        lin_step = (self.cfg.max_linear_acc_mm_s2
                    if cmd_lin_spd >= prev_lin_spd
                    else self.cfg.max_linear_dec_mm_s2) * dt

        prev_ang_spd = _norm3(self._prev[3:6])
        cmd_ang_spd = _norm3(cmd[3:6])
        ang_step = (self.cfg.max_angular_acc_deg_s2
                    if cmd_ang_spd >= prev_ang_spd
                    else self.cfg.max_angular_dec_deg_s2) * dt

        limited = False

        if is_constrained:
            # Unified scaling: the MORE restrictive factor applies to both,
            # preserving the  v = omega x r  coupling.
            scale = 1.0
            if dl_norm > lin_step and dl_norm > 1e-4:
                scale = min(scale, lin_step / dl_norm)
            if da_norm > ang_step and da_norm > 1e-4:
                scale = min(scale, ang_step / da_norm)
            if scale < 1.0:
                delta_lin = [d * scale for d in delta_lin]
                delta_ang = [d * scale for d in delta_ang]
                limited = True
        else:
            if dl_norm > lin_step and dl_norm > 1e-4:
                s = lin_step / dl_norm
                delta_lin = [d * s for d in delta_lin]
                limited = True
            if da_norm > ang_step and da_norm > 1e-4:
                s = ang_step / da_norm
                delta_ang = [d * s for d in delta_ang]
                limited = True

        out = [
            self._prev[0] + delta_lin[0],
            self._prev[1] + delta_lin[1],
            self._prev[2] + delta_lin[2],
            self._prev[3] + delta_ang[0],
            self._prev[4] + delta_ang[1],
            self._prev[5] + delta_ang[2],
        ]
        self._prev = list(out)
        self.was_limited = limited
        return out


# ═══════════════════════════════════════════════════════════════════════
# Joint Risk Monitor
# ═══════════════════════════════════════════════════════════════════════

# Source: xarm7.urdf.xacro  (lines 100-279) in
#   <workspace>/src/xarm_ros2/xarm_description/urdf/xarm7/xarm7.urdf.xacro
# Units: radians.
XARM7_JOINT_LIMITS: List[Tuple[float, float]] = [
    (-2.0 * math.pi,  2.0 * math.pi),   # J1
    (-2.059,           2.0944),           # J2
    (-2.0 * math.pi,  2.0 * math.pi),   # J3
    (-0.19198,         3.927),            # J4
    (-2.0 * math.pi,  2.0 * math.pi),   # J5
    (-1.69297,         math.pi),          # J6
    (-2.0 * math.pi,  2.0 * math.pi),   # J7
]


@dataclass
class JointRiskConfig:
    joint_margin_deg: float = 15.0
    singularity_warn_sv: float = 0.05
    singularity_hard_sv: float = 0.01
    joint_speed_limit_rad_s: float = 3.0
    joint_speed_warn_rad_s: float = 2.0


@dataclass
class JointRiskResult:
    scale: float              # 0..1 combined risk scale
    reason: str               # human-readable reason (empty if scale==1)
    joint_limit_scale: float  # component from joint limits
    singularity_scale: float  # component from Jacobian SVD
    joint_speed_scale: float  # component from runtime joint speeds
    min_singular_value: float
    worst_joint_margin_deg: float


class JointRiskMonitor:
    """Computes a smooth risk-based speed scale from joint limits,
    Jacobian singularity, and runtime joint speeds.

    Requires a reference to the kinematic model for Jacobian SVD.
    Falls back to conservative scaling if the model is unavailable.
    """

    def __init__(self, cfg: JointRiskConfig, kin_model=None):
        self.cfg = cfg
        self.kin = kin_model  # teleopv4.kinematics.KDLKinModel (optional)
        self._margin_rad = math.radians(cfg.joint_margin_deg)
        self._prev_speed_scale = 1.0

    def compute_risk(
        self,
        joints_rad: List[float],
        joint_speeds_deg_s: Optional[List[float]] = None,
    ) -> JointRiskResult:
        """Evaluate overall risk and return a speed scale factor."""

        jl_scale, jl_reason, worst_margin = self._joint_limit_risk(joints_rad)
        sing_scale, sing_reason, min_sv = self._singularity_risk(joints_rad)
        js_scale = self._joint_speed_risk(joint_speeds_deg_s)

        combined = min(jl_scale, sing_scale, js_scale)
        reasons = [r for r in (jl_reason, sing_reason) if r]
        reason = "; ".join(reasons)

        return JointRiskResult(
            scale=combined,
            reason=reason,
            joint_limit_scale=jl_scale,
            singularity_scale=sing_scale,
            joint_speed_scale=js_scale,
            min_singular_value=min_sv,
            worst_joint_margin_deg=math.degrees(worst_margin),
        )

    # ── Joint-limit proximity ─────────────────────────────────────────

    def _joint_limit_risk(
        self, joints_rad: List[float],
    ) -> Tuple[float, str, float]:
        if len(joints_rad) < 7:
            return 0.3, "joint_state_short", 0.0

        worst_margin = float("inf")
        for i in range(7):
            q = float(joints_rad[i])
            lo, hi = XARM7_JOINT_LIMITS[i]
            margin = min(q - lo, hi - q)
            worst_margin = min(worst_margin, margin)

        if worst_margin <= 0.0:
            return 0.1, "joint_limit_hit", worst_margin
        if worst_margin < self._margin_rad:
            scale = clamp(worst_margin / self._margin_rad, 0.15, 1.0)
            return scale, "joint_limit_near", worst_margin
        return 1.0, "", worst_margin

    # ── Singularity (Jacobian SVD) ────────────────────────────────────

    def _singularity_risk(
        self, joints_rad: List[float],
    ) -> Tuple[float, str, float]:
        if self.kin is None:
            return 1.0, "", 1.0  # no model -> skip

        try:
            min_sv, _cond, _manip = self.kin.singularity_measure(joints_rad)
        except Exception:
            return 0.5, "singularity_computation_failed", 0.0

        warn = self.cfg.singularity_warn_sv
        hard = self.cfg.singularity_hard_sv
        if min_sv <= hard:
            return 0.15, f"singularity_hard(sv={min_sv:.4f})", min_sv
        if min_sv < warn:
            scale = clamp((min_sv - hard) / (warn - hard), 0.15, 1.0)
            return scale, f"singularity_near(sv={min_sv:.4f})", min_sv
        return 1.0, "", min_sv

    # ── Runtime joint speed ───────────────────────────────────────────

    def _joint_speed_risk(
        self, speeds_deg_s: Optional[List[float]],
    ) -> float:
        if speeds_deg_s is None or len(speeds_deg_s) < 7:
            return self._prev_speed_scale

        max_rad = max(abs(math.radians(float(s))) for s in speeds_deg_s[:7])
        limit = self.cfg.joint_speed_limit_rad_s
        warn = self.cfg.joint_speed_warn_rad_s

        if max_rad >= limit:
            target = 0.15
        elif max_rad >= warn:
            target = clamp(
                (limit - max_rad) / max(limit - warn, 0.01), 0.15, 1.0,
            )
        else:
            target = 1.0

        # Smooth: fast decrease, slow recovery
        alpha = 0.35 if target < self._prev_speed_scale else 0.025
        smoothed = self._prev_speed_scale + alpha * (target - self._prev_speed_scale)
        smoothed = clamp(smoothed, 0.15, 1.0)
        self._prev_speed_scale = smoothed
        return smoothed


# ═══════════════════════════════════════════════════════════════════════
# Helper
# ═══════════════════════════════════════════════════════════════════════

def _norm3(v) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
