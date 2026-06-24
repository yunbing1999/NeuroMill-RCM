"""Joint-space fixed-point IK for TeleopV7.

Solves joint velocities so the locked TCP xyz point is the primary task,
while operator angular motion and 7-DOF posture comfort are secondary tasks.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class FixedPointIKConfig:
    pos_gain_s: float = 12.0
    max_correction_mm_s: float = 12.0
    damping: float = 0.025
    qdot_limit_rad_s: float = 0.55
    qddot_limit_rad_s2: float = 2.0
    nullspace_gain: float = 0.06
    hold_wz: bool = False


@dataclass
class FixedPointIKResult:
    qdot_rad_s: List[float]
    tip_error_mm: float
    correction_mm_s: List[float]
    angular_rad_s: List[float]
    limited: bool


class FixedPointIKSolver:
    """Hierarchical resolved-rate IK with fixed TCP xyz as top priority."""

    def __init__(self, kin_model, cfg: FixedPointIKConfig):
        self.kin = kin_model
        self.cfg = cfg
        self._prev_qdot = np.zeros(7, dtype=float)

    def reset(self) -> None:
        self._prev_qdot[:] = 0.0

    @staticmethod
    def _damped_pinv(A: np.ndarray, damping: float) -> np.ndarray:
        if A.size == 0:
            return A.T
        rows = A.shape[0]
        lam2 = max(float(damping), 1e-8) ** 2
        return A.T @ np.linalg.inv((A @ A.T) + (lam2 * np.eye(rows)))

    @staticmethod
    def _scale_to_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n <= max_norm or n < 1e-12:
            return v
        return v * (max_norm / n)

    def solve(
        self,
        q_rad: List[float],
        locked_pos_mm: List[float],
        current_pos_mm: List[float],
        desired_angular_rad_s: List[float],
        tcp_offset_m: Optional[List[float]],
        dt_s: float,
    ) -> FixedPointIKResult:
        q = np.asarray(q_rad[:7], dtype=float)
        lock = np.asarray(locked_pos_mm[:3], dtype=float) * 0.001
        cur = np.asarray(current_pos_mm[:3], dtype=float) * 0.001
        err_m = lock - cur
        err_mm = float(np.linalg.norm(err_m) * 1000.0)

        v_corr_m_s = float(self.cfg.pos_gain_s) * err_m
        max_corr_m_s = max(float(self.cfg.max_correction_mm_s), 0.1) * 0.001
        v_corr_m_s = self._scale_to_norm(v_corr_m_s, max_corr_m_s)

        J = self.kin.tool_jacobian(q.tolist(), tcp_offset_m)
        Jv = np.asarray(J[:3, :], dtype=float)
        Jw = np.asarray(J[3:6, :], dtype=float)
        damping = max(float(self.cfg.damping), 1e-6)

        # Priority 1: keep the locked xyz point fixed/corrected.
        Jv_pinv = self._damped_pinv(Jv, damping)
        qdot = Jv_pinv @ v_corr_m_s
        Nv = np.eye(7) - (Jv_pinv @ Jv)

        # Priority 2: follow operator-requested angular motion without
        # disturbing the fixed point. Use wx/wy by default and leave wz as
        # redundancy unless explicitly requested.
        w_des = np.asarray(desired_angular_rad_s[:3], dtype=float)
        if self.cfg.hold_wz:
            Jw_task = Jw
            w_task = w_des
        else:
            Jw_task = Jw[:2, :]
            w_task = w_des[:2]
        A2 = Jw_task @ Nv
        if A2.shape[0] > 0:
            A2_pinv = self._damped_pinv(A2, damping)
            qdot = qdot + (Nv @ (A2_pinv @ (w_task - (Jw_task @ qdot))))

        # Priority 3: use remaining 7-DOF redundancy to drift gently toward
        # a neutral posture. This helps avoid awkward elbow/wrist winding.
        A_task = np.vstack((Jv, Jw_task))
        A_task_pinv = self._damped_pinv(A_task, damping)
        N_task = np.eye(7) - (A_task_pinv @ A_task)
        q_center = -float(self.cfg.nullspace_gain) * q
        qdot = qdot + (N_task @ q_center)

        limited = False
        qdot_limit = max(float(self.cfg.qdot_limit_rad_s), 0.01)
        max_abs = float(np.max(np.abs(qdot)))
        if max_abs > qdot_limit:
            qdot = qdot * (qdot_limit / max_abs)
            limited = True

        dt = max(float(dt_s), 1e-4)
        max_step = max(float(self.cfg.qddot_limit_rad_s2), 0.01) * dt
        delta = qdot - self._prev_qdot
        delta_abs = float(np.max(np.abs(delta)))
        if delta_abs > max_step:
            qdot = self._prev_qdot + (delta * (max_step / delta_abs))
            limited = True

        self._prev_qdot = np.asarray(qdot, dtype=float)
        actual_w = Jw @ qdot
        return FixedPointIKResult(
            qdot_rad_s=[float(v) for v in qdot.tolist()],
            tip_error_mm=err_mm,
            correction_mm_s=[float(v) for v in (v_corr_m_s * 1000.0).tolist()],
            angular_rad_s=[float(v) for v in actual_w.tolist()],
            limited=limited,
        )
