"""Fixed-tip controller used by the active X/R3 constrained mode."""

from dataclasses import dataclass
from typing import List, Tuple

from .math_utils import vec3_norm, vec3_scale, vec3_sub


@dataclass
class TipLockConfig:
    max_angular_deg_s: float = 10.0
    correction_gain: float = 3.0
    correction_deadband_mm: float = 0.3


class TipLockController:
    """Keep the effective TCP/tool-tip position fixed in base frame."""

    def __init__(self, cfg: TipLockConfig):
        self.cfg = cfg
        self.active = False
        self.locked_pos_mm: List[float] = [0.0, 0.0, 0.0]
        self.last_pos_err_mm: float = 0.0

    def capture(self, tcp_pose_mm_deg: List[float]) -> None:
        """Record current effective tip xyz (base mm) as the locked point."""
        self.locked_pos_mm = [
            float(tcp_pose_mm_deg[0]),
            float(tcp_pose_mm_deg[1]),
            float(tcp_pose_mm_deg[2]),
        ]
        self.active = True
        self.last_pos_err_mm = 0.0

    def clear(self) -> None:
        self.active = False
        self.last_pos_err_mm = 0.0

    def compute_velocity(
        self,
        current_pose_mm_deg: List[float],
        stick_x: float,
        stick_y: float,
    ) -> Tuple[List[float], List[float]]:
        """Compute [vx,vy,vz] in mm/s and [wx,wy,wz] in deg/s."""
        if not self.active:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        cur_pos = [float(current_pose_mm_deg[i]) for i in range(3)]
        err = vec3_sub(self.locked_pos_mm, cur_pos)
        err_norm = vec3_norm(err)
        self.last_pos_err_mm = err_norm

        if err_norm > self.cfg.correction_deadband_mm:
            v_corr = vec3_scale(err, self.cfg.correction_gain)
        else:
            v_corr = [0.0, 0.0, 0.0]

        max_w = self.cfg.max_angular_deg_s
        wx = -stick_y * max_w
        wy = stick_x * max_w
        wz = 0.0
        return v_corr, [wx, wy, wz]
