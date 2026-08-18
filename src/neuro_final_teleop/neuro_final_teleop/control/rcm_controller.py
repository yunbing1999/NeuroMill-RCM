"""Joint-space remote-center-of-motion controller for TeleopV7.

The controller captures a fixed entry point in the robot base frame.
During active control, joint velocities will be calculated so the
instrument shaft continues to pass through that entry point.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class RCMConfig:
    """Numerical and safety configuration for RCM control."""

    correction_gain_s: float = 12.0
    max_correction_mm_s: float = 10.0
    damping: float = 0.025

    qdot_limit_rad_s: float = 0.40
    qddot_limit_rad_s2: float = 1.50

    # The xArm tool shaft is assumed to follow the tool Z axis.
    # Use -1.0 if the surgical instrument points along tool -Z.
    shaft_axis_sign: float = 1.0


@dataclass
class RCMResult:
    """RCM joint command and base-frame diagnostics."""

    qdot_rad_s: List[float]

    entry_point_mm: List[float]
    shaft_point_mm: List[float]
    lateral_error_vector_mm: List[float]
    lateral_error_mm: float

    correction_mm_s: List[float]
    angular_rad_s: List[float]
    insertion_mm_s: float
    limited: bool


class RCMController:
    """Resolved-rate controller for a captured remote center of motion."""

    def __init__(self, kin_model, cfg: RCMConfig):
        self.kin = kin_model
        self.cfg = cfg

        self._entry_point_m: Optional[np.ndarray] = None
        self._captured_axis: Optional[np.ndarray] = None
        self._prev_qdot = np.zeros(7, dtype=float)

    @property
    def active(self) -> bool:
        """True after a valid entry point has been captured."""

        return self._entry_point_m is not None

    @property
    def entry_point_m(self) -> Optional[List[float]]:
        """Captured entry point in the robot base frame, in metres."""

        if self._entry_point_m is None:
            return None
        return [float(v) for v in self._entry_point_m.tolist()]

    @property
    def captured_axis(self) -> Optional[List[float]]:
        """Tool-shaft direction at capture, expressed in the base frame."""

        if self._captured_axis is None:
            return None
        return [float(v) for v in self._captured_axis.tolist()]

    def reset(self) -> None:
        """Clear the captured point and all velocity history."""

        self._entry_point_m = None
        self._captured_axis = None
        self._prev_qdot[:] = 0.0

    def capture(
        self,
        q_rad: List[float],
        tcp_offset_m: Optional[List[float]],
    ) -> bool:
        """Capture the current effective tool tip as the RCM entry point.

        Parameters
        ----------
        q_rad:
            Current seven robot joint positions in radians.
        tcp_offset_m:
            Effective tool-tip translation relative to the flange, in metres.

        Returns
        -------
        bool
            True if capture succeeded.
        """

        if q_rad is None or len(q_rad) < 7:
            return False

        if tcp_offset_m is None or len(tcp_offset_m) < 3:
            tcp_offset_m = [0.0, 0.0, 0.0]

        try:
            tip_position_m, tool_rotation = self.kin.fk_tool(
                [float(v) for v in q_rad[:7]],
                [float(v) for v in tcp_offset_m[:3]],
            )
        except Exception:
            return False

        tip = np.asarray(tip_position_m, dtype=float).reshape(3)
        rotation = np.asarray(tool_rotation, dtype=float).reshape(3, 3)

        if not np.all(np.isfinite(tip)):
            return False
        if not np.all(np.isfinite(rotation)):
            return False

        shaft_axis = rotation[:, 2]
        shaft_axis = (
            float(self.cfg.shaft_axis_sign)
            * shaft_axis
        )

        axis_norm = float(np.linalg.norm(shaft_axis))
        if axis_norm < 1e-9:
            return False

        shaft_axis = shaft_axis / axis_norm

        self._entry_point_m = tip.copy()
        self._captured_axis = shaft_axis.copy()
        self._prev_qdot[:] = 0.0
        return True

    @staticmethod
    def _damped_pinv(
        matrix: np.ndarray,
        damping: float,
    ) -> np.ndarray:
        """Calculate a damped right pseudoinverse."""

        matrix = np.asarray(matrix, dtype=float)

        if matrix.size == 0:
            return matrix.T

        rows = matrix.shape[0]
        damping_sq = max(float(damping), 1e-8) ** 2

        return (
            matrix.T
            @ np.linalg.inv(
                matrix @ matrix.T
                + damping_sq * np.eye(rows)
            )
        )

    @staticmethod
    def _scale_to_norm(
        vector: np.ndarray,
        max_norm: float,
    ) -> np.ndarray:
        """Limit a vector while preserving its direction."""

        vector = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(vector))
        limit = max(float(max_norm), 0.0)

        if norm < 1e-12 or norm <= limit:
            return vector

        return vector * (limit / norm)

    @staticmethod
    def _skew(vector: np.ndarray) -> np.ndarray:
        """Return the cross-product matrix for a three-dimensional vector."""

        x, y, z = [float(v) for v in vector[:3]]

        return np.array(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0],
            ],
            dtype=float,
        )

    def solve(
        self,
        q_rad: List[float],
        desired_angular_rad_s: List[float],
        desired_insertion_m_s: float,
        tcp_offset_m: Optional[List[float]],
        dt_s: float,
    ) -> RCMResult:
        """Calculate joint velocity while preserving the captured RCM.

        Task priority:

        1. Correct lateral RCM error.
        2. Command insertion along the current shaft.
        3. Follow operator-requested angular velocity.
        """

        if not self.active or self._entry_point_m is None:
            raise RuntimeError("RCM solve requested before capture")

        if q_rad is None or len(q_rad) < 7:
            raise ValueError("RCM solve requires seven joint positions")

        if desired_angular_rad_s is None:
            desired_angular_rad_s = [0.0, 0.0, 0.0]

        if len(desired_angular_rad_s) < 3:
            raise ValueError(
                "desired_angular_rad_s must contain three values"
            )

        if tcp_offset_m is None or len(tcp_offset_m) < 3:
            tcp_offset_m = [0.0, 0.0, 0.0]

        q = np.asarray(q_rad[:7], dtype=float)
        desired_w = np.asarray(
            desired_angular_rad_s[:3],
            dtype=float,
        )

        tip_position_m, tool_rotation = self.kin.fk_tool(
            q.tolist(),
            [float(v) for v in tcp_offset_m[:3]],
        )

        tip = np.asarray(
            tip_position_m,
            dtype=float,
        ).reshape(3)

        rotation = np.asarray(
            tool_rotation,
            dtype=float,
        ).reshape(3, 3)

        shaft_axis = (
            float(self.cfg.shaft_axis_sign)
            * rotation[:, 2]
        )

        axis_norm = float(np.linalg.norm(shaft_axis))
        if axis_norm < 1e-9:
            raise RuntimeError("Current tool shaft axis is invalid")

        shaft_axis = shaft_axis / axis_norm

        # Vector from the current tool tip toward the captured entry point.
        tip_to_entry = self._entry_point_m - tip

        # Signed distance from the tip to the closest point on the shaft.
        shaft_distance_m = float(
            shaft_axis @ tip_to_entry
        )

        # Closest point on the current shaft line to the captured entry.
        shaft_point = tip + shaft_distance_m * shaft_axis

        # This error is perpendicular to the current shaft.
        lateral_error = self._entry_point_m - shaft_point
        lateral_error_mm = float(
            np.linalg.norm(lateral_error) * 1000.0
        )

        # Projection onto the plane perpendicular to the shaft.
        perpendicular_projector = (
            np.eye(3)
            - np.outer(shaft_axis, shaft_axis)
        )

        tool_jacobian = self.kin.tool_jacobian(
            q.tolist(),
            [float(v) for v in tcp_offset_m[:3]],
        )

        linear_jacobian = np.asarray(
            tool_jacobian[:3, :],
            dtype=float,
        )
        angular_jacobian = np.asarray(
            tool_jacobian[3:6, :],
            dtype=float,
        )

        # Lever arm from the tool tip to the shaft point nearest the entry.
        lever_arm = shaft_point - tip

        # Velocity Jacobian of that point on the instrument shaft.
        shaft_point_jacobian = (
            linear_jacobian
            - self._skew(lever_arm) @ angular_jacobian
        )

        # Only constrain motion perpendicular to the shaft.
        rcm_jacobian = (
            perpendicular_projector
            @ shaft_point_jacobian
        )

        correction_m_s = (
            float(self.cfg.correction_gain_s)
            * lateral_error
        )

        correction_limit_m_s = (
            max(
                float(self.cfg.max_correction_mm_s),
                0.1,
            )
            * 0.001
        )

        correction_m_s = self._scale_to_norm(
            correction_m_s,
            correction_limit_m_s,
        )

        damping = max(
            float(self.cfg.damping),
            1e-6,
        )

        identity_7 = np.eye(7)

        # -------------------------------------------------------------
        # Priority 1: lateral RCM correction
        # -------------------------------------------------------------

        rcm_pinv = self._damped_pinv(
            rcm_jacobian,
            damping,
        )

        qdot = rcm_pinv @ correction_m_s

        null_rcm = (
            identity_7
            - rcm_pinv @ rcm_jacobian
        )

        # -------------------------------------------------------------
        # Priority 2: insertion along the current shaft
        # -------------------------------------------------------------

        insertion_jacobian = (
            shaft_axis.reshape(1, 3)
            @ linear_jacobian
        )

        insertion_in_rcm_null = (
            insertion_jacobian
            @ null_rcm
        )

        insertion_pinv = self._damped_pinv(
            insertion_in_rcm_null,
            damping,
        )

        insertion_target = np.array(
            [float(desired_insertion_m_s)],
            dtype=float,
        )

        insertion_residual = (
            insertion_target
            - insertion_jacobian @ qdot
        )

        qdot = (
            qdot
            + null_rcm
            @ insertion_pinv
            @ insertion_residual
        )

        null_insertion = (
            null_rcm
            @ (
                identity_7
                - insertion_pinv
                @ insertion_in_rcm_null
            )
        )

        # -------------------------------------------------------------
        # Priority 3: operator angular velocity
        # -------------------------------------------------------------

        angular_in_null = (
            angular_jacobian
            @ null_insertion
        )

        angular_pinv = self._damped_pinv(
            angular_in_null,
            damping,
        )

        angular_residual = (
            desired_w
            - angular_jacobian @ qdot
        )

        qdot = (
            qdot
            + null_insertion
            @ angular_pinv
            @ angular_residual
        )

        # -------------------------------------------------------------
        # Joint velocity and acceleration limits
        # -------------------------------------------------------------

        limited = False

        qdot_limit = max(
            float(self.cfg.qdot_limit_rad_s),
            0.01,
        )

        max_joint_speed = float(
            np.max(np.abs(qdot))
        )

        if max_joint_speed > qdot_limit:
            qdot = (
                qdot
                * qdot_limit
                / max_joint_speed
            )
            limited = True

        dt = max(float(dt_s), 1e-4)

        max_velocity_step = (
            max(
                float(self.cfg.qddot_limit_rad_s2),
                0.01,
            )
            * dt
        )

        velocity_change = (
            qdot - self._prev_qdot
        )

        max_change = float(
            np.max(np.abs(velocity_change))
        )

        if max_change > max_velocity_step:
            qdot = (
                self._prev_qdot
                + velocity_change
                * max_velocity_step
                / max_change
            )
            limited = True

        self._prev_qdot = qdot.copy()

        # Calculate achieved velocities after all limiting.
        achieved_linear = linear_jacobian @ qdot
        achieved_angular = angular_jacobian @ qdot

        achieved_insertion_mm_s = float(
            shaft_axis @ achieved_linear
            * 1000.0
        )

        return RCMResult(
            qdot_rad_s=[float(v) for v in qdot],

            # Base-frame RCM geometry in millimetres.
            entry_point_mm=[
                float(v * 1000.0)
                for v in self._entry_point_m
            ],
            shaft_point_mm=[
                float(v * 1000.0)
                for v in shaft_point
            ],
            lateral_error_vector_mm=[
                float(v * 1000.0)
                for v in lateral_error
            ],
            lateral_error_mm=lateral_error_mm,

            correction_mm_s=[
                float(v * 1000.0)
                for v in correction_m_s
            ],
            angular_rad_s=[
                float(v)
                for v in achieved_angular
            ],
            insertion_mm_s=achieved_insertion_mm_s,
            limited=limited,
        )
