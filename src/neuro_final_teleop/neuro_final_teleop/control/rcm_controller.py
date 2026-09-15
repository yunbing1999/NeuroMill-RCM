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

    # Converts angular velocity into an equivalent linear task scale.
    characteristic_length_m: float = 0.0572957795
    max_insertion_depth_mm: float = 20.0
    max_withdrawal_depth_mm: float = 10.0
    travel_slowdown_mm: float = 5.0

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
    requested_insertion_mm_s: float
    target_insertion_mm_s: float
    insertion_mm_s: float
    insertion_depth_mm: float
    limited: bool
    travel_limited: bool
    joint_velocity_limited: bool
    joint_acceleration_limited: bool


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

    def _tool_state(self, q_rad, tcp_offset_m):
        """Return the current tool tip and normalized shaft axis."""

        tip, rotation = self.kin.fk_tool(
            [float(v) for v in q_rad[:7]],
            [float(v) for v in tcp_offset_m[:3]],
        )
        tip = np.asarray(tip, dtype=float).reshape(3)
        rotation = np.asarray(rotation, dtype=float).reshape(3, 3)

        if not np.all(np.isfinite(tip)) or not np.all(np.isfinite(rotation)):
            raise ValueError("Tool pose contains non-finite values")

        shaft_axis = self.cfg.shaft_axis_sign * rotation[:, 2]
        axis_norm = np.linalg.norm(shaft_axis)

        if axis_norm < 1e-9:
            raise ValueError("Tool shaft axis is invalid")

        return tip, shaft_axis / axis_norm

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
            tip, shaft_axis = self._tool_state(q_rad, tcp_offset_m)
        except Exception:
            return False

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

    def _limit_insertion(
        self,
        speed_m_s: float,
        depth_mm: float,
    ):
        """Slow or stop insertion near the configured travel limits."""

        if speed_m_s == 0.0:
            return 0.0, False

        max_in = max(float(self.cfg.max_insertion_depth_mm), 0.0)
        max_out = max(float(self.cfg.max_withdrawal_depth_mm), 0.0)
        slowdown = max(float(self.cfg.travel_slowdown_mm), 1e-6)

        remaining = (
            max_in - depth_mm
            if speed_m_s > 0.0
            else depth_mm + max_out
        )

        if remaining <= 0.0:
            return 0.0, True
        if remaining < slowdown:
            return speed_m_s * remaining / slowdown, True
        return speed_m_s, False

    def _limit_joint_motion(
        self,
        qdot: np.ndarray,
        dt_s: float,
    ):
        """Apply joint velocity and acceleration limits."""

        velocity_limited = False
        acceleration_limited = False

        speed_limit = max(float(self.cfg.qdot_limit_rad_s), 0.01)
        max_speed = float(np.max(np.abs(qdot)))

        if max_speed > speed_limit:
            qdot = qdot * speed_limit / max_speed
            velocity_limited = True

        dt = max(float(dt_s), 1e-4)
        step_limit = max(float(self.cfg.qddot_limit_rad_s2), 0.01) * dt
        velocity_change = qdot - self._prev_qdot
        max_change = float(np.max(np.abs(velocity_change)))

        if max_change > step_limit:
            qdot = self._prev_qdot + velocity_change * step_limit / max_change
            acceleration_limited = True

        self._prev_qdot = qdot.copy()
        return qdot, velocity_limited, acceleration_limited

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
        2. Follow the operator's insertion and angular commands.
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

        tip, shaft_axis = self._tool_state(q, tcp_offset_m)

        # -------------------------------------------------------------
        # RCM geometry
        # -------------------------------------------------------------

        # Vector from the current tool tip to the fixed entry point.
        tip_to_entry = self._entry_point_m - tip

        # Signed axial distance from the tip to the entry-point projection.
        shaft_distance_m = float(shaft_axis @ tip_to_entry)

        # Positive depth means insertion from the captured position.
        insertion_depth_mm = -shaft_distance_m * 1000.0

        # Point on the current shaft line closest to the fixed entry point.
        shaft_point = tip + shaft_distance_m * shaft_axis

        # Lateral error between the shaft line and the entry point.
        lateral_error = self._entry_point_m - shaft_point
        lateral_error_mm = np.linalg.norm(lateral_error) * 1000.0


        # -------------------------------------------------------------
        # RCM Jacobian
        # -------------------------------------------------------------

        # Tool-tip Jacobian includes the flange-to-TCP lever arm.
        tool_jacobian = self.kin.tool_jacobian(
            q.tolist(),
            tcp_offset_m[:3],
        )

        # Linear and angular parts of the 6x7 tool Jacobian.
        linear_jacobian = tool_jacobian[:3, :]
        angular_jacobian = tool_jacobian[3:6, :]

        # Remove velocity along the shaft; insertion is allowed by P1.
        perpendicular = np.eye(3) - np.outer(
            shaft_axis,
            shaft_axis,
        )

        # Vector from the tool tip to the closest shaft point.
        lever_arm = shaft_point - tip

        # Velocity Jacobian of the shaft point nearest the entry.
        shaft_point_jacobian = (
            linear_jacobian
            - self._skew(lever_arm) @ angular_jacobian
        )

        # P1 constrains only lateral motion of the shaft point.
        rcm_jacobian = perpendicular @ shaft_point_jacobian


        # -------------------------------------------------------------
        # P1 desired correction velocity
        # -------------------------------------------------------------

        # Proportional feedback drives lateral RCM error toward zero.
        correction_m_s = (
            self.cfg.correction_gain_s
            * lateral_error
        )

        # Convert the configured correction limit from mm/s to m/s.
        correction_limit_m_s = (
            max(self.cfg.max_correction_mm_s, 0.1)
            * 0.001
        )

        # Limit correction magnitude without changing its direction.
        correction_m_s = self._scale_to_norm(
            correction_m_s,
            correction_limit_m_s,
        )


        # -------------------------------------------------------------
        # P1 joint solution and null space
        # -------------------------------------------------------------

        # Damping improves numerical stability near singularities.
        damping = max(self.cfg.damping, 1e-6)

        # Map the desired lateral correction into joint velocity.
        rcm_pinv = self._damped_pinv(
            rcm_jacobian,
            damping,
        )

        # Highest-priority joint velocity:
        # qdot_1 = J_rcm# * v_rcm
        qdot = rcm_pinv @ correction_m_s

        # P2 may only use joint motion that does not disturb P1:
        # N1 = I - J_rcm# * J_rcm
        null_rcm = (
            np.eye(7)
            - rcm_pinv @ rcm_jacobian
        )


        # -------------------------------------------------------------
        # P2 scaling
        # -------------------------------------------------------------

        # Convert angular task error to an equivalent linear scale.
        characteristic_length_m = max(
            self.cfg.characteristic_length_m,
            1e-6,
        )

        # -------------------------------------------------------------
        # Priority 2: operator insertion and angular velocity
        # -------------------------------------------------------------

        insertion_jacobian = (
            shaft_axis.reshape(1, 3)
            @ linear_jacobian
        )

        operator_jacobian = np.vstack(
            (
                insertion_jacobian,
                characteristic_length_m * angular_jacobian,
            )
        )

        operator_in_rcm_null = (
            operator_jacobian
            @ null_rcm
        )

        operator_pinv = self._damped_pinv(
            operator_in_rcm_null,
            damping,
        )

        requested_insertion_mm_s = desired_insertion_m_s * 1000.0
        insertion_target_m_s, travel_limited = self._limit_insertion(
            float(desired_insertion_m_s),
            insertion_depth_mm,
        )
        target_insertion_mm_s = insertion_target_m_s * 1000.0

        operator_target = np.concatenate(
            (
                np.array(
                    [insertion_target_m_s],
                    dtype=float,
                ),
                characteristic_length_m * desired_w,
            )
        )

        operator_residual = (
            operator_target
            - operator_jacobian @ qdot
        )

        qdot = (
            qdot
            + null_rcm
            @ operator_pinv
            @ operator_residual
        )

        # -------------------------------------------------------------
        # Joint velocity and acceleration limits
        # -------------------------------------------------------------

        qdot, joint_velocity_limited, joint_acceleration_limited = (
            self._limit_joint_motion(qdot, dt_s)
        )

        limited = (
            travel_limited
            or joint_velocity_limited
            or joint_acceleration_limited
        )

        # Calculate achieved velocities after all limiting.
        achieved_linear = linear_jacobian @ qdot
        achieved_angular = angular_jacobian @ qdot

        achieved_insertion_mm_s = float(
            shaft_axis @ achieved_linear
            * 1000.0
        )

        return RCMResult(
            qdot_rad_s=qdot.tolist(),
            entry_point_mm=(self._entry_point_m * 1000.0).tolist(),
            shaft_point_mm=(shaft_point * 1000.0).tolist(),
            lateral_error_vector_mm=(lateral_error * 1000.0).tolist(),
            lateral_error_mm=float(lateral_error_mm),
            correction_mm_s=(correction_m_s * 1000.0).tolist(),
            angular_rad_s=achieved_angular.tolist(),
            requested_insertion_mm_s=requested_insertion_mm_s,
            target_insertion_mm_s=target_insertion_mm_s,
            insertion_mm_s=achieved_insertion_mm_s,
            insertion_depth_mm=insertion_depth_mm,
            limited=limited,
            travel_limited=travel_limited,
            joint_velocity_limited=joint_velocity_limited,
            joint_acceleration_limited=joint_acceleration_limited,
        )
