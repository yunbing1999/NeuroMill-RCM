"""
Pure math utilities for teleopv4.

No ROS, no hardware, no side-effects.  Every function is stateless and
can be unit-tested in isolation.
"""

import math
from typing import List, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Scalar helpers
# ---------------------------------------------------------------------------

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def deadzone_map(value: float, threshold: float) -> float:
    """Map value through deadzone: zero inside, rescaled outside."""
    if abs(value) < threshold:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)


def sigmoid_shape(value: float, gain: float = 1.6) -> float:
    """Smooth tanh-based shaping.  gain=0 is linear, larger gain is steeper."""
    value = clamp(value, -1.0, 1.0)
    denom = math.tanh(gain)
    if abs(denom) < 1e-9:
        return value
    return math.tanh(gain * value) / denom


# ---------------------------------------------------------------------------
# 3-vector helpers  (plain Python lists, no numpy needed)
# ---------------------------------------------------------------------------

def vec3_norm(v: List[float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def vec3_normalize(v: List[float]) -> List[float]:
    n = vec3_norm(v)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def vec3_cross(a: List[float], b: List[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vec3_dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec3_scale(v: List[float], s: float) -> List[float]:
    return [v[0] * s, v[1] * s, v[2] * s]


def vec3_add(a: List[float], b: List[float]) -> List[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec3_sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


# ---------------------------------------------------------------------------
# Rotation helpers  (numpy for matrix ops)
# ---------------------------------------------------------------------------

def skew_symmetric(v: List[float]) -> np.ndarray:
    """3x3 skew-symmetric matrix [v]x  such that  [v]x @ w == cross(v, w)."""
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0],
    ])


def rpy_deg_to_rotmat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX Euler (roll, pitch, yaw) in *degrees* -> 3x3 rotation matrix.

    Convention  R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    Same convention used by the xArm SDK for TCP orientation.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                 cp * cr],
    ])


def rotmat_to_rpy_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """3x3 rotation matrix -> (roll, pitch, yaw) in degrees.  ZYX convention."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def tool_z_axis_from_rpy_deg(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> List[float]:
    """Third column of the rotation matrix (tool Z in base frame)."""
    R = rpy_deg_to_rotmat(roll_deg, pitch_deg, yaw_deg)
    return [float(R[0, 2]), float(R[1, 2]), float(R[2, 2])]
