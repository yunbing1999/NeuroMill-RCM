"""
PyKDL-based kinematic model for xArm7.

Builds the kinematic chain from hardcoded parameters that were extracted
from the official xArm ROS2 description package.  Provides forward
kinematics, geometric Jacobian (flange and tool), and Jacobian SVD for
singularity detection.

Frame semantics (xArm Python SDK, typical on hardware):
    When **controller ``tcp_offset`` translation is zero**, ``get_position()``
    xyz matches ``fk_flange()`` (flange / mounting face in the model).

    When **non-zero TCP translation** (mm) is configured, ``get_position()``
    xyz is the **tool centre point (TCP)** in base frame.  Then
    ``validate_against_sdk`` compares SDK xyz to ``fk_tool()`` using the
    **effective** flange-frame translation
    ``controller_tcp_xyz + virtual_tip_xyz`` (mm), software-only beyond the
    controller.

    TCP offset **orientation** (roll/pitch/yaw) is not applied in ``fk_tool``
    for validation—only ``[x,y,z]`` mm.  Pure-Z (or small) tool extensions match
    well; large RPY offsets may need a future full transform.

DATA SOURCES
------------
Joint origins (xyz in metres, rpy in radians):
    File: <workspace>/src/xarm_ros2/xarm_description/config/
          kinematics/default/xarm7_default_kinematics.yaml

Joint limits (radians) and axes:
    File: <workspace>/src/xarm_ros2/xarm_description/urdf/
          xarm7/xarm7.urdf.xacro   (lines 94-280)

All seven joints are revolute with local axis [0, 0, 1].
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import PyKDL
except ImportError:
    raise ImportError(
        "PyKDL is required for teleopv4 kinematics.  "
        "Install: sudo apt install python3-pykdl"
    )


# ═══════════════════════════════════════════════════════════════════════
# xArm7 kinematic parameters  (DO NOT edit unless the robot changes)
# ═══════════════════════════════════════════════════════════════════════

# Source: xarm7_default_kinematics.yaml
# Each tuple: (x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad)
_JOINT_ORIGINS = [
    (0.0,     0.0,     0.267,   0.0,         0.0,  0.0),     # J1: link_base -> link1
    (0.0,     0.0,     0.0,    -math.pi/2,   0.0,  0.0),     # J2: link1 -> link2
    (0.0,    -0.293,   0.0,     math.pi/2,   0.0,  0.0),     # J3: link2 -> link3
    (0.0525,  0.0,     0.0,     math.pi/2,   0.0,  0.0),     # J4: link3 -> link4
    (0.0775, -0.3425,  0.0,     math.pi/2,   0.0,  0.0),     # J5: link4 -> link5
    (0.0,     0.0,     0.0,     math.pi/2,   0.0,  0.0),     # J6: link5 -> link6
    (0.076,   0.097,   0.0,    -math.pi/2,   0.0,  0.0),     # J7: link6 -> link7
]

# Source: xarm7.urdf.xacro  (lines 94-280)
# Each tuple: (lower_rad, upper_rad)
_JOINT_LIMITS = [
    (-2.0 * math.pi,  2.0 * math.pi),   # J1
    (-2.059,           2.0944),           # J2
    (-2.0 * math.pi,  2.0 * math.pi),   # J3
    (-0.19198,         3.927),            # J4
    (-2.0 * math.pi,  2.0 * math.pi),   # J5
    (-1.69297,         math.pi),          # J6
    (-2.0 * math.pi,  2.0 * math.pi),   # J7
]

# All joints: axis [0, 0, 1] in local frame (xarm7.urdf.xacro)
_JOINT_AXIS = PyKDL.Vector(0, 0, 1)

# xArm SDK get_position() already reports pose including flange/tool geometry.
# Keep model-side fixed flange offset at zero to avoid double-counting during
# FK validation against SDK xyz.
_FLANGE_OFFSET = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # (x, y, z, r, p, y) metres/rad

NUM_JOINTS = 7


# ═══════════════════════════════════════════════════════════════════════
# Validation result
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class ValidationResult:
    valid: bool
    position_error_mm: float
    detail: str


# ═══════════════════════════════════════════════════════════════════════
# Kinematic model
# ═══════════════════════════════════════════════════════════════════════

class KDLKinModel:
    """PyKDL kinematic model for xArm7.

    Usage
    -----
    >>> model = KDLKinModel()
    >>> pos, R = model.fk_flange([0.0]*7)
    >>> J = model.jacobian([0.0]*7)
    """

    def __init__(self):
        self._chain = self._build_chain()
        self._fk_solver = PyKDL.ChainFkSolverPos_recursive(self._chain)
        self._jac_solver = PyKDL.ChainJntToJacSolver(self._chain)

        self._validated = False
        self._validation_error_mm = float("inf")

    # ── Chain construction ────────────────────────────────────────────

    @staticmethod
    def _build_chain() -> PyKDL.Chain:
        """Build KDL chain using fixed+rotating segment pairs.

        URDF convention:  T_joint(q) = F_origin * R(axis, q)
        KDL  convention:  T_segment(q) = Joint(q) * F_tip

        To match URDF we decompose each joint into two KDL segments:
          1. Fixed segment carrying the origin transform F_origin.
          2. Rotating segment with axis [0,0,1] and identity tip frame.
        Product: F_origin * R_z(q), which matches the URDF exactly.
        """
        chain = PyKDL.Chain()
        for i, (x, y, z, roll, pitch, yaw) in enumerate(_JOINT_ORIGINS):
            origin_frame = PyKDL.Frame(
                PyKDL.Rotation.RPY(roll, pitch, yaw),
                PyKDL.Vector(x, y, z),
            )
            fixed_joint = PyKDL.Joint(
                f"origin_{i+1}", PyKDL.Joint.Fixed,
            )
            chain.addSegment(
                PyKDL.Segment(f"origin_{i+1}", fixed_joint, origin_frame),
            )

            rot_joint = PyKDL.Joint(
                f"joint{i+1}",
                PyKDL.Vector(0, 0, 0),
                _JOINT_AXIS,
                PyKDL.Joint.RotAxis,
            )
            chain.addSegment(
                PyKDL.Segment(f"link{i+1}", rot_joint, PyKDL.Frame.Identity()),
            )

        # Fixed flange segment: link7 origin -> flange face
        flange_frame = PyKDL.Frame(
            PyKDL.Rotation.RPY(
                _FLANGE_OFFSET[3], _FLANGE_OFFSET[4], _FLANGE_OFFSET[5],
            ),
            PyKDL.Vector(
                _FLANGE_OFFSET[0], _FLANGE_OFFSET[1], _FLANGE_OFFSET[2],
            ),
        )
        chain.addSegment(
            PyKDL.Segment(
                "flange",
                PyKDL.Joint("flange_fixed", PyKDL.Joint.Fixed),
                flange_frame,
            ),
        )
        return chain

    # ── Forward kinematics ────────────────────────────────────────────

    def fk_flange(
        self, q_rad: List[float],
    ) -> Tuple[List[float], np.ndarray]:
        """FK to flange face using current model fixed offset.

        Returns (position_m, 3x3_rotation_matrix).
        Position is in metres.
        """
        q = _list_to_jntarray(q_rad)
        frame = PyKDL.Frame()
        ret = self._fk_solver.JntToCart(q, frame)
        if ret < 0:
            raise RuntimeError(f"KDL FK failed (code {ret})")
        pos = [frame.p.x(), frame.p.y(), frame.p.z()]
        R = _kdl_rotation_to_numpy(frame.M)
        return pos, R

    def fk_tool(
        self,
        q_rad: List[float],
        tcp_offset_m: Optional[List[float]] = None,
    ) -> Tuple[List[float], np.ndarray]:
        """FK to **tool tip / TCP** (flange + translation in flange frame).

        Applies ``tcp_offset`` translation in the **flange** frame (metres)
        on top of ``fk_flange``: ``p_tcp = p_flange + R_flange @ offset_m``.

        tcp_offset_m: at least ``[x, y, z]`` in metres; orientation entries
        are ignored for the position part.
        """
        pos_f, R_f = self.fk_flange(q_rad)
        if tcp_offset_m is None or len(tcp_offset_m) < 3:
            return pos_f, R_f
        off_base = R_f @ np.array(tcp_offset_m[:3])
        pos_t = [pos_f[0] + off_base[0],
                 pos_f[1] + off_base[1],
                 pos_f[2] + off_base[2]]
        return pos_t, R_f

    # ── Jacobian ──────────────────────────────────────────────────────

    def jacobian(self, q_rad: List[float]) -> np.ndarray:
        """6x7 geometric Jacobian at the flange.

        Rows 0-2: linear velocity,  rows 3-5: angular velocity.
        Units: metres/rad for linear, rad/rad for angular.
        """
        q = _list_to_jntarray(q_rad)
        jac = PyKDL.Jacobian(NUM_JOINTS)
        ret = self._jac_solver.JntToJac(q, jac)
        if ret < 0:
            raise RuntimeError(f"KDL Jacobian failed (code {ret})")
        return _kdl_jacobian_to_numpy(jac)

    def tool_jacobian(
        self,
        q_rad: List[float],
        tcp_offset_m: Optional[List[float]] = None,
    ) -> np.ndarray:
        """6x7 Jacobian at the tool tip.

        Adjusts the linear part to account for the lever arm from flange
        to TCP:   J_v_tool = J_v - [p]_x @ J_w
        where p is the TCP offset in base frame.
        """
        J = self.jacobian(q_rad)
        if tcp_offset_m is None or len(tcp_offset_m) < 3:
            return J
        _, R_f = self.fk_flange(q_rad)
        p_base = R_f @ np.array(tcp_offset_m[:3])

        from .math_utils import skew_symmetric
        S = skew_symmetric(p_base.tolist())

        J_tool = J.copy()
        J_tool[:3, :] = J[:3, :] - S @ J[3:6, :]
        return J_tool

    # ── Singularity measure ───────────────────────────────────────────

    def singularity_measure(
        self,
        q_rad: List[float],
        tcp_offset_m: Optional[List[float]] = None,
    ) -> Tuple[float, float, float]:
        """Compute singularity metrics from Jacobian SVD.

        Returns (min_singular_value, condition_number, manipulability).
        Uses the tool Jacobian if tcp_offset_m is provided.
        """
        J = (self.tool_jacobian(q_rad, tcp_offset_m)
             if tcp_offset_m else self.jacobian(q_rad))
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv = float(sv[-1])
        cond = float(sv[0]) / max(min_sv, 1e-12)
        manip = float(np.prod(sv))
        return min_sv, cond, manip

    # ── Joint limits ──────────────────────────────────────────────────

    @staticmethod
    def joint_limits() -> List[Tuple[float, float]]:
        """Returns list of (lower_rad, upper_rad) for each joint."""
        return list(_JOINT_LIMITS)

    # ── Validation against SDK ────────────────────────────────────────

    @staticmethod
    def effective_tcp_translation_mm(
        controller_tcp_mm_deg: Optional[List[float]],
        virtual_tip_offset_mm: Optional[List[float]],
    ) -> List[float]:
        """Sum controller TCP translation (mm) + software virtual tip (mm).

        Same additive convention as UFACTORY ``tcp_offset`` xyz (flange frame);
        software-only virtual tip is never written to the controller.
        """
        c = [0.0, 0.0, 0.0]
        v = [0.0, 0.0, 0.0]
        if controller_tcp_mm_deg is not None and len(controller_tcp_mm_deg) >= 3:
            c = [float(controller_tcp_mm_deg[i]) for i in range(3)]
        if virtual_tip_offset_mm is not None and len(virtual_tip_offset_mm) >= 3:
            v = [float(virtual_tip_offset_mm[i]) for i in range(3)]
        return [c[i] + v[i] for i in range(3)]

    def validate_against_sdk(
        self,
        sdk_pose_mm_deg: List[float],
        joint_angles_rad: List[float],
        tolerance_mm: float = 2.0,
        tcp_offset_mm_deg: Optional[List[float]] = None,
        virtual_tip_offset_mm: Optional[List[float]] = None,
        validate_with_tcp: bool = True,
    ) -> ValidationResult:
        """Position-only FK vs SDK ``get_position()`` xyz.

        If ``validate_with_tcp`` is False: always ``fk_flange`` vs SDK.
        If effective ``|dx|+|dy|+|dz| <= 1`` mm: flange path.
        Else ``fk_tool`` with ``(controller+virtual)`` mm converted to metres.
        """
        sdk_pos_mm = [float(sdk_pose_mm_deg[i]) for i in range(3)]
        eff_mm = self.effective_tcp_translation_mm(
            tcp_offset_mm_deg, virtual_tip_offset_mm,
        )
        sum_eff = abs(eff_mm[0]) + abs(eff_mm[1]) + abs(eff_mm[2])

        if not validate_with_tcp:
            fk_pos_m, _ = self.fk_flange(joint_angles_rad)
            mode = "forced_flange"
            label_fk, label_sdk = "flange FK", "SDK xyz"
        elif sum_eff <= 1.0:
            fk_pos_m, _ = self.fk_flange(joint_angles_rad)
            mode = "flange"
            label_fk, label_sdk = "flange FK", "SDK xyz"
        else:
            tcp_m = [eff_mm[i] / 1000.0 for i in range(3)]
            fk_pos_m, _ = self.fk_tool(joint_angles_rad, tcp_m)
            mode = "tool_tcp_effective"
            label_fk, label_sdk = "tool FK(eff)", "SDK TCP"

        fk_pos_mm = [p * 1000.0 for p in fk_pos_m]
        err = math.sqrt(sum((a - b) ** 2
                            for a, b in zip(fk_pos_mm, sdk_pos_mm)))

        self._validation_error_mm = err
        self._validated = err < tolerance_mm

        detail = (
            f"{label_fk}({fk_pos_mm[0]:.1f},{fk_pos_mm[1]:.1f},{fk_pos_mm[2]:.1f}) "
            f"vs {label_sdk} ({sdk_pos_mm[0]:.1f},{sdk_pos_mm[1]:.1f},{sdk_pos_mm[2]:.1f}) "
            f"err={err:.2f}mm (tol={tolerance_mm:.1f}mm) | mode={mode}"
        )
        detail += (
            f" | eff_tcp_trans_mm=({eff_mm[0]:.2f},{eff_mm[1]:.2f},{eff_mm[2]:.2f})"
        )

        return ValidationResult(
            valid=self._validated,
            position_error_mm=err,
            detail=detail,
        )


    @property
    def is_validated(self) -> bool:
        return self._validated

    @property
    def validation_error_mm(self) -> float:
        return self._validation_error_mm


# ═══════════════════════════════════════════════════════════════════════
# Internal helpers
# ═══════════════════════════════════════════════════════════════════════

def _list_to_jntarray(vals: List[float]) -> PyKDL.JntArray:
    n = len(vals)
    q = PyKDL.JntArray(n)
    for i in range(n):
        q[i] = float(vals[i])
    return q


def _kdl_rotation_to_numpy(R: PyKDL.Rotation) -> np.ndarray:
    m = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            m[i, j] = R[i, j]
    return m


def _kdl_jacobian_to_numpy(jac: PyKDL.Jacobian) -> np.ndarray:
    rows = 6
    cols = jac.columns()
    J = np.zeros((rows, cols))
    for i in range(rows):
        for j in range(cols):
            J[i, j] = jac[i, j]
    return J
