"""
Configurable Xbox controller input layer for teleopv4.

Features
--------
- Axis indices and signs configurable via InputConfig (ROS params)
- Deadzone with rescaling
- EMA low-pass filter with fast-release path
- Edge detection (just-pressed) with debounce
- Joystick disconnect detection
- Haptic feedback helper
"""

import math
import time
from dataclasses import dataclass, field
from typing import Tuple

import pygame

from neuro_final_teleop.control.math_utils import clamp, deadzone_map


# ---------------------------------------------------------------------------
# Configuration (populated from ROS parameters by the node)
# ---------------------------------------------------------------------------

@dataclass
class InputConfig:
    """All tunables for the input layer.  Defaults match a standard
    Xbox controller on Linux / SDL2."""

    # Axis indices  (SDL joystick axis numbers)
    axis_lx_idx: int = 0
    axis_ly_idx: int = 1
    axis_rx_idx: int = 3
    axis_ry_idx: int = 4
    axis_lt_idx: int = 2
    axis_rt_idx: int = 5

    # Axis sign multipliers (flip +1/-1 to swap direction)
    lx_sign: float = 1.0
    ly_sign: float = 1.0
    rx_sign: float = 1.0
    ry_sign: float = 1.0

    # Button indices  (SDL joystick button numbers)
    btn_deadman_idx: int = 5   # RB
    btn_tip_lock_idx: int = 0  # A  -> Tip-Lock capture / unlock
    btn_b_idx: int = 1
    btn_tare_idx: int = 2      # X / Square -> FT tare
    btn_align_idx: int = 3     # Y  -> Orthogonal align (FREE_TELEOP only)
    btn_align_alt_idx: int = -1  # Optional alternate align index (e.g., PS layouts)
    btn_lb_idx: int = 4        # LB (spare / future use)
    btn_back_idx: int = 6      # BACK -> recovery / quit
    btn_r3_idx: int = 10       # Right-stick press -> momentary constrained control

    # D-pad hat index
    hat_idx: int = 0

    # Deadzone and filtering
    deadzone: float = 0.10
    filter_alpha: float = 0.50
    release_alpha: float = 0.92
    zero_snap: float = 0.02

    # Button debounce
    debounce_s: float = 0.35


# ---------------------------------------------------------------------------
# Snapshot returned every loop
# ---------------------------------------------------------------------------

@dataclass
class InputSnapshot:
    """Immutable snapshot of all controller inputs for one loop tick."""

    # Filtered analog values  (-1..1 for sticks, 0..1 for triggers)
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0

    # Raw (pre-filter) values for diagnostics
    raw_lx: float = 0.0
    raw_ly: float = 0.0
    raw_rx: float = 0.0
    raw_ry: float = 0.0

    # Button states (current frame)
    btn_rb: bool = False
    btn_a: bool = False
    btn_b: bool = False
    btn_x: bool = False
    btn_y: bool = False
    btn_lb: bool = False
    btn_back: bool = False
    btn_r3: bool = False

    # Edge detection: True only on the frame the button was first pressed
    a_edge: bool = False
    b_edge: bool = False
    x_edge: bool = False
    y_edge: bool = False
    lb_edge: bool = False
    back_edge: bool = False
    r3_edge: bool = False

    # D-pad  (x: -1/0/+1,  y: -1/0/+1)
    dpad: Tuple[int, int] = (0, 0)

    # Connection status
    connected: bool = True


# ---------------------------------------------------------------------------
# Input reader
# ---------------------------------------------------------------------------

class XboxInput:
    """Reads an Xbox controller via pygame / SDL and returns InputSnapshot."""

    def __init__(self, config: InputConfig):
        self.cfg = config
        self._joy = None

        # EMA filter state
        self._filtered = {
            "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0,
            "lt": 0.0, "rt": 0.0,
        }

        # Edge detection state: previous button values
        self._prev_btn = {
            "a": False,
            "b": False,
            "x": False,
            "y": False,
            "lb": False,
            "back": False,
            "r3": False,
        }
        self._edge_time = {
            "a": 0.0,
            "b": 0.0,
            "x": 0.0,
            "y": 0.0,
            "lb": 0.0,
            "back": 0.0,
            "r3": 0.0,
        }

        # Initialise pygame and joystick
        pygame.init()
        pygame.joystick.init()
        self._connect()

    # ── Connection ────────────────────────────────────────────────────

    def _connect(self) -> None:
        if pygame.joystick.get_count() == 0:
            raise RuntimeError(
                "No joystick found.  Connect an Xbox controller and retry."
            )
        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()

    @property
    def connected(self) -> bool:
        return self._joy is not None

    # ── Main read ─────────────────────────────────────────────────────

    def read(self) -> InputSnapshot:
        """Poll the controller and return a filtered InputSnapshot.

        Raises RuntimeError if the joystick is disconnected.
        """
        try:
            pygame.event.pump()
        except pygame.error as exc:
            return InputSnapshot(connected=False)

        if self._joy is None:
            return InputSnapshot(connected=False)

        cfg = self.cfg
        joy = self._joy

        def _safe_button(idx: int) -> bool:
            if idx < 0 or idx >= joy.get_numbuttons():
                return False
            return bool(joy.get_button(idx))

        # ── Raw analog axes ──────────────────────────────────────────
        raw_lx = joy.get_axis(cfg.axis_lx_idx) * cfg.lx_sign
        raw_ly = joy.get_axis(cfg.axis_ly_idx) * cfg.ly_sign
        raw_rx = joy.get_axis(cfg.axis_rx_idx) * cfg.rx_sign
        raw_ry = joy.get_axis(cfg.axis_ry_idx) * cfg.ry_sign

        # Triggers: SDL reports -1 (released) to +1 (pressed)
        raw_lt = (joy.get_axis(cfg.axis_lt_idx) + 1.0) / 2.0
        raw_rt = (joy.get_axis(cfg.axis_rt_idx) + 1.0) / 2.0

        # ── Deadzone ─────────────────────────────────────────────────
        dz_lx = deadzone_map(raw_lx, cfg.deadzone)
        dz_ly = deadzone_map(raw_ly, cfg.deadzone)
        dz_rx = deadzone_map(raw_rx, cfg.deadzone)
        dz_ry = deadzone_map(raw_ry, cfg.deadzone)
        dz_lt = 0.0 if raw_lt < cfg.deadzone else raw_lt
        dz_rt = 0.0 if raw_rt < cfg.deadzone else raw_rt

        # ── EMA filter ───────────────────────────────────────────────
        filt_lx = self._filter("lx", dz_lx)
        filt_ly = self._filter("ly", dz_ly)
        filt_rx = self._filter("rx", dz_rx)
        filt_ry = self._filter("ry", dz_ry)
        filt_lt = self._filter("lt", dz_lt)
        filt_rt = self._filter("rt", dz_rt)

        # ── Buttons ──────────────────────────────────────────────────
        btn_rb = _safe_button(cfg.btn_deadman_idx)
        btn_a = _safe_button(cfg.btn_tip_lock_idx)
        btn_b = _safe_button(cfg.btn_b_idx)
        btn_x = _safe_button(cfg.btn_tare_idx)
        btn_y = _safe_button(cfg.btn_align_idx)
        if not btn_y and cfg.btn_align_alt_idx >= 0:
            btn_y = _safe_button(cfg.btn_align_alt_idx)
        btn_lb = _safe_button(cfg.btn_lb_idx)
        btn_back = _safe_button(cfg.btn_back_idx)
        btn_r3 = _safe_button(cfg.btn_r3_idx)

        # ── Edge detection with debounce ─────────────────────────────
        a_edge = self._check_edge("a", btn_a)
        b_edge = self._check_edge("b", btn_b)
        x_edge = self._check_edge("x", btn_x)
        y_edge = self._check_edge("y", btn_y)
        lb_edge = self._check_edge("lb", btn_lb)
        back_edge = self._check_edge("back", btn_back)
        r3_edge = self._check_edge("r3", btn_r3)

        # ── D-pad ────────────────────────────────────────────────────
        dpad = (0, 0)
        if joy.get_numhats() > cfg.hat_idx:
            dpad = joy.get_hat(cfg.hat_idx)

        return InputSnapshot(
            lx=filt_lx, ly=filt_ly, rx=filt_rx, ry=filt_ry,
            lt=filt_lt, rt=filt_rt,
            raw_lx=raw_lx, raw_ly=raw_ly, raw_rx=raw_rx, raw_ry=raw_ry,
            btn_rb=btn_rb, btn_a=btn_a, btn_b=btn_b, btn_x=btn_x, btn_y=btn_y,
            btn_lb=btn_lb, btn_back=btn_back, btn_r3=btn_r3,
            a_edge=a_edge, b_edge=b_edge, x_edge=x_edge, y_edge=y_edge, lb_edge=lb_edge, back_edge=back_edge,
            r3_edge=r3_edge,
            dpad=dpad,
            connected=True,
        )

    # ── Filter ────────────────────────────────────────────────────────

    def _filter(self, key: str, value: float) -> float:
        """EMA with fast-release path and zero-snap."""
        prev = self._filtered[key]
        snap = self.cfg.zero_snap
        alpha = self.cfg.filter_alpha

        if abs(value) <= snap:
            self._filtered[key] = 0.0
            return 0.0

        # Faster response when releasing (moving toward zero or reversing)
        if abs(value) < abs(prev) or (prev * value < 0.0):
            alpha = max(alpha, self.cfg.release_alpha)

        new_val = (1.0 - alpha) * prev + alpha * value
        if abs(new_val) <= snap:
            new_val = 0.0
        self._filtered[key] = new_val
        return new_val

    # ── Edge detection ────────────────────────────────────────────────

    def _check_edge(self, key: str, current: bool) -> bool:
        """Returns True exactly once when a button transitions low->high,
        subject to a debounce window."""
        prev = self._prev_btn[key]
        self._prev_btn[key] = current
        if not (current and not prev):
            return False
        now = time.monotonic()
        if now - self._edge_time[key] < self.cfg.debounce_s:
            return False
        self._edge_time[key] = now
        return True

    # ── Reset ─────────────────────────────────────────────────────────

    def reset_filters(self) -> None:
        """Zero all filter memory.  Called on every state transition."""
        for k in self._filtered:
            self._filtered[k] = 0.0

    # ── Haptics ───────────────────────────────────────────────────────

    def rumble(
        self, strength: float = 0.8, duration_ms: int = 120,
    ) -> None:
        if self._joy is None:
            return
        try:
            s = clamp(strength, 0.0, 1.0)
            self._joy.rumble(s, s, duration_ms)
        except Exception:
            pass
