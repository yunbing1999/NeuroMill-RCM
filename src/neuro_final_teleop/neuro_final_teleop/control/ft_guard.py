"""
Force / Torque guard for teleopv4.

Features over teleopv3
----------------------
- Explicit data states: NO_DATA, FRESH, STALE
- Per-axis AND norm-based thresholds
- Hysteresis (separate engage / disengage thresholds)
- Soft slowdown zone between warning and hard limit
- Configurable stale-data policy  (ALLOW for bench, BLOCK_INWARD for real)
"""

import math
import time
from dataclasses import dataclass
from enum import Enum, unique
from typing import Tuple


@unique
class FTDataState(str, Enum):
    """State of the force/torque sensor data stream."""
    NO_DATA = "NO_DATA"    # never received any FT message
    FRESH = "FRESH"        # last message within max_age_s
    STALE = "STALE"        # last message older than max_age_s


@unique
class StalePolicy(str, Enum):
    """What to do when FT data is NO_DATA or STALE."""
    ALLOW = "allow"              # permit all motion (bench testing without sensor)
    BLOCK_INWARD = "block_inward"  # block inward insertion only


@dataclass
class FTGuardConfig:
    """All tunables for the FT guard.  Populated from ROS parameters."""

    # Per-axis hard limits (absolute value)
    fx_limit_n: float = 25.0
    fy_limit_n: float = 25.0
    fz_limit_n: float = 20.0
    tx_limit_nm: float = 2.0
    ty_limit_nm: float = 2.0
    tz_limit_nm: float = 1.5

    # Force / torque norm hard limits
    force_norm_limit_n: float = 30.0
    torque_norm_limit_nm: float = 2.5

    # Hysteresis ratio: unblock at  limit * hysteresis_ratio
    hysteresis_ratio: float = 0.80

    # Warning ratio: soft slowdown starts at  limit * warning_ratio
    warning_ratio: float = 0.70

    # Stale-data policy
    stale_policy: StalePolicy = StalePolicy.ALLOW
    max_age_s: float = 1.0


@dataclass
class FTReading:
    """Latest sensor reading."""
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    stamp_s: float = 0.0  # monotonic time of last update


@dataclass
class FTGuardResult:
    """Result of one guard evaluation cycle."""
    allowed_speed: float       # depth speed after guard (mm/s, may be reduced)
    scale: float               # 0..1 speed scale from soft slowdown
    blocked: bool              # True if motion was fully blocked
    reason: str                # human-readable reason (empty if OK)
    data_state: FTDataState    # current data freshness


class FTGuard:
    """Evaluates FT sensor data and gates inward insertion."""

    def __init__(self, config: FTGuardConfig):
        self.cfg = config
        self.reading = FTReading()
        self._blocked_latch = False   # hysteresis latch

    # ── Sensor update (called from ROS subscriber) ────────────────────

    def update(self, fx: float, fy: float, fz: float,
               tx: float, ty: float, tz: float, now_s: float) -> None:
        self.reading = FTReading(
            fx=fx, fy=fy, fz=fz, tx=tx, ty=ty, tz=tz, stamp_s=now_s,
        )

    # ── Data state ────────────────────────────────────────────────────

    def data_state(self, now_s: float) -> FTDataState:
        if self.reading.stamp_s <= 0.0:
            return FTDataState.NO_DATA
        if (now_s - self.reading.stamp_s) > self.cfg.max_age_s:
            return FTDataState.STALE
        return FTDataState.FRESH

    # ── Main evaluation ───────────────────────────────────────────────

    def evaluate_depth(
        self,
        depth_speed_mm_s: float,
        inward_positive: bool,
        now_s: float,
    ) -> FTGuardResult:
        """Evaluate whether the requested depth speed is safe.

        Parameters
        ----------
        depth_speed_mm_s : requested insertion speed (positive = inward if
                           inward_positive is True)
        inward_positive  : sign convention for "inward"
        now_s            : current monotonic time (seconds)

        Returns FTGuardResult with the (possibly reduced) speed.
        """
        ds = self.data_state(now_s)

        # Determine if the command is inward
        if inward_positive:
            is_inward = depth_speed_mm_s > 0.0
        else:
            is_inward = depth_speed_mm_s < 0.0

        # Outward motion is always allowed
        if not is_inward:
            return FTGuardResult(
                allowed_speed=depth_speed_mm_s,
                scale=1.0, blocked=False, reason="", data_state=ds,
            )

        # ── Stale / no-data policy ───────────────────────────────────
        if ds != FTDataState.FRESH:
            if self.cfg.stale_policy == StalePolicy.BLOCK_INWARD:
                return FTGuardResult(
                    allowed_speed=0.0, scale=0.0, blocked=True,
                    reason=f"FT data {ds.value}: inward blocked by stale_policy",
                    data_state=ds,
                )
            # StalePolicy.ALLOW: fall through — no FT-based blocking

        # If data is not fresh, we cannot evaluate thresholds
        if ds != FTDataState.FRESH:
            self._blocked_latch = False
            return FTGuardResult(
                allowed_speed=depth_speed_mm_s,
                scale=1.0, blocked=False,
                reason=f"FT data {ds.value}: allowed by stale_policy",
                data_state=ds,
            )

        # ── Per-axis and norm checks ─────────────────────────────────
        r = self.reading
        force_norm = math.sqrt(r.fx ** 2 + r.fy ** 2 + r.fz ** 2)
        torque_norm = math.sqrt(r.tx ** 2 + r.ty ** 2 + r.tz ** 2)

        # Compute worst-case ratio across all channels
        ratios = [
            abs(r.fx) / self.cfg.fx_limit_n,
            abs(r.fy) / self.cfg.fy_limit_n,
            abs(r.fz) / self.cfg.fz_limit_n,
            abs(r.tx) / self.cfg.tx_limit_nm,
            abs(r.ty) / self.cfg.ty_limit_nm,
            abs(r.tz) / self.cfg.tz_limit_nm,
            force_norm / self.cfg.force_norm_limit_n,
            torque_norm / self.cfg.torque_norm_limit_nm,
        ]
        max_ratio = max(ratios)

        # ── Hysteresis logic ─────────────────────────────────────────
        if self._blocked_latch:
            # Currently blocked: unblock only when ALL ratios drop below
            # the hysteresis threshold
            if max_ratio < self.cfg.hysteresis_ratio:
                self._blocked_latch = False
            else:
                return FTGuardResult(
                    allowed_speed=0.0, scale=0.0, blocked=True,
                    reason=self._format_reason(max_ratio, ratios),
                    data_state=ds,
                )

        # Hard block if any channel exceeds its limit
        if max_ratio >= 1.0:
            self._blocked_latch = True
            return FTGuardResult(
                allowed_speed=0.0, scale=0.0, blocked=True,
                reason=self._format_reason(max_ratio, ratios),
                data_state=ds,
            )

        # Soft slowdown in the warning zone
        if max_ratio >= self.cfg.warning_ratio:
            span = 1.0 - self.cfg.warning_ratio
            if span > 1e-6:
                scale = 1.0 - (max_ratio - self.cfg.warning_ratio) / span
            else:
                scale = 0.0
            scale = max(scale, 0.0)
            return FTGuardResult(
                allowed_speed=depth_speed_mm_s * scale,
                scale=scale, blocked=False,
                reason=f"FT soft slowdown ratio={max_ratio:.2f}",
                data_state=ds,
            )

        # All clear
        return FTGuardResult(
            allowed_speed=depth_speed_mm_s,
            scale=1.0, blocked=False, reason="", data_state=ds,
        )

    # ── Helpers ───────────────────────────────────────────────────────

    def _format_reason(self, max_ratio: float, ratios) -> str:
        r = self.reading
        return (
            f"FT guard: max_ratio={max_ratio:.2f} | "
            f"F=({r.fx:+.1f},{r.fy:+.1f},{r.fz:+.1f})N "
            f"T=({r.tx:+.2f},{r.ty:+.2f},{r.tz:+.2f})Nm"
        )

    @property
    def last_reading(self) -> FTReading:
        return self.reading
