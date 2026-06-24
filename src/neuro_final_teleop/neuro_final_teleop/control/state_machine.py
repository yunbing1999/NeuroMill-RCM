"""
Explicit state machine for teleopv4.

Every legal transition is enumerated.  The StateMachine class enforces
the transition table, logs every change, and calls user-registered
callbacks on exit/enter so the node can perform zero-on-transition
cleanup.
"""

import time
from enum import Enum, unique
from typing import Callable, Dict, Optional, Set


@unique
class TeleopState(str, Enum):
    """All states of the teleopv4 controller."""

    IDLE = "IDLE"
    """Deadman (RB) not held.  Zero velocity always commanded.  Safe resting state."""

    FREE_TELEOP = "FREE_TELEOP"
    """Deadman held, no constraint active.  Right stick Y -> Z; left stick XY
    uses node param ``left_stick_swap_xy`` (default true: LR->Y, UD->X)."""

    TIP_LOCK_CAPTURE = "TIP_LOCK_CAPTURE"
    """Transient (single-frame): records current TCP position as locked tip.
    Immediately transitions to TIP_LOCK_ACTIVE on success, or back to
    FREE_TELEOP on failure.  No velocity command is issued."""

    TIP_LOCK_ACTIVE = "TIP_LOCK_ACTIVE"
    """TCP position held fixed.  Left stick controls orientation around the
    locked tip (planar axes follow ``left_stick_swap_xy`` like FREE).
    NOTE: research prototype — depends on TCP offset calibration."""

    ALIGN_BUSY = "ALIGN_BUSY"
    """Orthogonal alignment in progress (mode-0 position move).  No
    velocity commands accepted.  Returns to FREE_TELEOP when done."""

    FAULT_LATCHED = "FAULT_LATCHED"
    """Entered on robot error, FT overload, joint-limit violation, or
    joystick disconnect.  Requires explicit recovery (BACK button while
    deadman held) to return to IDLE."""


# ── Transition table ─────────────────────────────────────────────────
# Key = source state, Value = set of allowed target states.
# Any transition NOT listed here is silently blocked.
ALLOWED_TRANSITIONS: Dict[TeleopState, Set[TeleopState]] = {
    TeleopState.IDLE: {
        TeleopState.FREE_TELEOP,        # deadman pressed
        TeleopState.FAULT_LATCHED,      # error detected while idle
    },
    TeleopState.FREE_TELEOP: {
        TeleopState.IDLE,               # deadman released
        TeleopState.TIP_LOCK_CAPTURE,   # button A
        TeleopState.ALIGN_BUSY,         # button Y
        TeleopState.FAULT_LATCHED,      # error
    },
    TeleopState.TIP_LOCK_CAPTURE: {
        TeleopState.TIP_LOCK_ACTIVE,    # capture succeeded
        TeleopState.FREE_TELEOP,        # capture failed
        TeleopState.IDLE,               # deadman released during capture
        TeleopState.FAULT_LATCHED,      # error
    },
    TeleopState.TIP_LOCK_ACTIVE: {
        TeleopState.FREE_TELEOP,        # button A (unlock)
        TeleopState.IDLE,               # deadman released
        TeleopState.FAULT_LATCHED,      # error
    },
    TeleopState.ALIGN_BUSY: {
        TeleopState.FREE_TELEOP,        # alignment complete
        TeleopState.IDLE,               # deadman released during align
        TeleopState.FAULT_LATCHED,      # error
    },
    TeleopState.FAULT_LATCHED: {
        TeleopState.IDLE,               # explicit recovery only
    },
}

# States where operator-commanded motion is generated
MOTION_STATES = frozenset({
    TeleopState.FREE_TELEOP,
    TeleopState.TIP_LOCK_ACTIVE,
})

# States using constrained (coupled lin/ang) control
CONSTRAINED_STATES = frozenset({
    TeleopState.TIP_LOCK_ACTIVE,
})


# Type alias for transition callbacks
TransitionCB = Callable[[TeleopState, TeleopState, str], None]


class StateMachine:
    """Manages teleopv4 state with guarded transitions and callbacks.

    on_exit(old, new, reason)  — called *before* the state changes.
    on_enter(old, new, reason) — called *after* the state changes.
    Both callbacks are the place to implement zero-on-transition.
    """

    def __init__(
        self,
        on_exit: Optional[TransitionCB] = None,
        on_enter: Optional[TransitionCB] = None,
        logger=None,
    ):
        self._state = TeleopState.IDLE
        self._prev_state = TeleopState.IDLE
        self._on_exit = on_exit
        self._on_enter = on_enter
        self._logger = logger
        self._transition_time = time.monotonic()
        self._transition_count = 0

    # ── Properties ────────────────────────────────────────────────────

    @property
    def state(self) -> TeleopState:
        return self._state

    @property
    def prev_state(self) -> TeleopState:
        return self._prev_state

    @property
    def time_in_state(self) -> float:
        """Seconds since the last transition."""
        return time.monotonic() - self._transition_time

    @property
    def transition_count(self) -> int:
        return self._transition_count

    # ── Queries ───────────────────────────────────────────────────────

    def is_constrained(self) -> bool:
        return self._state in CONSTRAINED_STATES

    def is_motion_allowed(self) -> bool:
        return self._state in MOTION_STATES

    # ── Transition ────────────────────────────────────────────────────

    def transition_to(self, target: TeleopState, reason: str = "") -> bool:
        """Attempt a state transition.  Returns True on success."""
        if target == self._state:
            return True

        allowed = ALLOWED_TRANSITIONS.get(self._state, set())
        if target not in allowed:
            if self._logger:
                self._logger.warn(
                    f"[SM] BLOCKED {self._state.value} -> {target.value} "
                    f"(reason: {reason})"
                )
            return False

        old = self._state

        if self._on_exit:
            self._on_exit(old, target, reason)

        self._prev_state = old
        self._state = target
        self._transition_time = time.monotonic()
        self._transition_count += 1

        if self._on_enter:
            self._on_enter(old, target, reason)

        if self._logger:
            self._logger.info(
                f"[SM] {old.value} -> {target.value} "
                f"(reason: {reason}) [#{self._transition_count}]"
            )
        return True

    def force_fault(self, reason: str = "") -> None:
        """Unconditional transition to FAULT_LATCHED from any state."""
        old = self._state
        if old == TeleopState.FAULT_LATCHED:
            return
        if self._on_exit:
            self._on_exit(old, TeleopState.FAULT_LATCHED, reason)
        self._prev_state = old
        self._state = TeleopState.FAULT_LATCHED
        self._transition_time = time.monotonic()
        self._transition_count += 1
        if self._on_enter:
            self._on_enter(old, TeleopState.FAULT_LATCHED, reason)
        if self._logger:
            self._logger.error(
                f"[SM] FAULT {old.value} -> FAULT_LATCHED "
                f"(reason: {reason}) [#{self._transition_count}]"
            )
