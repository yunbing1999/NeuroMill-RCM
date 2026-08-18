"""
DualSense controller input layer for neuro_final_teleop.
"""

import os
import re
import threading
import time
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

from dualsense_controller import DualSenseController

from neuro_final_teleop.control.math_utils import clamp, deadzone_map

try:
    from dualsense_controller import Feedback, SlopeFeedback
except ImportError:
    Feedback = None
    SlopeFeedback = None

try:
    from dualsense_controller import ContinuousResistance
except ImportError:
    ContinuousResistance = None


@dataclass
class InputConfig:
    deadzone: float = 0.10
    filter_alpha: float = 0.50
    release_alpha: float = 0.92
    zero_snap: float = 0.02
    debounce_s: float = 0.35
    startup_grace_s: float = 1.0
    lx_sign: float = 1.0
    ly_sign: float = 1.0
    rx_sign: float = 1.0
    ry_sign: float = 1.0
    target_vendor_id: int = 0x054C
    target_product_id: int = 0x0CE6
    trigger_start_position: int = 0
    trigger_end_position: int = 9
    trigger_preload_strength: int = 1
    trigger_max_strength: int = 8
    deadman_button: str = "r1"


@dataclass
class InputSnapshot:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0

    raw_lx: float = 0.0
    raw_ly: float = 0.0
    raw_rx: float = 0.0
    raw_ry: float = 0.0

    btn_rb: bool = False
    btn_a: bool = False
    btn_b: bool = False
    btn_x: bool = False
    btn_y: bool = False
    #btn_lb: bool = False
    #btn_back: bool = False
    #btn_r3: bool = False
    btn_lb: bool = False
    btn_back: bool = False
    btn_options: bool = False
    btn_r3: bool = False

    a_edge: bool = False
    b_edge: bool = False
    x_edge: bool = False
    y_edge: bool = False
    #lb_edge: bool = False
    #back_edge: bool = False
    #r3_edge: bool = False
    lb_edge: bool = False
    back_edge: bool = False
    options_edge: bool = False
    r3_edge: bool = False

    dpad: Tuple[int, int] = (0, 0)
    connected: bool = True

    _trigger_feedback_cb: Optional[Callable[[float], None]] = None

    def trigger_feedback(self, force_ratio: float) -> None:
        """Update R2 adaptive trigger resistance based on force ratio."""
        if self._trigger_feedback_cb is not None:
            self._trigger_feedback_cb(force_ratio)


class DualSenseInput:
    """Reads a PS5 DualSense controller via dualsense-controller."""

    def __init__(self, config: InputConfig):
        self.cfg = config
        self._init_time = time.monotonic()
        self._controller = None
        self._connect_error: Optional[str] = None
        self._bound_device_path: Optional[str] = None
        self._haptics_writable: bool = False
        self._connected_since_s: float = 0.0
        self._last_read_ok_s: float = 0.0
        self._last_snapshot = InputSnapshot()
        self._rumble_timer: Optional[threading.Timer] = None
        self._filtered = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0}
        #self._prev_btn = {
        #    "a": False,
        #    "b": False,
        #    "x": False,
        #    "y": False,
        #    "lb": False,
        #    "back": False,
        #    "r3": False,
        #}
        self._prev_btn = {
            "a": False,
            "b": False,
            "x": False,
            "y": False,
            "lb": False,
            "back": False,
            "options": False,
            "r3": False,
        }
        #self._edge_time = {
        #    "a": 0.0,
        #    "b": 0.0,
        #    "x": 0.0,
        #    "y": 0.0,
        #    "lb": 0.0,
        #    "back": 0.0,
        #    "r3": 0.0,
        #}
        self._edge_time = {
            "a": 0.0,
            "b": 0.0,
            "x": 0.0,
            "y": 0.0,
            "lb": 0.0,
            "back": 0.0,
            "options": 0.0,
            "r3": 0.0,
        }
        self._connect()

    @staticmethod
    def _parse_uevent(uevent_path: Path) -> Dict[str, str]:
        data: Dict[str, str] = {}
        text = uevent_path.read_text(encoding="utf-8", errors="ignore")
        for line in text.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                data[k.strip()] = v.strip()
        return data

    @staticmethod
    def _extract_vid_pid(uevent: Dict[str, str]) -> Optional[Tuple[int, int]]:
        hid_id = uevent.get("HID_ID", "")
        match = re.match(r"^[0-9A-Fa-f]+:([0-9A-Fa-f]+):([0-9A-Fa-f]+)$", hid_id)
        if not match:
            return None
        try:
            # int(..., 16) handles leading zeros, e.g. 0000054C.
            return int(match.group(1), 16), int(match.group(2), 16)
        except ValueError:
            return None

    def _iter_hidraw_devices(self):
        """Iterate all /dev/hidraw* nodes with parsed sysfs metadata."""
        for dev_node in sorted(Path("/dev").glob("hidraw*")):
            sysfs_name = dev_node.name
            uevent_path = Path("/sys/class/hidraw") / sysfs_name / "device" / "uevent"
            if not uevent_path.exists():
                continue
            try:
                uevent = self._parse_uevent(uevent_path)
            except Exception:
                continue
            yield str(dev_node), uevent

    def _hidraw_candidates(self):
        """Yield accessible hidraw nodes matching configured DualSense VID/PID."""
        for dev_node, uevent in self._iter_hidraw_devices():
            ids = self._extract_vid_pid(uevent)
            if ids is None:
                continue
            vid, pid = ids
            print(f"Checking {dev_node}: Found VID={vid:#06x}, PID={pid:#06x}")
            if vid != self.cfg.target_vendor_id or pid != self.cfg.target_product_id:
                continue
            # Read access is sufficient for control input; write access is only
            # needed for haptics and should not block teleop control.
            if not os.access(dev_node, os.R_OK):
                continue
            yield dev_node

    def _controller_from_path(self, dev_node: str):
        """Bind controller via DeviceInfo object for this hidraw path."""
        try:
            devices = DualSenseController.enumerate_devices()
        except Exception as exc:
            raise RuntimeError(f"DualSense enumerate_devices failed: {exc}") from exc

        selected = None
        for info in devices:
            path_val = getattr(info, "path", None)
            if isinstance(path_val, (bytes, bytearray)):
                path_str = path_val.decode(errors="ignore")
            else:
                path_str = str(path_val) if path_val is not None else ""
            if path_str == dev_node:
                selected = info
                break

        if selected is None:
            raise RuntimeError(f"No DeviceInfo found for {dev_node}")

        try:
            return DualSenseController(device_index_or_device_info=selected)
        except PermissionError:
            return None
        except OSError as exc:
            if "permission denied" in str(exc).lower():
                return None
            raise

    def _connect(self) -> None:
        self._controller = None
        self._connect_error = None
        self._bound_device_path = None

        candidate_nodes = list(self._hidraw_candidates())
        if not candidate_nodes:
            self._connect_error = (
                "No accessible DualSense hidraw node found for "
                f"VID:PID {self.cfg.target_vendor_id:#06x}:{self.cfg.target_product_id:#06x}"
            )
            return

        last_exc = None
        for dev_node in candidate_nodes:
            try:
                controller = self._controller_from_path(dev_node)
                if controller is None:
                    continue
                if hasattr(controller, "activate"):
                    try:
                        controller.activate()
                    except Exception:
                        # Some library versions start streaming lazily; keep device bound.
                        pass
                self._controller = controller
                self._bound_device_path = dev_node
                self._haptics_writable = os.access(dev_node, os.W_OK)
                now = time.monotonic()
                self._connected_since_s = now
                self._last_read_ok_s = now
                self._connect_error = None
                return
            except PermissionError:
                continue
            except OSError as exc:
                if "permission denied" in str(exc).lower():
                    continue
                last_exc = exc
            except Exception as exc:
                last_exc = exc

        if last_exc is not None:
            self._connect_error = str(last_exc)
        else:
            self._connect_error = "DualSense found but unavailable (permission or busy)."

    @property
    def connected(self) -> bool:
        return self._controller is not None

    @property
    def bound_device_path(self) -> Optional[str]:
        return self._bound_device_path

    @property
    def device(self):
        return self._controller

    def _read_axis(self, name: str, default: float = 0.0) -> float:
        if self._controller is None:
            return default
        # dualsense-controller exposes axis/trigger as property objects with .value
        # in many versions.
        value = getattr(self._controller, name, None)
        if value is None:
            return float(default)
        if hasattr(value, "value"):
            try:
                return float(value.value)
            except Exception:
                return float(default)
        if callable(value):
            try:
                return float(value())
            except Exception:
                return float(default)
        try:
            return float(value)
        except Exception:
            return float(default)

    def _read_axis_multi(self, names: Tuple[str, ...], default: float = 0.0) -> float:
        for n in names:
            v = self._read_axis(n, default=float("nan"))
            if not math.isnan(v):
                return float(v)
        return float(default)

    @staticmethod
    def _normalize_trigger(value: float) -> float:
        # Accept common API ranges:
        # - [-1, 1]  -> map to [0, 1]
        # - [0, 1]   -> passthrough
        # - [0, 255] -> scale to [0, 1]
        v = float(value)
        if v > 1.5:
            v = v / 255.0
        elif v < 0.0:
            v = (v + 1.0) / 2.0
        return clamp(v, 0.0, 1.0)

    def _read_button(self, name: str) -> bool:
        if self._controller is None:
            return False
        # dualsense-controller exposes buttons as ButtonProperty with .pressed
        # in many versions.
        value = getattr(self._controller, name, None)
        if value is None:
            return False
        if hasattr(value, "pressed"):
            try:
                return bool(value.pressed)
            except Exception:
                return False
        if hasattr(value, "value"):
            try:
                return bool(value.value)
            except Exception:
                return False
        if callable(value):
            try:
                return bool(value())
            except Exception:
                return False
        return bool(value)

    def _read_dpad(self) -> Tuple[int, int]:
        x = int(self._read_button("btn_right")) - int(self._read_button("btn_left"))
        y = int(self._read_button("btn_up")) - int(self._read_button("btn_down"))
        return (x, y)

    def read(self) -> InputSnapshot:
        if self._controller is None:
            return InputSnapshot(connected=False)
        try:
            raw_lx = self._read_axis("left_stick_x")
            raw_ly = self._read_axis("left_stick_y")
            raw_rx = self._read_axis("right_stick_x")
            raw_ry = self._read_axis("right_stick_y")
            raw_lx *= self.cfg.lx_sign
            raw_ly *= self.cfg.ly_sign
            raw_rx *= self.cfg.rx_sign
            raw_ry *= self.cfg.ry_sign
            raw_lt = self._normalize_trigger(
                self._read_axis_multi(
                    ("left_trigger", "trigger_l2", "l2", "left_trigger_value"), 0.0
                )
            )
            raw_rt = self._normalize_trigger(
                self._read_axis_multi(
                    ("right_trigger", "trigger_r2", "r2", "right_trigger_value"), 0.0
                )
            )
        except Exception:
            now = time.monotonic()
            if self.device is not None and (now - self._init_time) <= self.cfg.startup_grace_s:
                return InputSnapshot(connected=True, _trigger_feedback_cb=self.set_drilling_feedback)
            if self.device is not None and (now - self._last_read_ok_s) <= self.cfg.startup_grace_s:
                return InputSnapshot(
                    lx=self._last_snapshot.lx, ly=self._last_snapshot.ly,
                    rx=self._last_snapshot.rx, ry=self._last_snapshot.ry,
                    lt=self._last_snapshot.lt, rt=self._last_snapshot.rt,
                    raw_lx=self._last_snapshot.raw_lx, raw_ly=self._last_snapshot.raw_ly,
                    raw_rx=self._last_snapshot.raw_rx, raw_ry=self._last_snapshot.raw_ry,
                    btn_rb=self._last_snapshot.btn_rb, btn_a=self._last_snapshot.btn_a,
                    btn_b=self._last_snapshot.btn_b, btn_x=self._last_snapshot.btn_x, btn_y=self._last_snapshot.btn_y,
                    #btn_lb=self._last_snapshot.btn_lb, btn_back=self._last_snapshot.btn_back,
                    #btn_r3=self._last_snapshot.btn_r3,
                    btn_lb=self._last_snapshot.btn_lb,
                    btn_back=self._last_snapshot.btn_back,
                    btn_options=self._last_snapshot.btn_options,
                    btn_r3=self._last_snapshot.btn_r3,
                    #a_edge=False, b_edge=False, x_edge=False, y_edge=False, lb_edge=False, back_edge=False,
                    #r3_edge=False,
                    a_edge=False, b_edge=False, x_edge=False, y_edge=False,
                    lb_edge=False, back_edge=False,
                    options_edge=False, r3_edge=False,
                    dpad=self._last_snapshot.dpad, connected=True,
                    _trigger_feedback_cb=self.set_drilling_feedback,
                )
            return InputSnapshot(connected=False)

        cfg = self.cfg
        dz_lx = deadzone_map(raw_lx, cfg.deadzone)
        dz_ly = deadzone_map(raw_ly, cfg.deadzone)
        dz_rx = deadzone_map(raw_rx, cfg.deadzone)
        dz_ry = deadzone_map(raw_ry, cfg.deadzone)
        dz_lt = 0.0 if raw_lt < cfg.deadzone else raw_lt
        dz_rt = 0.0 if raw_rt < cfg.deadzone else raw_rt

        filt_lx = self._filter("lx", dz_lx)
        filt_ly = self._filter("ly", dz_ly)
        filt_rx = self._filter("rx", dz_rx)
        filt_ry = self._filter("ry", dz_ry)
        filt_lt = self._filter("lt", dz_lt)
        filt_rt = self._filter("rt", dz_rt)

        dpad = self._read_dpad()
        deadman_button = str(self.cfg.deadman_button).strip().lower()
        if deadman_button in ("dpad_down", "down", "d_down"):
            btn_rb = dpad[1] < 0
        elif deadman_button in ("dpad_up", "up", "d_up"):
            btn_rb = dpad[1] > 0
        elif deadman_button in ("dpad_left", "left", "d_left"):
            btn_rb = dpad[0] < 0
        elif deadman_button in ("dpad_right", "right", "d_right"):
            btn_rb = dpad[0] > 0
        elif deadman_button in ("circle", "o", "btn_circle"):
            btn_rb = self._read_button("btn_circle") or self._read_button("btn_r1")
        elif deadman_button in ("cross", "x", "btn_cross"):
            btn_rb = self._read_button("btn_cross")
        else:
            btn_rb = self._read_button("btn_r1")
        btn_a = self._read_button("btn_cross")
        btn_b = self._read_button("btn_circle")
        btn_x = self._read_button("btn_square")
        btn_y = self._read_button("btn_triangle")
        #btn_lb = self._read_button("btn_l1")
        #btn_back = self._read_button("btn_create")
        #btn_r3 = self._read_button("btn_r3")
        btn_lb = self._read_button("btn_l1")
        btn_back = self._read_button("btn_create")

        # Small button on the right side of the DualSense touchpad.
        # Support common property names used by library versions.
        btn_options = (
            self._read_button("btn_options")
            or self._read_button("btn_option")
        )

        btn_r3 = self._read_button("btn_r3")

        a_edge = self._check_edge("a", btn_a)
        b_edge = self._check_edge("b", btn_b)
        x_edge = self._check_edge("x", btn_x)
        y_edge = self._check_edge("y", btn_y)
        #lb_edge = self._check_edge("lb", btn_lb)
        #back_edge = self._check_edge("back", btn_back)
        #r3_edge = self._check_edge("r3", btn_r3)
        lb_edge = self._check_edge("lb", btn_lb)
        back_edge = self._check_edge("back", btn_back)
        options_edge = self._check_edge(
            "options",
            btn_options,
        )
        r3_edge = self._check_edge("r3", btn_r3)

        snapshot = InputSnapshot(
            lx=filt_lx,
            ly=filt_ly,
            rx=filt_rx,
            ry=filt_ry,
            lt=filt_lt,
            rt=filt_rt,
            raw_lx=raw_lx,
            raw_ly=raw_ly,
            raw_rx=raw_rx,
            raw_ry=raw_ry,
            btn_rb=btn_rb,
            btn_a=btn_a,
            btn_b=btn_b,
            btn_x=btn_x,
            btn_y=btn_y,
            #btn_lb=btn_lb,
            #btn_back=btn_back,
            #btn_r3=btn_r3,
            btn_lb=btn_lb,
            btn_back=btn_back,
            btn_options=btn_options,
            btn_r3=btn_r3,
            a_edge=a_edge,
            b_edge=b_edge,
            x_edge=x_edge,
            y_edge=y_edge,
            #lb_edge=lb_edge,
            #back_edge=back_edge,
            #r3_edge=r3_edge,
            lb_edge=lb_edge,
            back_edge=back_edge,
            options_edge=options_edge,
            r3_edge=r3_edge,
            dpad=dpad,
            connected=True,
            _trigger_feedback_cb=self.set_drilling_feedback,
        )
        self._last_read_ok_s = time.monotonic()
        self._last_snapshot = snapshot
        return snapshot

    def _filter(self, key: str, value: float) -> float:
        prev = self._filtered[key]
        snap = self.cfg.zero_snap
        alpha = self.cfg.filter_alpha
        if abs(value) <= snap:
            self._filtered[key] = 0.0
            return 0.0
        if abs(value) < abs(prev) or (prev * value < 0.0):
            alpha = max(alpha, self.cfg.release_alpha)
        new_val = (1.0 - alpha) * prev + alpha * value
        if abs(new_val) <= snap:
            new_val = 0.0
        self._filtered[key] = new_val
        return new_val

    def _check_edge(self, key: str, current: bool) -> bool:
        prev = self._prev_btn[key]
        self._prev_btn[key] = current
        if not (current and not prev):
            return False
        now = time.monotonic()
        if now - self._edge_time[key] < self.cfg.debounce_s:
            return False
        self._edge_time[key] = now
        return True

    def reset_filters(self) -> None:
        for key in self._filtered:
            self._filtered[key] = 0.0

    def _set_rumble_levels(self, left: float, right: float) -> bool:
        if self._controller is None:
            return False
        applied = False
        l = clamp(left, 0.0, 1.0)
        r = clamp(right, 0.0, 1.0)
        try:
            left_prop = getattr(self._controller, "left_rumble", None)
            if left_prop is not None and hasattr(left_prop, "set"):
                left_prop.set(l)
                applied = True
        except Exception:
            pass
        try:
            right_prop = getattr(self._controller, "right_rumble", None)
            if right_prop is not None and hasattr(right_prop, "set"):
                right_prop.set(r)
                applied = True
        except Exception:
            pass
        return applied

    def _schedule_rumble_stop(self, duration_ms: int) -> None:
        if self._rumble_timer is not None:
            try:
                self._rumble_timer.cancel()
            except Exception:
                pass
            self._rumble_timer = None
        delay_s = max(int(duration_ms), 1) / 1000.0

        def _stop():
            try:
                self._set_rumble_levels(0.0, 0.0)
            except Exception:
                pass

        self._rumble_timer = threading.Timer(delay_s, _stop)
        self._rumble_timer.daemon = True
        self._rumble_timer.start()

    def set_body_rumble(self, left: float, right: Optional[float] = None) -> None:
        if self._controller is None:
            return
        l = clamp(left, 0.0, 1.0)
        r = l if right is None else clamp(right, 0.0, 1.0)
        if self._rumble_timer is not None:
            try:
                self._rumble_timer.cancel()
            except Exception:
                pass
            self._rumble_timer = None
        if self._set_rumble_levels(l, r):
            return
        try:
            if hasattr(self._controller, "set_rumble"):
                self._controller.set_rumble(int(255 * l), int(255 * r), 80)
            elif hasattr(self._controller, "rumble"):
                self._controller.rumble(l, r, 80)
        except Exception:
            pass

    def _trigger_effect(self, side: str):
        if self._controller is None:
            return None
        trig_name = "left_trigger" if side == "left" else "right_trigger"
        trig = getattr(self._controller, trig_name, None)
        if trig is None:
            return None
        return getattr(trig, "effect", None)

    def _set_trigger_off(self, side: str) -> None:
        if self._controller is None:
            return
        try:
            eff = self._trigger_effect(side)
            if eff is not None:
                if hasattr(eff, "off"):
                    eff.off()
                    return
                if hasattr(eff, "no_resistance"):
                    eff.no_resistance()
                    return
        except Exception:
            pass

    def _set_right_trigger_off(self) -> None:
        self._set_trigger_off("right")

    def _set_trigger_continuous_resistance(
        self, side: str, start_position: int, force_255: int
    ) -> bool:
        """Apply 1..255 adaptive trigger resistance when the API supports it."""
        if self._controller is None:
            return False

        start_position = int(clamp(start_position, 0, 9))
        force_255 = int(clamp(force_255, 1, 255))

        direct = getattr(self._controller, "set_trigger_continuous_resistance", None)
        if callable(direct):
            try:
                direct(side, start_position, force_255)
                return True
            except TypeError:
                try:
                    direct(
                        side=side,
                        start_position=start_position,
                        force=force_255,
                    )
                    return True
                except Exception:
                    pass
            except Exception:
                pass

        eff = self._trigger_effect(side)
        if eff is None:
            return False

        method = getattr(eff, "continuous_resistance", None)
        if callable(method):
            try:
                method(start_position=start_position, force=force_255)
                return True
            except TypeError:
                try:
                    method(start_position, force_255)
                    return True
                except Exception:
                    pass
            except Exception:
                pass

        method = getattr(eff, "set_continuous_resistance", None)
        if callable(method):
            try:
                method(start_position=start_position, force=force_255)
                return True
            except TypeError:
                try:
                    method(start_position, force_255)
                    return True
                except Exception:
                    pass
            except Exception:
                pass

        return False

    def _set_trigger_resistance(self, side: str, force_ratio: float) -> None:
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio, 0.0, 1.0)
        start_position = int(clamp(self.cfg.trigger_start_position, 0, 9))
        if ratio <= 1e-4:
            self._set_trigger_off(side)
            return

        force_255 = int(clamp(round(ratio * 255.0), 1, 255))
        if self._set_trigger_continuous_resistance(side, start_position, force_255):
            return

        try:
            eff = self._trigger_effect(side)
            if eff is not None and hasattr(eff, "feedback"):
                strength_8 = int(clamp(round(ratio * 8.0), 1, 8))
                eff.feedback(start_position=start_position, strength=strength_8)
                return
        except Exception:
            pass

        strength = int(clamp(round(ratio * 8.0), 1, 8))
        if side == "left":
            self.set_trigger_profile_left(start_position=start_position, strength=strength)
        else:
            self.set_trigger_profile(start_position=start_position, strength=strength)

    def _set_trigger_hard_block(
        self, side: str, start_position: Optional[int] = None, strength: int = 8
    ) -> None:
        """Apply a distinct high-force barrier for safety-limit contact."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return

        pos = self.cfg.trigger_start_position if start_position is None else start_position
        pos = int(clamp(pos, 0, 9))
        strength_8 = int(clamp(strength, 1, 8))

        direct = getattr(self._controller, "set_trigger_hard_block", None)
        if callable(direct):
            try:
                direct(side, pos, strength_8)
                return
            except TypeError:
                try:
                    direct(side=side, start_position=pos, strength=strength_8)
                    return
                except Exception:
                    pass
            except Exception:
                pass

        try:
            eff = self._trigger_effect(side)
            if eff is not None:
                # Prefer a mode change/click-like profile over proportional force.
                method = getattr(eff, "hard_resistance", None)
                if callable(method):
                    try:
                        method(start_position=pos)
                        return
                    except TypeError:
                        try:
                            method(pos)
                            return
                        except Exception:
                            pass
                    except Exception:
                        pass

                method = getattr(eff, "section_resistance", None)
                if callable(method):
                    try:
                        method(start_position=pos, end_position=9, force=255)
                        return
                    except TypeError:
                        try:
                            method(pos, 9, 255)
                            return
                        except Exception:
                            pass
                    except Exception:
                        pass

                method = getattr(eff, "feedback", None)
                if callable(method):
                    try:
                        method(start_position=pos, strength=strength_8)
                        return
                    except TypeError:
                        try:
                            method(pos, strength_8)
                            return
                        except Exception:
                            pass
                    except Exception:
                        pass
        except Exception:
            pass

        if self._set_trigger_continuous_resistance(side, pos, 255):
            return
        if side == "left":
            self.set_trigger_profile_left(start_position=pos, strength=strength_8)
        else:
            self.set_trigger_profile(start_position=pos, strength=strength_8)

    def set_trigger_hard_block(self, start_position: Optional[int] = None, strength: int = 8) -> None:
        """Switch R2 to a hard barrier profile."""
        self._set_trigger_hard_block("right", start_position=start_position, strength=strength)

    def set_trigger_hard_block_left(self, start_position: Optional[int] = None, strength: int = 8) -> None:
        """Switch L2 to a hard barrier profile."""
        self._set_trigger_hard_block("left", start_position=start_position, strength=strength)

    def _set_trigger_progressive_resistance(self, side: str, force_ratio: float) -> None:
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio, 0.0, 1.0)
        if ratio <= 1e-4:
            self._set_trigger_off(side)
            return

        start_position = int(clamp(self.cfg.trigger_start_position, 0, 8))
        end_position = int(clamp(self.cfg.trigger_end_position, start_position + 1, 9))
        preload = int(clamp(self.cfg.trigger_preload_strength, 1, 8))
        max_strength = int(clamp(self.cfg.trigger_max_strength, preload, 8))
        end_strength = int(clamp(round(preload + ratio * (max_strength - preload)), preload, max_strength))

        try:
            eff = self._trigger_effect(side)
            if eff is not None and hasattr(eff, "slope_feedback"):
                eff.slope_feedback(
                    start_position=start_position,
                    end_position=end_position,
                    start_strength=preload,
                    end_strength=end_strength,
                )
                return
            if eff is not None and hasattr(eff, "multiple_position_feedback"):
                strengths = [0] * 10
                span = max(end_position - start_position, 1)
                for i in range(start_position, 10):
                    x = clamp((i - start_position) / span, 0.0, 1.0)
                    strengths[i] = int(clamp(round(preload + x * (end_strength - preload)), 1, 8))
                eff.multiple_position_feedback(tuple(strengths))
                return
        except Exception:
            pass

        self._set_trigger_resistance(side, ratio)

    def rumble(self, strength: float = 0.8, duration_ms: int = 120) -> None:
        if self._controller is None:
            return
        s = clamp(strength, 0.0, 1.0)
        try:
            if hasattr(self._controller, "set_rumble"):
                self._controller.set_rumble(int(255 * s), int(255 * s), duration_ms)
                return
            if hasattr(self._controller, "rumble"):
                self._controller.rumble(s, s, duration_ms)
                return
        except Exception:
            # fall through to property API fallback
            ...
        # dualsense-controller (property API): left_rumble/right_rumble states
        if self._set_rumble_levels(s, s):
            self._schedule_rumble_stop(duration_ms)

    def set_drilling_feedback(self, force_ratio: float, gain: float = 1.0) -> None:
        """Set adaptive trigger resistance on R2 from FT guard ratio."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio * max(gain, 0.0), 0.0, 1.0)
        self._set_trigger_resistance("right", ratio)

    def set_drilling_feedback_progressive(self, force_ratio: float, gain: float = 1.0) -> None:
        """Set progressive R2 resistance: light preload plus force-scaled ramp."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio * max(gain, 0.0), 0.0, 1.0)
        self._set_trigger_progressive_resistance("right", ratio)

    def set_drilling_feedback_left(self, force_ratio: float, gain: float = 1.0) -> None:
        """Set adaptive trigger resistance on L2 from FT guard ratio."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio * max(gain, 0.0), 0.0, 1.0)
        self._set_trigger_resistance("left", ratio)

    def set_drilling_feedback_progressive_left(self, force_ratio: float, gain: float = 1.0) -> None:
        """Set progressive L2 resistance: light preload plus force-scaled ramp."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        ratio = clamp(force_ratio * max(gain, 0.0), 0.0, 1.0)
        self._set_trigger_progressive_resistance("left", ratio)

    def set_trigger_profile(self, start_position: int, strength: int) -> None:
        """Apply explicit R2 adaptive trigger profile (0..8 range)."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        start_position = int(clamp(start_position, 0, 9))
        strength = int(clamp(strength, 0, 8))

        # dualsense-controller (property API)
        try:
            eff = self._trigger_effect("right")
            if eff is not None:
                if strength <= 0:
                    self._set_trigger_off("right")
                    return
                if hasattr(eff, "feedback"):
                    eff.feedback(start_position=start_position, strength=max(strength, 1))
                    return
                if hasattr(eff, "continuous_resistance"):
                    force_255 = int(round((max(strength, 1) / 8.0) * 255.0))
                    eff.continuous_resistance(start_position=start_position, force=force_255)
                    return
        except Exception:
            pass

        # Backward-compatible API paths
        effect = None
        try:
            if ContinuousResistance is not None:
                effect = ContinuousResistance(start_position=start_position, strength=strength)
            elif SlopeFeedback is not None:
                effect = SlopeFeedback(start_position=start_position, end_position=8, start_strength=strength, end_strength=strength)
            elif Feedback is not None:
                effect = Feedback(start_position=start_position, strength=strength)
        except Exception:
            effect = None

        try:
            if effect is not None and hasattr(self._controller, "set_right_trigger"):
                self._controller.set_right_trigger(effect)
            elif hasattr(self._controller, "set_right_trigger_feedback"):
                self._controller.set_right_trigger_feedback(start_position=start_position, strength=strength)
        except Exception:
            pass

    def set_trigger_profile_left(self, start_position: int, strength: int) -> None:
        """Apply explicit L2 adaptive trigger profile (0..8 range)."""
        if self._controller is None:
            return
        if not self._haptics_writable:
            return
        start_position = int(clamp(start_position, 0, 9))
        strength = int(clamp(strength, 0, 8))

        try:
            eff = self._trigger_effect("left")
            if eff is not None:
                if strength <= 0:
                    self._set_trigger_off("left")
                    return
                if hasattr(eff, "feedback"):
                    eff.feedback(start_position=start_position, strength=max(strength, 1))
                    return
                if hasattr(eff, "continuous_resistance"):
                    force_255 = int(round((max(strength, 1) / 8.0) * 255.0))
                    eff.continuous_resistance(start_position=start_position, force=force_255)
                    return
        except Exception:
            pass

    def set_trigger_vibration(self, intensity: float = 0.35) -> None:
        """Apply a brief R2 chatter-like vibration effect."""
        if self._controller is None:
            return
        i = clamp(intensity, 0.0, 1.0)
        if i <= 1e-3:
            self._set_trigger_off("right")
            return

        # dualsense-controller (property API)
        try:
            eff = self._trigger_effect("right")
            if eff is not None and hasattr(eff, "vibration"):
                amp = int(clamp(round(i * 8.0), 1, 8))
                eff.vibration(start_position=0, amplitude=amp, frequency=8)
                return
        except Exception:
            pass

        # Backward-compatible API paths
        try:
            if hasattr(self._controller, "set_right_trigger_vibration"):
                self._controller.set_right_trigger_vibration(int(255 * i))
            elif hasattr(self._controller, "set_rumble"):
                self._controller.set_rumble(int(180 * i), int(220 * i), 45)
            elif self._set_rumble_levels(0.65 * i, 0.85 * i):
                self._schedule_rumble_stop(60)
        except Exception:
            pass

    def set_trigger_vibration_left(self, intensity: float = 0.35) -> None:
        """Apply a brief L2 chatter-like vibration effect."""
        if self._controller is None:
            return
        i = clamp(intensity, 0.0, 1.0)
        if i <= 1e-3:
            self._set_trigger_off("left")
            return

        try:
            eff = self._trigger_effect("left")
            if eff is not None and hasattr(eff, "vibration"):
                amp = int(clamp(round(i * 8.0), 1, 8))
                eff.vibration(start_position=0, amplitude=amp, frequency=8)
                return
        except Exception:
            pass
