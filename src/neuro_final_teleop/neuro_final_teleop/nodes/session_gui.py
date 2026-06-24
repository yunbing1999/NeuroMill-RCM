import sys
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGridLayout, QFrame, QPushButton,
                             QTabWidget, QMessageBox, QGroupBox, QSizePolicy,
                             QCheckBox, QScrollArea)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QColor
import pyqtgraph as pg
import numpy as np
import subprocess
import datetime
import os
import signal
import time

# --- STYLE SETTINGS ---
BACKGROUND_COLOR = '#1e1e1e'
TEXT_COLOR = '#ffffff'
PLOT_BG_COLOR = '#121212'

# Colors: pastel tones for Forces, warm tones for Torques
COLORS = {
    'Fx': '#ff79c6',
    'Fy': '#bd93f9',
    'Fz': '#8be9fd',
    'Mx': '#ffb86c',
    'My': '#f1fa8c',
    'Mz': '#ff5555'
}

# --- LABELS ---
LABELS = {
    'Fx': "Force X (Lateral)",
    'Fy': "Force Y (Anterior)",
    'Fz': "Force Z (Axial Pressure)",
    'Mx': "Torque X (Roll)",
    'My': "Torque Y (Pitch)",
    'Mz': "Torque Z (Yaw)"
}

# Display-only sign mapping for "human-intuitive" interpretation.
# Raw ROS topic data remains untouched; this only affects GUI numbers/plots.
INTUITIVE_DISPLAY_SIGN = {
    'Fx': -1.0, 'Fy': -1.0, 'Fz': -1.0,
    'Mx': -1.0, 'My': -1.0, 'Mz': -1.0,
}

GUIDE_IMAGE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "ps5_controller_guide.jpg")
NOISE_CALIBRATION_DURATION_S = 10.0

# Name of the teleop node we toggle haptic-evaluation flags on. This must
# match the `name=` argument used in the launch file. Adjust here if the
# launch file ever renames the node.
TELEOP_NODE_NAME = "neuro_final_teleop"

# All per-channel haptic enable flags we mirror from the backend so the
# evaluation panel can toggle vibration vs adaptive trigger without
# touching any config file or the control logic itself.
HAPTIC_VIBRATION_FLAGS = (
    "v7_ft_haptic_rumble_enable",
    "v7_ft_haptic_contact_pulse_enable",
    "v7_ft_haptic_release_pulse_enable",
    "v7_ft_haptic_trigger_vibration_enable",
    "v7_motion_haptic_enable",
)
HAPTIC_TRIGGER_FLAGS = (
    "v7_ft_haptic_trigger_enable",
)
HAPTIC_ALL_FLAGS = HAPTIC_VIBRATION_FLAGS + HAPTIC_TRIGGER_FLAGS

HAPTIC_MODE_BOTH = "both"
HAPTIC_MODE_VIBRATION_ONLY = "vibration_only"
HAPTIC_MODE_TRIGGER_ONLY = "trigger_only"
HAPTIC_MODE_OFF = "off"


# ============================================================
# 1) ROS 2 NODE
# ============================================================
class RosNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.subscription = self.create_subscription(
            WrenchStamped, '/xarm/ft_data', self.listener_callback, 10)
        self.debug_subscription = self.create_subscription(
            String, '/neuro_final/ft_haptic_debug', self.debug_callback, 10)

        self.tare_client = self.create_client(Trigger, '/xarm/tare_sensor')

        # Service clients that toggle the existing per-channel haptic
        # enables on the teleop node. We use the standard rclpy parameter
        # services so no new topics/services have to be added on the
        # backend side.
        self.set_param_client = self.create_client(
            SetParameters, f'/{TELEOP_NODE_NAME}/set_parameters'
        )
        self.get_param_client = self.create_client(
            GetParameters, f'/{TELEOP_NODE_NAME}/get_parameters'
        )
        # Filled in asynchronously the first time the get_parameters
        # service answers. Until then BOTH falls back to True for the
        # known-enabled channels, which matches the default config.
        self.haptic_baseline = None

        self.current_raw_data = {
            'Fx': 0.0, 'Fy': 0.0, 'Fz': 0.0,
            'Mx': 0.0, 'My': 0.0, 'Mz': 0.0,
            't': 0.0
        }
        self.current_backend_debug = None

    def listener_callback(self, msg):
        self.current_raw_data['Fx'] = msg.wrench.force.x
        self.current_raw_data['Fy'] = msg.wrench.force.y
        self.current_raw_data['Fz'] = msg.wrench.force.z
        self.current_raw_data['Mx'] = msg.wrench.torque.x
        self.current_raw_data['My'] = msg.wrench.torque.y
        self.current_raw_data['Mz'] = msg.wrench.torque.z

        # Use the message timestamp from bridge.py
        self.current_raw_data['t'] = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9
        )

    def debug_callback(self, msg):
        try:
            data = json.loads(msg.data)
            data['t'] = float(data.get('stamp', 0.0))
            self.current_backend_debug = data
        except Exception:
            pass

    def snapshot(self):
        backend = self.current_backend_debug
        if backend is not None:
            def field_dict(name):
                value = backend.get(name, {})
                return dict(value) if isinstance(value, dict) else {}

            backend = {
                **backend,
                'raw': field_dict('raw'),
                'filtered': field_dict('filtered'),
                'raw_sensor': field_dict('raw_sensor'),
                'filtered_sensor': field_dict('filtered_sensor'),
                'sensor_analysis': field_dict('sensor_analysis'),
                'haptic_output': field_dict('haptic_output'),
            }
        return {
            'raw': dict(self.current_raw_data),
            'backend': backend,
        }

    def send_tare_request(self):
        if not self.tare_client.wait_for_service(timeout_sec=1.0):
            return False
        req = Trigger.Request()
        self.tare_client.call_async(req)
        return True

    def request_haptic_baseline(self):
        """Ask the teleop node what the configured haptic enables are.

        The result is stored asynchronously on ``self.haptic_baseline`` so
        the BOTH evaluation mode can restore the exact values the user
        booted with, no matter which yaml config they loaded.
        """
        if not self.get_param_client.service_is_ready():
            return False
        req = GetParameters.Request()
        req.names = list(HAPTIC_ALL_FLAGS)
        future = self.get_param_client.call_async(req)

        def _done(fut):
            try:
                result = fut.result()
            except Exception:
                return
            if result is None or not getattr(result, "values", None):
                return
            baseline = {}
            for name, value in zip(HAPTIC_ALL_FLAGS, result.values):
                if value.type == ParameterType.PARAMETER_BOOL:
                    baseline[name] = bool(value.bool_value)
                else:
                    baseline[name] = False
            self.haptic_baseline = baseline

        future.add_done_callback(_done)
        return True

    def send_haptic_param_update(self, updates):
        """Push a {param_name: bool} dict to the teleop node.

        Returns True if the call was issued (the actual ack arrives
        asynchronously via the rclpy executor in the GUI worker thread).
        """
        if not updates:
            return False
        if not self.set_param_client.service_is_ready():
            return False
        req = SetParameters.Request()
        for name, value in updates.items():
            param = Parameter()
            param.name = str(name)
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = bool(value)
            req.parameters.append(param)
        self.set_param_client.call_async(req)
        return True


# ============================================================
# 2) BACKGROUND WORKER THREAD
# ============================================================
class RosThread(QThread):
    """Spins ROS 2 in the background so the GUI never freezes."""
    new_data_signal = pyqtSignal(dict)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self._is_running = True

    def run(self):
        while rclpy.ok() and self._is_running:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.01)
                # Emit a copy so GUI thread gets a stable snapshot
                self.new_data_signal.emit(self.ros_node.snapshot())
            except Exception:
                pass

    def stop(self):
        self._is_running = False
        self.quit()
        self.wait()


# ============================================================
# 3) MAIN WINDOW GUI
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.display_intuitive = True
        self.show_backend_lpf = False
        self._last_raw_sample_t = None
        self._last_backend_sample_t = None
        self.recording_process = None
        self.recording_root = os.environ.get(
            "NEURO_FINAL_RECORDING_ROOT",
            os.path.expanduser("~/NeuroFinal/recordings"),
        )
        os.makedirs(self.recording_root, exist_ok=True)
        self._log_file = None
        self.is_recording = False
        self.noise_calibration_samples = []
        self.noise_calibration_active = False
        self.noise_calibration_start_s = 0.0
        self.noise_calibration_started_iso = None
        self.noise_stats = None
        self._last_noise_calibration_sample_t = None
        self._last_contact_debug_sample_t = None
        self.contact_debug_time_buffer = []
        self.contact_debug_force_buffer = []
        self.contact_debug_ratio_buffer = []
        self.contact_debug_contact_buffer = []
        self._contact_debug_ratio_scale_n = 1.0

        self.setWindowTitle("Neuro-Surgical Robotics Interface")
        self.resize(1400, 900)
        self.setMinimumSize(1260, 820)
        self.setStyleSheet(f"background-color: {BACKGROUND_COLOR}; color: {TEXT_COLOR};")

        # Main Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #444; }
            QTabBar::tab { background: #333; color: #aaa; padding: 10px; min-width: 150px; }
            QTabBar::tab:selected { background: #555; color: white; font-weight: bold; }
        """)
        self.setCentralWidget(self.tabs)

        # Tab 1: Monitor
        self.monitor_tab = QWidget()
        self.setup_monitor_ui()
        self.tabs.addTab(self.monitor_tab, "LIVE SURGERY MONITOR")

        # Tab 2: Xbox Guide
        self.guide_tab = QWidget()
        self.setup_guide_ui()
        self.tabs.addTab(self.guide_tab, "CONTROLLER GUIDE")

        # --- MULTITHREADING SETUP ---
        self.ros_thread = RosThread(self.ros_node)
        self.ros_thread.new_data_signal.connect(self.update_gui)
        self.ros_thread.start()

    def setup_monitor_ui(self):
        layout = QHBoxLayout(self.monitor_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # --- LEFT PANEL: NUMBERS & BUTTONS ---
        left_panel = QFrame()
        left_panel.setFixedWidth(460)
        left_panel.setStyleSheet("border-right: 2px solid #333; padding-right: 12px;")
        left_outer_layout = QVBoxLayout(left_panel)
        left_outer_layout.setContentsMargins(0, 0, 0, 0)
        left_outer_layout.setSpacing(0)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollArea > QWidget > QWidget {
                background-color: transparent;
            }
            QScrollBar:vertical {
                background: #252526;
                width: 10px;
                margin: 0;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #555;
                min-height: 32px;
                border-radius: 5px;
            }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {
                height: 0;
            }
        """)

        left_content = QWidget()
        left_content.setStyleSheet("background-color: transparent; border: none;")
        left_layout = QVBoxLayout(left_content)
        left_layout.setContentsMargins(6, 6, 6, 6)
        left_layout.setSpacing(8)

        title = QLabel("REAL-TIME SENSOR")
        title.setFont(QFont("Arial", 17, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #ecf0f1; margin-bottom: 10px;")
        left_layout.addWidget(title)

        self.value_labels = {}
        keys = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        units = ['N', 'N', 'N', 'Nm', 'Nm', 'Nm']

        for i, key in enumerate(keys):
            box = QFrame()
            box.setMinimumHeight(92)
            box.setStyleSheet(f"""
                background-color: #252526; 
                border-left: 5px solid {COLORS[key]}; 
                border-radius: 4px; 
                margin-bottom: 5px;
            """)
            box_layout = QVBoxLayout(box)
            box_layout.setContentsMargins(10, 6, 10, 6)
            box_layout.setSpacing(2)

            lbl_title = QLabel(LABELS[key])
            lbl_title.setWordWrap(True)
            lbl_title.setFont(QFont("Arial", 10, QFont.Bold))
            lbl_title.setStyleSheet("color: #bdc3c7; border: none;")

            val_layout = QHBoxLayout()
            val_layout.setContentsMargins(0, 0, 0, 0)
            val_layout.setSpacing(6)

            lbl_val = QLabel("0.00")
            font_size = 24
            lbl_val.setFont(QFont("Consolas", font_size, QFont.Bold))
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl_val.setStyleSheet("color: white; border: none;")
            lbl_val.setMinimumWidth(160)

            lbl_unit = QLabel(units[i])
            lbl_unit.setFont(QFont("Arial", 13))
            lbl_unit.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
            lbl_unit.setStyleSheet("color: #7f8c8d; border: none; padding-bottom: 5px;")
            lbl_unit.setMinimumWidth(36)

            val_layout.addStretch()
            val_layout.addWidget(lbl_val)
            val_layout.addWidget(lbl_unit)

            self.value_labels[key] = lbl_val

            box_layout.addWidget(lbl_title)
            box_layout.addLayout(val_layout)
            left_layout.addWidget(box)

        # --- ACTIONS SECTION ---
        action_group = QGroupBox("SURGICAL CONTROLS")
        action_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #555;
                margin-top: 10px;
                font-weight: bold;
                padding-top: 14px;
                border-radius: 4px;
            }
        """)
        action_layout = QVBoxLayout(action_group)
        action_layout.setContentsMargins(10, 10, 10, 10)
        action_layout.setSpacing(8)

        self.btn_tare = QPushButton("RESET SENSOR (TARE)")
        self.btn_tare.setFixedHeight(48)
        self.btn_tare.setStyleSheet("""
            QPushButton { background-color: #d35400; color: white; font-weight: bold; border-radius: 5px; font-size: 12px; padding: 4px 8px; }
            QPushButton:hover { background-color: #e67e22; }
        """)
        self.btn_tare.clicked.connect(self.handle_tare)
        action_layout.addWidget(self.btn_tare)

        self.btn_display_mode = QPushButton("DISPLAY: INTUITIVE (SIGN-CORRECTED)")
        self.btn_display_mode.setFixedHeight(44)
        self.btn_display_mode.setStyleSheet("""
            QPushButton { background-color: #2c3e50; color: white; font-weight: bold; border-radius: 5px; font-size: 11px; padding: 4px 8px; }
            QPushButton:hover { background-color: #34495e; }
        """)
        self.btn_display_mode.clicked.connect(self.toggle_display_mode)
        action_layout.addWidget(self.btn_display_mode)

        self.chk_backend_lpf = QCheckBox("Show backend LPF values")
        self.chk_backend_lpf.setStyleSheet("""
            QCheckBox { color: #ecf0f1; font-weight: bold; padding: 6px; }
            QCheckBox::indicator { width: 18px; height: 18px; }
        """)
        self.chk_backend_lpf.stateChanged.connect(self.toggle_backend_lpf_view)
        action_layout.addWidget(self.chk_backend_lpf)

        self.lbl_view_status = QLabel("View: RAW FT SENSOR")
        self.lbl_view_status.setAlignment(Qt.AlignCenter)
        self.lbl_view_status.setWordWrap(True)
        self.lbl_view_status.setStyleSheet("color: #8be9fd; font-size: 12px; font-weight: bold;")
        action_layout.addWidget(self.lbl_view_status)

        self.btn_record = QPushButton("START RECORDING")
        self.btn_record.setFixedHeight(48)
        self.btn_record.setStyleSheet("""
            QPushButton { background-color: #c0392b; color: white; font-weight: bold; border-radius: 5px; font-size: 12px; padding: 4px 8px; }
            QPushButton:hover { background-color: #e74c3c; }
        """)
        self.btn_record.clicked.connect(self.toggle_recording)
        action_layout.addWidget(self.btn_record)

        self.lbl_status = QLabel("System Status: READY")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet("color: #7f8c8d; font-size: 12px; margin-top: 5px;")
        action_layout.addWidget(self.lbl_status)

        left_layout.addWidget(action_group)

        eval_group = QGroupBox("HAPTIC EVALUATION MODE")
        eval_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #555;
                margin-top: 8px;
                font-weight: bold;
                padding-top: 14px;
                border-radius: 4px;
            }
        """)
        eval_layout = QGridLayout(eval_group)
        eval_layout.setContentsMargins(10, 10, 10, 10)
        eval_layout.setHorizontalSpacing(6)
        eval_layout.setVerticalSpacing(6)

        # 4-way mode selector. The buttons only push ROS parameter updates
        # to the existing per-channel haptic enables on neuro_final_teleop; no
        # FT, contact, or motion logic is altered.
        self.haptic_mode_buttons = {}
        mode_specs = [
            (HAPTIC_MODE_BOTH, "BOTH"),
            (HAPTIC_MODE_VIBRATION_ONLY, "VIBRATION ONLY"),
            (HAPTIC_MODE_TRIGGER_ONLY, "ADAPTIVE TRIGGER ONLY"),
            (HAPTIC_MODE_OFF, "OFF"),
        ]
        for idx, (mode_id, label_text) in enumerate(mode_specs):
            btn = QPushButton(label_text)
            btn.setFixedHeight(34)
            btn.setCheckable(True)
            btn.setStyleSheet(self._haptic_mode_button_style(False))
            btn.clicked.connect(lambda _checked=False, m=mode_id: self.set_haptic_mode(m))
            row = idx // 2
            col = idx % 2
            eval_layout.addWidget(btn, row, col)
            self.haptic_mode_buttons[mode_id] = btn

        self.lbl_haptic_mode_status = QLabel("Mode: BOTH (waiting for neuro_final_teleop...)")
        self.lbl_haptic_mode_status.setWordWrap(True)
        self.lbl_haptic_mode_status.setAlignment(Qt.AlignCenter)
        self.lbl_haptic_mode_status.setStyleSheet(
            "color: #8be9fd; border: none; font-weight: bold; font-size: 11px;"
        )
        eval_layout.addWidget(self.lbl_haptic_mode_status, 2, 0, 1, 2)

        left_layout.addWidget(eval_group)
        self.current_haptic_mode = HAPTIC_MODE_BOTH
        self._haptic_baseline_logged = False
        self._refresh_haptic_mode_buttons()

        # Periodically try to grab the backend's baseline values and keep
        # the mode label fresh. This is a slow timer (every 1s) so it
        # never competes with the live FT plotting.
        self._haptic_mode_timer = QTimer(self)
        self._haptic_mode_timer.setInterval(1000)
        self._haptic_mode_timer.timeout.connect(self._haptic_mode_tick)
        self._haptic_mode_timer.start()

        backend_group = QGroupBox("BACKEND FT / HAPTIC STATUS")
        backend_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #555;
                margin-top: 8px;
                font-weight: bold;
                padding-top: 14px;
                border-radius: 4px;
            }
        """)
        backend_layout = QGridLayout(backend_group)
        backend_layout.setContentsMargins(12, 14, 12, 12)
        backend_layout.setHorizontalSpacing(14)
        backend_layout.setVerticalSpacing(7)
        backend_layout.setColumnMinimumWidth(0, 205)
        backend_layout.setColumnMinimumWidth(1, 170)
        backend_layout.setColumnStretch(0, 1)
        backend_layout.setColumnStretch(1, 1)
        self.backend_info_labels = {}
        backend_fields = [
            (None, "MEASUREMENT"),
            ("view_mode", "View mode"),
            ("debug_topic", "FT debug topic"),
            ("force_norm_filtered", "Filtered norm"),
            ("force_norm_raw", "Raw norm"),
            ("selected_force_value", "Selected before deadband"),
            ("selected_force_after_deadband", "Selected after deadband"),
            ("baseline_force_n", "Baseline force"),
            ("shift_from_baseline_n", "Shift from baseline"),
            ("noise_band_n", "Noise band (ON)"),
            ("lpf_alpha", "LPF alpha"),
            ("ft_fresh", "FT fresh"),
            (None, "HAPTIC DECISION"),
            ("contact_active", "Contact active"),
            ("contact_candidate", "Contact candidate"),
            ("actual_sent_ratio", "Actual sent ratio"),
            ("target_haptic_ratio", "Target haptic ratio"),
            ("smoothed_haptic_ratio", "Smoothed haptic ratio"),
            ("hard_block_active", "Hard block"),
            ("deadman_active", "Deadman active"),
            ("depth_input_active", "Depth input active"),
            ("active_trigger_side", "Active trigger"),
            ("motion_active", "Motion active"),
            ("haptic_block_reason", "Haptic block reason"),
        ]
        backend_row = 0
        for key, label_text in backend_fields:
            if key is None:
                section = QLabel(label_text)
                section.setMinimumHeight(28)
                section.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                section.setStyleSheet(
                    "color: #8be9fd; border: none; font-weight: bold; "
                    "font-size: 12px; margin-top: 6px; padding-top: 4px;"
                )
                backend_layout.addWidget(section, backend_row, 0, 1, 2)
                backend_layout.setRowMinimumHeight(backend_row, 28)
                backend_row += 1
                continue
            lbl = QLabel(f"{label_text}:")
            lbl.setMinimumHeight(24)
            lbl.setWordWrap(True)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl.setStyleSheet("color: #bdc3c7; border: none; font-size: 11px;")
            val = QLabel("--")
            val.setMinimumHeight(24)
            val.setMinimumWidth(150)
            val.setTextInteractionFlags(Qt.TextSelectableByMouse)
            val.setStyleSheet(
                "color: #ecf0f1; border: none; font-family: Consolas; "
                "font-size: 11px; padding-left: 4px;"
            )
            if key == "haptic_block_reason":
                val.setWordWrap(True)
                val.setMinimumHeight(42)
                val.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                backend_layout.addWidget(lbl, backend_row, 0, 1, 2)
                backend_layout.setRowMinimumHeight(backend_row, 24)
                backend_row += 1
                backend_layout.addWidget(val, backend_row, 0, 1, 2)
                backend_layout.setRowMinimumHeight(backend_row, 42)
                backend_row += 1
            else:
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                backend_layout.addWidget(lbl, backend_row, 0)
                backend_layout.addWidget(val, backend_row, 1)
                backend_layout.setRowMinimumHeight(backend_row, 26)
                backend_row += 1
            self.backend_info_labels[key] = val
        left_layout.addWidget(backend_group)
        left_layout.addStretch()
        left_scroll.setWidget(left_content)
        left_outer_layout.addWidget(left_scroll)
        layout.addWidget(left_panel)

        # --- RIGHT PANEL: PLOTS ---
        right_panel = QWidget()
        grid_layout = QGridLayout(right_panel)
        grid_layout.setContentsMargins(4, 4, 4, 4)
        grid_layout.setHorizontalSpacing(8)
        grid_layout.setVerticalSpacing(8)

        self.plots = {}
        self.curves = {}
        self.base_plot_titles = {}

        # Keep same structure, but now hold time + raw data
        self.time_window_sec = 5.0  # show only the last 5 seconds
        self.max_samples = 2000  # enough for high-rate raw data

        self.data_buffers = {key: [] for key in keys}
        self.time_buffers = {key: [] for key in keys}

        positions = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)]

        for i, key in enumerate(keys):
            full_title = f"{LABELS[key]} [{units[i]}]"
            self.base_plot_titles[key] = full_title
            plot = pg.PlotWidget(title=full_title)
            plot.setBackground(PLOT_BG_COLOR)
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel('bottom', 'Time', units='s')
            plot.setLabel('left', units[i])
            plot.getPlotItem().titleLabel.setText(full_title, color="#eaeaea", size="10pt")

            # Force ranges requested by user.
            if key in ['Fx', 'Fy', 'Fz']:
                plot.setYRange(-50, 50)
            else:
                plot.setYRange(-2, 2)

            pen = pg.mkPen(color=COLORS[key], width=2)
            self.curves[key] = plot.plot(self.time_buffers[key], self.data_buffers[key], pen=pen)
            self.plots[key] = plot

            row, col = positions[i]
            grid_layout.addWidget(plot, row, col)

        self.contact_debug_plot = pg.PlotWidget(title="CONTACT DEBUG TIMELINE")
        self.contact_debug_plot.setBackground(PLOT_BG_COLOR)
        self.contact_debug_plot.showGrid(x=True, y=True, alpha=0.3)
        self.contact_debug_plot.setLabel('bottom', 'Time', units='s')
        self.contact_debug_plot.setLabel('left', 'Force / scaled haptic')
        self.contact_debug_plot.getPlotItem().titleLabel.setText(
            "CONTACT DEBUG TIMELINE", color="#eaeaea", size="10pt"
        )
        self.contact_debug_plot.addLegend(offset=(10, 10))
        self.contact_force_curve = self.contact_debug_plot.plot(
            [], [], pen=pg.mkPen(color="#ffffff", width=2),
            name="selected filtered force [N]"
        )
        self.contact_haptic_curve = self.contact_debug_plot.plot(
            [], [], pen=pg.mkPen(color="#ffb86c", width=2),
            name="actual haptic ratio x scale"
        )
        self.contact_active_curve = self.contact_debug_plot.plot(
            [], [], pen=pg.mkPen(color="#50fa7b", width=2, style=Qt.DashLine),
            name="contact active marker"
        )
        self.noise_on_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color="#ff5555", width=2, style=Qt.DashLine)
        )
        self.noise_off_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color="#f1fa8c", width=2, style=Qt.DashLine)
        )
        self.noise_on_line.setVisible(False)
        self.noise_off_line.setVisible(False)
        self.contact_debug_plot.addItem(self.noise_on_line)
        self.contact_debug_plot.addItem(self.noise_off_line)
        # Backend baseline + adaptive noise band: drawn live from the debug
        # topic so what the backend sees is always visible next to what the
        # GUI's offline calibration suggested.
        self.backend_baseline_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color="#8be9fd", width=2, style=Qt.DotLine)
        )
        self.backend_shift_on_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen(color="#ff79c6", width=2, style=Qt.DotLine)
        )
        self.backend_baseline_line.setVisible(False)
        self.backend_shift_on_line.setVisible(False)
        self.contact_debug_plot.addItem(self.backend_baseline_line)
        self.contact_debug_plot.addItem(self.backend_shift_on_line)
        grid_layout.addWidget(self.contact_debug_plot, 2, 0, 1, 3)

        noise_group = QGroupBox("FREE-SPACE NOISE BAND")
        noise_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #555;
                margin-top: 8px;
                font-weight: bold;
                padding-top: 14px;
                border-radius: 4px;
            }
        """)
        noise_layout = QGridLayout(noise_group)
        noise_layout.setContentsMargins(10, 10, 10, 10)
        noise_layout.setHorizontalSpacing(14)
        noise_layout.setVerticalSpacing(4)

        self.btn_noise_calibrate = QPushButton("START NOISE CALIBRATION")
        self.btn_noise_calibrate.setFixedHeight(36)
        self.btn_noise_calibrate.setStyleSheet("""
            QPushButton { background-color: #2c3e50; color: white; font-weight: bold; border-radius: 5px; padding: 4px 8px; }
            QPushButton:hover { background-color: #34495e; }
        """)
        self.btn_noise_calibrate.clicked.connect(self.start_noise_calibration)
        noise_layout.addWidget(self.btn_noise_calibrate, 0, 0, 1, 2)

        self.btn_noise_reset = QPushButton("RESET NOISE CALIBRATION")
        self.btn_noise_reset.setFixedHeight(36)
        self.btn_noise_reset.setStyleSheet("""
            QPushButton { background-color: #7f8c8d; color: white; font-weight: bold; border-radius: 5px; padding: 4px 8px; }
            QPushButton:hover { background-color: #95a5a6; }
        """)
        self.btn_noise_reset.clicked.connect(self.reset_noise_calibration)
        noise_layout.addWidget(self.btn_noise_reset, 0, 2, 1, 2)

        self.noise_info_labels = {}
        noise_fields = [
            ("status", "Status"),
            ("mean", "Noise mean"),
            ("std", "Noise std"),
            ("minmax", "Noise min/max"),
            ("peak", "Noise peak"),
            ("suggest_on", "Suggested contact ON"),
            ("suggest_off", "Suggested contact OFF"),
            ("inside", "Force inside noise band"),
            ("above_on", "Above suggested contact ON"),
            ("backend_contact", "Contact active from backend"),
            ("actual_haptic", "Actual haptic output"),
        ]
        for idx, (key, label_text) in enumerate(noise_fields):
            row = 1 + (idx // 2)
            col = (idx % 2) * 2
            lbl = QLabel(f"{label_text}:")
            lbl.setStyleSheet("color: #bdc3c7; border: none;")
            val = QLabel("--")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setStyleSheet("color: #ecf0f1; border: none; font-family: Consolas;")
            noise_layout.addWidget(lbl, row, col)
            noise_layout.addWidget(val, row, col + 1)
            self.noise_info_labels[key] = val
        self._set_noise_status("Idle")
        grid_layout.addWidget(noise_group, 3, 0, 1, 3)

        grid_layout.setRowStretch(0, 1)
        grid_layout.setRowStretch(1, 1)
        grid_layout.setRowStretch(2, 1)
        grid_layout.setRowStretch(3, 0)

        self._refresh_plot_titles()
        layout.addWidget(right_panel)

    def setup_guide_ui(self):
        layout = QVBoxLayout(self.guide_tab)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_label)
        self.load_guide_image()

    def load_guide_image(self):
        if os.path.exists(GUIDE_IMAGE_PATH):
            pixmap = QPixmap(GUIDE_IMAGE_PATH)
            w = self.guide_tab.width() + 200
            h = self.guide_tab.height() + 200
            scaled_pixmap = pixmap.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pixmap)
        else:
            self.image_label.setText(
                f"IMAGE NOT FOUND!\nPlease save the image as:\n'{GUIDE_IMAGE_PATH}'\nin the same folder as this script.")
            self.image_label.setFont(QFont("Arial", 14, QFont.Bold))
            self.image_label.setStyleSheet("color: #e74c3c;")

    def resizeEvent(self, event):
        if self.tabs.currentWidget() == self.guide_tab:
            self.load_guide_image()
        super().resizeEvent(event)

    def handle_tare(self):
        success = self.ros_node.send_tare_request()
        if success:
            self.lbl_status.setText("Status: SENSOR RESET (TARE) OK")
            self.lbl_status.setStyleSheet("color: #2ecc71; font-weight: bold;")
            QTimer.singleShot(3000, lambda: self.lbl_status.setText("System Status: READY"))
        else:
            self.lbl_status.setText("Status: TARE FAILED!")
            self.lbl_status.setStyleSheet("color: red; font-weight: bold;")

    def toggle_recording(self):
        if not self.is_recording:
            timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            session_name = f"surgery_{timestamp}"

            session_dir = os.path.join(self.recording_root, session_name)
            bag_dir = os.path.join(session_dir, "bag")
            os.makedirs(session_dir, exist_ok=True)

            log_path = os.path.join(session_dir, "rosbag.log")
            self._log_file = open(log_path, "w", buffering=1)

            cmd = [
                "ros2", "bag", "record",
                "-o", bag_dir,
                "/xarm/ft_data",
                "/neuro_final/ft_haptic_debug",
                "/joint_states",
                "/tf",
                "/tf_static",
                "/zed/zed_node/rgb/color/rect/image",
                "/zed/zed_node/rgb/color/rect/camera_info"
            ]

            try:
                self.recording_process = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid,
                    stdout=self._log_file,
                    stderr=subprocess.STDOUT
                )
                self.is_recording = True
                self._current_session_dir = session_dir

                self.btn_record.setText("STOP RECORDING")
                self.btn_record.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
                self.lbl_status.setText(f"Status: RECORDING... Session: {session_name}")
                self.lbl_status.setStyleSheet("color: #f1c40f; font-weight: bold;")

            except Exception as e:
                self.lbl_status.setText(f"Status: REC FAILED: {e}")
                self.lbl_status.setStyleSheet("color: red;")
                try:
                    self._log_file.close()
                except Exception:
                    pass
                self._log_file = None
        else:
            if self.recording_process:
                try:
                    os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
                    self.recording_process.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if self.recording_process:
                        self.recording_process.kill()
                self.recording_process = None

                if self._log_file:
                    try:
                        self._log_file.close()
                    except Exception:
                        pass
                    self._log_file = None

            self.is_recording = False
            self.btn_record.setText("START RECORDING")
            self.btn_record.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
            saved_path = getattr(self, "_current_session_dir", "recordings/")
            self.lbl_status.setText(f"Status: DATA SAVED.  ({saved_path})")
            self.lbl_status.setStyleSheet("color: #ecf0f1;")

    def _display_value(self, key: str, raw_value: float) -> float:
        if self.display_intuitive:
            return float(INTUITIVE_DISPLAY_SIGN.get(key, 1.0)) * float(raw_value)
        return float(raw_value)

    def toggle_display_mode(self):
        self.display_intuitive = not self.display_intuitive
        if self.display_intuitive:
            self.btn_display_mode.setText("DISPLAY: INTUITIVE (SIGN-CORRECTED)")
            self.lbl_status.setText("Status: DISPLAY = INTUITIVE (raw stream unchanged)")
        else:
            self.btn_display_mode.setText("DISPLAY: RAW SENSOR FRAME")
            self.lbl_status.setText("Status: DISPLAY = RAW SENSOR FRAME")
        self.lbl_status.setStyleSheet("color: #ecf0f1; font-weight: bold;")

    def toggle_backend_lpf_view(self, state):
        self.show_backend_lpf = state == Qt.Checked
        self._clear_plot_buffers()
        if self.show_backend_lpf:
            self.lbl_view_status.setText("View: BACKEND FILTERED SENSOR")
        else:
            self.lbl_view_status.setText("View: RAW FT SENSOR")
        self._refresh_plot_titles()
        self._update_backend_info(None)

    def _clear_plot_buffers(self):
        for key in self.data_buffers:
            self.data_buffers[key].clear()
            self.time_buffers[key].clear()
            self.curves[key].setData([], [])
        self._last_raw_sample_t = None
        self._last_backend_sample_t = None

    def _refresh_plot_titles(self):
        mode = "BACKEND FILTERED SENSOR" if self.show_backend_lpf else "RAW FT SENSOR"
        for key, plot in self.plots.items():
            title = f"{self.base_plot_titles[key]} - {mode}"
            plot.getPlotItem().titleLabel.setText(title, color="#eaeaea", size="10pt")

    def _backend_filtered_values(self, backend):
        if backend is None:
            return {}, False
        filtered_sensor = backend.get('filtered_sensor')
        if isinstance(filtered_sensor, dict) and filtered_sensor:
            return filtered_sensor, False
        old_filtered = backend.get('filtered')
        if isinstance(old_filtered, dict) and old_filtered:
            return old_filtered, True
        return {}, False

    def _backend_measurement(self, backend, key, default=None):
        if backend is None:
            return default
        sensor = backend.get('sensor_analysis', {})
        haptic = backend.get('haptic_output', {})
        if key in sensor:
            return sensor.get(key, default)
        if key in haptic:
            return haptic.get(key, default)
        legacy_key = {
            "filtered_force_norm_n": "force_norm_filtered",
            "raw_force_norm_n": "force_norm_raw",
        }.get(key)
        if legacy_key is not None and legacy_key in backend:
            return backend.get(legacy_key, default)
        return backend.get(key, default)

    def _format_bool(self, value, true_text="YES", false_text="NO"):
        if value is None:
            return "--"
        return true_text if bool(value) else false_text

    def _format_backend_value(self, key, backend, using_old_filtered=False):
        view_mode = "BACKEND FILTERED SENSOR" if self.show_backend_lpf else "RAW"
        if key == "view_mode":
            return view_mode
        if key == "debug_topic":
            if backend is None:
                return "waiting"
            return "OK (old filtered)" if using_old_filtered else "OK"
        if backend is None:
            return "--"

        if key == "force_norm_filtered":
            value = self._backend_measurement(backend, "filtered_force_norm_n")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "force_norm_raw":
            value = self._backend_measurement(backend, "raw_force_norm_n")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "selected_force_value":
            value = self._backend_measurement(backend, "selected_force_value")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "selected_force_after_deadband":
            selected = self._backend_measurement(backend, "selected_force_value")
            deadband = self._backend_measurement(backend, "deadband_n", 0.0)
            if selected is None:
                return "--"
            return f"{max(float(selected) - float(deadband or 0.0), 0.0):.3f} N"
        if key == "baseline_force_n":
            value = self._backend_measurement(backend, "baseline_force_n")
            if value is None:
                value = self._backend_measurement(backend, "contact_baseline_n")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "shift_from_baseline_n":
            value = self._backend_measurement(backend, "shift_from_baseline_n")
            if value is None:
                value = self._backend_measurement(backend, "contact_shift_delta_n")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "noise_band_n":
            value = self._backend_measurement(backend, "noise_band_n")
            if value is None:
                value = self._backend_measurement(backend, "contact_shift_threshold_n")
            return f"{float(value):.3f} N" if value is not None else "--"
        if key == "lpf_alpha":
            value = self._backend_measurement(backend, "lpf_alpha")
            return f"{float(value):.3f}" if value is not None else "--"
        if key == "ft_fresh":
            return self._format_bool(self._backend_measurement(backend, "ft_fresh"))

        if key in ("contact_active", "hard_block_active"):
            return self._format_bool(self._backend_measurement(backend, key), "ON", "OFF")
        if key == "contact_candidate":
            value = self._backend_measurement(backend, key)
            if value is None:
                value = self._backend_measurement(backend, "contact_candidate_active")
            return self._format_bool(value, "ON", "OFF")
        if key in ("actual_sent_ratio", "target_haptic_ratio", "smoothed_haptic_ratio"):
            value = self._backend_measurement(backend, key)
            return f"{float(value):.3f}" if value is not None else "--"
        if key in ("deadman_active", "depth_input_active", "motion_active"):
            return self._format_bool(self._backend_measurement(backend, key))
        if key == "active_trigger_side":
            return str(self._backend_measurement(backend, key, "none"))
        if key == "haptic_block_reason":
            value = self._backend_measurement(backend, "haptic_block_reason")
            if value is None:
                value = self._backend_measurement(backend, "reason")
            return str(value) if value is not None else "--"
        return "--"

    def _update_backend_info(self, backend, using_old_filtered=False):
        for key, label in self.backend_info_labels.items():
            label.setText(self._format_backend_value(key, backend, using_old_filtered))

    def _set_noise_status(self, text):
        label = getattr(self, "noise_info_labels", {}).get("status")
        if label is not None:
            label.setText(text)

    def _set_noise_label(self, key, text):
        label = getattr(self, "noise_info_labels", {}).get(key)
        if label is not None:
            label.setText(text)

    def _backend_force_sample(self, backend):
        if backend is None:
            return None
        filtered, _ = self._backend_filtered_values(backend)
        if not filtered:
            return None

        force_value = self._backend_measurement(backend, "selected_force_before_deadband_n")
        if force_value is None:
            force_value = self._backend_measurement(backend, "filtered_force_norm_n")
        if force_value is None:
            try:
                fx = float(filtered.get("Fx", 0.0))
                fy = float(filtered.get("Fy", 0.0))
                fz = float(filtered.get("Fz", 0.0))
                force_value = float(np.sqrt((fx * fx) + (fy * fy) + (fz * fz)))
            except Exception:
                return None

        sample = {
            "timestamp": float(backend.get("stamp", backend.get("t", 0.0))),
            "force_n": float(force_value),
        }
        for key in ("Fx", "Fy", "Fz", "Mx", "My", "Mz"):
            sample[key] = float(filtered.get(key, 0.0))
        return sample

    def start_noise_calibration(self):
        self.noise_calibration_samples = []
        self.noise_stats = None
        self.noise_calibration_active = True
        self.noise_calibration_start_s = time.monotonic()
        self.noise_calibration_started_iso = datetime.datetime.now().isoformat()
        self._last_noise_calibration_sample_t = None
        self.noise_on_line.setVisible(False)
        self.noise_off_line.setVisible(False)
        self._set_noise_status(f"Collecting {NOISE_CALIBRATION_DURATION_S:.1f}s")
        for key in ("mean", "std", "minmax", "peak", "suggest_on", "suggest_off"):
            self._set_noise_label(key, "--")

    def reset_noise_calibration(self):
        self.noise_calibration_samples = []
        self.noise_stats = None
        self.noise_calibration_active = False
        self.noise_calibration_start_s = 0.0
        self.noise_calibration_started_iso = None
        self._last_noise_calibration_sample_t = None
        self.noise_on_line.setVisible(False)
        self.noise_off_line.setVisible(False)
        self._set_noise_status("Idle")
        for key in (
            "mean", "std", "minmax", "peak", "suggest_on", "suggest_off",
            "inside", "above_on", "backend_contact", "actual_haptic",
        ):
            self._set_noise_label(key, "--")
        self._refresh_contact_debug_plot()

    def _process_noise_calibration(self, sample):
        if not self.noise_calibration_active:
            return

        elapsed = time.monotonic() - self.noise_calibration_start_s
        remaining = max(NOISE_CALIBRATION_DURATION_S - elapsed, 0.0)

        if sample is not None:
            sample_t = sample["timestamp"]
            if self._last_noise_calibration_sample_t != sample_t:
                self.noise_calibration_samples.append(dict(sample))
                self._last_noise_calibration_sample_t = sample_t

        self._set_noise_status(
            f"Collecting {remaining:.1f}s | samples={len(self.noise_calibration_samples)}"
        )

        if elapsed >= NOISE_CALIBRATION_DURATION_S:
            self.noise_calibration_active = False
            self._finish_noise_calibration()

    def _finish_noise_calibration(self):
        if not self.noise_calibration_samples:
            self.noise_stats = None
            self._set_noise_status("No backend samples")
            return

        forces = np.array(
            [float(sample["force_n"]) for sample in self.noise_calibration_samples],
            dtype=float,
        )
        noise_mean_n = float(np.mean(forces))
        noise_std_n = float(np.std(forces))
        noise_min_n = float(np.min(forces))
        noise_max_n = float(np.max(forces))
        noise_peak_abs_n = float(np.max(np.abs(forces)))
        suggested_contact_on_n = max(
            noise_mean_n + (3.0 * noise_std_n),
            noise_max_n + 0.2,
        )
        suggested_contact_off_n = 0.5 * suggested_contact_on_n

        self.noise_stats = {
            "calibration_start_time": self.noise_calibration_started_iso,
            "duration_s": NOISE_CALIBRATION_DURATION_S,
            "sample_count": int(len(self.noise_calibration_samples)),
            "noise_mean_n": noise_mean_n,
            "noise_std_n": noise_std_n,
            "noise_min_n": noise_min_n,
            "noise_max_n": noise_max_n,
            "noise_peak_abs_n": noise_peak_abs_n,
            "suggested_contact_on_n": suggested_contact_on_n,
            "suggested_contact_off_n": suggested_contact_off_n,
        }

        self._set_noise_status(f"Done | samples={len(self.noise_calibration_samples)}")
        self._set_noise_label("mean", f"{noise_mean_n:.3f} N")
        self._set_noise_label("std", f"{noise_std_n:.3f} N")
        self._set_noise_label("minmax", f"{noise_min_n:.3f} / {noise_max_n:.3f} N")
        self._set_noise_label("peak", f"{noise_peak_abs_n:.3f} N")
        self._set_noise_label("suggest_on", f"{suggested_contact_on_n:.3f} N")
        self._set_noise_label("suggest_off", f"{suggested_contact_off_n:.3f} N")

        self.noise_on_line.setValue(suggested_contact_on_n)
        self.noise_off_line.setValue(suggested_contact_off_n)
        self.noise_on_line.setVisible(True)
        self.noise_off_line.setVisible(True)
        self._write_noise_calibration_json()
        self._refresh_contact_debug_plot()

    def _write_noise_calibration_json(self):
        if not self.is_recording or not self.noise_stats:
            return
        session_dir = getattr(self, "_current_session_dir", None)
        if not session_dir:
            return
        path = os.path.join(session_dir, "noise_calibration.json")
        try:
            with open(path, "w") as f:
                json.dump(self.noise_stats, f, indent=2)
        except Exception as exc:
            self.lbl_status.setText(f"Status: NOISE JSON WRITE FAILED: {exc}")
            self.lbl_status.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def _update_noise_status_indicators(self, sample, backend):
        actual = self._backend_measurement(backend, "actual_sent_ratio") if backend else None
        contact = self._backend_measurement(backend, "contact_active") if backend else None

        self._set_noise_label(
            "backend_contact",
            self._format_bool(contact, "ON", "OFF"),
        )
        self._set_noise_label(
            "actual_haptic",
            f"{float(actual):.3f}" if actual is not None else "--",
        )

        if sample is None or not self.noise_stats:
            self._set_noise_label("inside", "--")
            self._set_noise_label("above_on", "--")
            return

        force_n = float(sample["force_n"])
        on_n = float(self.noise_stats["suggested_contact_on_n"])
        inside = force_n < on_n
        above_on = force_n >= on_n
        self._set_noise_label("inside", "YES" if inside else "NO")
        self._set_noise_label("above_on", "YES" if above_on else "NO")

    def _append_contact_debug_sample(self, sample, backend):
        if sample is None:
            return
        sample_t = sample["timestamp"]
        if self._last_contact_debug_sample_t == sample_t:
            return
        self._last_contact_debug_sample_t = sample_t

        actual = self._backend_measurement(backend, "actual_sent_ratio", 0.0)
        contact = self._backend_measurement(backend, "contact_active", False)
        baseline = self._backend_measurement(backend, "baseline_force_n")
        if baseline is None:
            baseline = self._backend_measurement(backend, "contact_baseline_n")
        shift_on = self._backend_measurement(backend, "noise_band_n")
        if shift_on is None:
            shift_on = self._backend_measurement(backend, "contact_shift_threshold_n")
        if baseline is not None:
            self.backend_baseline_line.setValue(float(baseline))
            self.backend_baseline_line.setVisible(True)
            if shift_on is not None:
                self.backend_shift_on_line.setValue(float(baseline) + float(shift_on))
                self.backend_shift_on_line.setVisible(True)
        else:
            self.backend_baseline_line.setVisible(False)
            self.backend_shift_on_line.setVisible(False)
        self.contact_debug_time_buffer.append(sample_t)
        self.contact_debug_force_buffer.append(float(sample["force_n"]))
        self.contact_debug_ratio_buffer.append(float(actual or 0.0))
        self.contact_debug_contact_buffer.append(1.0 if bool(contact) else 0.0)

        if len(self.contact_debug_time_buffer) > self.max_samples:
            self.contact_debug_time_buffer = self.contact_debug_time_buffer[-self.max_samples:]
            self.contact_debug_force_buffer = self.contact_debug_force_buffer[-self.max_samples:]
            self.contact_debug_ratio_buffer = self.contact_debug_ratio_buffer[-self.max_samples:]
            self.contact_debug_contact_buffer = self.contact_debug_contact_buffer[-self.max_samples:]

        t_min = sample_t - self.time_window_sec
        while self.contact_debug_time_buffer and self.contact_debug_time_buffer[0] < t_min:
            self.contact_debug_time_buffer.pop(0)
            self.contact_debug_force_buffer.pop(0)
            self.contact_debug_ratio_buffer.pop(0)
            self.contact_debug_contact_buffer.pop(0)

        self._refresh_contact_debug_plot()

    def _refresh_contact_debug_plot(self):
        if not getattr(self, "contact_debug_time_buffer", None):
            self.contact_force_curve.setData([], [])
            self.contact_haptic_curve.setData([], [])
            self.contact_active_curve.setData([], [])
            return

        t0 = self.contact_debug_time_buffer[0]
        x = [tt - t0 for tt in self.contact_debug_time_buffer]
        max_force = max([abs(v) for v in self.contact_debug_force_buffer] + [1.0])
        suggested_on = 0.0
        if self.noise_stats:
            suggested_on = float(self.noise_stats.get("suggested_contact_on_n", 0.0))
        scale_n = max(1.0, suggested_on, max_force * 1.2)
        self._contact_debug_ratio_scale_n = scale_n

        haptic_y = [float(v) * scale_n for v in self.contact_debug_ratio_buffer]
        contact_y = [float(v) * scale_n for v in self.contact_debug_contact_buffer]

        self.contact_force_curve.setData(x, self.contact_debug_force_buffer)
        self.contact_haptic_curve.setData(x, haptic_y)
        self.contact_active_curve.setData(x, contact_y)
        self.contact_debug_plot.setXRange(0, self.time_window_sec)
        self.contact_debug_plot.setYRange(-0.05 * scale_n, 1.15 * scale_n)

    def update_gui(self, data):
        """Called safely by the background thread whenever new data is ready."""
        keys = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        raw_data = data.get('raw', {})
        backend = data.get('backend')
        backend_sample = self._backend_force_sample(backend)
        self._process_noise_calibration(backend_sample)
        self._append_contact_debug_sample(backend_sample, backend)
        self._update_noise_status_indicators(backend_sample, backend)

        if self.show_backend_lpf:
            if backend is None:
                self.lbl_view_status.setText("Waiting for /neuro_final/ft_haptic_debug")
                self._update_backend_info(None)
                return
            values, using_old_filtered = self._backend_filtered_values(backend)
            if not values:
                self.lbl_view_status.setText("Waiting for filtered_sensor in /neuro_final/ft_haptic_debug")
                self._update_backend_info(backend, using_old_filtered)
                return
            if using_old_filtered:
                self.lbl_view_status.setText("WARNING: using old backend filtered field")
            else:
                self.lbl_view_status.setText("View: BACKEND FILTERED SENSOR")
            t = float(backend.get('stamp', backend.get('t', 0.0)))
            self._update_backend_info(backend, using_old_filtered)
            if self._last_backend_sample_t == t:
                return
            self._last_backend_sample_t = t
        else:
            values = raw_data
            t = float(raw_data.get('t', 0.0))
            self.lbl_view_status.setText("View: RAW FT SENSOR")
            self._update_backend_info(backend)
            if self._last_raw_sample_t == t:
                return
            self._last_raw_sample_t = t

        for key in keys:
            raw_val = float(values.get(key, 0.0))
            val = self._display_value(key, raw_val)
            self.value_labels[key].setText(f"{val:.2f}")

            # Append newest raw sample
            self.time_buffers[key].append(t)
            self.data_buffers[key].append(val)

            # Prevent unlimited growth
            if len(self.time_buffers[key]) > self.max_samples:
                self.time_buffers[key] = self.time_buffers[key][-self.max_samples:]
                self.data_buffers[key] = self.data_buffers[key][-self.max_samples:]

            # Keep only the last self.time_window_sec seconds
            t_min = t - self.time_window_sec
            while self.time_buffers[key] and self.time_buffers[key][0] < t_min:
                self.time_buffers[key].pop(0)
                self.data_buffers[key].pop(0)

            # Plot relative time so visible x-axis starts from 0
            if self.time_buffers[key]:
                t0 = self.time_buffers[key][0]
                x = [tt - t0 for tt in self.time_buffers[key]]
                self.curves[key].setData(x, self.data_buffers[key])
                self.plots[key].setXRange(0, self.time_window_sec)

    def _haptic_mode_button_style(self, selected: bool) -> str:
        if selected:
            return (
                "QPushButton { background-color: #50fa7b; color: #1e1e1e; "
                "font-weight: bold; border-radius: 5px; font-size: 11px; "
                "padding: 4px 8px; }"
                "QPushButton:hover { background-color: #5cffa0; }"
            )
        return (
            "QPushButton { background-color: #2c3e50; color: white; "
            "font-weight: bold; border-radius: 5px; font-size: 11px; "
            "padding: 4px 8px; }"
            "QPushButton:hover { background-color: #34495e; }"
        )

    def _refresh_haptic_mode_buttons(self):
        for mode_id, btn in self.haptic_mode_buttons.items():
            is_active = (mode_id == self.current_haptic_mode)
            btn.blockSignals(True)
            btn.setChecked(is_active)
            btn.blockSignals(False)
            btn.setStyleSheet(self._haptic_mode_button_style(is_active))

    def _haptic_baseline_or_default(self):
        # If the backend hasn't answered get_parameters yet, assume the
        # default config (all three primary channels on, optional channels
        # off). The user can always click BOTH again once the link is up.
        baseline = self.ros_node.haptic_baseline
        if baseline:
            return baseline
        fallback = {name: False for name in HAPTIC_ALL_FLAGS}
        fallback["v7_ft_haptic_trigger_enable"] = True
        fallback["v7_ft_haptic_rumble_enable"] = True
        fallback["v7_ft_haptic_contact_pulse_enable"] = True
        return fallback

    def _haptic_updates_for_mode(self, mode_id: str):
        baseline = self._haptic_baseline_or_default()
        updates = {}
        if mode_id == HAPTIC_MODE_BOTH:
            for name in HAPTIC_ALL_FLAGS:
                updates[name] = bool(baseline.get(name, False))
        elif mode_id == HAPTIC_MODE_VIBRATION_ONLY:
            for name in HAPTIC_VIBRATION_FLAGS:
                updates[name] = bool(baseline.get(name, False))
            for name in HAPTIC_TRIGGER_FLAGS:
                updates[name] = False
        elif mode_id == HAPTIC_MODE_TRIGGER_ONLY:
            for name in HAPTIC_VIBRATION_FLAGS:
                updates[name] = False
            for name in HAPTIC_TRIGGER_FLAGS:
                updates[name] = bool(baseline.get(name, False))
        elif mode_id == HAPTIC_MODE_OFF:
            for name in HAPTIC_ALL_FLAGS:
                updates[name] = False
        return updates

    def set_haptic_mode(self, mode_id: str):
        if mode_id not in (
            HAPTIC_MODE_BOTH,
            HAPTIC_MODE_VIBRATION_ONLY,
            HAPTIC_MODE_TRIGGER_ONLY,
            HAPTIC_MODE_OFF,
        ):
            return
        updates = self._haptic_updates_for_mode(mode_id)
        ok = self.ros_node.send_haptic_param_update(updates)
        if not ok:
            self.lbl_haptic_mode_status.setText(
                "Mode change pending: neuro_final_teleop parameter service not ready"
            )
            self.lbl_haptic_mode_status.setStyleSheet(
                "color: #f1c40f; border: none; font-weight: bold; font-size: 11px;"
            )
            return
        self.current_haptic_mode = mode_id
        self._refresh_haptic_mode_buttons()
        labels = {
            HAPTIC_MODE_BOTH: "Mode: BOTH (vibration + adaptive trigger)",
            HAPTIC_MODE_VIBRATION_ONLY: "Mode: VIBRATION ONLY (adaptive trigger muted)",
            HAPTIC_MODE_TRIGGER_ONLY: "Mode: ADAPTIVE TRIGGER ONLY (vibration muted)",
            HAPTIC_MODE_OFF: "Mode: OFF (all haptics muted)",
        }
        self.lbl_haptic_mode_status.setText(labels[mode_id])
        self.lbl_haptic_mode_status.setStyleSheet(
            "color: #50fa7b; border: none; font-weight: bold; font-size: 11px;"
        )

    def _haptic_mode_tick(self):
        # Try to grab the original config values once the backend service
        # is ready. After that, the timer mostly just keeps the status
        # label honest.
        if self.ros_node.haptic_baseline is None:
            self.ros_node.request_haptic_baseline()
        elif not self._haptic_baseline_logged:
            self._haptic_baseline_logged = True
            if self.current_haptic_mode == HAPTIC_MODE_BOTH:
                self.lbl_haptic_mode_status.setText(
                    "Mode: BOTH (vibration + adaptive trigger)"
                )
                self.lbl_haptic_mode_status.setStyleSheet(
                    "color: #50fa7b; border: none; font-weight: bold; font-size: 11px;"
                )

    def closeEvent(self, event):
        """Safely shuts down the background thread when the user closes the window."""
        self.ros_thread.stop()
        super().closeEvent(event)


def main():
    rclpy.init()
    ros_node = RosNode()
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        if window.recording_process:
            try:
                os.killpg(os.getpgid(window.recording_process.pid), signal.SIGINT)
                window.recording_process.wait(timeout=2)
            except Exception:
                pass
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
