import sys
import json
import csv
import time
import random
import numpy as np
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QComboBox, QCheckBox, QFileDialog, 
                             QGroupBox, QSplitter, QFrame, QScrollArea, QRadioButton, QButtonGroup,
                             QTabWidget, QTextEdit, QSpinBox, QDoubleSpinBox, QSlider)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QThread, QObject
from PyQt6.QtGui import QColor, QPalette, QFont
import pyqtgraph as pg

# ==========================================
# Constants & Configuration
# ==========================================
DEFAULT_BAUD_RATE = 2000000
BAUD_RATES = [9600, 115200, 500000, 921600, 1000000, 2000000]

# Grafana-style Colors
COLOR_BG = "#161719"
COLOR_PANEL = "#1f1f20"
COLOR_TEXT = "#c7d0d9"
COLOR_ACCENT_1 = "#00bcd4" # Cyan
COLOR_ACCENT_2 = "#e91e63" # Pink
COLOR_ACCENT_3 = "#8bc34a" # Green
COLOR_ACCENT_4 = "#ff9800" # Orange
COLOR_ACCENT_5 = "#9c27b0" # Purple

STYLESHEET = f"""
QMainWindow {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
}}
QWidget {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
    font-family: 'Segoe UI', sans-serif;
    font-size: 10pt;
}}
QGroupBox {{
    border: 1px solid #333;
    border-radius: 5px;
    margin-top: 10px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 3px;
}}
QPushButton {{
    background-color: {COLOR_PANEL};
    border: 1px solid #333;
    border-radius: 4px;
    padding: 5px 10px;
    color: {COLOR_TEXT};
}}
QPushButton:hover {{
    background-color: #2a2a2b;
    border-color: {COLOR_ACCENT_1};
}}
QPushButton:pressed {{
    background-color: {COLOR_ACCENT_1};
    color: #000;
}}
QComboBox {{
    background-color: {COLOR_PANEL};
    border: 1px solid #333;
    border-radius: 4px;
    padding: 5px;
}}
QCheckBox {{
    spacing: 5px;
}}
QCheckBox::indicator {{
    width: 15px;
    height: 15px;
    border-radius: 3px;
    border: 1px solid #555;
    background-color: {COLOR_PANEL};
}}
QCheckBox::indicator:checked {{
    background-color: {COLOR_ACCENT_3};
    border-color: {COLOR_ACCENT_3};
}}
QLabel {{
    color: {COLOR_TEXT};
}}
"""

# ==========================================
# Serial Worker
# ==========================================
class SerialWorker(QThread):
    data_received = pyqtSignal(dict)
    raw_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None
        self.pending_commands = []

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2) # Wait for reset

            while self.running and self.ser.is_open:
                # Send pending commands
                while self.pending_commands:
                    cmd = self.pending_commands.pop(0)
                    self.ser.write((cmd + '\n').encode())
                    time.sleep(0.05)

                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.raw_received.emit(line)
                        if line.startswith('{') and line.endswith('}'):
                            try:
                                data = json.loads(line)
                                self.data_received.emit(data)
                            except json.JSONDecodeError:
                                pass
        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

    def send_command(self, cmd):
        self.pending_commands.append(cmd)

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.wait()

# ==========================================
# Signal Generator (Simulation)
# ==========================================
class SignalGenerator:
    def __init__(self):
        self.t = 0
    
    def next_sample(self):
        self.t += 0.01
        # Generate some sine waves + noise
        mx = 10 * np.sin(2 * np.pi * 1.0 * self.t) + np.random.normal(0, 0.5)
        my = 10 * np.sin(2 * np.pi * 2.0 * self.t + np.pi/4) + np.random.normal(0, 0.5)
        mz = 10 * np.sin(2 * np.pi * 0.5 * self.t + np.pi/2) + np.random.normal(0, 0.5)
        mag = np.sqrt(mx**2 + my**2 + mz**2)
        cur = 5 + 2 * np.sin(2 * np.pi * 5.0 * self.t) + np.random.normal(0, 0.2)
        
        return {
            "mx": mx, "my": my, "mz": mz, 
            "mag": mag, "cur": cur, "slip": 0,
            "t": self.t * 1000 # simulate ms timestamp
        }

# ==========================================
# Main Application
# ==========================================
class AdaptiveGripperGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Adaptive Gripper Analysis")
        self.resize(1200, 800)
        
        # Data Storage
        self.buffer_size = 1000
        self.data = {
            'mx': [], 'my': [], 'mz': [], 'mag': [], 
            'rmx': [], 'rmy': [], 'rmz': [],
            'cur': [], 'slip': [], 's_ind': [],
            'srv': [], 'grp': [],
            'timestamp': []
        }
        # Buffer for despiking (median filter)
        self.spike_buffer = {k: [] for k in self.data.keys() if k != 'timestamp'}
        
        self.fft_data = {'freqs': [], 'mags': []}
        self.recorded_events = [] # For start/end of segments (Labeling)
        self.current_segment_start = None
        
        # Recording & Replay
        self.is_recording = False
        self.recording_data = [] # List of dicts for the current recording session
        self.recording_fft = []  # List of dicts for FFT data
        self.replay_data = []    # List of dicts for replay
        self.replay_fft_data = [] # List of dicts for replay FFT
        self.replay_index = 0
        self.replay_buffer = {
            'mx': [], 'my': [], 'mz': [], 'mag': [], 
            'cur': [], 'slip': [], 'srv': [], 'grp': [], 'timestamp': []
        }
        
        self.serial_thread = None
        self.sim_generator = SignalGenerator()
        self.is_simulating = False
        self.is_connected = False
        
        # Setup UI
        self.setup_ui()
        self.setup_plotting()
        self.apply_styles()
        
        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33) # ~30 FPS

        # Timer for replay
        self.replay_timer = QTimer()
        self.replay_timer.timeout.connect(self.update_replay_loop)

    def setup_ui(self):
        # Central Widget & Main Layout
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # --- Sidebar ---
        scroll_sidebar = QScrollArea()
        scroll_sidebar.setWidgetResizable(True)
        scroll_sidebar.setFixedWidth(300)
        scroll_sidebar.setStyleSheet(f"background-color: {COLOR_PANEL}; border: none;")
        
        sidebar = QWidget()
        sidebar.setStyleSheet(f"background-color: {COLOR_PANEL};")
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(15, 15, 15, 15)
        sidebar_layout.setSpacing(15)
        
        scroll_sidebar.setWidget(sidebar)

        # Title
        title = QLabel("SIGNAL CONTROLS")
        title.setStyleSheet(f"color: {COLOR_ACCENT_1}; font-weight: bold; font-size: 12pt;")
        sidebar_layout.addWidget(title)

        # 1. Connection
        grp_conn = QGroupBox("Connection")
        v_conn = QVBoxLayout()
        
        # Port Selection
        self.combo_ports = QComboBox()
        self.refresh_ports()
        v_conn.addWidget(QLabel("Port:"))
        v_conn.addWidget(self.combo_ports)
        
        # Baud Selection
        self.combo_baud = QComboBox()
        for b in BAUD_RATES:
            self.combo_baud.addItem(str(b))
        self.combo_baud.setCurrentText(str(DEFAULT_BAUD_RATE))
        v_conn.addWidget(QLabel("Baud Rate:"))
        v_conn.addWidget(self.combo_baud)

        # Connect/Disconnect
        self.btn_connect = QPushButton("Connect Serial")
        self.btn_connect.clicked.connect(self.toggle_connection)
        v_conn.addWidget(self.btn_connect)
        
        # Refresh Ports
        btn_refresh = QPushButton("Refresh Ports")
        btn_refresh.clicked.connect(self.refresh_ports)
        v_conn.addWidget(btn_refresh)

        grp_conn.setLayout(v_conn)
        sidebar_layout.addWidget(grp_conn)

        # 2. Source Mode
        grp_source = QGroupBox("Data Source")
        v_source = QVBoxLayout()
        self.radio_serial = QRadioButton("Serial Stream")
        self.radio_sim = QRadioButton("Simulation")
        self.radio_serial.setChecked(True)
        self.radio_group = QButtonGroup()
        self.radio_group.addButton(self.radio_serial)
        self.radio_group.addButton(self.radio_sim)
        v_source.addWidget(self.radio_serial)
        v_source.addWidget(self.radio_sim)
        
        self.radio_sim.toggled.connect(self.toggle_simulation)
        
        grp_source.setLayout(v_source)
        sidebar_layout.addWidget(grp_source)

        # 3. Telemetry & Plots
        self.setup_telemetry_ui(sidebar_layout)

        # 4. FFT Configuration
        grp_fft_cfg = QGroupBox("FFT Configuration")
        v_fft_cfg = QVBoxLayout()
        
        h_samp = QHBoxLayout()
        h_samp.addWidget(QLabel("Samples:"))
        self.spin_fft_samples = QSpinBox()
        self.spin_fft_samples.setRange(32, 2048)
        self.spin_fft_samples.setValue(128)
        h_samp.addWidget(self.spin_fft_samples)
        v_fft_cfg.addLayout(h_samp)
        
        h_rate = QHBoxLayout()
        h_rate.addWidget(QLabel("Rate (Hz):"))
        self.spin_fft_rate = QSpinBox()
        self.spin_fft_rate.setRange(1, 10000)
        self.spin_fft_rate.setValue(2000) # Default to 2kHz based on scan cycle
        h_rate.addWidget(self.spin_fft_rate)
        v_fft_cfg.addLayout(h_rate)
        
        grp_fft_cfg.setLayout(v_fft_cfg)
        sidebar_layout.addWidget(grp_fft_cfg)

        # 5. Analysis
        grp_ana = QGroupBox("Analysis")
        v_ana = QVBoxLayout()
        
        # Despike Checkbox
        self.chk_despike = QCheckBox("Despike (Median Filter)")
        self.chk_despike.setChecked(True)
        self.chk_despike.setToolTip("Removes single-sample outliers/spikes using a 3-point median filter.")
        v_ana.addWidget(self.chk_despike)
        
        self.lbl_freq = QLabel("Dominant Freq: 0.0 Hz")
        self.lbl_freq.setStyleSheet(f"color: {COLOR_ACCENT_1}; font-size: 11pt; font-weight: bold;")
        v_ana.addWidget(self.lbl_freq)
        grp_ana.setLayout(v_ana)
        sidebar_layout.addWidget(grp_ana)

        # 7. Plot Settings (Y-Axis)
        grp_plot = QGroupBox("Plot Settings")
        v_plot = QVBoxLayout()
        
        self.chk_auto_scale = QCheckBox("Auto Scale Y-Axis")
        self.chk_auto_scale.setChecked(True)
        self.chk_auto_scale.toggled.connect(self.toggle_axis_scale)
        v_plot.addWidget(self.chk_auto_scale)
        
        # Center DC Option
        self.chk_center_dc = QCheckBox("Center at DC (Avg)")
        self.chk_center_dc.toggled.connect(self.toggle_center_dc)
        v_plot.addWidget(self.chk_center_dc)
        
        # Range setting for Center DC
        h_center_range = QHBoxLayout()
        h_center_range.addWidget(QLabel("Range (+/-):"))
        self.spin_center_range = QDoubleSpinBox()
        self.spin_center_range.setRange(0.1, 1000.0)
        self.spin_center_range.setValue(20.0)
        self.spin_center_range.setEnabled(False)
        h_center_range.addWidget(self.spin_center_range)
        v_plot.addLayout(h_center_range)
        
        # Manual Min/Max
        h_range = QHBoxLayout()
        self.spin_y_min = QSpinBox()
        self.spin_y_min.setRange(-1000, 1000)
        self.spin_y_min.setValue(-10)
        self.spin_y_min.setSuffix(" min")
        self.spin_y_min.setEnabled(False)
        self.spin_y_min.valueChanged.connect(self.update_axis_range)
        
        self.spin_y_max = QSpinBox()
        self.spin_y_max.setRange(-1000, 1000)
        self.spin_y_max.setValue(10)
        self.spin_y_max.setSuffix(" max")
        self.spin_y_max.setEnabled(False)
        self.spin_y_max.valueChanged.connect(self.update_axis_range)

        h_range.addWidget(self.spin_y_min)
        h_range.addWidget(self.spin_y_max)
        v_plot.addLayout(h_range)
        
        grp_plot.setLayout(v_plot)
        sidebar_layout.addWidget(grp_plot)

        # 8. Data Actions
        grp_act = QGroupBox("Data & Labeling")
        v_act = QVBoxLayout()
        
        # Recording Controls
        hbox_rec = QHBoxLayout()
        self.btn_record = QPushButton("Start Recording")
        self.btn_record.setStyleSheet(f"background-color: {COLOR_PANEL}; color: {COLOR_ACCENT_3};")
        self.btn_record.clicked.connect(self.toggle_recording)
        hbox_rec.addWidget(self.btn_record)
        v_act.addLayout(hbox_rec)

        hbox_label = QHBoxLayout()
        self.btn_success = QPushButton("Mark Success")
        self.btn_success.setStyleSheet(f"background-color: {COLOR_ACCENT_3}; color: #000;")
        self.btn_fail = QPushButton("Mark Fail")
        self.btn_fail.setStyleSheet(f"background-color: {COLOR_ACCENT_2}; color: #fff;")
        hbox_label.addWidget(self.btn_success)
        hbox_label.addWidget(self.btn_fail)
        
        self.btn_success.clicked.connect(lambda: self.mark_event("Success"))
        self.btn_fail.clicked.connect(lambda: self.mark_event("Failure"))
        
        self.btn_export = QPushButton("EXPORT DATA (CSV)")
        self.btn_export.setMinimumHeight(40)
        self.btn_export.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_export.setStyleSheet(f"""
            background-color: {COLOR_ACCENT_4}; 
            color: #000; 
            font-weight: bold; 
            font-size: 12pt;
            border-radius: 5px;
            border: 2px solid #d68100;
        """)
        self.btn_export.clicked.connect(self.export_data)

        v_act.addLayout(hbox_label)
        v_act.addWidget(self.btn_export)
        
        grp_act.setLayout(v_act)
        sidebar_layout.addWidget(grp_act)
        
        sidebar_layout.addStretch()
        main_layout.addWidget(scroll_sidebar)

        # --- Main Content Area (Tabs) ---
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{ border: 1px solid #444; }}
            QTabBar::tab {{
                background: {COLOR_PANEL};
                color: {COLOR_TEXT};
                padding: 8px 20px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background: #333;
                border-bottom: 2px solid {COLOR_ACCENT_1};
            }}
        """)
        
        # Tab 1: Visualizer
        tab_viz = QWidget()
        layout_viz = QVBoxLayout(tab_viz)
        
        splitter = QSplitter(Qt.Orientation.Vertical)
        
        # Time Plot
        self.plot_time = pg.PlotWidget(title="Time-Series Data")
        self.plot_time.setBackground(COLOR_BG)
        self.plot_time.showGrid(x=True, y=True, alpha=0.3)
        self.plot_time.getAxis('bottom').setPen(COLOR_TEXT)
        self.plot_time.getAxis('left').setPen(COLOR_TEXT)
        
        # FFT Plot
        self.plot_fft = pg.PlotWidget(title="Real-Time FFT (Frequency Domain)")
        self.plot_fft.setBackground(COLOR_BG)
        self.plot_fft.showGrid(x=True, y=True, alpha=0.3)
        self.plot_fft.setLabel('bottom', "Frequency", units='Hz')
        self.plot_fft.setLabel('left', "Magnitude")

        splitter.addWidget(self.plot_time)
        splitter.addWidget(self.plot_fft)
        
        layout_viz.addWidget(splitter)
        
        # Tab 2: Replay
        self.tab_replay = QWidget()
        self.setup_replay_ui(self.tab_replay)
        
        # Tab 3: Raw Output
        self.tab_raw = QWidget()
        layout_raw = QVBoxLayout(self.tab_raw)
        self.text_raw = QTextEdit()
        self.text_raw.setReadOnly(True)
        self.text_raw.setStyleSheet(f"background-color: #000; color: #0f0; font-family: Consolas, monospace;")
        layout_raw.addWidget(self.text_raw)
        
        self.tabs.addTab(tab_viz, "Visualizer")
        self.tabs.addTab(self.tab_replay, "Replay")
        self.tabs.addTab(self.tab_raw, "Raw Serial")
        
        main_layout.addWidget(self.tabs, stretch=1)
    
    def setup_replay_ui(self, parent):
        layout = QVBoxLayout(parent)
        
        splitter = QSplitter(Qt.Orientation.Vertical)
        
        # Plot Time Series
        self.plot_replay = pg.PlotWidget(title="Replay Data")
        self.plot_replay.setBackground(COLOR_BG)
        self.plot_replay.showGrid(x=True, y=True, alpha=0.3)
        self.plot_replay.getAxis('bottom').setPen(COLOR_TEXT)
        self.plot_replay.getAxis('left').setPen(COLOR_TEXT)
        splitter.addWidget(self.plot_replay)
        
        # Plot FFT
        self.plot_replay_fft = pg.PlotWidget(title="Replay FFT")
        self.plot_replay_fft.setBackground(COLOR_BG)
        self.plot_replay_fft.showGrid(x=True, y=True, alpha=0.3)
        self.plot_replay_fft.setLabel('bottom', "Frequency", units='Hz')
        self.plot_replay_fft.setLabel('left', "Magnitude")
        splitter.addWidget(self.plot_replay_fft)
        
        layout.addWidget(splitter)
        
        # Replay Curves (Time Series)
        self.replay_curves = {}
        # Same colors as main
        c_mx = COLOR_ACCENT_2 
        c_my = COLOR_ACCENT_3 
        c_mz = COLOR_ACCENT_1
        c_mag = "#ffffff"
        c_cur = COLOR_ACCENT_4
        c_raw = "#777777"
        c_slip = "#ff0000"
        c_srv = COLOR_ACCENT_5
        
        # Mag Filtered
        self.replay_curves['mx'] = self.plot_replay.plot(pen=pg.mkPen(c_mx, width=2), name='Mag X')
        self.replay_curves['my'] = self.plot_replay.plot(pen=pg.mkPen(c_my, width=2), name='Mag Y')
        self.replay_curves['mz'] = self.plot_replay.plot(pen=pg.mkPen(c_mz, width=2), name='Mag Z')
        self.replay_curves['mag'] = self.plot_replay.plot(pen=pg.mkPen(c_mag, width=2), name='Magnitude')
        
        # Mag Raw
        self.replay_curves['rmx'] = self.plot_replay.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw X')
        self.replay_curves['rmy'] = self.plot_replay.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw Y')
        self.replay_curves['rmz'] = self.plot_replay.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw Z')

        # Current
        self.replay_curves['cur'] = self.plot_replay.plot(pen=pg.mkPen(c_cur, width=2), name='Current')
        
        # Slip
        self.replay_curves['slip'] = self.plot_replay.plot(pen=pg.mkPen(c_slip, width=2), name='Slip State')
        self.replay_curves['s_ind'] = self.plot_replay.plot(pen=pg.mkPen(c_slip, width=1, style=Qt.PenStyle.DotLine), name='Slip Ind')
        
        # Servo
        self.replay_curves['srv'] = self.plot_replay.plot(pen=pg.mkPen(c_srv, width=2), name='Servo')
        self.replay_curves['grp'] = self.plot_replay.plot(pen=pg.mkPen(c_srv, width=1, style=Qt.PenStyle.DashLine), name='Grip')

        # Replay Curve (FFT)
        self.curve_replay_fft = self.plot_replay_fft.plot(pen=pg.mkPen(COLOR_ACCENT_1, width=2, fillLevel=0, brush=(0, 188, 212, 50)))

        # Initially hide all
        for c in self.replay_curves.values():
            c.setVisible(False)
        
        # Controls
        h_controls = QHBoxLayout()
        
        self.btn_load_replay = QPushButton("Load File")
        self.btn_load_replay.clicked.connect(self.load_replay_file)
        h_controls.addWidget(self.btn_load_replay)
        
        self.btn_play_replay = QPushButton("Play")
        self.btn_play_replay.clicked.connect(self.toggle_replay)
        h_controls.addWidget(self.btn_play_replay)
        
        self.slider_replay = QSlider(Qt.Orientation.Horizontal)
        self.slider_replay.setRange(0, 100)
        self.slider_replay.sliderMoved.connect(self.on_replay_slider_move)
        h_controls.addWidget(self.slider_replay)
        
        self.lbl_replay_time = QLabel("0.0s")
        h_controls.addWidget(self.lbl_replay_time)
        
        layout.addLayout(h_controls)

    def setup_telemetry_ui(self, layout):
        grp = QGroupBox("Telemetry & Plotting")
        vbox = QVBoxLayout()
        vbox.setSpacing(10)
        
        # Helper to create a group
        def create_stream_group(label, command, plot_keys):
            # Main Checkbox (Command)
            chk_cmd = QCheckBox(label)
            chk_cmd.toggled.connect(lambda c: self.send_stream_command(command, c))
            
            # Sub Checkboxes (Plotting)
            sub_layout = QVBoxLayout()
            sub_layout.setContentsMargins(20, 0, 0, 0)
            
            sub_checks = {}
            for key, name in plot_keys.items():
                chk = QCheckBox(name)
                chk.setChecked(False) # Default off
                chk.setVisible(False) # Initially hidden until parent enabled? Or just disabled
                chk.toggled.connect(lambda c, k=key: self.toggle_plot_visibility(k, c))
                sub_layout.addWidget(chk)
                sub_checks[key] = chk
                
            # Link visibility/enable
            def on_main_toggle(checked):
                for chk in sub_checks.values():
                    chk.setVisible(checked)
                    if not checked:
                        chk.setChecked(False)
            
            chk_cmd.toggled.connect(on_main_toggle)
            
            vbox.addWidget(chk_cmd)
            vbox.addLayout(sub_layout)
            return chk_cmd, sub_checks

        # 1. Mag Filtered
        self.grp_mag, self.chk_mag = create_stream_group("Mag Filtered", "mag_filtered", {
            'mx': "Mag X", 'my': "Mag Y", 'mz': "Mag Z", 'mag': "Magnitude"
        })
        
        # 2. Mag Raw
        self.grp_raw, self.chk_raw = create_stream_group("Mag Raw", "mag_raw", {
            'rmx': "Raw X", 'rmy': "Raw Y", 'rmz': "Raw Z"
        })

        # 3. Current
        self.grp_cur, self.chk_cur = create_stream_group("Current", "current", {
            'cur': "Current (mA)"
        })

        # 4. Slip
        self.grp_slip, self.chk_slip = create_stream_group("Slip Detection", "slip", {
            'slip': "Slip State", 's_ind': "Slip Indicator"
        })

        # 5. Servo
        self.grp_srv, self.chk_srv = create_stream_group("Servo & Mode", "servo", {
            'srv': "Servo Pos", 'grp': "Grip State"
        })
        
        # 6. FFT Mode
        self.chk_cmd_fft = QCheckBox("FFT Mode (Exclusive)")
        self.chk_cmd_fft.toggled.connect(lambda c: self.send_stream_command("fft", c))
        vbox.addWidget(self.chk_cmd_fft)

        grp.setLayout(vbox)
        layout.addWidget(grp)

    def toggle_plot_visibility(self, key, visible):
        if key in self.curves:
            self.curves[key].setVisible(visible)
        if key in self.replay_curves:
            self.replay_curves[key].setVisible(visible)

    def send_stream_command(self, key, enabled):
        if self.is_connected and self.serial_thread:
            cmd = json.dumps({key: enabled})
            self.serial_thread.send_command(cmd)
            # Log sent command
            self.text_raw.append(f">> SENT: {cmd}")

    def toggle_axis_scale(self, checked):
        # Enable/Disable spinboxes
        self.spin_y_min.setEnabled(False)
        self.spin_y_max.setEnabled(False)
        self.spin_center_range.setEnabled(False)
        
        if checked:
            # Auto Scale ON
            self.chk_center_dc.setChecked(False) # Mutually exclusive
            self.plot_time.enableAutoRange(axis='y')
            self.plot_replay.enableAutoRange(axis='y')
        else:
            # Auto Scale OFF - check if Center DC is ON, otherwise Manual
            if not self.chk_center_dc.isChecked():
                self.spin_y_min.setEnabled(True)
                self.spin_y_max.setEnabled(True)
                self.update_axis_range()

    def toggle_center_dc(self, checked):
        if checked:
            # Center DC ON
            self.chk_auto_scale.setChecked(False) # Mutually exclusive
            self.spin_center_range.setEnabled(True)
            self.spin_y_min.setEnabled(False)
            self.spin_y_max.setEnabled(False)
            # Disable auto range so we can set it manually in loop
            self.plot_time.disableAutoRange(axis='y')
            self.plot_replay.disableAutoRange(axis='y')
        else:
            # Center DC OFF
            self.spin_center_range.setEnabled(False)
            # If Auto Scale is also OFF, enable manual
            if not self.chk_auto_scale.isChecked():
                self.spin_y_min.setEnabled(True)
                self.spin_y_max.setEnabled(True)
                self.update_axis_range()

    def update_axis_range(self):
        if not self.chk_auto_scale.isChecked() and not self.chk_center_dc.isChecked():
            ymin = self.spin_y_min.value()
            ymax = self.spin_y_max.value()
            if ymin < ymax:
                self.plot_time.setYRange(ymin, ymax)
                self.plot_replay.setYRange(ymin, ymax)

    def setup_plotting(self):
        # Create curves
        self.curves = {}
        # Colors
        c_mx = COLOR_ACCENT_2 # Pink
        c_my = COLOR_ACCENT_3 # Green
        c_mz = COLOR_ACCENT_1 # Cyan
        c_mag = "#ffffff"     # White
        c_cur = COLOR_ACCENT_4 # Orange
        c_raw = "#777777"     # Grey
        c_slip = "#ff0000"    # Red
        c_srv = COLOR_ACCENT_5 # Purple
        
        # Mag Filtered
        self.curves['mx'] = self.plot_time.plot(pen=pg.mkPen(c_mx, width=2), name='Mag X')
        self.curves['my'] = self.plot_time.plot(pen=pg.mkPen(c_my, width=2), name='Mag Y')
        self.curves['mz'] = self.plot_time.plot(pen=pg.mkPen(c_mz, width=2), name='Mag Z')
        self.curves['mag'] = self.plot_time.plot(pen=pg.mkPen(c_mag, width=2), name='Magnitude')
        
        # Mag Raw
        self.curves['rmx'] = self.plot_time.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw X')
        self.curves['rmy'] = self.plot_time.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw Y')
        self.curves['rmz'] = self.plot_time.plot(pen=pg.mkPen(c_raw, width=1, style=Qt.PenStyle.DashLine), name='Raw Z')

        # Current
        self.curves['cur'] = self.plot_time.plot(pen=pg.mkPen(c_cur, width=2), name='Current')
        
        # Slip
        self.curves['slip'] = self.plot_time.plot(pen=pg.mkPen(c_slip, width=2), name='Slip State')
        self.curves['s_ind'] = self.plot_time.plot(pen=pg.mkPen(c_slip, width=1, style=Qt.PenStyle.DotLine), name='Slip Ind')
        
        # Servo
        self.curves['srv'] = self.plot_time.plot(pen=pg.mkPen(c_srv, width=2), name='Servo')
        self.curves['grp'] = self.plot_time.plot(pen=pg.mkPen(c_srv, width=1, style=Qt.PenStyle.DashLine), name='Grip')
        
        # Initially hide all
        for c in self.curves.values():
            c.setVisible(False)

        self.curve_fft = self.plot_fft.plot(pen=pg.mkPen(COLOR_ACCENT_1, width=2, fillLevel=0, brush=(0, 188, 212, 50)))

    def apply_styles(self):
        self.setStyleSheet(STYLESHEET)

    def refresh_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_simulation(self, checked):
        self.is_simulating = checked
        if checked:
            if self.is_connected:
                self.toggle_connection() # Disconnect serial if active
            self.btn_connect.setEnabled(False)
            self.combo_ports.setEnabled(False)
        else:
            self.btn_connect.setEnabled(True)
            self.combo_ports.setEnabled(True)

    def toggle_connection(self):
        if not self.is_connected:
            # Reset all telemetry plotting checkboxes before connecting
            self.grp_mag.setChecked(False)
            self.grp_raw.setChecked(False)
            self.grp_cur.setChecked(False)
            self.grp_slip.setChecked(False)
            self.grp_srv.setChecked(False)
            self.chk_cmd_fft.setChecked(False)

            port = self.combo_ports.currentText()
            try:
                baud = int(self.combo_baud.currentText())
            except ValueError:
                baud = DEFAULT_BAUD_RATE
                
            if not port:
                return

            self.serial_thread = SerialWorker(port, baud)
            self.serial_thread.data_received.connect(self.handle_data)
            self.serial_thread.raw_received.connect(self.handle_raw)
            self.serial_thread.error_occurred.connect(self.handle_error)
            self.serial_thread.start()
            
            self.is_connected = True
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet(f"background-color: {COLOR_ACCENT_2}; color: white;")
            self.radio_sim.setEnabled(False)
        else:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread = None
            
            self.is_connected = False
            self.btn_connect.setText("Connect Serial")
            self.btn_connect.setStyleSheet("")
            self.radio_sim.setEnabled(True)
            
            # Toggle off all telemetry plotting
            self.grp_mag.setChecked(False)
            self.grp_raw.setChecked(False)
            self.grp_cur.setChecked(False)
            self.grp_slip.setChecked(False)
            self.grp_srv.setChecked(False)
            self.chk_cmd_fft.setChecked(False)

    def handle_error(self, msg):
        self.text_raw.append(f"!! ERROR: {msg}")
        self.toggle_connection() # Reset UI

    def handle_raw(self, line):
        self.text_raw.append(line)
        # Scroll to bottom
        sb = self.text_raw.verticalScrollBar()
        sb.setValue(sb.maximum())

    def handle_data(self, data):
        # Process incoming JSON
        
        # Add reception timestamp (ms)
        # Use PC time if 't' (device time) is missing or 0
        current_time_ms = time.time() * 1000.0
        data['recv_ts'] = current_time_ms
        
        if 't' not in data or data['t'] == 0:
            data['t'] = current_time_ms

        # 1. FFT Data
        if data.get('type') == 'fft':
            # Record FFT if recording
            if self.is_recording:
                self.recording_fft.append(data)
                
            fft_vals = data.get('data', [])
            if fft_vals:
                self.process_external_fft(fft_vals)
            return

        # 2. Time Series Data
        self.append_data(data)

    def process_external_fft(self, fft_vals):
        # Update FFT plot with external data
        # Calculate X axis (Frequency)
        sample_rate = self.spin_fft_rate.value()
        # FFT size is typically (N/2) + 1 or N/2 depending on implementation
        # User example has 64 bins. If N=128, rfft gives 65 bins (0..Nyquist).
        
        num_bins = len(fft_vals)
        if num_bins < 2: 
            return
            
        freqs = np.linspace(0, sample_rate / 2, num_bins)
        
        # Store for update loop (or update directly here if performant enough)
        self.fft_data['freqs'] = freqs
        self.fft_data['mags'] = fft_vals
        
        # Find dominant freq
        try:
            # Skip DC (index 0) if it's huge
            idx_peak = np.argmax(fft_vals[1:]) + 1
            dom_freq = freqs[idx_peak]
            self.lbl_freq.setText(f"Dominant Freq: {dom_freq:.1f} Hz")
        except:
            pass

    def append_data(self, data):
        # Helper to safely append
        ts = data.get('t', 0)
        
        # If recording, save entire data packet (RAW data, before filtering)
        # We typically want to record the raw data so we can post-process differently if needed.
        # But if the spikes are serial errors, maybe we want to record filtered?
        # Usually recording raw is safer, but for "WYSIWYG" export, let's record raw and let replay filter it if enabled?
        # Or filter before recording? 
        # Given the request is about "serial communication problems", these are artifacts, not signal.
        # Let's filter BEFORE storage so everything (Plots, Recording, Export) is clean.
        
        # All keys we track
        keys = ['mx', 'my', 'mz', 'mag', 
                'rmx', 'rmy', 'rmz', 
                'cur', 'slip', 's_ind', 
                'srv', 'grp']
                
        filtered_data = {}
        
        # Apply Despiking (3-point Median Filter)
        for key in keys:
            raw_val = data.get(key, 0.0)
            
            # Update spike buffer
            if key in self.spike_buffer:
                self.spike_buffer[key].append(raw_val)
                if len(self.spike_buffer[key]) > 3:
                    self.spike_buffer[key].pop(0)
            
            val_to_store = raw_val
            
            # Only apply filter to analog-ish signals, not discrete states like 'slip' if not needed
            # But 'slip' is 0/1, median filter works fine on it too (will debounce it).
            if self.chk_despike.isChecked() and key in self.spike_buffer:
                buf = self.spike_buffer[key]
                if len(buf) == 3:
                    # Median of 3
                    # Sort takes ~O(N log N), for N=3 it's tiny.
                    val_to_store = sorted(buf)[1]
            
            filtered_data[key] = val_to_store

        # Update the data dict for plotting
        for key in keys:
            self.data[key].append(filtered_data[key])
            
        self.data['timestamp'].append(ts)
        
        # Keep buffer size
        if len(self.data['timestamp']) > self.buffer_size:
            for k in self.data:
                self.data[k] = self.data[k][-self.buffer_size:]

        # Recording (Save the FILTERED data if despike is on, to avoid saving corruption)
        # If the user wants raw, they can uncheck despike.
        if self.is_recording:
            # Create a copy of data to save, injecting filtered values
            record_packet = data.copy()
            for k, v in filtered_data.items():
                record_packet[k] = v
            self.recording_data.append(record_packet)

    def toggle_recording(self):
        if not self.is_recording:
            # Start Recording
            self.is_recording = True
            self.recording_data = [] # Clear previous recording
            self.recording_fft = []  # Clear previous FFT recording
            self.btn_record.setText("Stop Recording")
            self.btn_record.setStyleSheet(f"background-color: {COLOR_ACCENT_2}; color: white;")
        else:
            # Stop Recording
            self.is_recording = False
            self.btn_record.setText("Start Recording")
            self.btn_record.setStyleSheet(f"background-color: {COLOR_PANEL}; color: {COLOR_ACCENT_3};")
            print(f"Recording stopped. captured {len(self.recording_data)} samples and {len(self.recording_fft)} FFT frames.")

    def update_loop(self):
        # 1. Generate Sim Data if needed
        if self.is_simulating:
            # Generate a few samples per frame to simulate speed
            for _ in range(3): 
                d = self.sim_generator.next_sample()
                self.append_data(d)
                
            # Simulate FFT update occasionally
            if np.random.random() < 0.1:
                # Mock FFT
                freqs = np.linspace(0, 50, 64)
                mags = np.random.random(64) * 10
                mags[10] = 50 # Peak at ~8Hz
                self.fft_data['freqs'] = freqs
                self.fft_data['mags'] = mags

        # 2. Update Time Plot
        if len(self.data['timestamp']) > 1:
            # Calculate DC average if needed
            visible_values = []
            
            # Update data for all curves (visibility handled by checkboxes)
            for key, curve in self.curves.items():
                if key in self.data and curve.isVisible():
                    y_data = self.data[key]
                    curve.setData(y_data)
                    if self.chk_center_dc.isChecked():
                        visible_values.extend(y_data)
            
            # Handle Center DC Scaling
            if self.chk_center_dc.isChecked() and visible_values:
                avg = np.mean(visible_values)
                rng = self.spin_center_range.value()
                self.plot_time.setYRange(avg - rng, avg + rng, padding=0)
                    
        # 3. Update FFT Plot
        # If we have external FFT data, use it.
        # Otherwise, if in SIM mode or default, we might calculate it (but prompt said take from controller)
        if len(self.fft_data['freqs']) > 0 and len(self.fft_data['mags']) > 0:
             self.curve_fft.setData(self.fft_data['freqs'], self.fft_data['mags'])
        else:
            # Fallback local FFT calculation if no external data and we have time series? 
            # User instructions imply we should rely on controller for FFT when connected.
            # But let's keep local FFT for simulation or if user wants it (but we don't have a toggle for local FFT anymore).
            # For now, if no external data is coming, the plot stays empty.
            pass

    def mark_event(self, label):
        # Record the current timestamp/index and label
        if not self.data['timestamp']:
            return
        
        t = self.data['timestamp'][-1]
        self.recorded_events.append({
            'timestamp': t,
            'label': label,
            'data_index': len(self.data['timestamp'])
        })
        print(f"Event Marked: {label} at T={t}")
        # Optional: Add a visual marker on the plot (not implemented here for brevity)

    def export_data(self):
        # Ask for directory to create the recording folder in
        parent_dir = QFileDialog.getExistingDirectory(self, "Select Directory to Save Recording")
        if not parent_dir:
            return

        try:
            import os
            from datetime import datetime
            
            # Create folder: recording_YYYY_MM_DD_HH_MM_SS
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            folder_name = f"recording_{timestamp}"
            save_dir = os.path.join(parent_dir, folder_name)
            os.makedirs(save_dir, exist_ok=True)
            
            # 1. Export Main Time Series Data -> signals.csv
            signals_path = os.path.join(save_dir, "signals.csv")
            
            with open(signals_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header
                keys = ['timestamp', 'recv_ts', 'mx', 'my', 'mz', 'mag', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']
                writer.writerow(keys + ['label'])
                
                # Decide source: Recording Data or Live Buffer
                source_data = []
                
                if self.recording_data:
                    # Use the recording buffer
                    for row_data in self.recording_data:
                        row = []
                        # Map keys
                        t = row_data.get('t', 0)
                        row.append(t)
                        row.append(row_data.get('recv_ts', 0))
                        for k in keys[2:]: # skip timestamps
                            row.append(row_data.get(k, 0))
                        
                        # Check for label
                        lbl = ""
                        for ev in self.recorded_events:
                            if abs(ev['timestamp'] - t) < 1.0: # 1ms tolerance
                                lbl = ev['label']
                                break
                        row.append(lbl)
                        source_data.append(row)
                else:
                     # Use circular buffer
                    events_map = {e['timestamp']: e['label'] for e in self.recorded_events}
                    
                    for i in range(len(self.data['timestamp'])):
                        t = self.data['timestamp'][i]
                        row = [t, 0] 
                        for k in keys[2:]:
                            row.append(self.data[k][i])
                        
                        lbl = events_map.get(t, "")
                        row.append(lbl)
                        source_data.append(row)
                        
                writer.writerows(source_data)
            
            print(f"Exported signals to {signals_path}")

            # 2. Export FFT Data -> FFT.csv
            if self.recording_fft:
                fft_path = os.path.join(save_dir, "FFT.csv")
                    
                with open(fft_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Header
                    num_bins = len(self.recording_fft[0].get('data', []))
                    header = ['timestamp', 'recv_ts'] + [f'bin_{i}' for i in range(num_bins)]
                    writer.writerow(header)
                    
                    for frame in self.recording_fft:
                        t = frame.get('t', 0)
                        recv_ts = frame.get('recv_ts', 0)
                        bins = frame.get('data', [])
                        row = [t, recv_ts] + bins
                        writer.writerow(row)
                        
                print(f"Exported FFT data to {fft_path}")
            
            # Show confirmation
            self.text_raw.append(f">> SAVED RECORDING: {folder_name}")

        except Exception as e:
            print(f"Export failed: {e}")
            self.text_raw.append(f"!! EXPORT FAILED: {e}")

    def load_replay_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Replay Data", "", "CSV Files (*.csv)")
        if not path:
            return
            
        try:
            self.replay_data = []
            self.replay_fft_data = []
            
            # Temporary buffers
            temp_signals = []
            temp_fft = []
            
            # 1. Load Main Data (Signals)
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    item = {}
                    for k, v in row.items():
                        if k == 'label':
                            item[k] = v
                        else:
                            try:
                                item[k] = float(v)
                            except:
                                item[k] = 0.0
                    temp_signals.append(item)
            
            # 2. Check for and Load FFT Data
            if path.lower().endswith('signals.csv'):
                # If we loaded signals.csv, look for FFT.csv in same folder
                import os
                dir_name = os.path.dirname(path)
                fft_path = os.path.join(dir_name, "FFT.csv")
            elif path.lower().endswith('.csv'):
                # Fallback for old style or generic
                fft_path = path[:-4] + "_fft.csv"
            else:
                fft_path = path + "_fft.csv"
                
            import os
            if os.path.exists(fft_path):
                try:
                    with open(fft_path, 'r') as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            # Parse metadata
                            fft_item = {}
                            # Extract timestamp and recv_ts explicitly
                            if 'timestamp' in row: fft_item['timestamp'] = float(row['timestamp'])
                            if 'recv_ts' in row: fft_item['recv_ts'] = float(row['recv_ts'])
                            
                            # Bins
                            bins = []
                            bin_keys = [k for k in row.keys() if k.startswith('bin_')]
                            bin_keys.sort(key=lambda x: int(x.split('_')[1]))
                            for k in bin_keys:
                                try:
                                    bins.append(float(row[k]))
                                except:
                                    bins.append(0.0)
                            fft_item['data'] = bins
                            temp_fft.append(fft_item)
                            
                    print(f"Loaded {len(temp_fft)} FFT frames.")
                except Exception as e:
                    print(f"Failed to load FFT sidecar: {e}")

            # 3. Time Synchronization Logic
            # Determine which timestamp field to use ('timestamp' or 'recv_ts')
            # If signals 'timestamp' and FFT 'timestamp' are disjoint, try 'recv_ts'.
            
            def get_time_range(data_list, key):
                if not data_list: return 0, 0
                valid = [d[key] for d in data_list if key in d and d[key] != 0]
                if not valid: return 0, 0
                return min(valid), max(valid)

            use_recv_ts = False
            
            if temp_signals and temp_fft:
                sig_min, sig_max = get_time_range(temp_signals, 'timestamp')
                fft_min, fft_max = get_time_range(temp_fft, 'timestamp')
                
                # Check overlap
                overlap = max(0, min(sig_max, fft_max) - max(sig_min, fft_min))
                
                # If overlap is small (e.g. < 100ms) compared to duration, or completely disjoint
                # Check if the gap is huge (e.g. > 1 minute) which implies epoch mismatch
                gap = 0
                if overlap == 0:
                    gap = abs(fft_min - sig_min)
                
                print(f"Time Sync Check: Sig[{sig_min:.0f}-{sig_max:.0f}] FFT[{fft_min:.0f}-{fft_max:.0f}] Gap={gap:.0f}")
                
                # If gap is large (> 10 seconds), try recv_ts
                if gap > 10000: 
                    print("Timestamps diverged significantly. Checking recv_ts...")
                    sig_r_min, sig_r_max = get_time_range(temp_signals, 'recv_ts')
                    fft_r_min, fft_r_max = get_time_range(temp_fft, 'recv_ts')
                    
                    overlap_r = max(0, min(sig_r_max, fft_r_max) - max(sig_r_min, fft_r_min))
                    gap_r = 0
                    if overlap_r == 0:
                        gap_r = abs(fft_r_min - sig_r_min)
                        
                    print(f"Recv_ts Sync Check: Sig[{sig_r_min:.0f}-{sig_r_max:.0f}] FFT[{fft_r_min:.0f}-{fft_r_max:.0f}] Gap={gap_r:.0f}")
                    
                    if gap_r < 10000: # Reasonable sync in wall clock
                        use_recv_ts = True
                        print(">> Switching to recv_ts for synchronization.")

            # 4. Assign 't' based on decision
            time_key = 'recv_ts' if use_recv_ts else 'timestamp'
            
            # Process Signals
            self.replay_data = []
            for item in temp_signals:
                # Fallback logic if primary key is missing
                ts = item.get(time_key, 0)
                if ts == 0: 
                    ts = item.get('timestamp', 0)
                if ts == 0:
                    ts = item.get('t', 0)
                
                item['t'] = ts
                self.replay_data.append(item)

            # Process FFT
            self.replay_fft_data = []
            for item in temp_fft:
                ts = item.get(time_key, 0)
                if ts == 0:
                    ts = item.get('timestamp', 0)
                
                item['t'] = ts
                self.replay_fft_data.append(item)

            # 5. Normalize and Finalize
            if self.replay_data:
                # Sort
                self.replay_data.sort(key=lambda x: x['t'])
                if self.replay_fft_data:
                    self.replay_fft_data.sort(key=lambda x: x['t'])
                
                # Configure view from first row
                self.configure_view_from_row(self.replay_data[0])
                
                # Normalize time
                start_time = self.replay_data[0]['t']
                
                for d in self.replay_data:
                    d['t'] = d['t'] - start_time
                    
                for d in self.replay_fft_data:
                    d['t'] = d['t'] - start_time

                self.slider_replay.setRange(0, len(self.replay_data) - 1)
                self.slider_replay.setValue(0)
                self.replay_index = 0
                self.update_replay_plot_snapshot()
                print(f"Loaded {len(self.replay_data)} samples for replay.")
                
        except Exception as e:
            print(f"Replay Load Failed: {e}")
            import traceback
            traceback.print_exc()

    def configure_view_from_row(self, row):
        # Helper to enable checkboxes based on data presence (non-zero)
        def check_group(grp_chk, sub_chks, keys):
            has_data = False
            for k in keys:
                if abs(row.get(k, 0)) > 0.000001:
                    has_data = True
                    break
            
            if has_data:
                grp_chk.setChecked(True)
                for k in keys:
                    # If the specific key has data, check it
                    if k in sub_chks and abs(row.get(k, 0)) > 0.000001:
                        sub_chks[k].setChecked(True)
            else:
                grp_chk.setChecked(False)
        
        # Mag Filtered
        check_group(self.grp_mag, self.chk_mag, ['mx', 'my', 'mz', 'mag'])
        # Mag Raw
        check_group(self.grp_raw, self.chk_raw, ['rmx', 'rmy', 'rmz'])
        # Current
        check_group(self.grp_cur, self.chk_cur, ['cur'])
        # Slip
        check_group(self.grp_slip, self.chk_slip, ['slip', 's_ind'])
        # Servo
        check_group(self.grp_srv, self.chk_srv, ['srv', 'grp'])

    def toggle_replay(self):
        if self.replay_timer.isActive():
            self.replay_timer.stop()
            self.btn_play_replay.setText("Play")
        else:
            if not self.replay_data:
                return
            self.replay_timer.start(33) # 30ms playback
            self.btn_play_replay.setText("Pause")

    def on_replay_slider_move(self, val):
        self.replay_index = val
        self.update_replay_plot_snapshot()

    def update_replay_loop(self):
        if self.replay_index < len(self.replay_data) - 1:
            self.replay_index += 1
            self.slider_replay.setValue(self.replay_index)
            self.update_replay_plot_snapshot()
        else:
            self.toggle_replay() # Stop at end

    def update_replay_plot_snapshot(self):
        if not self.replay_data:
            return
            
        # Show a window of data
        window = 500
        start_idx = max(0, self.replay_index - window)
        end_idx = self.replay_index + 1
        
        subset = self.replay_data[start_idx:end_idx]
        
        t = [d.get('t', 0) for d in subset]
        
        visible_values = []

        # Helper to update if visible
        for key, curve in self.replay_curves.items():
            if curve.isVisible():
                y = [d.get(key, 0) for d in subset]
                curve.setData(t, y)
                if self.chk_center_dc.isChecked():
                    visible_values.extend(y)
        
        # Handle Center DC Scaling for Replay
        if self.chk_center_dc.isChecked() and visible_values:
            avg = np.mean(visible_values)
            rng = self.spin_center_range.value()
            self.plot_replay.setYRange(avg - rng, avg + rng, padding=0)
        
        # Update FFT if available
        cur_t = self.replay_data[self.replay_index].get('t', 0)
        
        if self.replay_fft_data:
            # Find the most recent FFT frame <= cur_t
            # Since data is sorted, we can search
            # Simple linear search backward from last known index or just linear scan if small enough
            # For robustness, let's find the frame with closest timestamp
            
            # Efficient search:
            import bisect
            # Create a list of timestamps
            # (Optimization: store this list separately to avoid rebuilding)
            # For now, simple iteration since replay loop is 33ms
            
            best_frame = None
            for frame in self.replay_fft_data:
                if frame['t'] > cur_t:
                    break
                best_frame = frame
                
            # Or better: keep track of an index? 
            # If we scrub backwards, index approach needs reset.
            # Let's do the simple loop, it's fast enough for <10k frames
            
            if best_frame:
                fft_vals = best_frame['data']
                # X axis
                sample_rate = self.spin_fft_rate.value()
                num_bins = len(fft_vals)
                freqs = np.linspace(0, sample_rate / 2, num_bins)
                self.curve_replay_fft.setData(freqs, fft_vals)
            else:
                self.curve_replay_fft.clear()
        
        # Update label
        self.lbl_replay_time.setText(f"{cur_t:.2f} ms")
        
        # Force auto-range on X for replay to follow the data
        if t:
            self.plot_replay.setXRange(min(t), max(t) + 0.1, padding=0)

    def import_data(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import Data", "", "CSV Files (*.csv);;JSON Files (*.json)")
        if not path:
            return

        try:
            # Clear current data
            for k in self.data:
                self.data[k] = []
            self.recorded_events = []
            
            if path.endswith('.csv'):
                with open(path, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        # Parse row
                        try:
                            t = float(row.get('timestamp', 0))
                            lbl = row.get('label', '')

                            for k in ['mx', 'my', 'mz', 'mag', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']:
                                self.data[k].append(float(row.get(k, 0)))
                            
                            self.data['timestamp'].append(t)
                            
                            if lbl:
                                self.recorded_events.append({'timestamp': t, 'label': lbl})
                        except ValueError:
                            continue
                            
            elif path.endswith('.json'):
                # Assuming JSON is a list of objects or a struct
                with open(path, 'r') as f:
                    content = json.load(f)
                    # Check format
                    if isinstance(content, list):
                        for item in content:
                            self.append_data(item)
                    elif isinstance(content, dict) and 'data' in content:
                         # Handle specific export formats if defined
                         pass
                         
            print(f"Imported from {path}")
            # Refresh plots immediately
            self.update_loop()
            
        except Exception as e:
            print(f"Import failed: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Set dark palette for standard Qt widgets
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(COLOR_BG))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Base, QColor(COLOR_PANEL))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(COLOR_BG))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Text, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Button, QColor(COLOR_PANEL))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, QColor(COLOR_ACCENT_1))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(COLOR_ACCENT_1))
    palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(palette)
    
    window = AdaptiveGripperGUI()
    window.show()
    sys.exit(app.exec())

