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
                             QTabWidget, QTextEdit, QSpinBox)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QThread, QObject
from PyQt6.QtGui import QColor, QPalette, QFont
import pyqtgraph as pg

# ==========================================
# Constants & Configuration
# ==========================================
DEFAULT_BAUD_RATE = 921600
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
            if self.ser and self.ser.is_open:
                self.ser.close()

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
            'cur': [], 'slip': [], 'timestamp': []
        }
        self.fft_data = {'freqs': [], 'mags': []}
        self.recorded_events = [] # For start/end of segments (Labeling)
        self.current_segment_start = None
        
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

        # 3. Telemetry Control (Commands)
        grp_tele = QGroupBox("Telemetry Control")
        v_tele = QVBoxLayout()
        
        self.chk_cmd_mag = QCheckBox("Mag Filtered")
        self.chk_cmd_mag.toggled.connect(lambda c: self.send_stream_command("mag_filtered", c))
        
        self.chk_cmd_raw = QCheckBox("Mag Raw")
        self.chk_cmd_raw.toggled.connect(lambda c: self.send_stream_command("mag_raw", c))
        
        self.chk_cmd_cur = QCheckBox("Current")
        self.chk_cmd_cur.toggled.connect(lambda c: self.send_stream_command("current", c))
        
        self.chk_cmd_slip = QCheckBox("Slip")
        self.chk_cmd_slip.toggled.connect(lambda c: self.send_stream_command("slip", c))
        
        self.chk_cmd_srv = QCheckBox("Servo/Mode")
        self.chk_cmd_srv.toggled.connect(lambda c: self.send_stream_command("servo", c))
        
        self.chk_cmd_fft = QCheckBox("FFT Mode (Exclusive)")
        self.chk_cmd_fft.toggled.connect(lambda c: self.send_stream_command("fft", c))
        
        v_tele.addWidget(self.chk_cmd_mag)
        v_tele.addWidget(self.chk_cmd_raw)
        v_tele.addWidget(self.chk_cmd_cur)
        v_tele.addWidget(self.chk_cmd_slip)
        v_tele.addWidget(self.chk_cmd_srv)
        v_tele.addWidget(self.chk_cmd_fft)
        
        grp_tele.setLayout(v_tele)
        sidebar_layout.addWidget(grp_tele)

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

        # 5. Plot Overlays
        grp_sig = QGroupBox("Plot Overlays")
        v_sig = QVBoxLayout()
        self.check_mx = QCheckBox("Mag X (Red)")
        self.check_my = QCheckBox("Mag Y (Green)")
        self.check_mz = QCheckBox("Mag Z (Blue)")
        self.check_mag = QCheckBox("Magnitude (White)")
        self.check_cur = QCheckBox("Current (Yellow)")
        
        self.check_mag.setChecked(True)
        
        v_sig.addWidget(self.check_mx)
        v_sig.addWidget(self.check_my)
        v_sig.addWidget(self.check_mz)
        v_sig.addWidget(self.check_mag)
        v_sig.addWidget(self.check_cur)
        grp_sig.setLayout(v_sig)
        sidebar_layout.addWidget(grp_sig)

        # 6. Analysis
        grp_ana = QGroupBox("Analysis")
        v_ana = QVBoxLayout()
        self.lbl_freq = QLabel("Dominant Freq: 0.0 Hz")
        self.lbl_freq.setStyleSheet(f"color: {COLOR_ACCENT_1}; font-size: 11pt; font-weight: bold;")
        v_ana.addWidget(self.lbl_freq)
        grp_ana.setLayout(v_ana)
        sidebar_layout.addWidget(grp_ana)

        # 7. Data Actions
        grp_act = QGroupBox("Data & Labeling")
        v_act = QVBoxLayout()
        
        hbox_label = QHBoxLayout()
        self.btn_success = QPushButton("Mark Success")
        self.btn_success.setStyleSheet(f"background-color: {COLOR_ACCENT_3}; color: #000;")
        self.btn_fail = QPushButton("Mark Fail")
        self.btn_fail.setStyleSheet(f"background-color: {COLOR_ACCENT_2}; color: #fff;")
        hbox_label.addWidget(self.btn_success)
        hbox_label.addWidget(self.btn_fail)
        
        self.btn_success.clicked.connect(lambda: self.mark_event("Success"))
        self.btn_fail.clicked.connect(lambda: self.mark_event("Failure"))
        
        self.btn_export = QPushButton("Export Data (CSV)")
        self.btn_export.clicked.connect(self.export_data)
        self.btn_import = QPushButton("Import Data (JSON/CSV)")
        self.btn_import.clicked.connect(self.import_data)

        v_act.addLayout(hbox_label)
        v_act.addWidget(self.btn_export)
        v_act.addWidget(self.btn_import)
        
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
        
        # Tab 2: Raw Output
        self.tab_raw = QWidget()
        layout_raw = QVBoxLayout(self.tab_raw)
        self.text_raw = QTextEdit()
        self.text_raw.setReadOnly(True)
        self.text_raw.setStyleSheet(f"background-color: #000; color: #0f0; font-family: Consolas, monospace;")
        layout_raw.addWidget(self.text_raw)
        
        self.tabs.addTab(tab_viz, "Visualizer")
        self.tabs.addTab(self.tab_raw, "Raw Serial")
        
        main_layout.addWidget(self.tabs, stretch=1)
    
    def send_stream_command(self, key, enabled):
        if self.is_connected and self.serial_thread:
            cmd = json.dumps({key: enabled})
            self.serial_thread.send_command(cmd)
            # Log sent command
            self.text_raw.append(f">> SENT: {cmd}")

    def setup_plotting(self):
        # Create curves
        self.curves = {}
        # Colors: R, G, B, W, Y
        self.curves['mx'] = self.plot_time.plot(pen=pg.mkPen(COLOR_ACCENT_2, width=2), name='Mag X')
        self.curves['my'] = self.plot_time.plot(pen=pg.mkPen(COLOR_ACCENT_3, width=2), name='Mag Y')
        self.curves['mz'] = self.plot_time.plot(pen=pg.mkPen(COLOR_ACCENT_1, width=2), name='Mag Z')
        self.curves['mag'] = self.plot_time.plot(pen=pg.mkPen('#ffffff', width=2), name='Magnitude')
        self.curves['cur'] = self.plot_time.plot(pen=pg.mkPen(COLOR_ACCENT_4, width=2), name='Current')
        
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
        
        # 1. FFT Data
        if data.get('type') == 'fft':
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
        
        for key in ['mx', 'my', 'mz', 'mag', 'cur', 'slip']:
            val = data.get(key, 0.0)
            self.data[key].append(val)
            
        self.data['timestamp'].append(ts)
        
        # Keep buffer size
        if len(self.data['timestamp']) > self.buffer_size:
            for k in self.data:
                self.data[k] = self.data[k][-self.buffer_size:]

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
            # Update visibility based on checkboxes
            if self.check_mx.isChecked():
                self.curves['mx'].setData(self.data['mx'])
                self.curves['mx'].setVisible(True)
            else:
                self.curves['mx'].setVisible(False)
                
            if self.check_my.isChecked():
                self.curves['my'].setData(self.data['my'])
                self.curves['my'].setVisible(True)
            else:
                self.curves['my'].setVisible(False)
                
            if self.check_mz.isChecked():
                self.curves['mz'].setData(self.data['mz'])
                self.curves['mz'].setVisible(True)
            else:
                self.curves['mz'].setVisible(False)
                
            if self.check_mag.isChecked():
                self.curves['mag'].setData(self.data['mag'])
                self.curves['mag'].setVisible(True)
            else:
                self.curves['mag'].setVisible(False)
                
            if self.check_cur.isChecked():
                self.curves['cur'].setData(self.data['cur'])
                self.curves['cur'].setVisible(True)
            else:
                self.curves['cur'].setVisible(False)

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
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV", "", "CSV Files (*.csv)")
        if path:
            try:
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Header
                    keys = ['timestamp', 'mx', 'my', 'mz', 'mag', 'cur', 'slip']
                    writer.writerow(keys + ['label'])
                    
                    # Create a lookup for events
                    # Events are stored with timestamp.
                    # We will match if timestamp is within a small epsilon or exact match
                    events_map = {e['timestamp']: e['label'] for e in self.recorded_events}
                    
                    for i in range(len(self.data['timestamp'])):
                        t = self.data['timestamp'][i]
                        row = [self.data[k][i] for k in keys]
                        
                        # Check for label (exact match for now, or match if close)
                        lbl = events_map.get(t, "")
                        writer.writerow(row + [lbl])
                        
                print(f"Exported to {path}")
            except Exception as e:
                print(f"Export failed: {e}")

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
                            mx = float(row.get('mx', 0))
                            my = float(row.get('my', 0))
                            mz = float(row.get('mz', 0))
                            mag = float(row.get('mag', 0))
                            cur = float(row.get('cur', 0))
                            slip = float(row.get('slip', 0))
                            lbl = row.get('label', '')
                            
                            self.data['timestamp'].append(t)
                            self.data['mx'].append(mx)
                            self.data['my'].append(my)
                            self.data['mz'].append(mz)
                            self.data['mag'].append(mag)
                            self.data['cur'].append(cur)
                            self.data['slip'].append(slip)
                            
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

